//! worker 线程 —— 所有阻塞 IO 的唯一归属地。
//!
//! # 为什么必须有这一层
//!
//! [`scope_core::CommandBus`] 是同步的：`connect()` 最坏约 1.7 s（PING 500 ms × 3 次重试），
//! 控制命令最坏约 660 ms，等触发最长 2 s。放在 UI 线程里，窗口直接假死。
//!
//! ```text
//! ┌─ UI 线程 ──────────┐        ┌─ worker 线程 ────────┐
//! │ App  每帧 drain()  │ ◄──rx──│ CommandBus<Transport>│
//! │                   │ ──tx──►│ 阻塞 IO 全在这里      │
//! └───────────────────┘        └──────────────────────┘
//!         ▲                             │
//!         └── ctx.request_repaint() ◄───┘
//! ```
//!
//! # 两条容易踩的线
//!
//! 1. **`Request` 通道必须是无界的**（`std::sync::mpsc` 默认）。用 `sync_channel`
//!    的话队列一满，UI 线程的 `send` 就会阻塞 —— 这是整套设计里唯一的真死锁入口。
//! 2. **`request_repaint` 只在状态真正变化时调**。放进高频轮询会长期烧掉一个核。

use crate::msg::{describe_error, Request, Update};
use scope_core::{CommandBus, ScopeError, State};
use scope_device::{Transport, TransportKind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

/// worker 的句柄。UI 侧只拿这三个东西。
pub struct Worker {
    tx: Sender<Request>,
    rx: Receiver<Update>,
    cancel: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Worker {
    /// 起一个 worker 线程。
    ///
    /// `ctx` 用于在结果就绪时唤醒 UI —— `egui::Context` 是 `Clone + Send`，
    /// 从 worker 调用 `request_repaint` 是标准用法，无死锁风险。
    pub fn spawn(ctx: egui::Context) -> Worker {
        let (tx, req_rx) = mpsc::channel::<Request>();
        let (upd_tx, rx) = mpsc::channel::<Update>();
        let cancel = Arc::new(AtomicBool::new(false));

        let handle = {
            let cancel = Arc::clone(&cancel);
            std::thread::Builder::new()
                .name("scope-worker".into())
                .spawn(move || {
                    let mut worker = WorkerState {
                        bus: None,
                        cancel,
                        tx: upd_tx,
                        ctx,
                    };
                    worker.run(req_rx);
                })
                .expect("无法创建 worker 线程")
        };

        Worker {
            tx,
            rx,
            cancel,
            handle: Some(handle),
        }
    }

    /// 发一个请求。**不会阻塞**（通道无界），UI 永不卡。
    pub fn send(&self, req: Request) {
        // 发送失败只可能是 worker 已退出 —— 那不是 UI 需要处理的情况
        let _ = self.tx.send(req);
    }

    /// 排空所有待处理的回执。**不阻塞**，每帧调用一次。
    pub fn drain(&self, mut on_update: impl FnMut(Update)) {
        while let Ok(u) = self.rx.try_recv() {
            on_update(u);
        }
    }

    /// 请求取消当前长操作。
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// worker 是否已经死了（panic 或提前退出）。
    ///
    /// GUI 最大的风险不是死锁，而是「worker 悄悄没了，UI 永远转圈」。
    pub fn is_dead(&self) -> bool {
        self.handle.as_ref().is_some_and(|h| h.is_finished())
    }

    /// 收尾：请求关闭并**限时**等待。
    ///
    /// 不能无超时 join —— worker 可能正阻塞在 2 s 的等待触发里，
    /// 那样关窗口会卡住。这里给它 300 ms 发一次 STOP 的机会。
    pub fn shutdown(&mut self) {
        self.cancel.store(true, Ordering::Relaxed);
        let _ = self.tx.send(Request::Shutdown);
        if let Some(h) = self.handle.take() {
            let deadline = std::time::Instant::now() + std::time::Duration::from_millis(300);
            while !h.is_finished() && std::time::Instant::now() < deadline {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            // 到点还没停就放弃 join —— 线程会随进程一起结束
        }
    }
}

/// worker 线程内部的状态。
struct WorkerState {
    bus: Option<CommandBus<Transport>>,
    cancel: Arc<AtomicBool>,
    tx: Sender<Update>,
    ctx: egui::Context,
}

impl WorkerState {
    /// 在已连接的设备上执行一段操作。
    ///
    /// 存在的意义是把 `self.bus.as_mut()` 的借用**限制在一个表达式内** ——
    /// 直接用 `let Some(bus) = self.bus.as_mut() else { self.fail(..) }` 的话，
    /// 借用会跨过后面的 `self.emit(..)` / `self.fail(..)`，编译器不让过。
    fn with_bus<T>(
        &mut self,
        action: &'static str,
        f: impl FnOnce(&mut CommandBus<Transport>) -> Result<T, ScopeError>,
    ) -> Result<T, ScopeError> {
        match self.bus.as_mut() {
            Some(bus) => f(bus),
            None => Err(ScopeError::BadState {
                current: State::Idle,
                action,
                hint: "尚未连接设备，先点「连接」".into(),
            }),
        }
    }

    /// 发回执并唤醒 UI。
    fn emit(&self, u: Update) {
        if self.tx.send(u).is_ok() {
            // 只在真有变化时唤醒，不要放进高频轮询
            self.ctx.request_repaint();
        }
    }

    /// 切换模拟器场景后重新握手，刷新 `info`（含 ch_count）。
    ///
    /// 只重发 GET_INFO + GET_CONFIG，**不重建 Transport、不丢连接状态** ——
    /// 采集循环按 `bus.info.ch_count` 决定拉几个通道，不刷新的话
    /// 从单通道场景切到 I2C 双通道只会显示一条泳道。
    fn refresh_after_scenario_switch(&mut self) {
        // 借用限制在一个表达式内（与 `with_bus` 同一条规矩）
        let done = match self.bus.as_mut() {
            Some(bus) => match bus.connect() {
                Ok(info) => {
                    let state = bus.get_status().map(|s| s.state).unwrap_or(State::Idle);
                    Some((info, bus.config.clone(), state, bus.is_simulated()))
                }
                Err(e) => {
                    self.fail(&e);
                    None
                }
            },
            None => None,
        };
        if let Some((info, config, state, simulated)) = done {
            self.emit(Update::Connected {
                info,
                config,
                state,
                simulated,
            });
        }
    }

    fn fail(&self, e: &ScopeError) {
        let (text, hint) = describe_error(e);
        self.emit(Update::Failed { text, hint });
    }

    fn busy(&self, op: &'static str, active: bool) {
        self.emit(Update::Busy { op, active });
    }

    fn run(&mut self, rx: Receiver<Request>) {
        // 收到 Shutdown 或发送端全部丢弃时退出
        while let Ok(req) = rx.recv() {
            let stop = matches!(req, Request::Shutdown);
            self.handle(req);
            if stop {
                break;
            }
        }
        // 收尾：给真机留一次 STOP 的机会
        if let Some(bus) = self.bus.as_mut() {
            let _ = bus.stop();
        }
    }

    fn handle(&mut self, req: Request) {
        match req {
            Request::Shutdown => {}

            Request::ListPorts => {
                self.emit(Update::Ports(scope_transport_serial::list_ports()));
            }

            Request::Connect {
                transport,
                port,
                baud,
                scenario,
            } => {
                self.bus = None; // 旧连接一律丢弃，避免两根链路同时活着
                self.busy("连接", true);

                let port_dev = match transport {
                    TransportKind::Sim => Transport::sim(scenario),
                    TransportKind::Serial => match Transport::serial(&port, baud) {
                        Ok(d) => d,
                        Err(e) => {
                            self.fail(&ScopeError::Link(e));
                            self.busy("连接", false);
                            return;
                        }
                    },
                };

                let mut bus = CommandBus::new(port_dev);
                match bus.connect() {
                    Ok(info) => {
                        // `connect()` 只发 GET_INFO + GET_CONFIG，**不发 GET_STATUS**。
                        // 不补这一次的话 `bus.state` 一直是 None，
                        // `guard_config_allowed` 就失去判据 —— 已武装时照样能改配置。
                        let state = bus.get_status().map(|s| s.state).unwrap_or(State::Idle);
                        self.emit(Update::Connected {
                            info,
                            config: bus.config.clone(),
                            state,
                            // `is_simulated()` 由传输层如实回答（Transport 转发给 SimDevice）。
                            // 漏转发的话这里会一直报 false，模拟器面板就永远不出现。
                            simulated: bus.is_simulated(),
                        });
                        self.bus = Some(bus);
                    }
                    Err(e) => self.fail(&e),
                }
                self.busy("连接", false);
            }

            Request::Disconnect => {
                if let Some(bus) = self.bus.as_mut() {
                    let _ = bus.stop();
                }
                self.bus = None;
                self.emit(Update::Disconnected);
            }

            Request::ApplyConfig {
                rate_hz,
                samples,
                trigger_mode,
                trigger_source,
                trigger_edge,
                trigger_level_lsb,
                pre_samples,
                holdoff_us,
            } => {
                self.busy("应用配置", true);
                let r = self.with_bus("应用配置", |bus| {
                    bus.set_sample_rate(rate_hz)?;
                    bus.set_trigger(
                        trigger_mode,
                        trigger_source,
                        trigger_edge,
                        trigger_level_lsb,
                        pre_samples,
                        holdoff_us,
                    )?;
                    bus.set_acq(0, samples, 0, 1)?;
                    // 回显值才是真值 —— `SET_SAMPLE_RATE` 会被定时器分频量化
                    Ok(bus.get_config().ok().or_else(|| bus.config.clone()))
                });
                match r {
                    Ok(Some(cfg)) => self.emit(Update::ConfigApplied(Box::new(cfg))),
                    Ok(None) => {}
                    Err(e) => self.fail(&e),
                }
                self.busy("应用配置", false);
            }

            Request::Acquire {
                samples,
                rate_hz,
                trigger_level_lsb,
                timeout_ms,
            } => {
                self.cancel.store(false, Ordering::Relaxed);
                self.busy("采集中", true);

                let params = scope_core::AcquireParams {
                    samples,
                    rate_hz,
                    trigger_level_lsb,
                    timeout: std::time::Duration::from_millis(timeout_ms),
                };
                // 取消标志先克隆出来 —— 闭包里不能同时借 self
                let cancel = Arc::clone(&self.cancel);
                let r = self.with_bus("采集", |bus| {
                    scope_core::acquire_cancellable(bus, &params, &cancel)
                });
                match r {
                    Ok(cap) => self.emit(Update::Acquired(Box::new(cap))),
                    Err(e) => self.fail(&e),
                }
                self.busy("采集中", false);
            }

            Request::SetScenario(s) => {
                let r = self.with_bus("切换场景", |bus| {
                    // `set_scenario` 会清空已采集的数据（capture_id 立即失效），
                    // 但**不重置状态机** —— 所以先 STOP 再切，顺序不能反。
                    let _ = bus.stop();
                    match bus.port_mut().sim_mut() {
                        Some(sim) => {
                            sim.set_scenario(s);
                            Ok(())
                        }
                        None => Err(ScopeError::Unsupported(
                            "当前连接的不是模拟器，无法切换波形场景".into(),
                        )),
                    }
                });
                match r {
                    Ok(()) => {
                        // **场景换了通道数也会换**（单通道 ↔ I2C 双通道）——
                        // 必须重新 connect 刷新 info，否则采集循环按旧的
                        // `ch_count` 拉数据，双通道场景只显示一条泳道
                        // （2026-10-07 观察：从其他场景切到 I2C 只显示单通道）。
                        // 与 MCP 的 `sim_set_scenario` 同一条规矩。
                        self.refresh_after_scenario_switch();
                    }
                    Err(e) => self.fail(&e),
                }
            }

            Request::SetFaults(faults) => {
                let r = self.with_bus("注入故障", |bus| match bus.port_mut().sim_mut() {
                    Some(sim) => {
                        sim.faults = *faults;
                        Ok(())
                    }
                    None => Err(ScopeError::Unsupported(
                        "当前连接的不是模拟器，无法注入故障".into(),
                    )),
                });
                match r {
                    Ok(()) => self.emit(Update::StateChanged(State::Idle)),
                    Err(e) => self.fail(&e),
                }
            }

            Request::Reset => {
                self.busy("复位", true);
                let r = self.with_bus("复位", |bus| {
                    bus.reset()?;
                    Ok(bus.config.clone())
                });
                match r {
                    Ok(cfg) => {
                        self.emit(Update::StateChanged(State::Idle));
                        if let Some(cfg) = cfg {
                            self.emit(Update::ConfigApplied(Box::new(cfg)));
                        }
                    }
                    Err(e) => self.fail(&e),
                }
                self.busy("复位", false);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scope_sim::Scenario;

    /// 轮询等一个满足条件的回执（worker 是异步的，测试要等它）。
    fn wait_for(w: &Worker, mut pred: impl FnMut(&Update) -> bool) -> Option<Update> {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let mut found = None;
            w.drain(|u| {
                if found.is_none() && pred(&u) {
                    found = Some(u);
                }
            });
            if let Some(u) = found {
                return Some(u);
            }
            if std::time::Instant::now() > deadline {
                return None;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    /// **切场景必须刷新 `info` 的通道数。**
    ///
    /// 2026-10-07 的 bug：从单通道场景切到 I2C 双通道，界面只显示一条泳道 ——
    /// 采集循环按缓存的 `bus.info.ch_count` 拉数据，而切场景只改了模拟器
    /// 内部、没重新握手。
    ///
    /// 变异（杀掉）：把 `refresh_after_scenario_switch` 里的 `bus.connect()`
    /// 删掉（或改成只 emit StateChanged）—— 这条红（等不到第二次 Connected，
    /// 或第二次的 ch_count 还是 1）。
    #[test]
    fn switching_scenario_refreshes_the_channel_count() {
        let mut w = Worker::spawn(egui::Context::default());

        w.send(Request::Connect {
            transport: TransportKind::Sim,
            port: String::new(),
            baud: 921_600,
            scenario: Scenario::Sine1k3v3,
        });
        let first = wait_for(&w, |u| matches!(u, Update::Connected { .. }))
            .expect("第一次连接应在 10 s 内完成");
        match &first {
            Update::Connected { info, .. } => {
                assert_eq!(info.ch_count, 1, "sine 场景是单通道");
            }
            _ => unreachable!(),
        }

        w.send(Request::SetScenario(Scenario::I2c100k));
        let second =
            wait_for(&w, |u| matches!(u, Update::Connected { .. })).expect("切场景后应重新握手");
        match &second {
            Update::Connected { info, .. } => {
                assert_eq!(
                    info.ch_count, 2,
                    "切到 i2c_100k 后 info 必须刷新成 2 通道 —— 否则只显示单通道"
                );
            }
            _ => unreachable!(),
        }

        w.shutdown();
    }
}
