//! 应用状态与布局。
//!
//! # 状态放哪
//!
//! | 状态 | 位置 | 理由 |
//! |---|---|---|
//! | `CommandBus` / 设备真值快照 | worker | 阻塞 IO 只能有一个所有者 |
//! | 采集结果 `CaptureStore` | **本线程** | `Capture: Clone` 很便宜，UI 每帧要随机访问；放 worker 侧就要加锁，而锁一旦落在 worker 手里，2 s 等待期间 UI 会被卡住 |
//! | 解码结果 | 本线程，脏标记缓存 | 4096 点解码很快，但没必要每帧重算 |
//! | 「期望值」与「回显值」 | **分开存** | 采样率会被量化，界面必须能画出「请求 vs 实际」的差异 |

use crate::ai::AiState;
use crate::drive::{self, DriveEvent, DrivePhase, DriveState, Handoff, TrajectoryItem};
use crate::font::FontOutcome;
use crate::msg::{Request, TransportKind, Update};
use crate::panels;
use crate::worker::Worker;
use scope_core::{
    CalibrationStore, Capture, CaptureStore, DeviceConfig, DeviceInfo, I2cDecode, I2cDecodeConfig,
    ScaleSet, State,
};
use scope_sim::Scenario;
use std::sync::atomic::{AtomicBool, Ordering};

/// 应用主状态。
pub struct App {
    /// worker 句柄（所有阻塞 IO 的去处）。
    pub(crate) worker: Worker,

    // ── 连接意图 ──
    /// 传输选择。
    pub(crate) transport_kind: TransportKind,
    /// 串口名。
    pub(crate) port: String,
    /// 波特率。
    pub(crate) baud: u32,
    /// 模拟器场景。
    pub(crate) scenario: Scenario,
    /// 已枚举到的串口。
    pub(crate) ports: Vec<(String, String, bool)>,

    // ── 设备真值（全部来自设备回显）──
    /// 设备能力。
    pub(crate) info: Option<DeviceInfo>,
    /// 设备当前配置（**回显值**）。
    pub(crate) config: Option<DeviceConfig>,
    /// 设备当前状态。
    pub(crate) state: Option<State>,
    /// 是否已连接。
    pub(crate) connected: bool,
    /// 「连续刷新」：收到一帧后立即采下一帧。默认关（docs/09 §8）。
    /// 连续帧不进历史 —— store 只留最近 16 条（容量由 core 决定），
    /// 30 fps 连跑几秒就会把用户手动采的记录全顶掉。
    pub(crate) continuous: bool,

    // ── 期望配置（UI 意图，与真值分开）──
    /// 请求的采样率。
    pub(crate) want_rate: u32,
    /// 请求的点数。
    pub(crate) want_samples: u16,
    /// 触发模式 `0=auto / 1=normal / 2=single`。
    pub(crate) want_trigger_mode: u8,
    /// 触发边沿 `0=上升 / 1=下降`。
    pub(crate) want_trigger_edge: u8,
    /// 触发电平（ADC LSB）。
    pub(crate) want_trigger_level: u16,

    // ── 通道标定 ──
    /// 标定表（按设备 uid 索引）。**只读** —— 写入走 `scope-cli cal set`。
    pub(crate) calib: CalibrationStore,
    /// 当前这一窗该用哪套换算。
    ///
    /// **断开连接时不清**（与 `info`/`config` 不同）—— 因为 `capture` 也不清，
    /// 波形还在屏幕上，纵轴标签必须跟显示的数据一致。
    ///
    /// ⚠ 代价：历史采集里换过设备的那几条，会按**当前**设备的标定渲染。
    /// 真正的修法是给 `Capture` 加 uid 溯源，那会牵动 core、MCP 的 JSON
    /// 与 GUI 的镜像，成本高。本轮接受，在这里写明。
    pub(crate) calib_view: ScaleSet,

    // ── 采集与解码 ──
    /// 当前显示的采集。
    pub(crate) capture: Option<Capture>,
    /// 历史采集（最近 16 条，容量由 core 决定）。
    pub(crate) store: CaptureStore,
    /// 解码结果。
    pub(crate) decode: Option<I2cDecode>,
    /// 解码参数。
    pub(crate) decode_cfg: I2cDecodeConfig,
    /// 解码是否需要重算。
    decode_dirty: AtomicBool,
    /// 当前 SCL/SDA 通道选择。
    pub(crate) scl_channel: usize,
    pub(crate) sda_channel: usize,
    /// 解码协议选择（I2C / UART）。场景切换时给合理默认，用户可改。
    pub(crate) decode_protocol: scope_core::Protocol,
    /// UART 波特率。8N1 其余参数固定（复用 [`scope_core::UartConfig`] 语义）；
    /// 0 会被 core 显式报错，不做隐式推断。
    pub(crate) uart_baud: u32,
    /// UART 解码结果。
    pub(crate) uart_decode: Option<scope_core::DecodedFrames>,
    /// 用户编辑的 CSV 导出路径；空串 = 用默认 `captures/capture_<id>.csv`。
    pub(crate) csv_path: String,

    // ── UI 态 ──
    /// 正在进行的操作名。
    pub(crate) busy: Option<&'static str>,
    /// 已发出、还没等到回执（`Acquired` / `Failed`）的 Acquire。
    ///
    /// 连续刷新的首帧发送用它做守卫，**不能用 `busy`**：`busy` 在
    /// 「应用配置 / 复位」期间同样是 Some，而这两个操作成功只回
    /// `ConfigApplied` / `StateChanged`，永远不会有 Acquired 来续采 ——
    /// 拿 `busy` 守卫会把开关卡在「开着但什么都不采」。
    pub(crate) acquire_in_flight: bool,
    /// 最后一次错误：`(说明, 怎么办)`。
    pub(crate) last_error: Option<(String, Option<String>)>,
    /// 滚动日志。
    pub(crate) log: Vec<String>,
    /// 字体加载结果（没找到时要在界面上说一声）。
    pub(crate) font_notice: Option<String>,
    /// 下次绘制时把图重置到全览。
    pub(crate) fit_pending: bool,
    /// 显示电压还是 LSB。
    pub(crate) show_volts: bool,
    /// 泳道是否重叠显示（默认分开，即示波器的 stacked 模式）。
    pub(crate) lanes_overlap: bool,
    /// 当前连接的是不是模拟器 —— 决定「故障注入」面板要不要出现。
    pub(crate) simulated: bool,
    /// 待注入的故障配置（UI 意图，点「注入」才生效）。
    pub(crate) faults: scope_sim::FaultInjection,
    /// 帮助页是否打开。
    pub(crate) show_help: bool,
    /// `--demo` 的状态机：0=未开始 1=已发连接 2=已发采集。
    demo_stage: u8,

    // ── AI 分析 ──
    /// AI 面板的全部状态（配置、在途请求、结果）。
    ///
    /// 单独成组是有意的：这块**不碰设备**，和 `worker` 那条链路没有任何交集
    /// 除了「读当前采集」。共用一个 `AiState` 字段比往 `App` 上再摊十几个字段好读。
    pub(crate) ai: AiState,

    /// 「让 AI 自己配置并采集」的状态（阶段、会话句柄、轨迹、交接快照）。
    ///
    /// ⚠ 与 `worker` 是**互斥**的：这个非 Idle 时设备在子进程手上，
    /// `worker` 那边即使还活着也够不到设备 —— 见 [`DrivePhase::device_is_elsewhere`]。
    pub(crate) drive: DriveState,
}

impl App {
    /// 建一个 App 并挂上 worker。
    pub fn new(
        ctx: &egui::Context,
        font: FontOutcome,
        demo: bool,
        demo_cont: bool,
        scenario: Option<Scenario>,
    ) -> App {
        let worker = Worker::spawn(ctx.clone());
        // 首启预选模拟器 + I2C 场景 —— 零硬件的人双击就能看到东西
        worker.send(Request::ListPorts);

        // AI 状态与它的配置来源说明。**只调一次 `load`** ——
        // 它内部会建 `AiWorker`、读一次配置文件，调两次就等于建了两个 worker。
        let (ai, ai_note) = AiState::load(ctx.clone());

        let mut app = App {
            worker,
            transport_kind: TransportKind::Sim,
            port: String::new(),
            baud: 921_600,
            scenario: scenario.unwrap_or(Scenario::I2c100k),
            ports: Vec::new(),

            info: None,
            config: None,
            state: None,
            connected: false,
            // `--demo-cont`：演示模式直接开连续刷新 —— 用于脚本验证
            // 「连续采集下缩放/平移是否仍可用」这类交互问题
            continuous: demo_cont,

            want_rate: scope_core::f103::MAX_SAMPLE_RATE_HZ,
            want_samples: 4096,
            // auto —— 与设备上电默认一致（`firmware/App/acq.c` 的 `acq_init`）。
            // 默认 normal 时，接一个直流或没接信号会**必然**等不到边沿、
            // 2 s 超时，用户看到的是「采集没结果」。
            want_trigger_mode: 0,
            want_trigger_edge: 0, // 上升
            want_trigger_level: 2048,

            calib: CalibrationStore::load_default(),
            calib_view: ScaleSet::uncalibrated(0),

            capture: None,
            store: CaptureStore::default(),
            decode: None,
            decode_cfg: I2cDecodeConfig::default(),
            decode_dirty: AtomicBool::new(false),
            scl_channel: 0,
            sda_channel: 1,
            decode_protocol: scope_core::Protocol::I2c,
            uart_baud: 115_200,
            uart_decode: None,
            csv_path: String::new(),

            busy: None,
            acquire_in_flight: false,
            last_error: None,
            log: Vec::new(),
            font_notice: font.notice(),
            fit_pending: false,
            show_volts: true,
            lanes_overlap: false,
            simulated: false,
            faults: scope_sim::FaultInjection::default(),
            show_help: false,
            demo_stage: if demo { 0 } else { 3 },

            ai,
            drive: DriveState::default(),
        };

        // 字体去向写进日志 —— 加载成功也留一条，方便排查「为什么中文是方框」
        match &font {
            FontOutcome::Loaded { path } => app.note(format!("界面字体：{path}")),
            FontOutcome::NotFound { .. } => app.note("⚠ 未找到中文字体，中文会显示为方框"),
        }

        // AI 配置的来源说明记入日志。
        //
        // **不放在面板上常驻显示**：它描述的是「文件在哪、读到没有」这类部署细节，
        // 属于日志内容；放在操作界面上既挤占空间，又每次重绘都在重复同一句话。
        // 密钥的存储方式与安全边界另见帮助页「注意事项」。
        if let Some(note) = ai_note {
            app.note(format!("AI 配置：{note}"));
        }

        // `--demo` 下把 AI 面板一并展开：这个开关的用途之一就是核对布局，
        // 而收起状态的面板截不到任何东西（见 main.rs 里 `--demo` 那段注释）。
        app.ai.panel_open = demo;

        app
    }

    // ══════════════════════════════════════════════════════════════
    // AI 驱动设备：交接 / 状态机 / 事件
    // ══════════════════════════════════════════════════════════════

    /// 设备此刻是不是不在 GUI 手上。
    ///
    /// 界面上一堆控件靠它禁用 —— 而这**不是 UI 的偏好，是串口独占的必然**：
    /// 设备已经让给子进程了，那些按钮点下去也够不到东西。
    pub(crate) fn device_is_elsewhere(&self) -> bool {
        self.drive.phase.device_is_elsewhere()
    }

    /// 现在能不能把设备交给 AI。
    pub(crate) fn can_start_drive(&self) -> bool {
        self.connected && !self.is_busy() && !self.drive.phase.is_busy()
    }

    /// 用户点了「交给 AI」：**先把设备让出去**。
    ///
    /// 顺序不能反 —— 必须先 `Disconnect` 把串口放开，子进程才连得上。
    /// 真正的会话在 [`Self::step_drive`] 里等 `Update::Disconnected` 之后才起。
    pub(crate) fn start_drive(&mut self) {
        if !self.can_start_drive() {
            return;
        }
        let task = self.ai.question.trim().to_string();
        if task.is_empty() {
            self.note("需求为空。请在输入框中填写。");
            return;
        }

        // 快照 —— 会话期间与收回来之后都以它为唯一数据来源
        self.drive.handoff = Some(Handoff {
            link: self.link_description(),
            simulated: self.simulated,
            config: self.config.clone(),
            transport: self.transport_kind,
            port: self.port.clone(),
            baud: self.baud,
            scenario: self.scenario,
        });
        self.drive.task = task;
        self.drive.begin();
        self.drive.phase = DrivePhase::Disconnecting;

        // 停掉 --demo 自动机 —— 否则它会在 AI 跑的时候插一脚
        self.demo_stage = 3;

        // 交接期间设备不在 GUI 手上：续采（或刚排队的那一发）会撞上断开，
        // 报一条与事实不符的错误。连续刷新是「断开即停」的语义，这里先关。
        self.continuous = false;

        self.note("设备交接：GUI 断开，释放串口");
        self.worker.send(Request::Disconnect);
    }

    /// 推一步状态机。在 `logic()` 里每帧调用。
    pub(crate) fn step_drive(&mut self, ctx: &egui::Context) {
        match self.drive.phase {
            // 断开完成 → 起会话
            DrivePhase::Disconnecting if !self.connected => {
                self.spawn_drive(ctx);
            }
            // 重新连上 → 回到 Idle
            DrivePhase::Reconnecting if self.connected => {
                self.drive.phase = DrivePhase::Idle;
                self.note(format!(
                    "设备已收回。AI 的改动保留在设备上，可用「{}」改回",
                    drive::RESTORE_LABEL
                ));
            }
            _ => {}
        }
    }

    /// 真正起会话线程（设备已经让出去了）。
    fn spawn_drive(&mut self, ctx: &egui::Context) {
        let Some(h) = self.drive.handoff.clone() else {
            self.drive.phase = DrivePhase::Idle;
            return;
        };
        let job = drive::DriveJob {
            task: self.drive.task.clone(),
            cfg: self.ai.cfg.clone(),
            connect_args: h.connect_args(),
        };
        let generation = self.drive.next_generation();
        self.drive.handle = Some(drive::spawn_session(job, generation, ctx.clone()));
        self.drive.phase = DrivePhase::Running;
    }

    /// 收一条会话事件。
    fn apply_drive(&mut self, e: DriveEvent) {
        match e {
            DriveEvent::Phase(p) => {
                self.drive.push(TrajectoryItem::Phase(p.clone()));
                self.note(format!("AI：{p}"));
            }
            DriveEvent::ToolCall { name, args } => {
                self.drive.push(TrajectoryItem::Call {
                    name: name.clone(),
                    args,
                });
            }
            DriveEvent::ToolResult { name, ok, summary } => {
                self.drive.push(TrajectoryItem::Result {
                    name: name.clone(),
                    ok,
                    summary,
                });
            }
            DriveEvent::Warnings(ws) => {
                for w in ws {
                    self.drive.push(TrajectoryItem::Warning(w.clone()));
                    self.note(format!("AI 警告：{w}"));
                }
            }
            DriveEvent::Capture(c) => {
                let cap = *c;
                self.drive.push(TrajectoryItem::Capture {
                    id: cap.id,
                    points: cap.channels.first().map(|v| v.len()).unwrap_or(0),
                    channels: cap.channels.len(),
                });
                self.note(format!(
                    "AI 采到 #{}（{} 点）—— 已上屏",
                    cap.id,
                    cap.channels.first().map(|v| v.len()).unwrap_or(0)
                ));
                // 与 worker 那条路走同一套：进历史、换当前显示、标脏重算解码
                self.store.push(cap.clone());
                self.capture = Some(cap);
                self.decode_dirty.store(true, Ordering::Relaxed);
                self.fit_pending = true;
            }
            DriveEvent::Finished {
                text,
                turns,
                hit_limit,
            } => {
                self.drive.final_text = Some(text);
                self.drive.push(TrajectoryItem::Phase(if hit_limit {
                    format!("已达轮数上限而停止（{turns} 轮）")
                } else {
                    format!("跑完（{turns} 轮）")
                }));
                self.return_device();
            }
            // ⚠ 出错也**必须**把设备收回来 —— 否则界面永远停在
            // 「AI 正在操作设备」，而设备谁也拿不回来。
            DriveEvent::Failed(e) => {
                self.note(format!("AI 会话失败：{}（{}）", e.message, e.hint));
                self.drive.push(TrajectoryItem::Phase(format!(
                    "✗ {} —— {}",
                    e.message, e.hint
                )));
                self.return_device();
            }
        }
    }

    /// 把设备收回来 —— 按交接快照连回**同一个目标**。
    fn return_device(&mut self) {
        let Some(h) = self.drive.handoff.clone() else {
            self.drive.phase = DrivePhase::Idle;
            return;
        };
        self.drive.phase = DrivePhase::Reconnecting;
        self.note("会话结束，正在收回设备");
        self.worker.send(Request::Connect {
            transport: h.transport,
            port: h.port,
            baud: h.baud,
            scenario: h.scenario,
        });
    }

    /// 用户点了「终止」。
    ///
    /// **直接杀**，不走优雅收场 —— 用户点了终止就该立刻停。
    /// 代价是真机可能停在 Armed：重连读到状态后会提示点复位。
    pub(crate) fn stop_drive(&mut self) {
        let Some(h) = self.drive.handle.take() else {
            return;
        };
        // 作废这一代 —— 旧线程还会吐几条事件出来，不能混进下一次会话
        self.drive.next_generation();
        h.terminate();
        self.note("会话已终止（子进程已结束）。设备上保留其最后的配置。");
        self.return_device();
    }

    /// 按交接快照把设备设置改回去。
    pub(crate) fn restore_handoff_config(&mut self) {
        let Some(h) = self.drive.handoff.as_ref() else {
            return;
        };
        let Some(c) = h.config.as_ref() else {
            self.note("交接时未读到设备配置，无法恢复");
            return;
        };
        // 把「期望值」同步成快照 —— 用户点一次「应用到设备」即可
        self.want_rate = c.rate_hz;
        self.want_samples = c.capture_samples;
        self.want_trigger_mode = c.trigger_mode;
        self.want_trigger_edge = c.trigger_edge;
        self.want_trigger_level = c.trigger_level_lsb;
        self.note("「期望配置」已填回交接时的值，由「应用到设备」下发");
    }

    /// 记一行日志。
    pub(crate) fn note(&mut self, line: impl Into<String>) {
        self.log.push(line.into());
        // 只留最近 200 行 —— 日志面板是给人扫的，不是归档
        if self.log.len() > 200 {
            self.log.drain(..self.log.len() - 200);
        }
    }

    /// 是否有长操作在进行。
    pub(crate) fn is_busy(&self) -> bool {
        self.busy.is_some()
    }

    /// 链路的一句话描述（给证据包和界面用）。
    ///
    /// **模拟器要写出场景名**：用户在界面上换了场景之后，证据包里
    /// 如果只写「模拟器」，模型就没法把结论和具体波形对上。
    ///
    /// # ⚠ AI 期间要用交接快照，不能按当前状态拼
    ///
    /// AI 跑的时候 GUI 是**断开**的：`connected=false`、`simulated=false`、
    /// `port` 是空的。照着当前状态拼会输出「serial @ 921600」——
    /// **把模拟器说成串口**，而且这话会进证据包、进锚点行，没人看得出来。
    ///
    /// 所以设备不在手上时，一律用交接那一刻的快照。
    pub(crate) fn link_description(&self) -> String {
        if self.device_is_elsewhere() {
            if let Some(h) = &self.drive.handoff {
                return h.link.clone();
            }
        }
        if self.simulated {
            format!("sim({})", self.scenario.name())
        } else if self.port.trim().is_empty() {
            format!("serial @ {}", self.baud)
        } else {
            format!("{} @ {}", self.port, self.baud)
        }
    }

    /// 当前这份数据是不是模拟器产生的。
    ///
    /// 与 [`Self::link_description`] 同理：AI 期间按当前状态读会得到 `false`，
    /// 于是证据包里那句「这是【模拟器】数据」就没了 ——
    /// 而那正是最不该丢的一句。
    pub(crate) fn evidence_simulated(&self) -> bool {
        if self.device_is_elsewhere() {
            if let Some(h) = &self.drive.handoff {
                return h.simulated;
            }
        }
        self.simulated
    }

    /// 面板顶部那行锚点，例如 `#3 · 4096 点 @ 857142 Hz · sim(i2c_100k) · 触发点 9`。
    ///
    /// **必须常显。** 没有它，用户会把模拟器上的结论当成自己板子的结论 ——
    /// 这是三个独立设计里都被列为最严重的那条风险。
    pub(crate) fn capture_anchor(&self) -> String {
        let Some(cap) = &self.capture else {
            return "（没有采集）".to_string();
        };
        let n = cap.channels.first().map(|c| c.len()).unwrap_or(0);
        let trig = match cap.trigger_index {
            Some(i) => format!(" · 触发点 {i}"),
            None => " · 无触发".to_string(),
        };
        format!(
            "#{} · {} 点 @ {} Hz · {}{}{}",
            cap.id,
            n,
            cap.rate_hz,
            self.link_description(),
            trig,
            if cap.overrun { " · ⚠溢出" } else { "" },
        )
    }

    /// 把当前采集打包成证据包。**纯组装，不做任何计算** ——
    /// 所有数字都是 core 已经算好的。
    ///
    /// 返回 `None` 表示还没有采集可分析。
    pub(crate) fn build_evidence(&self) -> Option<String> {
        let cap = self.capture.as_ref()?;
        let link = self.link_description();
        let question = self.ai.question.trim();

        // ⚠ 数据源与配置**都要走「设备不在手上时用交接快照」那条路** ——
        // AI 期间按当前状态读会得到 simulated=false、config=None，
        // 于是证据包里「这是模拟器数据」这句会消失，而那正是最不该丢的一句。
        let simulated = self.evidence_simulated();
        let config = if self.device_is_elsewhere() {
            self.drive.handoff.as_ref().and_then(|h| h.config.as_ref())
        } else {
            self.config.as_ref()
        };

        // 证据包统一消费 `DecodedFrames`；GUI 内部仍保留 `I2cDecode`
        // （画泳道要逐事件，转换一次的成本可忽略）。UART 结果本身就是
        // `DecodedFrames`，直接克隆。
        let frames = if self.decode_protocol == scope_core::Protocol::I2c {
            self.decode
                .as_ref()
                .map(|d| scope_core::to_decoded(d.clone()))
        } else {
            self.uart_decode.clone()
        };
        let decode_req = build_decode_request(
            self.decode_protocol,
            self.decode.is_some(),
            self.uart_decode.is_some(),
            self.scl_channel,
            self.sda_channel,
            &self.decode_cfg,
            self.uart_baud,
        );

        Some(scope_core::build_evidence(&scope_core::EvidenceInput {
            capture: cap,
            scale: &self.calib_view,
            link: &link,
            simulated,
            config,
            decode: frames.as_ref(),
            decode_req: decode_req.as_ref(),
            question: if question.is_empty() {
                None
            } else {
                Some(question)
            },
        }))
    }

    /// 处理一条 worker 回执。
    fn apply(&mut self, u: Update) {
        match u {
            Update::Ports(list) => {
                if self.port.is_empty() {
                    // 预选第一个疑似目标
                    if let Some((name, _, true)) = list.iter().find(|p| p.2) {
                        self.port = name.clone();
                    }
                }
                self.ports = list;
            }

            Update::Connected {
                info,
                config,
                state,
                simulated,
            } => {
                self.note(format!(
                    "已连接 {} 通道 / 上限 {} Hz",
                    info.ch_count, info.rate_max_hz
                ));
                // 标定表按 uid 查。**重新读一遍文件** —— 用户可能在两次采集
                // 之间在另一个终端跑了 `cal set`。
                self.calib.reload();
                self.calib_view = self
                    .calib
                    .scales_for(Some(&info.uid), info.ch_count as usize);
                if !self.calib_view.all_calibrated() {
                    self.note(self.calib_view.summary_note());
                }
                self.info = Some(info);
                self.config = config;
                self.state = Some(state);
                self.connected = true;
                self.simulated = simulated;
                self.last_error = None;
            }

            Update::Disconnected => {
                self.note("已断开");
                self.connected = false;
                // 断开即复位开关 —— 否则迟到的 Acquired 回执会按开关
                // 继续向一个已经不存在的连接发采集
                self.continuous = false;
                // 显式清零：该标志平时靠 worker 的「每个 Acquire 必得终态
                // （Acquired / Failed）」不变量兜底，清零后该不变量不再承重。
                self.acquire_in_flight = false;
                self.simulated = false;
                self.info = None;
                self.config = None;
                self.state = None;
                // `calib_view` **故意不清**：`capture` 也不清，波形还在屏幕上，
                // 纵轴标签必须跟显示的数据一致。见字段上的说明。
            }

            Update::ConfigApplied(cfg) => {
                // 回显值才是真值。采样率被量化时，这里就是用户能看到差异的地方。
                if cfg.rate_hz != self.want_rate {
                    self.note(format!(
                        "采样率被量化：请求 {} Hz → 实际 {} Hz",
                        self.want_rate, cfg.rate_hz
                    ));
                }
                self.config = Some(*cfg);
            }

            Update::Acquired(cap) => {
                self.acquire_in_flight = false;
                let cap = *cap;
                let mut msg = format!(
                    "采集完成：{} 点 @ {} Hz（{:.3} ms）",
                    cap.len(),
                    cap.rate_hz,
                    cap.duration_us() as f64 / 1000.0
                );
                if cap.overrun {
                    msg.push_str("  ⚠ 发生溢出，数据不完整");
                }
                if cap.trigger_index.is_none() {
                    msg.push_str("  · 无触发点（软触发）");
                }
                self.note(msg);
                // 这次采集成功了，上一次的错误就不再适用 —— 不清的话，
                // 之前那条红字会一直挂在底栏
                self.last_error = None;
                // 连续模式下**不重置视口**：每帧 reset 会把用户的滚轮缩放 /
                // 拖拽清掉，等于连续运行时无法查看细节。会话第一帧的适配
                // 由勾选框打开时置的一次 `fit_pending` 负责。
                self.fit_pending = !self.continuous;

                // 通道数可能变少（比如从双通道场景切到单通道），
                // 把通道选择夹回合法范围 —— 越界的话解码会直接报错。
                let n = self.capture.as_ref().map(|c| c.channels.len()).unwrap_or(0);
                if n > 0 {
                    self.scl_channel = self.scl_channel.min(n - 1);
                    self.sda_channel = self.sda_channel.min(n - 1);
                }
                // 连续帧不进历史 —— store 只留最近 16 条（容量由 core 决定），
                // 30 fps 连跑几秒就会把用户手动采的记录全顶掉（docs/09 §8）。
                if !self.continuous {
                    self.store.push(cap.clone());
                }
                self.capture = Some(cap);
                self.decode_dirty.store(true, Ordering::Relaxed);
                // 连续刷新：收到一帧立即要下一帧。循环由 UI 线程驱动 ——
                // 每收到一帧就再发一个 Acquire，worker 一行不改。
                if self.continuous {
                    self.acquire_in_flight = true;
                    self.worker.send(Request::Acquire {
                        samples: self.want_samples,
                        rate_hz: self.want_rate,
                        trigger_level_lsb: self.want_trigger_level,
                        timeout_ms: 2000,
                    });
                }
            }

            Update::StateChanged(s) => {
                self.state = Some(s);
            }

            Update::Busy { op, active } => {
                self.busy = if active { Some(op) } else { None };
            }

            Update::Failed { text, hint } => {
                self.acquire_in_flight = false;
                self.note(format!("✗ {text}"));
                self.last_error = Some((text, hint));
                // 连续刷新的失败即停止循环 —— 不静默重试（docs/09 §8）。
                if self.continuous {
                    self.continuous = false;
                    self.note("连续刷新已停止（上次采集失败）");
                }
            }
        }
    }

    /// 按需重算解码。
    fn refresh_decode(&mut self) {
        if !self.decode_dirty.swap(false, Ordering::Relaxed) {
            return;
        }
        let Some(cap) = self.capture.as_ref() else {
            self.decode = None;
            self.uart_decode = None;
            return;
        };

        match self.decode_protocol {
            scope_core::Protocol::I2c => {
                self.uart_decode = None;
                // I2C 解码至少要两条线。单通道采集（正弦 / 方波 / 直流…）**不是错误**，
                // 只是这个场景没有 I2C 可解 —— 静默跳过。
                // 回归：之前这里会一路走到 decode_capture 报 InvalidParam，
                // 于是用户采集正弦波时也会看到一条红字错误。
                if cap.channels.len() < 2 {
                    self.decode = None;
                    return;
                }
                let cfg = I2cDecodeConfig {
                    scl_channel: self.scl_channel,
                    sda_channel: self.sda_channel,
                    levels: self.decode_cfg.levels,
                    debounce_ns: self.decode_cfg.debounce_ns,
                };
                match scope_core::decode_capture(cap, &cfg) {
                    Ok(d) => {
                        self.note(format!("解码完成：共 {} 帧", d.frame_count()));
                        self.decode = Some(d);
                    }
                    Err(e) => {
                        let (text, hint) = crate::msg::describe_error(&e);
                        self.note(format!("✗ 解码失败：{text}"));
                        self.last_error = Some((text, hint));
                        self.decode = None;
                    }
                }
            }
            scope_core::Protocol::Uart => {
                self.decode = None;
                // 通道留空 = core 自动取通道 0；波特率 0 由 core 显式报错。
                let req = scope_core::DecodeRequest {
                    protocol: scope_core::Protocol::Uart,
                    channels: vec![],
                    params: scope_core::DecodeParams::Uart(scope_core::UartConfig {
                        baud: self.uart_baud,
                        ..scope_core::UartConfig::default()
                    }),
                };
                match scope_core::decoder::decode(cap, &req)
                    .map_err(scope_core::ScopeError::from)
                {
                    Ok(d) => {
                        self.note(format!("解码完成：共 {} 帧", d.frame_count()));
                        self.uart_decode = Some(d);
                    }
                    Err(e) => {
                        let (text, hint) = crate::msg::describe_error(&e);
                        self.note(format!("✗ 解码失败：{text}"));
                        self.last_error = Some((text, hint));
                        self.uart_decode = None;
                    }
                }
            }
        }
    }

    /// 请求解码重算（参数变了就调它）。
    pub(crate) fn mark_decode_dirty(&mut self) {
        self.decode_dirty.store(true, Ordering::Relaxed);
    }

    /// 场景切换时给解码协议一个合理默认（用户可改）：
    /// `uart_115k` → UART，`i2c_*` → I2C；模拟信号场景保持现状。
    pub(crate) fn sync_decode_protocol(&mut self) {
        if let Some(p) = default_protocol_for(self.scenario) {
            if self.decode_protocol != p {
                self.decode_protocol = p;
                self.mark_decode_dirty();
            }
        }
    }
}

/// 场景 → 解码协议默认。模拟信号场景返回 `None`（保持用户当前选择）。
fn default_protocol_for(scenario: Scenario) -> Option<scope_core::Protocol> {
    match scenario {
        Scenario::Uart115k => Some(scope_core::Protocol::Uart),
        Scenario::I2c100k | Scenario::I2c400k | Scenario::I2cNack => {
            Some(scope_core::Protocol::I2c)
        }
        _ => None,
    }
}

/// 按当前选择构造解码请求 —— `build_evidence` 与测试共用这一份逻辑，
/// 避免请求形状在两处各写一遍（同物两名缺陷）。
pub(crate) fn build_decode_request(
    protocol: scope_core::Protocol,
    has_i2c: bool,
    has_uart: bool,
    scl_channel: usize,
    sda_channel: usize,
    decode_cfg: &I2cDecodeConfig,
    uart_baud: u32,
) -> Option<scope_core::DecodeRequest> {
    match protocol {
        scope_core::Protocol::I2c if has_i2c => Some(scope_core::DecodeRequest {
            protocol: scope_core::Protocol::I2c,
            channels: vec![],
            params: scope_core::DecodeParams::I2c(I2cDecodeConfig {
                scl_channel,
                sda_channel,
                levels: decode_cfg.levels,
                debounce_ns: decode_cfg.debounce_ns,
            }),
        }),
        scope_core::Protocol::Uart if has_uart => Some(scope_core::DecodeRequest {
            protocol: scope_core::Protocol::Uart,
            channels: vec![],
            params: scope_core::DecodeParams::Uart(scope_core::UartConfig {
                baud: uart_baud,
                ..scope_core::UartConfig::default()
            }),
        }),
        _ => None,
    }
}

/// 把一次采集导出为 CSV —— 与 CLI / MCP 同一序列化
/// （`Capture::to_csv`，含首行 `# rate_hz=` 契约）。
///
/// 写入前过 [`scope_core::persist::check_writable`] 闸门：
/// 导出路径不得落入项目目录（当前工作目录），与配置文件的防泄漏语义一致。
/// 返回写入的完整路径。
pub(crate) fn export_csv(
    path: &str,
    cap: &scope_core::Capture,
    scales: &scope_core::ScaleSet,
) -> Result<String, String> {
    let target = std::path::PathBuf::from(path);
    let parent = target
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| std::path::PathBuf::from("."));
    let cwd = std::env::current_dir()
        .map_err(|e| format!("无法获取当前工作目录（{e}）。无法确认导出位置安全性，拒绝写入"))?;
    scope_core::persist::check_writable(&parent, &cwd, "导出的 CSV")?;
    let body = cap.to_csv(scales.as_slice());
    std::fs::write(&target, body).map_err(|e| format!("写 CSV 失败：{e}"))?;
    Ok(target.display().to_string())
}

/// CSV 导出的默认文件名（用户未改路径时使用）。
pub(crate) fn default_csv_path(cap: &scope_core::Capture) -> String {
    format!("captures/capture_{}.csv", cap.id)
}

impl eframe::App for App {
    /// 非绘制逻辑。**worker 消息只在这里排空。**
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let mut pending: Vec<Update> = Vec::new();
        self.worker.drain(|u| pending.push(u));
        for u in pending {
            self.apply(u);
        }

        // AI 回执走**另一条通道**（另一条线程），同样先收集再应用。
        let mut ai_pending = Vec::new();
        self.ai.worker.drain(|u| ai_pending.push(u));
        for u in ai_pending {
            self.ai.apply(u);
        }

        if self.worker.is_dead() {
            // worker 悄悄死了的话，UI 会永远转圈 —— 这是最坏的用户体验
            self.busy = None;
            if self.last_error.is_none() {
                self.last_error =
                    Some(("后台工作线程已退出".into(), Some("重启应用即可恢复".into())));
            }
        }

        // ── AI 驱动会话 ──
        //
        // 先接收事件（写入 Vec 再处理，避免在闭包里可变借走 App），
        // 再推状态机。迟到的会话事件靠世代号丢弃。
        let mut drive_events = Vec::new();
        let mut gen = 0;
        if let Some(h) = &self.drive.handle {
            gen = h.generation;
            h.drain(|e| drive_events.push(e));
        }
        for e in drive_events {
            if self.drive.accepts(gen) {
                self.apply_drive(e);
            }
        }
        self.step_drive(ctx);

        // --demo：先连模拟器，连上了再采一次。
        // 用级联的 stage 而不是一个 bool —— 连接是异步的，得等回执。
        match self.demo_stage {
            0 if !self.is_busy() => {
                self.demo_stage = 1;
                self.worker.send(Request::Connect {
                    transport: TransportKind::Sim,
                    port: String::new(),
                    baud: 921_600,
                    scenario: self.scenario,
                });
            }
            // 连接失败就一直停在 1 —— 不重试，免得每帧刷屏
            1 if self.connected && !self.is_busy() => {
                self.demo_stage = 2;
                self.acquire_in_flight = true;
                // 与勾选框路径一致：会话第一帧适配默认窗口
                // （`--demo-cont` 下后续帧不再重置视口，见 Update::Acquired）
                self.fit_pending = true;
                self.worker.send(Request::Acquire {
                    samples: self.want_samples,
                    rate_hz: self.want_rate,
                    trigger_level_lsb: self.want_trigger_level,
                    timeout_ms: 2000,
                });
            }
            _ => {}
        }

        self.refresh_decode();
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("topbar").show(ui, |ui| panels::toolbar(self, ui));
        egui::Panel::bottom("statusbar").show(ui, |ui| panels::status_bar(self, ui));
        egui::Panel::left("side")
            .resizable(true)
            .default_size(288.0)
            .size_range(240.0..=460.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    panels::device(self, ui);
                    ui.separator();
                    panels::config(self, ui);
                    ui.separator();
                    panels::history(self, ui);
                    // 故障注入只在模拟器连接时才有意义 —— 真机上没有这个开关
                    if self.simulated {
                        ui.separator();
                        panels::faults(self, ui);
                    }
                    ui.separator();
                    panels::log(self, ui);
                });
            });
        // 详情面板必须自己占一个面板。`Plot::show` 会**占用全部可用高度**，
        // 直接跟在它后面的东西会被整个挤出屏幕 —— 交易表和色标就是这么消失的。
        //
        // 有采集就显示：**测量对任何信号都适用**，不只是 I2C。里面的解码部分
        // 会自己在通道数 < 2 时让位。
        // 看帮助时把详情面板收起来 —— 帮助页内容本来就长，再被底部占掉 270px
        // 就只能滚动着看，没必要。
        if self.capture.is_some() && !self.show_help {
            egui::Panel::bottom("detail")
                .resizable(true)
                .default_size(270.0)
                .size_range(90.0..=560.0)
                .show(ui, |ui| {
                    // 内容可能比面板高（测量 + 色标 + 质量 + 交易表），
                    // 没有这层 ScrollArea 的话超出的部分会被直接裁掉
                    egui::ScrollArea::vertical()
                        .id_salt("detail_scroll")
                        .auto_shrink([false, false])
                        .show(ui, |ui| panels::detail_panel(self, ui));
                });
        }

        // AI 面板：右侧独立一栏。
        //
        // 为什么不塞进底部的 `detail`：那里已经有测量 + 色标 + 质量 + 交易表，
        // 再挤进去只会让两边都难读。右侧栏还有个好处 —— 分析结论可以和波形
        // **并排**看，不用来回滚。
        if self.ai.panel_open {
            egui::Panel::right("ai")
                .resizable(true)
                .default_size(380.0)
                .size_range(300.0..=640.0)
                .show(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .id_salt("ai_scroll")
                        .auto_shrink([false, false])
                        .show(ui, |ui| panels::ai_panel(self, ui));
                });
        }

        egui::CentralPanel::default().show(ui, |ui| {
            // 帮助页顶掉波形（不做成浮层 Window —— 0.36 的 Window API 我还没核实过）
            if self.show_help {
                panels::help_page(self, ui);
            } else {
                panels::plot(self, ui);
            }
        });
    }

    /// 收尾。注意签名**不带 `glow::Context`** —— eframe 0.36 的默认渲染后端是
    /// wgpu，`glow` 是可选特性，没开时 trait 用的是这个无参版本。
    fn on_exit(&mut self) {
        // ⚠ **先杀 AI 的子进程，再关 worker。**
        //
        // 从前这里只 shutdown worker。而 `scope-mcp` 子进程**不归它管** ——
        // AI 跑着的时候关窗口，会留下一个**孤儿 `scope-mcp` 一直占着 COM7**，
        // 下次开程序连不上，而且任务管理器里那个进程看起来跟本程序无关。
        //
        // `terminate()` 内部是 kill + wait：wait 不能省，Windows 上句柄没释放，
        // 下次重连会偶发「端口被占用」。
        if let Some(h) = self.drive.handle.take() {
            h.terminate();
        }
        self.worker.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A1：协议选择决定 `DecodeRequest` 的形状 —— I2C 带通道配置，
    /// UART 带波特率（8N1 固定、通道留空自动取 0）。
    #[test]
    fn decode_request_follows_the_selected_protocol() {
        let cfg = I2cDecodeConfig::default();
        let req = build_decode_request(
            scope_core::Protocol::I2c,
            true,
            false,
            0,
            1,
            &cfg,
            115_200,
        )
        .expect("I2C 有结果时应构造请求");
        assert_eq!(req.protocol, scope_core::Protocol::I2c);
        assert!(matches!(req.params, scope_core::DecodeParams::I2c(_)));

        let req = build_decode_request(
            scope_core::Protocol::Uart,
            false,
            true,
            0,
            1,
            &cfg,
            9600,
        )
        .expect("UART 有结果时应构造请求");
        assert_eq!(req.protocol, scope_core::Protocol::Uart);
        assert!(req.channels.is_empty(), "UART 通道留空 = core 自动取通道 0");
        match req.params {
            scope_core::DecodeParams::Uart(u) => {
                assert_eq!(u.baud, 9600, "波特率应来自用户输入");
                assert_eq!(u.data_bits, 8);
                assert_eq!(u.parity, scope_core::Parity::None);
                assert_eq!(u.stop_bits, 1);
            }
            other => panic!("UART 选择下应是 Uart 参数：{other:?}"),
        }

        // 没有对应协议的结果时不发请求 —— 证据包的 decode_req 与结果成对。
        assert!(
            build_decode_request(scope_core::Protocol::I2c, false, true, 0, 1, &cfg, 115_200)
                .is_none()
        );
    }

    /// C4：场景切换给解码协议合理默认；模拟信号场景保持现状。
    #[test]
    fn scenario_switch_defaults_the_decode_protocol() {
        assert_eq!(
            default_protocol_for(Scenario::Uart115k),
            Some(scope_core::Protocol::Uart)
        );
        assert_eq!(
            default_protocol_for(Scenario::I2c100k),
            Some(scope_core::Protocol::I2c)
        );
        assert_eq!(
            default_protocol_for(Scenario::I2cNack),
            Some(scope_core::Protocol::I2c)
        );
        // 模拟信号场景：None = 保持用户当前选择
        assert_eq!(default_protocol_for(Scenario::Sine1k3v3), None);
    }

    /// A2：GUI 导出的 CSV 与 core 的 `Capture::to_csv` 逐字节一致；
    /// 项目目录（当前工作目录）之内拒绝写入。
    #[test]
    fn export_csv_matches_core_to_csv_byte_for_byte() {
        let mut gen = scope_sim::WaveformGen::new(scope_sim::Scenario::Sine1k3v3, 1);
        let channels = gen.generate_multi(1, 256, 857_142);
        let mut cap = scope_core::Capture::new(1, 857_142, 1, 256);
        cap.channels = channels;
        let scales = scope_core::ScaleSet::uncalibrated(1);

        let dir = std::env::temp_dir().join("scope-gui-export-test");
        let path = dir.join("capture_1.csv");
        let written =
            export_csv(&path.to_string_lossy(), &cap, &scales).expect("临时目录在项目之外，应允许写入");
        assert!(written.contains("capture_1.csv"), "应写默认文件名：{written}");
        let got = std::fs::read_to_string(&path).expect("应能读回导出文件");
        assert_eq!(
            got,
            cap.to_csv(scales.as_slice()),
            "GUI 导出与 core to_csv 必须逐字节一致"
        );

        // 闸门：当前工作目录之内拒绝写入（与配置文件同一条 persist 语义）
        let inside = std::path::Path::new("captures").join("should_fail.csv");
        assert!(
            export_csv(&inside.to_string_lossy(), &cap, &scales).is_err(),
            "项目目录内的导出必须被拒"
        );
        let _ = std::fs::remove_file(&path);
    }
}
