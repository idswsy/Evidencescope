//! 完整的设备模型 —— 不是「假的」，是**行为等价**的。
//!
//! 它跑的是同一份帧编解码、同一个状态机、同一套错误码，
//! 只是把「定时器触发 → ADC → DMA」换成了波形合成器。
//!
//! 因此在模拟器上验证通过的 Agent 逻辑，接到真机上不需要改一行。
//!
//! 有意复刻的 F103 真实行为：
//! - **采样率量化**：只提供定时器分频能达到的档位，并回显 `actual_hz`
//! - **触发语义**：auto 模式约 200 ms 无触发强制完成（否则 Agent 永久阻塞）
//! - **状态机约束**：ARMED 下发配置命令回 `BUSY`
//! - **seq 去重缓存**：单槽，1 s 窗口，保证 ARM 重试安全
//! - **不按波特率节流**：`byte_rate()` 返回 `u32::MAX`。模拟分片间隔只会让
//!   测试变慢，而 `byte_rate` 的真实用途是让命令层**估算超时**。
//!   （这句注释从前写的是「按波特率节流，模拟真实的分片间隔」，与实现相反，
//!   并被 README 与 docs/01 各抄了一遍。）
//! - **故障注入**：丢帧 / CRC 错 / 延迟尖峰

use crate::waveform::{Scenario, WaveformGen, FULL_SCALE_LSB};
use scope_core::device::DevicePort;
use scope_core::error::LinkError;
use scope_proto::*;
use std::collections::VecDeque;

/// 故障注入配置。
#[derive(Debug, Clone)]
pub struct FaultInjection {
    /// 每 N 个帧丢弃一个（0 = 关闭）。
    pub drop_every_n_frames: u32,
    /// 每 N 个帧破坏一个的 CRC（0 = 关闭）。
    pub crc_err_every_n_frames: u32,
    /// 随机延迟尖峰的概率（0.0 .. 1.0）。
    pub latency_spike_probability: f64,
    /// 延迟尖峰的时长（ms）。
    pub latency_spike_ms: u64,
    /// 永不触发 —— 用于验证 `scope_capture` 的超时路径不卡死。
    pub no_trigger: bool,
    /// 制造一次采集溢出。
    pub force_overrun: bool,
}

impl Default for FaultInjection {
    fn default() -> Self {
        FaultInjection {
            drop_every_n_frames: 0,
            crc_err_every_n_frames: 0,
            latency_spike_probability: 0.0,
            latency_spike_ms: 0,
            no_trigger: false,
            force_overrun: false,
        }
    }
}

impl FaultInjection {
    /// 第 `n` 个发出帧该不该被丢掉。
    ///
    /// 抽成方法是为了让「什么算丢帧」只有一处定义 —— 计数与丢弃动作
    /// 必须用同一个判据，否则统计出来的数和实际丢的对不上。
    pub fn drops_frame(&self, n: u32) -> bool {
        self.drop_every_n_frames > 0 && n % self.drop_every_n_frames == 0
    }

    /// 第 `n` 个发出帧的 CRC 该不该被破坏。
    pub fn breaks_crc(&self, n: u32) -> bool {
        self.crc_err_every_n_frames > 0 && n % self.crc_err_every_n_frames == 0
    }

    /// 是否完全干净（无注入）。
    pub fn is_clean(&self) -> bool {
        self.drop_every_n_frames == 0
            && self.crc_err_every_n_frames == 0
            && self.latency_spike_probability <= 0.0
            && !self.no_trigger
            && !self.force_overrun
    }
}

/// `GET_INFO.caps` 的声明口径。
///
/// 模拟器自述是「复刻真实行为」，但它的 `caps` 一度描述的是**它自己会做什么**，
/// 而不是**真机做得到什么** —— 于是对着模拟器开发的 Agent 会规划出真机必然
/// 拒绝的动作（`FORCE_TRIGGER` / MINMAX / AC 耦合）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapsMode {
    /// 与真机 `DEV_CAPS` 完全一致（默认）。
    ///
    /// 真机只有 `CAP_TRIG_AUTO | CAP_TRIG_NORMAL | CAP_TRIG_SINGLE | CAP_FMT_PACK12`
    /// 四位；**即使 I2C 场景是双通道，也不声明 `CAP_CH_DUAL`** —— 那描述的是
    /// 真板的能力，不是模拟场景的通道数。
    RealDevice,
    /// 模拟器完整能力：额外声明软触发 / MINMAX / AC 耦合 / 数字 I2C / 双通道。
    ///
    /// 只用于模拟器专项测试与演示，不用于「对着它开发真机 Agent」的场景。
    IdealSim,
}

/// 设备配置快照。
#[derive(Debug, Clone)]
struct Config {
    rate_hz: u32,
    acq_mode: u8,
    capture_samples: u16,
    format: u8,
    decimation: u16,
    trigger_mode: u8,
    trigger_source: u8,
    trigger_edge: u8,
    trigger_level_lsb: u16,
    pre_samples: u16,
    holdoff_us: u32,
    ch0_enable: u8,
    ch0_coupling: u8,
    /// 通道 0 的量程档位索引。**本板是手拨开关，只记录不生效**（见 docs/02 §5）。
    ch0_range_idx: u8,
    /// 通道 0 的垂直偏移（ADC LSB，i16）。
    ///
    /// `GET_CONFIG` 不回报这两个字段（协议里 payload 是定长 17 字节），
    /// 所以只能靠 `SET_CHANNEL` 记下来。它们由 `SET_CHANNEL` 的**回显**带给主机。
    ch0_offset_lsb: i16,
    /// 通道 1 的使能。
    ///
    /// 分开存是有原因的：从前 `cmd_set_channel` 无论 `ch` 是几都往 `ch0_*`
    /// 里写 —— 配 CH2 会**改掉 CH1 的配置**。
    ch1_enable: u8,
    /// 通道 1 的耦合。
    ch1_coupling: u8,
    /// 通道 1 的量程档位索引（同 ch0，只记录）。
    ch1_range_idx: u8,
    /// 通道 1 的垂直偏移。
    ch1_offset_lsb: i16,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            rate_hz: 857_142,
            acq_mode: 0,
            capture_samples: 4096,
            format: 0,
            decimation: 1,
            // 必须与 `firmware/App/acq.c` 的 `acq_init()` 一致 —— 见那里的长注释。
            // 默认 AUTO 而不是 NORMAL：直流信号在边沿触发下**任何电平都不会触发**，
            // 默认成 NORMAL 会让「接上、没接信号、点采集」必然超时。
            trigger_mode: 0,
            trigger_source: 0,
            trigger_edge: 0,
            trigger_level_lsb: 2048,
            pre_samples: 2048,
            holdoff_us: 1000,
            ch0_enable: 1,
            ch0_coupling: 0,
            ch0_range_idx: 0,
            ch0_offset_lsb: 0,
            ch1_enable: 1,
            ch1_coupling: 0,
            ch1_range_idx: 0,
            ch1_offset_lsb: 0,
        }
    }
}

/// 模拟器设备。
pub struct SimDevice {
    scenario: Scenario,
    wave: WaveformGen,

    state: State,
    cfg: Config,

    /// 待发给主机的字节。
    tx: VecDeque<u8>,
    /// 主机写入的字节缓冲（用来喂解析器）。
    rx: Vec<u8>,
    parser: Parser,
    frame: Frame,

    /// 已冻结的采集数据（单次模式）。
    captured: Option<Captured>,
    next_capture_id: u16,

    /// 单槽 seq 去重缓存 —— 复刻设备的非幂等命令重试安全机制。
    dedup: Option<(u16, u16, Vec<u8>)>,

    /// **测试专用**：READ_BUFFER 无论请求什么格式都按 RAW16 回。
    /// 见 [`SimDevice::set_read_buffer_force_raw16`]。
    force_raw16: bool,

    /// `GET_INFO.caps` 的声明口径，默认 [`CapsMode::RealDevice`]。
    caps_mode: CapsMode,

    /// 故障注入设置。直接赋值即可：
    /// `dev.faults.drop_every_n_frames = 50;`
    pub faults: FaultInjection,
    frames_sent: u32,

    // ── 链路健康计数（GET_STATUS 上报）──────────────────────────
    //
    // 从前 `cmd_get_status` 把 `overrun_samples` / `rx_crc_err` /
    // `rx_dropped` / `tx_dropped` 全部**写死为 0**，于是主机侧读到
    // 「零溢出、零 CRC 错」时，那根本不是统计出来的，而是没人统计。
    // 一个 Agent 会把它当成「链路很干净」的证据。
    /// 累计被丢弃的样点数（采集溢出）。
    overrun_samples: u32,
    /// 累计被故障注入丢掉的发送帧数。
    tx_dropped: u32,
    /// 最近一次错误码（`queue_error` 回的那个）。
    last_error_code: u16,

    /// 上一次触发的时间（用于 auto 模式超时与 holdoff）。
    armed_at: Option<std::time::Instant>,
    triggered: bool,
}

#[derive(Debug, Clone)]
struct Captured {
    id: u16,
    channels: Vec<Vec<u16>>,
    trigger_index: Option<u32>,
    overrun: bool,
}

impl SimDevice {
    /// 用指定场景创建模拟器。
    pub fn new(scenario: Scenario) -> SimDevice {
        SimDevice {
            scenario,
            wave: WaveformGen::new(scenario, 0x5EED_1234),
            state: State::Idle,
            cfg: Config::default(),
            tx: VecDeque::new(),
            rx: Vec::new(),
            parser: Parser::with_capacity(MAX_FRAME_RX),
            frame: Frame {
                header: Header {
                    ver: VER,
                    flags: 0,
                    seq: 0,
                    cmd_raw: 0,
                    len: 0,
                },
                payload: Vec::new(),
            },
            captured: None,
            next_capture_id: 1,
            dedup: None,
            force_raw16: false,
            caps_mode: CapsMode::RealDevice,
            faults: FaultInjection::default(),
            frames_sent: 0,
            overrun_samples: 0,
            tx_dropped: 0,
            last_error_code: 0,
            armed_at: None,
            triggered: false,
        }
    }

    /// 当前场景。
    pub fn scenario(&self) -> Scenario {
        self.scenario
    }

    /// 切换场景。
    pub fn set_scenario(&mut self, s: Scenario) {
        self.scenario = s;
        self.wave.set_scenario(s);
        self.captured = None;
    }

    /// 换波形随机种子。
    ///
    /// `new()` 固定用 `0x5EED_1234`，所以 `noise` 场景每次生成的序列完全一致 ——
    /// 那是为了回归测试可断言。想在演示里看到不同的噪声就换种子。
    pub fn set_seed(&mut self, seed: u64) {
        self.wave.set_seed(seed);
        self.captured = None;
    }

    /// **测试专用**：READ_BUFFER 无论主机请求什么格式都按 RAW16 回
    /// （分片头 `format` 也如实回显 0）。
    ///
    /// 真机上设备完全有权按自己的情况回别的格式；主机侧的正确性要求是
    /// 「按分片头**回显**的 format 解码」，**不是**「按自己请求的值解码」。
    /// 这个开关用来在端到端测试里制造「请求 PACK12、回包 RAW16」，
    /// 给那条属性一个红灯（见 scope-mcp 的
    /// `capture_decodes_by_the_echoed_chunk_format_not_by_the_request`）。
    ///
    /// 刻意**不**放进 [`FaultInjection`]：这不是链路故障，而是合法的设备行为，
    /// 也不该牵动 MCP `InjectArgs` 的 schema。
    pub fn set_read_buffer_force_raw16(&mut self, on: bool) {
        self.force_raw16 = on;
    }

    /// 设备报告的通道数。
    pub fn channel_count(&self) -> u8 {
        self.scenario.channel_count() as u8
    }

    /// 设置 `GET_INFO.caps` 的声明口径。
    ///
    /// 默认 [`CapsMode::RealDevice`]；只有模拟器专项测试才应切到
    /// [`CapsMode::IdealSim`]。
    pub fn set_caps_mode(&mut self, mode: CapsMode) {
        self.caps_mode = mode;
    }

    /// 当前 `GET_INFO.caps` 的值。
    ///
    /// 抽成方法是为了让「声明什么」只有一处定义，测试也能直接对照。
    pub fn caps(&self) -> u32 {
        let mut caps = CAP_TRIG_AUTO | CAP_TRIG_NORMAL | CAP_TRIG_SINGLE | CAP_FMT_PACK12;
        if self.caps_mode == CapsMode::IdealSim {
            caps |= CAP_TRIG_SOFT | CAP_COUPLING_AC | CAP_FMT_MINMAX | CAP_DIGITAL_I2C;
            if self.channel_count() >= 2 {
                caps |= CAP_CH_DUAL;
            }
        }
        caps
    }

    /// 当前状态机状态。测试与诊断用（正常路径应通过 `GET_STATUS` 获取）。
    pub fn state(&self) -> State {
        self.state
    }

    // ── 内部：响应编码 ──────────────────────────────────────────

    fn queue(&mut self, flags: u8, seq: u16, cmd: Cmd, payload: Vec<u8>) {
        self.frames_sent = self.frames_sent.wrapping_add(1);

        // 故障注入：丢帧
        if self.faults.drops_frame(self.frames_sent) {
            self.tx_dropped = self.tx_dropped.wrapping_add(1);
            return;
        }

        let f = Frame {
            header: Header {
                ver: VER,
                flags,
                seq,
                cmd_raw: cmd as u16,
                len: payload.len() as u16,
            },
            payload,
        };
        let mut bytes = f.encode();

        // 故障注入：破坏 CRC（改最后一个字节）
        if self.faults.breaks_crc(self.frames_sent) {
            if let Some(last) = bytes.last_mut() {
                *last ^= 0xFF;
            }
        }

        self.tx.extend(bytes);
    }

    /// 构造错误响应帧：`{code:u16, severity:u8, msg_len:u8, msg[≤32]}`。
    ///
    /// 错误响应统一用 `CMD_ERROR` 作为帧命令码，原始命令码由主机从
    /// 自己发出的请求里对应 —— 这样错误帧的解析路径只有一条。
    fn queue_error(&mut self, seq: u16, _hash_cmd: u16, code: u16, msg: &str) {
        // 让 GET_STATUS 能回答「上一次出错是什么」——从前这个字段写死 0
        self.last_error_code = code;
        let msg_bytes = msg.as_bytes();
        let n = msg_bytes.len().min(32);
        let mut p = Vec::with_capacity(4 + n);
        p.extend_from_slice(&code.to_le_bytes());
        p.push(1); // severity = err
        p.push(n as u8);
        p.extend_from_slice(&msg_bytes[..n]);

        self.queue(flags::RESP | flags::ERROR, seq, Cmd::Error, p);
    }

    fn queue_ack(&mut self, seq: u16, cmd: Cmd) {
        self.queue(flags::RESP, seq, cmd, Vec::new());
    }

    fn queue_event(&mut self, cmd: Cmd, payload: Vec<u8>) {
        self.queue(flags::EVENT, 0, cmd, payload);
    }

    // ── 内部：命令执行 ──────────────────────────────────────────

    fn handle(&mut self, hdr: Header, payload: &[u8]) {
        let seq = hdr.seq;
        let cmd_raw = hdr.cmd_raw;

        // seq 去重缓存：同一 seq 在 1s 内重发 → 回放上次响应，不重复执行
        if let Some((cached_seq, cached_cmd, cached_resp)) = &self.dedup {
            if *cached_seq == seq && *cached_cmd == cmd_raw {
                let r = cached_resp.clone();
                self.tx.extend(r);
                return;
            }
        }

        let tx_before = self.tx.len();

        let cmd = match Cmd::try_from_u16(cmd_raw) {
            Some(c) => c,
            None => {
                // 未知 CMD：此时 payload 已被解析器按 LEN 正确吞掉，
                // 所以这里回错误是安全的（不会造成帧错位）
                self.queue_error(seq, cmd_raw, 0x0001, "unknown cmd");
                self.cache_dedup(seq, cmd_raw, tx_before);
                return;
            }
        };

        if !cmd_allowed(self.state, cmd) {
            self.queue_error(seq, cmd_raw, 0x0004, "busy: stop first");
            self.cache_dedup(seq, cmd_raw, tx_before);
            return;
        }

        match cmd {
            Cmd::GetInfo => self.cmd_get_info(seq),
            Cmd::Ping => self.cmd_ping(seq, payload),
            Cmd::GetStatus => self.cmd_get_status(seq),
            Cmd::Reset => {
                self.state = State::Idle;
                self.cfg = Config::default();
                self.captured = None;
                self.dedup = None;
                self.queue_ack(seq, cmd);
            }
            Cmd::GetConfig => self.cmd_get_config(seq),
            Cmd::SetSampleRate => self.cmd_set_rate(seq, payload),
            Cmd::SetChannel => self.cmd_set_channel(seq, payload),
            Cmd::SetTrigger => self.cmd_set_trigger(seq, payload),
            Cmd::SetAcq => self.cmd_set_acq(seq, payload),
            Cmd::Arm => self.cmd_arm(seq),
            Cmd::Stop => {
                self.state = State::Idle;
                self.queue_ack(seq, cmd);
            }
            Cmd::ForceTrigger => {
                if self.state == State::Armed {
                    self.complete_capture();
                    self.queue_ack(seq, cmd);
                } else {
                    self.queue_error(seq, cmd_raw, 0x0005, "not armed");
                }
            }
            Cmd::ReadBuffer => self.cmd_read_buffer(seq, payload),
            Cmd::Measure => self.cmd_measure(seq, payload),
            Cmd::Echo => {
                self.queue(flags::RESP, seq, cmd, payload.to_vec());
            }
            Cmd::SetLogLevel => self.queue_ack(seq, cmd),
            Cmd::GetLastError => {
                let mut p = Vec::new();
                p.extend_from_slice(&0u16.to_le_bytes());
                p.push(0);
                p.extend_from_slice(&0u32.to_le_bytes());
                p.push(0);
                self.queue(flags::RESP, seq, cmd, p);
            }
            // 事件类命令不会从主机发出
            Cmd::EventTrigger | Cmd::EventOverrun | Cmd::EventLog | Cmd::Error => {
                self.queue_error(seq, cmd_raw, 0x0001, "event is device->host");
            }
            Cmd::MemRead | Cmd::MemWrite => {
                self.queue_error(seq, cmd_raw, 0x000A, "debug build only");
            }
        }

        self.cache_dedup(seq, cmd_raw, tx_before);
    }

    fn cache_dedup(&mut self, seq: u16, cmd: u16, tx_before: usize) {
        // 缓存本次产生的字节，供同 seq 重发时原样回放
        let produced: Vec<u8> = self.tx.iter().skip(tx_before).copied().collect();
        if !produced.is_empty() {
            self.dedup = Some((seq, cmd, produced));
        }
    }

    // ── 各命令 ─────────────────────────────────────────────────

    fn cmd_get_info(&mut self, seq: u16) {
        let mut p = Vec::with_capacity(45);
        p.push(VER);
        p.extend_from_slice(&0x0001_0000u32.to_le_bytes()); // fw_ver 1.0.0
        p.extend_from_slice(&0x0103u16.to_le_bytes()); // model: F103 立创底板
        p.extend_from_slice(&[
            0x5E, 0xED, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        ]); // uid
        p.extend_from_slice(&TICK_HZ.to_le_bytes());
        p.push(12); // adc_bits
        p.push(self.channel_count());
        p.extend_from_slice(&1000u32.to_le_bytes()); // rate_min
        p.extend_from_slice(&scope_core::f103::MAX_SAMPLE_RATE_HZ.to_le_bytes()); // rate_max = 857142
        p.extend_from_slice(&(scope_core::f103::MAX_CAPTURE_SAMPLES as u16).to_le_bytes());
        p.extend_from_slice(&(MAX_PAYLOAD_RX as u16).to_le_bytes());
        p.extend_from_slice(&(MAX_PAYLOAD_TX as u16).to_le_bytes());
        p.extend_from_slice(&(PREFERRED_CHUNK as u16).to_le_bytes());

        let caps = self.caps();
        p.extend_from_slice(&caps.to_le_bytes());

        debug_assert_eq!(p.len(), 45, "GET_INFO payload 必须是 45 字节");
        self.queue(flags::RESP, seq, Cmd::GetInfo, p);
    }

    fn cmd_ping(&mut self, seq: u16, payload: &[u8]) {
        let n = payload.first().copied().unwrap_or(0) as usize;
        let mut p = Vec::with_capacity(1 + n + 4);
        p.push(n as u8);
        p.extend_from_slice(payload.get(1..1 + n).unwrap_or(&[]));
        p.extend_from_slice(&self.tick_us().to_le_bytes());
        self.queue(flags::RESP, seq, Cmd::Ping, p);
    }

    fn cmd_get_status(&mut self, seq: u16) {
        let mut p = Vec::with_capacity(33);
        p.push(self.state as u8);
        // bit0 = 发生过溢出。位含义见 docs/03-protocol.md
        let err_flags: u16 = u16::from(self.overrun_samples > 0);
        p.extend_from_slice(&err_flags.to_le_bytes());
        // 环里已填充的样点数：DONE 报本次采集的长度，其余状态报 0
        let ring_fill = self
            .captured
            .as_ref()
            .filter(|_| self.state == State::Done)
            .map(|c| c.channels.first().map(|v| v.len()).unwrap_or(0) as u16)
            .unwrap_or(0);
        p.extend_from_slice(&ring_fill.to_le_bytes());
        p.extend_from_slice(
            &self
                .captured
                .as_ref()
                .map(|c| c.id)
                .unwrap_or(0)
                .to_le_bytes(),
        );
        let ti = self
            .captured
            .as_ref()
            .and_then(|c| c.trigger_index)
            .unwrap_or(0xFFFF_FFFF);
        p.extend_from_slice(&ti.to_le_bytes());
        // ── 下面四个是「这份数据可不可信」的判据。**必须是真统计出来的值。**
        //
        // 回归：这四行从前全是写死的 `0`。主机侧读到「零溢出、零 CRC 错」时，
        // 那根本不是采集下来的证据，而是**没人统计过** —— 而 Agent 会把它
        // 当成「链路很干净」的依据。（`Parser` 其实一直在统计
        // `crc_err_count` / `dropped_count`，只是没人往上报。）
        p.extend_from_slice(&self.overrun_samples.to_le_bytes());
        p.extend_from_slice(&(self.parser.crc_err_count.min(u16::MAX as u32) as u16).to_le_bytes());
        p.extend_from_slice(&(self.parser.dropped_count.min(u16::MAX as u32) as u16).to_le_bytes());
        p.extend_from_slice(&self.tx_dropped.to_le_bytes());
        p.extend_from_slice(&self.uptime_ms().to_le_bytes());
        p.extend_from_slice(&self.tick_us().to_le_bytes());
        p.extend_from_slice(&self.last_error_code.to_le_bytes());
        debug_assert_eq!(p.len(), 33);
        self.queue(flags::RESP, seq, Cmd::GetStatus, p);
    }

    fn cmd_get_config(&mut self, seq: u16) {
        let mut p = Vec::with_capacity(17);
        p.extend_from_slice(&self.cfg.rate_hz.to_le_bytes());
        p.push(self.cfg.acq_mode);
        p.extend_from_slice(&self.cfg.capture_samples.to_le_bytes());
        p.push(self.cfg.format);
        p.extend_from_slice(&self.cfg.decimation.to_le_bytes());
        p.push(self.cfg.trigger_mode);
        p.push(self.cfg.trigger_source);
        p.push(self.cfg.trigger_edge);
        p.extend_from_slice(&self.cfg.trigger_level_lsb.to_le_bytes());
        p.push(self.cfg.ch0_enable);
        p.push(self.cfg.ch0_coupling);
        self.queue(flags::RESP, seq, Cmd::GetConfig, p);
    }

    fn cmd_set_rate(&mut self, seq: u16, payload: &[u8]) {
        if payload.len() < 4 {
            self.queue_error(seq, Cmd::SetSampleRate as u16, 0x0002, "bad len");
            return;
        }
        let requested = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        let actual = scope_core::f103::nearest_achievable_rate(requested);
        self.cfg.rate_hz = actual;
        // 必须回显实际值 —— 这是协议纪律
        self.queue(
            flags::RESP,
            seq,
            Cmd::SetSampleRate,
            actual.to_le_bytes().to_vec(),
        );
    }

    fn cmd_set_channel(&mut self, seq: u16, payload: &[u8]) {
        if payload.len() < 6 {
            self.queue_error(seq, Cmd::SetChannel as u16, 0x0002, "bad len");
            return;
        }
        if payload[0] >= self.channel_count() {
            self.queue_error(
                seq,
                Cmd::SetChannel as u16,
                0x0003,
                "channel not present on this board",
            );
            return;
        }
        // payload: {ch:u8, enable:u8, range_idx:u8, coupling:u8, offset_lsb:i16}
        let enable = payload[1];
        let range_idx = payload[2];
        let coupling = payload[3];
        let offset_lsb = i16::from_le_bytes([payload[4], payload[5]]);
        match payload[0] {
            0 => {
                self.cfg.ch0_enable = enable;
                self.cfg.ch0_range_idx = range_idx;
                self.cfg.ch0_coupling = coupling;
                self.cfg.ch0_offset_lsb = offset_lsb;
            }
            1 => {
                self.cfg.ch1_enable = enable;
                self.cfg.ch1_range_idx = range_idx;
                self.cfg.ch1_coupling = coupling;
                self.cfg.ch1_offset_lsb = offset_lsb;
            }
            _ => {}
        }
        // 回显生效值（协议规定），主机据此知道当前通道配置
        self.queue(flags::RESP, seq, Cmd::SetChannel, payload.to_vec());
    }

    fn cmd_set_trigger(&mut self, seq: u16, payload: &[u8]) {
        if payload.len() < 11 {
            self.queue_error(seq, Cmd::SetTrigger as u16, 0x0002, "bad len");
            return;
        }
        self.cfg.trigger_mode = payload[0];
        self.cfg.trigger_source = payload[1];
        self.cfg.trigger_edge = payload[2];
        self.cfg.trigger_level_lsb = u16::from_le_bytes([payload[3], payload[4]]);
        self.cfg.pre_samples = u16::from_le_bytes([payload[5], payload[6]]);
        self.cfg.holdoff_us = u32::from_le_bytes([payload[7], payload[8], payload[9], payload[10]]);
        self.queue(flags::RESP, seq, Cmd::SetTrigger, payload.to_vec());
    }

    fn cmd_set_acq(&mut self, seq: u16, payload: &[u8]) {
        if payload.len() < 6 {
            self.queue_error(seq, Cmd::SetAcq as u16, 0x0002, "bad len");
            return;
        }
        let samples = u16::from_le_bytes([payload[1], payload[2]]);
        if samples as u32 > scope_core::f103::MAX_CAPTURE_SAMPLES {
            self.queue_error(seq, Cmd::SetAcq as u16, 0x0003, "capture_samples too large");
            return;
        }
        self.cfg.acq_mode = payload[0];
        self.cfg.capture_samples = samples;
        self.cfg.format = payload[3];
        self.cfg.decimation = u16::from_le_bytes([payload[4], payload[5]]);
        self.queue(flags::RESP, seq, Cmd::SetAcq, payload.to_vec());
    }

    fn cmd_arm(&mut self, seq: u16) {
        self.state = State::Armed;
        self.armed_at = Some(std::time::Instant::now());
        self.triggered = false;
        self.captured = None;
        self.queue_ack(seq, Cmd::Arm);
    }

    fn cmd_read_buffer(&mut self, seq: u16, payload: &[u8]) {
        if payload.len() < 10 {
            self.queue_error(seq, Cmd::ReadBuffer as u16, 0x0002, "bad len");
            return;
        }
        let capture_id = u16::from_le_bytes([payload[0], payload[1]]);
        let start = u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
        let count = u16::from_le_bytes([payload[6], payload[7]]);
        // 测试开关：模拟「设备按 RAW16 回，尽管主机请求的是 PACK12」。
        // 真机上这是合法行为；主机必须按分片头回显的 format 解码。
        let format = if self.force_raw16 { 0 } else { payload[8] };
        let ch = payload[9];

        let cap = match &self.captured {
            Some(c) if c.id == capture_id => c.clone(),
            Some(_) => {
                self.queue_error(seq, Cmd::ReadBuffer as u16, 0x0006, "capture_id mismatch");
                return;
            }
            None => {
                self.queue_error(seq, Cmd::ReadBuffer as u16, 0x0006, "no data");
                return;
            }
        };

        let channel = match cap.channels.get(ch as usize) {
            Some(c) => c,
            None => {
                self.queue_error(seq, Cmd::ReadBuffer as u16, 0x0003, "bad channel");
                return;
            }
        };

        // 校验请求的样点数装得进一个 payload —— 分片头算在 payload 之内，
        // 所以 RAW16 的实际上限是 (2060 - 12) / 2 = 1024 样点。
        let max_samples = (MAX_PAYLOAD_TX - CHUNK_HEADER_LEN) / 2;
        if (count as usize) > max_samples {
            self.queue_error(
                seq,
                Cmd::ReadBuffer as u16,
                0x0003,
                "count exceeds max payload",
            );
            return;
        }

        let start_i = start as usize;
        if start_i > channel.len() {
            self.queue_error(seq, Cmd::ReadBuffer as u16, 0x0003, "start out of range");
            return;
        }
        let end = (start_i + count as usize).min(channel.len());
        let slice = &channel[start_i..end];
        let n = slice.len() as u16;

        let mut p = Vec::with_capacity(CHUNK_HEADER_LEN + slice.len() * 2);
        // **模拟器约定**：发生过 overrun 的采集，它的分片数据一律标为无效。
        //
        // 用「约定」而不是「复刻真机」是有意的 —— 固件还是 0 行，串口路径
        // 从未与真机对过话，协议只定义了这个 bit 的含义（「数据无效 /
        // 期间发生 overrun」），**没有**规定「整次采集的每一片都要置位」。
        // 这条是模拟器与主机之间的约定，不是对硬件的断言。
        //
        // 主机侧确实只认这个标志位（`command.rs` 写着 EVENT_OVERRUN 不改
        // 状态机，调用方靠它判断这份数据能不能用）。
        //
        // 回归：从前这里只打 `CHUNK_FLAG_LAST`，于是 `force_overrun` 这个
        // 故障**注入了但主机侧完全观察不到** —— 一个「制造溢出」的开关，
        // 打开后所有现象与没开时一模一样。
        let mut flags = if end >= channel.len() {
            CHUNK_FLAG_LAST
        } else {
            0
        };
        if cap.overrun {
            flags |= CHUNK_FLAG_INVALID;
        }
        let ch_hdr = ChunkHeader {
            capture_id,
            start_sample: start,
            count: n,
            decimation: self.cfg.decimation,
            format,
            flags,
        };
        p.extend_from_slice(&ch_hdr.encode());

        match format {
            0 => {
                // RAW16
                for &s in slice {
                    p.extend_from_slice(&s.to_le_bytes());
                }
            }
            1 => {
                // PACK12
                let mut i = 0;
                while i + 1 < slice.len() {
                    p.extend_from_slice(&pack12_pair(slice[i], slice[i + 1]));
                    i += 2;
                }
                if i < slice.len() {
                    p.extend_from_slice(&pack12_pair(slice[i], 0));
                }
            }
            2 => {
                // MINMAX：每 decimation 个样点出一个 (min,max)
                let d = self.cfg.decimation.max(1) as usize;
                for chunk in slice.chunks(d) {
                    let lo = *chunk.iter().min().unwrap_or(&0);
                    let hi = *chunk.iter().max().unwrap_or(&0);
                    p.extend_from_slice(&lo.to_le_bytes());
                    p.extend_from_slice(&hi.to_le_bytes());
                }
            }
            _ => {
                self.queue_error(seq, Cmd::ReadBuffer as u16, 0x0003, "bad format");
                return;
            }
        }

        self.queue(flags::RESP, seq, Cmd::ReadBuffer, p);
    }

    fn cmd_measure(&mut self, seq: u16, payload: &[u8]) {
        if payload.len() < 14 {
            self.queue_error(seq, Cmd::Measure as u16, 0x0002, "bad len");
            return;
        }
        let capture_id = u16::from_le_bytes([payload[0], payload[1]]);
        let cap = match &self.captured {
            Some(c) if c.id == capture_id => c.clone(),
            _ => {
                self.queue_error(seq, Cmd::Measure as u16, 0x0006, "no data");
                return;
            }
        };
        let s = match cap.channels.first() {
            Some(c) if !c.is_empty() => c,
            _ => {
                self.queue_error(seq, Cmd::Measure as u16, 0x0006, "empty");
                return;
            }
        };

        let min = *s.iter().min().unwrap();
        let max = *s.iter().max().unwrap();
        // u64 累加：4095² × 4096 ≈ 2³⁶，u32 必溢出
        let sum: u64 = s.iter().map(|&v| v as u64).sum();
        let n = s.len() as u64;
        let mean_x256 = ((sum as f64 / n as f64) * 256.0) as i32;

        let mut rising = 0u32;
        let mut prev = s[0];
        for &v in s.iter().skip(1) {
            if prev < 2048 && v >= 2048 {
                rising += 1;
            }
            prev = v;
        }

        let mut p = Vec::with_capacity(30);
        p.extend_from_slice(&min.to_le_bytes());
        p.extend_from_slice(&max.to_le_bytes());
        p.extend_from_slice(&(max - min).to_le_bytes());
        p.extend_from_slice(&mean_x256.to_le_bytes());
        p.extend_from_slice(&0u32.to_le_bytes()); // rms_x256
        p.extend_from_slice(&(rising as u16).to_le_bytes());
        p.extend_from_slice(&0u16.to_le_bytes()); // falling
        p.extend_from_slice(&0u32.to_le_bytes()); // period
        p.extend_from_slice(&0u16.to_le_bytes()); // duty
        p.extend_from_slice(&0u16.to_le_bytes()); // rise time
        p.extend_from_slice(&0u16.to_le_bytes()); // fall time
                                                  // quality: 若有效沿太少就置标志，而不是回看似精确的数字
        let q: u16 = if rising < 2 { 1 << 2 } else { 0 };
        p.extend_from_slice(&q.to_le_bytes());
        debug_assert_eq!(p.len(), 30);

        self.queue(flags::RESP, seq, Cmd::Measure, p);
    }

    // ── 采集完成 ───────────────────────────────────────────────

    fn complete_capture(&mut self) {
        let n = self.cfg.capture_samples as usize;
        let ch_count = self.channel_count() as usize;

        // 必须用 generate_multi：逐通道调 generate 会让第二路整体偏移 n 个样点，
        // 两路时间轴对不上 —— 对 I2C（SCL/SDA 必须同时刻）是致命的。
        let mut channels = self.wave.generate_multi(ch_count, n, self.cfg.rate_hz);

        // **按通道配置改变输出** —— 从前 `ch0_enable` / `offset_lsb` 只是被存下来
        // 并在 `GET_CONFIG` 里回显，**没有任何代码用它**：禁用一个通道，
        // 采集数据一字不变。这会使回显配置与实际采集行为不一致。
        for (ch, samples) in channels.iter_mut().enumerate() {
            let (enable, offset) = match ch {
                0 => (self.cfg.ch0_enable != 0, i32::from(self.cfg.ch0_offset_lsb)),
                _ => (self.cfg.ch1_enable != 0, i32::from(self.cfg.ch1_offset_lsb)),
            };
            if !enable {
                // 通道关掉 → 平坦的 0。**不要**在这里保持原样：那会让
                // 「关了这个通道」与「这个通道是平的」看起来一模一样。
                samples.iter_mut().for_each(|v| *v = 0);
            } else if offset != 0 {
                samples.iter_mut().for_each(|v| {
                    *v = (i32::from(*v) + offset).clamp(0, FULL_SCALE_LSB as i32) as u16;
                });
            }
        }

        // 触发点：在中间找一个上穿电平的位置（模拟真实触发搜索）
        let trigger_index = if self.faults.no_trigger {
            None
        } else {
            find_trigger(
                &channels[0],
                self.cfg.trigger_level_lsb,
                self.cfg.trigger_edge == 0,
            )
        };

        let id = self.next_capture_id;
        self.next_capture_id = self.next_capture_id.wrapping_add(1).max(1);

        let overrun = self.faults.force_overrun;
        if overrun {
            // 溢出的语义是「这些样点没被存下来」——按采集长度计入累计值
            let n = self.cfg.capture_samples as u32;
            self.overrun_samples = self.overrun_samples.saturating_add(n);
        }
        self.captured = Some(Captured {
            id,
            channels,
            trigger_index,
            overrun,
        });
        self.state = State::Done;

        let cap = self.captured.as_ref().unwrap();
        let mut p = Vec::with_capacity(18);
        p.extend_from_slice(&id.to_le_bytes());
        p.extend_from_slice(&cap.trigger_index.unwrap_or(0xFFFF_FFFF).to_le_bytes());
        p.extend_from_slice(&self.tick_us().to_le_bytes());
        p.extend_from_slice(&self.cfg.rate_hz.to_le_bytes());
        p.extend_from_slice(&(cap.channels[0].len() as u32).to_le_bytes());

        if cap.overrun {
            let mut o = Vec::with_capacity(6);
            o.extend_from_slice(&64u32.to_le_bytes());
            o.extend_from_slice(&id.to_le_bytes());
            self.queue_event(Cmd::EventOverrun, o);
        }
        self.queue_event(Cmd::EventTrigger, p);
    }

    /// 推进模拟器的时间：ARMED 状态下的触发判定。
    ///
    /// 两条路径必须**分开**，因为它们的语义完全不同：
    ///
    /// | 模式 | 行为 | no_trigger 是否生效 |
    /// |---|---|---|
    /// | `auto` (0) | 约 200 ms 无触发 → 强制完成 | **不生效** |
    /// | `normal`/`single` | 信号满足条件才完成 | 生效（模拟"信号一直不触发"） |
    ///
    /// `no_trigger` 之所以不能作用于 auto —— 真机的 auto 模式就是会超时完成的，
    /// 而 Agent 正是靠这一条才不会永久阻塞。把它一起屏蔽掉，
    /// 模拟器就会表现为"Agent 卡死"，与真机行为不符。
    fn tick(&mut self) {
        if self.state != State::Armed || self.triggered {
            return;
        }
        let Some(armed_at) = self.armed_at else {
            return;
        };
        let elapsed = armed_at.elapsed();

        // ── auto 模式：约 200 ms 无触发就强制完成 ──
        if self.cfg.trigger_mode == 0 {
            if elapsed.as_millis() >= 200 {
                self.complete_capture();
            }
            return;
        }

        // ── normal / single：模拟一段触发延迟后完成 ──
        // 注入的"永不触发"只在这一支生效
        if self.faults.no_trigger {
            return;
        }
        if elapsed.as_millis() >= 5 {
            self.triggered = true;
            self.complete_capture();
        }
    }

    fn tick_us(&self) -> u32 {
        // 1 MHz 时钟 → 微秒计数；u32 自然回绕约 71.6 分钟（规格里写明了）
        self.uptime_ms().wrapping_mul(1000)
    }

    fn uptime_ms(&self) -> u32 {
        use std::sync::OnceLock;
        static START: OnceLock<std::time::Instant> = OnceLock::new();
        let start = START.get_or_init(std::time::Instant::now);
        start.elapsed().as_millis() as u32
    }
}

/// 在样本里搜触发点。
///
/// 迟滞必须是一个**状态机**，不能写成「相邻两点跨越整条迟滞带」——
/// 那样的话缓变信号永远触发不了：1 kHz 正弦在 857 kSPS 下每样点只变化
/// 约 14 LSB，而迟滞带是 32 LSB 宽，单步根本跨不过去。
///
/// 正确语义（与真实示波器的施密特触发一致）：
/// 1. 信号必须先跌到 `lo` 以下置位（armed）
/// 2. 之后升到 `hi` 以上才算一次上升沿
/// 3. 落在带内的值既不置位也不触发 —— 这就是迟滞抑制抖动的原理
fn find_trigger(s: &[u16], level: u16, rising: bool) -> Option<u32> {
    const HYST: u16 = 16;
    let lo = level.saturating_sub(HYST);
    let hi = level.saturating_add(HYST).min(4095);

    // 初始是否已置位，取决于第一个样点落在迟滞带的哪一侧
    let first = *s.first()?;
    let mut primed = if rising { first <= lo } else { first >= hi };

    for (i, &v) in s.iter().enumerate() {
        if !rising {
            // 下降沿：先确认升到 hi 以上，再等跌到 lo 以下
            if v >= hi {
                primed = true;
            } else if v <= lo && primed {
                return Some(i as u32);
            }
            continue;
        }

        if v <= lo {
            primed = true;
        } else if v >= hi && primed {
            return Some(i as u32);
        }
        // 带内的值：不改状态、不触发
    }
    None
}

impl DevicePort for SimDevice {
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), LinkError> {
        // 把新字节和上次没喂完的拼起来。data 是局部变量 ——
        // 这样解析器借用它的时候不会和 &mut self 冲突。
        let mut data = std::mem::take(&mut self.rx);
        data.extend_from_slice(bytes);

        let mut first = true;
        loop {
            // 只有第一次要把新字节交给解析器；
            // 后续轮次是在排空解析器内部缓冲里已经攒下的帧
            let chunk: &[u8] = if first { &data } else { &[] };
            first = false;

            match self.parser.feed(chunk, &mut self.frame) {
                ParseStatus::Ok | ParseStatus::BadVer => {
                    let hdr = self.frame.header;
                    let payload = self.frame.payload.clone();

                    // 主版本不符时仍必须应答 GET_INFO / PING，其余回 VERSION_MISMATCH
                    let is_basic =
                        hdr.cmd_raw == Cmd::GetInfo as u16 || hdr.cmd_raw == Cmd::Ping as u16;
                    if !hdr.ver_matches() && !is_basic {
                        self.queue_error(hdr.seq, hdr.cmd_raw, 0x0009, "version mismatch");
                    } else {
                        self.handle(hdr, &payload);
                    }
                }
                ParseStatus::NeedMore => break,
                // 解析器已自行重同步，继续扫下一帧
                ParseStatus::CrcErr | ParseStatus::BadLen => continue,
            }
        }
        Ok(())
    }

    fn read_some(&mut self) -> Result<Vec<u8>, LinkError> {
        self.tick();

        if self.tx.is_empty() {
            return Ok(Vec::new());
        }

        // 故障注入：延迟尖峰
        if self.faults.latency_spike_probability > 0.0 && self.frames_sent % 7 == 0 {
            // 只在极端情况下触发，避免简单场景变慢
            let roll = (self.frames_sent as f64 * 0.137).fract();
            if roll < self.faults.latency_spike_probability {
                std::thread::sleep(std::time::Duration::from_millis(
                    self.faults.latency_spike_ms,
                ));
            }
        }

        // 模拟链路的字节率：一次最多给 64 字节，逼出真实的粘包/拆包路径
        let n = self.tx.len().min(64);
        Ok(self.tx.drain(..n).collect())
    }

    fn describe(&self) -> String {
        format!("sim({})", self.scenario.name())
    }

    fn is_simulated(&self) -> bool {
        true
    }

    fn byte_rate(&self) -> u32 {
        u32::MAX // 模拟器不受链路限制
    }
}

#[cfg(test)]
mod trigger_tests {
    use super::find_trigger;

    #[test]
    fn finds_rising_edge_on_slow_ramp() {
        // 缓变信号：每样点只走 8 LSB，远小于 32 LSB 的迟滞带宽。
        // 这是回归用例 —— 之前在单步跨越判据下会返回 None。
        let s: Vec<u16> = (1000u32..3000).step_by(8).map(|v| v as u16).collect();
        let t = find_trigger(&s, 2048, true).expect("缓变信号必须能触发");
        assert!(s[t as usize] as u32 >= 2064);
        assert!(t > 0);
    }

    #[test]
    fn hysteresis_ignores_noise_around_level() {
        // 在阈值附近抖动 ±8 LSB（带内）不应触发；
        // 只有真正跌破 lo 再升过 hi 才算。
        let mut s = vec![2048u16; 50];
        for i in 0..40 {
            s.push(if i % 2 == 0 { 2040 } else { 2056 }); // 全在 2032..2064 带内
        }
        s.push(2000); // 跌破 lo
        s.extend(std::iter::repeat(2060u16).take(10));
        s.push(3000); // 升过 hi

        let t = find_trigger(&s, 2048, true);
        assert!(t.is_some(), "跌破再升起应当触发");
        assert_eq!(s[t.unwrap() as usize], 3000);
    }

    #[test]
    fn falling_edge_works_symmetrically() {
        let s: Vec<u16> = (1000u32..3000).rev().step_by(8).map(|v| v as u16).collect();
        let t = find_trigger(&s, 2048, false).expect("下降沿必须能触发");
        assert!(s[t as usize] as u32 <= 2032);
    }

    #[test]
    fn no_trigger_when_signal_never_crosses() {
        // 完全在阈值上方的平坦信号不应触发
        let s = vec![3000u16; 100];
        assert_eq!(find_trigger(&s, 2048, true), None);
    }

    #[test]
    fn empty_input_is_handled() {
        assert_eq!(find_trigger(&[], 2048, true), None);
    }
}

#[cfg(test)]
mod caps_tests {
    use super::{CapsMode, SimDevice};
    use crate::waveform::Scenario;
    use scope_proto::*;

    #[test]
    fn default_caps_match_the_real_device() {
        // 默认口径必须与真机 DEV_CAPS 逐位一致：只有四位。
        let dev = SimDevice::new(Scenario::I2c100k);
        assert_eq!(dev.channel_count(), 2, "I2C 场景仍是双通道");
        assert_eq!(
            dev.caps(),
            CAP_TRIG_AUTO | CAP_TRIG_NORMAL | CAP_TRIG_SINGLE | CAP_FMT_PACK12,
            "默认真机口径不得多声明任何一位"
        );
        assert_eq!(dev.caps() & CAP_CH_DUAL, 0, "真板只有 1 通道");
    }

    #[test]
    fn ideal_caps_keep_the_simulator_extra_bits() {
        let mut dev = SimDevice::new(Scenario::I2c100k);
        dev.set_caps_mode(CapsMode::IdealSim);
        for bit in [
            CAP_TRIG_SOFT,
            CAP_COUPLING_AC,
            CAP_FMT_MINMAX,
            CAP_DIGITAL_I2C,
            CAP_CH_DUAL,
        ] {
            assert_ne!(dev.caps() & bit, 0, "IdealSim 应保留模拟器完整能力");
        }
    }
}
