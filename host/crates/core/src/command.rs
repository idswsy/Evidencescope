//! [`CommandBus`] —— 协议之上的命令层。
//!
//! 职责：
//! - 把高层 API 调用编码成帧下发，把响应解码回结构体
//! - 管理 `seq`、超时、重试
//! - 维护设备状态缓存（当前时基、量程、触发设置）
//! - 对上层（CLI / MCP / GUI）提供与硬件无关的接口
//!
//! ## 两条硬纪律
//!
//! 1. **`SET_*` 一律以设备回显为准**。F103 的采样率被定时器分频量化，
//!    主机必须以 `actual_hz` 建时间轴，不能用自己的请求值。
//! 2. **非幂等命令不得并发流水**。设备只有 1 槽去重缓存，
//!    并发两个 `ARM` 会让回放机制失效。

use crate::device::{DevicePort, FrameReader};
use crate::error::{DeviceError, LinkError, Result, ScopeError};
use scope_proto::{
    cmd_allowed, Cmd, ErrorCode, Frame, Header, ParseStatus, Severity, State, CHUNK_HEADER_LEN,
};
use std::time::Duration;

/// 控制命令的默认超时。
pub const TIMEOUT_CONTROL: Duration = Duration::from_millis(200);
/// `PING` 的超时。
pub const TIMEOUT_PING: Duration = Duration::from_millis(500);
/// 触发等待的默认上限。
pub const TIMEOUT_TRIGGER: Duration = Duration::from_millis(2000);

/// 重试策略。
#[derive(Debug, Clone, Copy)]
pub struct RetryPolicy {
    /// 最大尝试次数（含首次）。
    pub max_attempts: u32,
    /// 两次尝试之间的等待。
    pub backoff: Duration,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        RetryPolicy {
            max_attempts: 3,
            backoff: Duration::from_millis(20),
        }
    }
}

/// 设备上报的能力（`GET_INFO` 响应）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    /// 协议版本字节。
    pub proto_ver: u8,
    /// 固件版本。
    pub fw_ver: u32,
    /// 型号。
    pub model: u16,
    /// 唯一 ID（标定表按它索引）。
    pub uid: [u8; 12],
    /// 设备时钟频率。
    pub tick_hz: u32,
    /// ADC 位数。
    pub adc_bits: u8,
    /// 通道数。
    pub ch_count: u8,
    /// 支持的最低采样率。
    pub rate_min_hz: u32,
    /// 支持的最高采样率。**F103 上是 857142，不是 1000000。**
    pub rate_max_hz: u32,
    /// 单次采集最大样点数。
    pub capture_max_samples: u16,
    /// 设备可接收的 payload 上限。
    pub max_rx_payload: u16,
    /// 设备可发出的 payload 上限。
    pub max_tx_payload: u16,
    /// 建议分片样点数。
    pub preferred_chunk_samples: u16,
    /// 能力位掩码。
    pub caps: u32,
}

impl DeviceInfo {
    /// `GET_INFO` 响应 payload 的长度（与 C 侧 `info_resp_t` 一致）。
    pub const PAYLOAD_LEN: usize = 45;

    /// 从 `GET_INFO` 响应 payload 解析。
    ///
    /// 字段偏移由 `proto/protocol.h` 的 `_Static_assert` 锁死，
    /// 两边一旦不同步，C 端编译期就会失败。
    pub fn decode(p: &[u8]) -> Option<DeviceInfo> {
        if p.len() < Self::PAYLOAD_LEN {
            return None;
        }
        let mut uid = [0u8; 12];
        uid.copy_from_slice(&p[7..19]);
        Some(DeviceInfo {
            proto_ver: p[0],
            fw_ver: u32::from_le_bytes([p[1], p[2], p[3], p[4]]),
            model: u16::from_le_bytes([p[5], p[6]]),
            uid,
            tick_hz: u32::from_le_bytes([p[19], p[20], p[21], p[22]]),
            adc_bits: p[23],
            ch_count: p[24],
            rate_min_hz: u32::from_le_bytes([p[25], p[26], p[27], p[28]]),
            rate_max_hz: u32::from_le_bytes([p[29], p[30], p[31], p[32]]),
            capture_max_samples: u16::from_le_bytes([p[33], p[34]]),
            max_rx_payload: u16::from_le_bytes([p[35], p[36]]),
            max_tx_payload: u16::from_le_bytes([p[37], p[38]]),
            preferred_chunk_samples: u16::from_le_bytes([p[39], p[40]]),
            caps: u32::from_le_bytes([p[41], p[42], p[43], p[44]]),
        })
    }
}

/// 设备当前配置镜像（`GET_CONFIG` 响应）。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceConfig {
    /// 实际生效的采样率。
    pub rate_hz: u32,
    /// 采集模式。
    pub acq_mode: u8,
    /// 单次采集样点数。
    pub capture_samples: u16,
    /// 波形格式。
    pub format: u8,
    /// 抽点倍数。
    pub decimation: u16,
    /// 触发电平（ADC LSB）。
    pub trigger_level_lsb: u16,
    /// 触发模式。
    pub trigger_mode: u8,
    /// 触发边沿。
    pub trigger_edge: u8,
    /// 触发源。
    pub trigger_source: u8,
    /// 通道 0 启用状态。
    pub ch0_enable: u8,
    /// 通道 0 耦合。
    pub ch0_coupling: u8,
}

/// `GET_STATUS` 的完整响应（33 字节，见 `docs/03-protocol.md` §5.1）。
///
/// **为什么要有这个结构**：从前 `get_status()` 只读第一个字节（`state`），
/// 把 payload 其余 32 字节**直接丢掉**。于是协议里定义的溢出计数、CRC 错误
/// 计数、丢帧计数一路都到不了上层 —— 一个 Agent 读到「溢出 0 次」时，
/// 那根本不是它采集下来的数据算出来的，而是**没人算过**。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceStatus {
    /// 当前状态机状态。
    pub state: State,
    /// 错误标志位（含义见 `docs/03-protocol.md`）。
    pub err_flags: u16,
    /// 环缓冲里已填充的样点数。
    pub ring_fill_samples: u16,
    /// 最近一次采集的 id。
    pub last_capture_id: u16,
    /// 最近一次触发的样点下标；`None` = 还没触发过（线上是 `0xFFFFFFFF`）。
    pub last_trigger_index: Option<u32>,
    /// **累计被丢弃的样点数**（采集环溢出）。判断一份采集可不可信要看它。
    pub overrun_samples: u32,
    /// 累计收到的 CRC 错误帧数。
    pub rx_crc_err: u16,
    /// 累计丢弃的接收字节数（重同步时跳过的）。
    pub rx_dropped: u16,
    /// 累计丢弃的发送字节数。
    pub tx_dropped: u32,
    /// 设备开机以来的毫秒数。
    pub uptime_ms: u32,
    /// 设备侧微秒级 tick（时间对齐用）。
    pub tick_us: u32,
    /// 最近一次错误的错误码（0 = 无）。
    pub last_error_code: u16,
}

impl DeviceStatus {
    /// 从 33 字节 payload 解析。长度不足返回 `None`（**不 panic**）。
    ///
    /// 字段偏移见 `docs/03-protocol.md` §5.1 的 payload 定义：
    ///
    /// ```text
    ///  0  state            u8
    ///  1  err_flags        u16
    ///  3  ring_fill        u16
    ///  5  last_capture_id  u16
    ///  7  last_trigger_idx u32
    /// 11  overrun_samples  u32
    /// 15  rx_crc_err       u16
    /// 17  rx_dropped       u16
    /// 19  tx_dropped       u32
    /// 23  uptime_ms        u32
    /// 27  tick_us          u32
    /// 31  last_error_code  u16   → 共 33
    /// ```
    ///
    /// ⚠ **这些偏移是手写的，错了不会编译不过。** 第一版从 `overrun_samples`
    /// 起整体少了 2 字节，于是把「没有触发过」哨兵 `0xFFFFFFFF` 的高半字
    /// 读成了溢出计数 —— 一台**刚连上、什么都没做**的设备报告「溢出 65535 次」。
    /// `decode_reads_every_field_at_the_right_offset` 那条测试守着这件事。
    pub fn decode(p: &[u8]) -> Option<DeviceStatus> {
        if p.len() < 33 {
            return None;
        }
        let u16at = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let u32at = |i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let ti = u32at(7);
        Some(DeviceStatus {
            state: State::from_u8(p[0]).unwrap_or(State::Idle),
            err_flags: u16at(1),
            ring_fill_samples: u16at(3),
            last_capture_id: u16at(5),
            // 协议里 `0xFFFFFFFF` 是「没有触发过」的哨兵，不是下标
            last_trigger_index: (ti != 0xFFFF_FFFF).then_some(ti),
            overrun_samples: u32at(11),
            rx_crc_err: u16at(15),
            rx_dropped: u16at(17),
            tx_dropped: u32at(19),
            uptime_ms: u32at(23),
            tick_us: u32at(27),
            last_error_code: u16at(31),
        })
    }

    /// 这份数据可不可信 —— 有溢出或链路错误就不是干净的采集。
    ///
    /// 给上层一句话的判据，免得每个消费方各写一遍阈值。
    pub fn link_is_clean(&self) -> bool {
        self.overrun_samples == 0 && self.rx_crc_err == 0 && self.rx_dropped == 0
    }
}

/// 一次特征（transaction）的结果。
#[derive(Debug, Clone)]
pub struct Response {
    /// 序号。
    pub seq: u16,
    /// 命令码。
    pub cmd: u16,
    /// 标志位。
    pub flags: u8,
    /// payload。
    pub payload: Vec<u8>,
}

/// 命令总线。
pub struct CommandBus<P: DevicePort> {
    port: P,
    reader: FrameReader,
    seq: u16,
    retry: RetryPolicy,

    /// 最近一次 `GET_INFO` 的结果。
    pub info: Option<DeviceInfo>,
    /// 最近一次 `GET_CONFIG` 的结果（设备真值，不是主机的意图）。
    pub config: Option<DeviceConfig>,
    /// 最近一次已知状态。
    pub state: Option<State>,
}

impl<P: DevicePort> CommandBus<P> {
    /// 包装一个已连通的 [`DevicePort`]。
    pub fn new(port: P) -> Self {
        CommandBus {
            port,
            reader: FrameReader::new(),
            seq: 0,
            retry: RetryPolicy::default(),
            info: None,
            config: None,
            state: None,
        }
    }

    /// 设置重试策略。
    pub fn with_retry(mut self, retry: RetryPolicy) -> Self {
        self.retry = retry;
        self
    }

    /// 底层链路描述。
    pub fn describe(&self) -> String {
        self.port.describe()
    }

    /// 是否为模拟器。
    pub fn is_simulated(&self) -> bool {
        self.port.is_simulated()
    }

    /// 取回底层端口的可变引用（模拟器场景下用于故障注入等）。
    pub fn port_mut(&mut self) -> &mut P {
        &mut self.port
    }

    fn next_seq(&mut self) -> u16 {
        self.seq = self.seq.wrapping_add(1);
        self.seq
    }

    // ── 核心：一次请求-响应事务 ─────────────────────────────────

    /// 发送一条命令并等待匹配 `seq` 的响应。
    ///
    /// 重试语义（对应协议规格 §8）：
    /// - 幂等命令：直接按原 `seq` 重发
    /// - 非幂等命令（`ARM`）：同样按原 `seq` 重发，靠**设备侧单槽
    ///   去重缓存**回放上次响应，保证不会重复执行
    /// - 超时后先排空接收缓冲、重同步链路，再重试
    pub fn transaction(
        &mut self,
        cmd: Cmd,
        payload: Vec<u8>,
        timeout: Duration,
    ) -> Result<Response> {
        let seq = self.next_seq();
        let frame = Frame {
            header: Header {
                ver: scope_proto::VER,
                flags: 0,
                seq,
                cmd_raw: cmd as u16,
                len: payload.len() as u16,
            },
            payload,
        };
        // 主机 → 设备方向的上限是 MAX_PAYLOAD_RX（512），**比设备发来的小得多**。
        //
        // 必须在这里拦下：设备的解析缓冲只有 MAX_FRAME_RX（524 字节），
        // 收到的帧一旦装不下，解析器会把它逐字节削掉，既不解析也不回错误帧
        // —— 主机只能看到三次无意义的超时，完全不知道是自己发太大了。
        if frame.payload.len() > scope_proto::MAX_PAYLOAD_RX {
            return Err(ScopeError::InvalidParam {
                field: "payload",
                value: format!("{} 字节", frame.payload.len()),
                reason: format!(
                    "主机→设备方向上限是 {} 字节（设备接收缓冲 {} 字节）；\
                     设备发来的分片上限是 {} 字节，别把两者搞混",
                    scope_proto::MAX_PAYLOAD_RX,
                    scope_proto::MAX_FRAME_RX,
                    scope_proto::MAX_PAYLOAD_TX
                ),
            });
        }

        let bytes = frame.try_encode().ok_or_else(|| ScopeError::InvalidParam {
            field: "payload",
            value: format!("{} 字节", frame.payload.len()),
            reason: format!("超过帧编码上限 {} 字节", scope_proto::MAX_PAYLOAD_TX),
        })?;

        let mut last_err: Option<ScopeError> = None;

        for attempt in 0..self.retry.max_attempts {
            if attempt > 0 {
                // 超时后不要立刻重发：先排空接收缓冲、重同步链路
                self.drain_rx();
                std::thread::sleep(self.retry.backoff);
            }

            self.port.write_all(&bytes).map_err(ScopeError::Link)?;

            match self.await_response(seq, timeout) {
                Ok(resp) => return Ok(resp),
                Err(e @ ScopeError::Link(LinkError::Timeout(_))) => {
                    last_err = Some(e);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }

        Err(last_err.unwrap_or(ScopeError::Link(LinkError::Disconnected)))
    }

    /// 等待指定 `seq` 的响应；丢弃不匹配的帧（事件帧等）。
    fn await_response(&mut self, seq: u16, timeout: Duration) -> Result<Response> {
        let deadline = std::time::Instant::now() + timeout;

        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Err(ScopeError::Link(LinkError::Timeout(timeout)));
            }

            let frame = self.reader.next_frame(&mut self.port, remaining)?;

            // 设备主动上报的事件帧：更新状态缓存后继续等响应
            if frame.header.flags & scope_proto::flags::EVENT != 0 {
                self.observe_event(&frame);
                continue;
            }

            if frame.header.seq != seq {
                continue; // 陈旧的响应/事件，丢弃
            }

            if frame.header.flags & scope_proto::flags::ERROR != 0 {
                return Err(ScopeError::Device(decode_error(&frame.payload)));
            }

            return Ok(Response {
                seq: frame.header.seq,
                cmd: frame.header.cmd_raw,
                flags: frame.header.flags,
                payload: frame.payload,
            });
        }
    }

    /// 把设备主动上报的事件吸收进状态缓存。
    fn observe_event(&mut self, frame: &Frame) {
        // EventOverrun / EventLog 不改状态机，调用方要靠 GET_STATUS 里的
        // overrun_samples 计数看到它们，所以这里只处理 EventTrigger。
        if Cmd::try_from_u16(frame.header.cmd_raw) == Some(Cmd::EventTrigger) {
            self.state = Some(State::Done);
        }
    }

    /// 排空接收缓冲中所有已到达的帧（重试前调用）。
    fn drain_rx(&mut self) {
        let short = Duration::from_millis(5);
        let deadline = std::time::Instant::now() + short;
        while std::time::Instant::now() < deadline {
            if self.reader.next_frame(&mut self.port, short).is_err() {
                break;
            }
        }
    }

    // ── 高层命令 ───────────────────────────────────────────────

    /// 建立会话：`GET_INFO` + `GET_CONFIG`。
    ///
    /// **这是连接后的第一条命令**，任何其他调用都应在它之后。
    pub fn connect(&mut self) -> Result<DeviceInfo> {
        let resp = self.transaction(Cmd::GetInfo, Vec::new(), TIMEOUT_PING)?;
        let info = DeviceInfo::decode(&resp.payload).ok_or_else(|| ScopeError::InvalidParam {
            field: "GET_INFO.payload",
            value: format!("{} 字节", resp.payload.len()),
            reason: format!(
                "响应长度应为 {} 字节，固件可能未实现或版本不符",
                DeviceInfo::PAYLOAD_LEN
            ),
        })?;
        self.info = Some(info.clone());

        // 配置读取失败不应阻塞连接
        if let Ok(cfg) = self.get_config() {
            self.config = Some(cfg);
        }
        Ok(info)
    }

    /// `PING`：链路保活 + 往返时延测量。
    pub fn ping(&mut self, data: &[u8]) -> Result<(Vec<u8>, Duration)> {
        let mut payload = Vec::with_capacity(1 + data.len());
        payload.push(data.len().min(32) as u8);
        payload.extend_from_slice(&data[..data.len().min(32)]);

        let t0 = std::time::Instant::now();
        let resp = self.transaction(Cmd::Ping, payload, TIMEOUT_PING)?;
        let rtt = t0.elapsed();

        // 响应 = len + data + tick_us(4)
        if resp.payload.is_empty() {
            return Ok((Vec::new(), rtt));
        }
        let n = resp.payload[0] as usize;
        let echoed = resp.payload.get(1..1 + n).unwrap_or(&[]).to_vec();
        Ok((echoed, rtt))
    }

    /// `GET_STATUS`：任何状态下都可调用。
    pub fn get_status(&mut self) -> Result<DeviceStatus> {
        let resp = self.transaction(Cmd::GetStatus, Vec::new(), TIMEOUT_CONTROL)?;
        // 解析失败**不当成 Idle 混过去** —— 那会让「设备回了一个我们不认识的
        // 状态」看起来像「设备空闲」。此时应直接报错。
        let st = DeviceStatus::decode(&resp.payload).ok_or_else(|| {
            ScopeError::Unsupported(format!(
                "GET_STATUS 响应长度 {} 字节，协议规定 33 字节",
                resp.payload.len()
            ))
        })?;
        self.state = Some(st.state);
        Ok(st)
    }

    /// 重新读取设备配置真值。
    pub fn get_config(&mut self) -> Result<DeviceConfig> {
        let resp = self.transaction(Cmd::GetConfig, Vec::new(), TIMEOUT_CONTROL)?;
        let p = &resp.payload;
        if p.len() < 4 {
            return Err(ScopeError::InvalidParam {
                field: "GET_CONFIG.payload",
                value: format!("{} 字节", p.len()),
                reason: "响应过短".into(),
            });
        }
        let cfg = DeviceConfig {
            rate_hz: u32::from_le_bytes([p[0], p[1], p[2], p[3]]),
            acq_mode: *p.get(4).unwrap_or(&0),
            capture_samples: p
                .get(5..7)
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .unwrap_or(0),
            format: *p.get(7).unwrap_or(&0),
            decimation: p
                .get(8..10)
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .unwrap_or(1),
            trigger_mode: *p.get(10).unwrap_or(&0),
            trigger_source: *p.get(11).unwrap_or(&0),
            trigger_edge: *p.get(12).unwrap_or(&0),
            trigger_level_lsb: p
                .get(13..15)
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .unwrap_or(2048),
            ch0_enable: *p.get(15).unwrap_or(&1),
            ch0_coupling: *p.get(16).unwrap_or(&0),
        };
        self.config = Some(cfg.clone());
        Ok(cfg)
    }

    /// `SET_SAMPLE_RATE`。
    ///
    /// **返回设备量化后的实际值**，不是请求值。时间轴必须用返回值。
    ///
    /// 本地校验规则：先把请求吸附到定时器能产生的档位，**再**判断这个档位
    /// 是否落在设备声明的 `[rate_min, rate_max]` 内。
    ///
    /// 顺序很重要 —— 反过来会把 `857143` 这种「只差 1 Hz」的常见近似值
    /// 直接判死，而它显然应该被量化成 `857142`。
    /// 真正离谱的请求（如 2 MHz）仍然会被挡下。
    pub fn set_sample_rate(&mut self, requested_hz: u32) -> Result<u32> {
        self.guard_config_allowed("SET_SAMPLE_RATE")?;

        if let Some(info) = &self.info {
            let snapped = crate::f103::nearest_achievable_rate(requested_hz);
            if snapped < info.rate_min_hz || snapped > info.rate_max_hz {
                return Err(ScopeError::InvalidParam {
                    field: "sample_rate_hz",
                    value: requested_hz.to_string(),
                    reason: format!(
                        "本机支持范围 {}..{} Hz（F103 在 72MHz+USB 下 ADCCLK 只能到 12MHz，\
                         因此上限是 857142 而不是 1000000）",
                        info.rate_min_hz, info.rate_max_hz
                    ),
                });
            }
        }

        let resp = self.transaction(
            Cmd::SetSampleRate,
            requested_hz.to_le_bytes().to_vec(),
            TIMEOUT_CONTROL,
        )?;

        let actual = resp
            .payload
            .get(0..4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .ok_or_else(|| ScopeError::InvalidParam {
                field: "SET_SAMPLE_RATE.resp",
                value: format!("{} 字节", resp.payload.len()),
                reason: "设备必须回显 actual_hz".into(),
            })?;

        if let Some(cfg) = &mut self.config {
            cfg.rate_hz = actual;
        }
        Ok(actual)
    }

    /// `SET_TRIGGER`。
    pub fn set_trigger(
        &mut self,
        mode: u8,
        source: u8,
        edge: u8,
        level_lsb: u16,
        pre_samples: u16,
        holdoff_us: u32,
    ) -> Result<()> {
        self.guard_config_allowed("SET_TRIGGER")?;

        if level_lsb > 4095 {
            return Err(ScopeError::InvalidParam {
                field: "level_lsb",
                value: level_lsb.to_string(),
                reason: "12-bit ADC 电平范围是 0..4095".into(),
            });
        }

        let mut p = Vec::with_capacity(11);
        p.push(mode);
        p.push(source);
        p.push(edge);
        p.extend_from_slice(&level_lsb.to_le_bytes());
        p.extend_from_slice(&pre_samples.to_le_bytes());
        p.extend_from_slice(&holdoff_us.to_le_bytes());

        let _ = self.transaction(Cmd::SetTrigger, p, TIMEOUT_CONTROL)?;

        if let Some(cfg) = &mut self.config {
            cfg.trigger_mode = mode;
            cfg.trigger_source = source;
            cfg.trigger_edge = edge;
            cfg.trigger_level_lsb = level_lsb;
        }
        Ok(())
    }

    /// `SET_ACQ`。
    pub fn set_acq(
        &mut self,
        mode: u8,
        capture_samples: u16,
        format: u8,
        decimation: u16,
    ) -> Result<()> {
        self.guard_config_allowed("SET_ACQ")?;

        let max = self
            .info
            .as_ref()
            .map(|i| i.capture_max_samples)
            .unwrap_or(scope_proto::CAPTURE_MAX_SAMPLES as u16);
        if capture_samples > max {
            return Err(ScopeError::InvalidParam {
                field: "capture_samples",
                value: capture_samples.to_string(),
                reason: format!(
                    "超过设备上限 {max} 点（8 KB 环 / u16 = 4096，这是 F103 的物理上限）"
                ),
            });
        }
        let max_decim = crate::f103::MAX_DECIMATION as u16;
        if decimation == 0 || decimation > max_decim {
            return Err(ScopeError::InvalidParam {
                field: "decimation",
                value: decimation.to_string(),
                reason: format!("范围 1..={max_decim}"),
            });
        }

        let mut p = Vec::with_capacity(6);
        p.push(mode);
        p.extend_from_slice(&capture_samples.to_le_bytes());
        p.push(format);
        p.extend_from_slice(&decimation.to_le_bytes());
        let _ = self.transaction(Cmd::SetAcq, p, TIMEOUT_CONTROL)?;

        if let Some(cfg) = &mut self.config {
            cfg.acq_mode = mode;
            cfg.capture_samples = capture_samples;
            cfg.format = format;
            cfg.decimation = decimation;
        }
        Ok(())
    }

    /// `SET_CHANNEL`。
    ///
    /// 电平与偏移**一律用 ADC LSB 整数**，i16 所以负偏移合法（把波形挪进屏幕）。
    /// 伏特换算、量程衰减、AC/DC 校正全部留在显示层（ADR-006）。
    pub fn set_channel(
        &mut self,
        ch: u8,
        enable: bool,
        range_idx: u8,
        coupling: u8,
        offset_lsb: i16,
    ) -> Result<()> {
        self.guard_config_allowed("SET_CHANNEL")?;

        let ch_count = self.info.as_ref().map(|i| i.ch_count).unwrap_or(1);
        if ch >= ch_count {
            return Err(ScopeError::InvalidParam {
                field: "ch",
                value: ch.to_string(),
                reason: format!("该设备只有 {ch_count} 个通道（以 GET_INFO 上报为准）"),
            });
        }

        let mut p = Vec::with_capacity(6);
        p.push(ch);
        p.push(u8::from(enable));
        p.push(range_idx);
        p.push(coupling);
        p.extend_from_slice(&offset_lsb.to_le_bytes());
        let _ = self.transaction(Cmd::SetChannel, p, TIMEOUT_CONTROL)?;

        // 主机侧的配置镜像**只更新 ch0** —— `DeviceConfig` 就是照
        // `GET_CONFIG` 的 17 字节建的，里面只有通道 0 的两个字段。
        //
        // 回归：这里从前无论 `ch` 是几都往 `ch0_*` 里写，于是「配一下 CH2」
        // 会把镜像里的 CH1 使能改成 CH2 的 —— `scope_status` 报的是镜像，
        // Agent 会看到 CH1 被关了，而设备上其实没有。
        if ch == 0 {
            if let Some(cfg) = &mut self.config {
                cfg.ch0_enable = u8::from(enable);
                cfg.ch0_coupling = coupling;
            }
        }
        Ok(())
    }

    /// `RESET` —— 设备回到上电初始态。
    ///
    /// **这是从 [`State::Fault`] 里出来的唯一办法。** `cmd_allowed` 规定 Fault 态下
    /// 只接受 Reset，其余命令一律被拒 —— 没有这个方法，设备一旦进 Fault，
    /// 上位机就彻底够不着它了（CLI 和 GUI 都够不着，只能重启板子）。
    ///
    /// 副作用是配置全部回出厂值，所以顺手重取一次 `GET_CONFIG`，
    /// 免得缓存里留着已经不成立的旧值。
    pub fn reset(&mut self) -> Result<()> {
        let _ = self.transaction(Cmd::Reset, Vec::new(), TIMEOUT_CONTROL)?;
        self.state = Some(State::Idle);
        self.config = Some(self.get_config()?);
        Ok(())
    }

    /// `ARM`。**非幂等** —— 内部靠设备侧 seq 去重缓存保证重试安全。
    pub fn arm(&mut self) -> Result<()> {
        if let Some(state) = self.state {
            if !cmd_allowed(state, Cmd::Arm) {
                return Err(ScopeError::BadState {
                    current: state,
                    action: "ARM",
                    hint: "先发 STOP 再重新 ARM".into(),
                });
            }
        }
        let _ = self.transaction(Cmd::Arm, Vec::new(), TIMEOUT_CONTROL)?;
        self.state = Some(State::Armed);
        Ok(())
    }

    /// `STOP`。幂等。
    pub fn stop(&mut self) -> Result<()> {
        let _ = self.transaction(Cmd::Stop, Vec::new(), TIMEOUT_CONTROL)?;
        self.state = Some(State::Idle);
        Ok(())
    }

    /// `FORCE_TRIGGER`。仅 `ARMED` 有效。
    pub fn force_trigger(&mut self) -> Result<()> {
        let _ = self.transaction(Cmd::ForceTrigger, Vec::new(), TIMEOUT_CONTROL)?;
        Ok(())
    }

    /// `READ_BUFFER` —— 按绝对样点号拉取一片波形。
    ///
    /// 因为 `start_sample` 是绝对偏移，本操作**天然幂等**：
    /// 丢片时用同一个偏移重拉即可。
    pub fn read_buffer(
        &mut self,
        capture_id: u16,
        start_sample: u32,
        count: u16,
        format: u8,
        ch: u8,
    ) -> Result<Vec<u8>> {
        let mut p = Vec::with_capacity(10);
        p.extend_from_slice(&capture_id.to_le_bytes());
        p.extend_from_slice(&start_sample.to_le_bytes());
        p.extend_from_slice(&count.to_le_bytes());
        p.push(format);
        p.push(ch);

        // 动态超时：按分片大小估算传输时间
        let timeout = self
            .port
            .recv_timeout_for(count as usize * 2 + CHUNK_HEADER_LEN);
        let resp = self.transaction(Cmd::ReadBuffer, p, timeout)?;
        Ok(resp.payload)
    }

    /// 等待 `EVENT_TRIGGER`，返回其 payload。
    ///
    /// **超时返回 `Ok(None)` 而不是 `Err`** —— 「没等到触发」是一种正常结果，
    /// 不是链路故障。调用方（CLI / MCP）据此给出「检查触发电平」之类的建议，
    /// 而不是报一个让人以为是串口坏了的 IO 错误。
    pub fn wait_trigger(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }

            let frame = match self.reader.next_frame(&mut self.port, remaining) {
                Ok(f) => f,
                // 等超时 = 没触发，是正常结果
                Err(LinkError::Timeout(_)) => return Ok(None),
                Err(e) => return Err(ScopeError::Link(e)),
            };

            if frame.header.flags & scope_proto::flags::EVENT != 0
                && Cmd::try_from_u16(frame.header.cmd_raw) == Some(Cmd::EventTrigger)
            {
                self.state = Some(State::Done);
                return Ok(Some(frame.payload));
            }
            // 其他事件帧（OVERRUN / LOG）忽略，继续等
        }
    }

    fn guard_config_allowed(&self, action: &'static str) -> Result<()> {
        if let Some(state) = self.state {
            if matches!(state, State::Armed | State::Streaming) {
                return Err(ScopeError::BadState {
                    current: state,
                    action,
                    hint: "采集进行中只能配置触发前先 STOP".into(),
                });
            }
        }
        Ok(())
    }
}

/// 设备状态的中文名。
///
/// CLI / GUI / MCP 共用一份 —— 三处各写一份必然漂移。
pub fn state_name(s: State) -> &'static str {
    match s {
        State::Idle => "空闲",
        State::Armed => "已武装",
        State::Streaming => "流推送中",
        State::Done => "采集完成",
        State::Fault => "故障",
    }
}

/// 解析错误帧 payload：`{code:u16, severity:u8, msg_len:u8, msg[]}`。
fn decode_error(p: &[u8]) -> DeviceError {
    if p.len() < 4 {
        return DeviceError {
            code: ErrorCode::Internal,
            severity: Severity::Fatal,
            message: format!("错误帧过短（{} 字节）", p.len()),
        };
    }
    let code_raw = u16::from_le_bytes([p[0], p[1]]);
    let code = match code_raw {
        0x0001 => ErrorCode::UnknownCmd,
        0x0002 => ErrorCode::BadLen,
        0x0003 => ErrorCode::BadParam,
        0x0004 => ErrorCode::Busy,
        0x0005 => ErrorCode::BadState,
        0x0006 => ErrorCode::NoData,
        0x0007 => ErrorCode::Overrun,
        0x0008 => ErrorCode::Timeout,
        0x0009 => ErrorCode::VersionMismatch,
        0x000A => ErrorCode::Unsupported,
        0x000B => ErrorCode::FlashErr,
        _ => ErrorCode::Internal,
    };
    let severity = match p[2] {
        0 => Severity::Warn,
        2 => Severity::Fatal,
        _ => Severity::Err,
    };
    let msg_len = (p[3] as usize).min(p.len() - 4);
    let message = String::from_utf8_lossy(&p[4..4 + msg_len]).into_owned();

    DeviceError {
        code,
        severity,
        message,
    }
}

/// 供测试与模拟器使用：把 [`ParseStatus`] 转成人类可读文本。
pub fn parse_status_str(s: ParseStatus) -> &'static str {
    match s {
        ParseStatus::Ok => "ok",
        ParseStatus::NeedMore => "need-more",
        ParseStatus::CrcErr => "crc-error",
        ParseStatus::BadLen => "bad-len",
        ParseStatus::BadVer => "bad-version",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 造一个 33 字节的 `GET_STATUS` payload，每个字段给一个**唯一**的值。
    ///
    /// 唯一性是有意的：任何偏移错误都会让某个断言对不上，而用 0/1 之类的
    /// 值就抓不出来（第一版的偏移错了 2 字节，正是因为哨兵和溢出计数
    /// 恰好长得很像才差点蒙混过去）。
    fn status_payload() -> Vec<u8> {
        let mut p = Vec::with_capacity(33);
        p.push(State::Fault as u8);
        p.extend_from_slice(&0x1111u16.to_le_bytes());
        p.extend_from_slice(&0x2222u16.to_le_bytes());
        p.extend_from_slice(&0x3333u16.to_le_bytes());
        p.extend_from_slice(&0x4444_4444u32.to_le_bytes());
        p.extend_from_slice(&0x5555_5555u32.to_le_bytes());
        p.extend_from_slice(&0x6666u16.to_le_bytes());
        p.extend_from_slice(&0x7777u16.to_le_bytes());
        p.extend_from_slice(&0x8888_8888u32.to_le_bytes());
        p.extend_from_slice(&0x9999_9999u32.to_le_bytes());
        p.extend_from_slice(&0xAAAA_AAAAu32.to_le_bytes());
        p.extend_from_slice(&0xBBBBu16.to_le_bytes());
        assert_eq!(p.len(), 33);
        p
    }

    #[test]
    fn decode_reads_every_field_at_the_right_offset() {
        // 回归：第一版从 `overrun_samples` 起整体少了 2 字节。症状是
        // 一台**刚连上、什么都没做**的设备报告「溢出 65535 次」——
        // 那个 0xFFFF 其实是「没有触发过」哨兵 `0xFFFFFFFF` 的高半字。
        // 一个 Agent 看到非零溢出，会开始怀疑一份本来完全干净的数据。
        let st = DeviceStatus::decode(&status_payload()).expect("33 字节应当能解");
        assert_eq!(st.state, State::Fault);
        assert_eq!(st.err_flags, 0x1111);
        assert_eq!(st.ring_fill_samples, 0x2222);
        assert_eq!(st.last_capture_id, 0x3333);
        assert_eq!(st.last_trigger_index, Some(0x4444_4444));
        assert_eq!(st.overrun_samples, 0x5555_5555);
        assert_eq!(st.rx_crc_err, 0x6666);
        assert_eq!(st.rx_dropped, 0x7777);
        assert_eq!(st.tx_dropped, 0x8888_8888);
        assert_eq!(st.uptime_ms, 0x9999_9999);
        assert_eq!(st.tick_us, 0xAAAA_AAAA);
        assert_eq!(st.last_error_code, 0xBBBB);
        assert!(!st.link_is_clean(), "有溢出就不该算干净");
    }

    #[test]
    fn a_short_status_payload_is_rejected_not_guessed() {
        assert!(
            DeviceStatus::decode(&[0u8; 32]).is_none(),
            "少一个字节也不行"
        );
        assert!(DeviceStatus::decode(&[]).is_none());
    }

    #[test]
    fn the_no_trigger_sentinel_becomes_none_not_a_huge_index() {
        let mut p = status_payload();
        p[7..11].copy_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
        assert_eq!(
            DeviceStatus::decode(&p).unwrap().last_trigger_index,
            None,
            "0xFFFFFFFF 是「没有触发过」，不是下标 4294967295"
        );
    }

    #[test]
    fn link_is_clean_only_when_all_three_counters_are_zero() {
        let mut p = status_payload();
        // 先把三个计数清零 —— 它是干净的
        p[11..15].copy_from_slice(&0u32.to_le_bytes()); // overrun
        p[15..17].copy_from_slice(&0u16.to_le_bytes()); // rx_crc_err
        p[17..19].copy_from_slice(&0u16.to_le_bytes()); // rx_dropped
        assert!(DeviceStatus::decode(&p).unwrap().link_is_clean());

        // 任意一个非零就不干净 —— 逐个验，免得只挡了一个
        for (name, at) in [("overrun", 11usize), ("rx_crc_err", 15), ("rx_dropped", 17)] {
            let mut q = p.clone();
            q[at] = 1;
            assert!(
                !DeviceStatus::decode(&q).unwrap().link_is_clean(),
                "{name} 非零时不该算干净"
            );
        }
    }
}
