//! # scope-proto —— F103 Agent Scope 线协议（Rust 端）
//!
//! 本 crate 是 [`docs/03-protocol.md`] 的 Rust 实现，与 C 端
//! `proto/protocol.{h,c}` **逐字节等价**。
//!
//! 两端一致性由同一份黄金向量保证：
//! [`proto/tests/vectors.json`] 同时被 `cargo test -p scope-proto` 与
//! `make -C proto test` 执行。任何一端偏离契约，它的测试立刻红。
//!
//! ## 设计纪律
//!
//! - 小端、定长、不用自描述编码
//! - 控制链路永不出现 `f32`
//! - 未知 CMD 必须按 LEN 吞掉整帧再回 `UNKNOWN_CMD`
//! - 出错时以 +1 字节步进重同步（不是按帧长跳过）
//!
//! [`docs/03-protocol.md`]: ../../../../docs/03-protocol.md
//! [`proto/tests/vectors.json`]: ../../../../proto/tests/vectors.json

#![deny(clippy::all)]
#![warn(missing_docs)]

use serde::{Deserialize, Serialize};

// ══════════════════════════════════════════════════════════════════════
// 常量
// ══════════════════════════════════════════════════════════════════════

/// 同步头第一字节。
pub const SYNC0: u8 = 0xAA;
/// 同步头第二字节。
pub const SYNC1: u8 = 0x55;

/// 协议主版本。
pub const VER_MAJOR: u8 = 1;
/// 协议次版本。
pub const VER_MINOR: u8 = 0;
/// 线上协议版本字节（高 4 位主 / 低 4 位次）。
pub const VER: u8 = (VER_MAJOR << 4) | VER_MINOR;

// ── GET_INFO.caps 位掩码 ──────────────────────────────────────────
//
// 与 `proto/protocol.h` 的 `CAP_*` **逐位一致**。C 端与 Rust 端各自定义是
// 历史原因；新增/修改位时必须两处同步（CI 目前只校验协议版本号）。
/// 支持双通道同步采样。
pub const CAP_CH_DUAL: u32 = 1 << 0;
/// 支持自动触发。
pub const CAP_TRIG_AUTO: u32 = 1 << 1;
/// 支持普通触发。
pub const CAP_TRIG_NORMAL: u32 = 1 << 2;
/// 支持单次触发。
pub const CAP_TRIG_SINGLE: u32 = 1 << 3;
/// 支持软触发。
pub const CAP_TRIG_SOFT: u32 = 1 << 4;
/// 支持 AC 耦合。
pub const CAP_COUPLING_AC: u32 = 1 << 5;
/// 支持 PACK12 波形格式。
pub const CAP_FMT_PACK12: u32 = 1 << 6;
/// 支持 MINMAX 波形格式。
pub const CAP_FMT_MINMAX: u32 = 1 << 7;
/// 支持设备侧测量。
pub const CAP_MEAS_ON_DEVICE: u32 = 1 << 8;
/// 支持数字通路 I2C 解码。
pub const CAP_DIGITAL_I2C: u32 = 1 << 9;

/// 固定帧头长度：SYNC(2) + VER(1) + FLAGS(1) + SEQ(2) + CMD(2) + LEN(2)。
pub const HEADER_LEN: usize = 10;
/// CRC16 字段长度。
pub const CRC_LEN: usize = 2;
/// 最小帧长（`LEN = 0`）。
pub const FRAME_MIN: usize = HEADER_LEN + CRC_LEN;

/// 分片头长度（`READ_BUFFER` 响应前 12 字节）。
pub const CHUNK_HEADER_LEN: usize = 12;

/// 一个分片最多装多少样点（RAW16 格式）。
pub const CHUNK_SAMPLES_MAX: usize = 1024;

/// 设备 → 主机的 payload 上限。
///
/// = 分片头 12 B + 1024 样点 × u16 = **2060**。
/// 注意分片头**算在 payload 之内** —— 写成 2048 会让 1024 点的分片超限。
pub const MAX_PAYLOAD_TX: usize = CHUNK_HEADER_LEN + CHUNK_SAMPLES_MAX * 2;
/// 主机 → 设备的 payload 上限。
pub const MAX_PAYLOAD_RX: usize = 512;
/// 设备 → 主机的最大帧长。
pub const MAX_FRAME_TX: usize = HEADER_LEN + MAX_PAYLOAD_TX + CRC_LEN;
/// 主机 → 设备的最大帧长。
pub const MAX_FRAME_RX: usize = HEADER_LEN + MAX_PAYLOAD_RX + CRC_LEN;
/// 采集深度上限（8 KB 环 = 4096 点 × u16）。
pub const CAPTURE_MAX_SAMPLES: usize = 4096;
/// 默认分片大小。
pub const PREFERRED_CHUNK: usize = 1024;
/// 设备时钟频率：1 MHz（`u32` 微秒计数，约 71.6 分钟回绕）。
pub const TICK_HZ: u32 = 1_000_000;

// ══════════════════════════════════════════════════════════════════════
// FLAGS
// ══════════════════════════════════════════════════════════════════════

/// 帧标志位。
pub mod flags {
    /// 1 = 响应。
    pub const RESP: u8 = 1 << 0;
    /// 1 = 错误。
    pub const ERROR: u8 = 1 << 1;
    /// 分片未结束。
    pub const MORE: u8 = 1 << 2;
    /// 分片结束。
    pub const LAST: u8 = 1 << 3;
    /// 设备主动上报。
    pub const EVENT: u8 = 1 << 4;
    /// 免回复。
    pub const NO_REPLY: u8 = 1 << 5;
}

// ══════════════════════════════════════════════════════════════════════
// 命令码
// ══════════════════════════════════════════════════════════════════════

/// 命令码。编号一经分配永不复用。
///
/// 高字节 = 类（`0x01` 系统 / `0x02` 配置 / `0x03` 采集 / `0x04` 读取 /
/// `0x05` 测量 / `0x06` 调试与事件 / `0x07` 错误），低字节 = 类内编号。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u16)]
#[serde(into = "u16", from = "u16")]
pub enum Cmd {
    // ── 系统类 0x01xx ──
    /// 获取设备信息与能力。连接后的第一条命令。
    GetInfo = 0x0101,
    /// 链路保活 / RTT 估计。
    Ping = 0x0102,
    /// 获取当前状态。任何状态可调。
    GetStatus = 0x0103,
    /// 软复位（需 magic）。
    Reset = 0x0104,

    // ── 配置类 0x02xx ──
    /// 读回全部可设置项。
    GetConfig = 0x0200,
    /// 设置采样率（设备回显量化后的实际值）。
    SetSampleRate = 0x0201,
    /// 设置通道。
    SetChannel = 0x0202,
    /// 设置触发。
    SetTrigger = 0x0203,
    /// 设置采集参数。
    SetAcq = 0x0204,

    // ── 采集类 0x03xx ──
    /// 武装采集。非幂等。
    Arm = 0x0301,
    /// 停止。幂等。
    Stop = 0x0302,
    /// 软件触发。仅 ARMED 有效。
    ForceTrigger = 0x0303,

    // ── 读取类 0x04xx ──
    /// 按绝对样点号读取波形分片。
    ReadBuffer = 0x0401,

    // ── 测量类 0x05xx ──
    /// 设备侧定点测量。
    Measure = 0x0501,

    // ── 调试与事件 0x06xx ──
    /// 链路吞吐与无差错验证。
    Echo = 0x0601,
    /// 设置日志级别。
    SetLogLevel = 0x0602,
    /// 读取最后一次错误的详情。
    GetLastError = 0x0603,
    /// 读内存。仅 DEBUG 构建，绝不注册给 MCP。
    MemRead = 0x0610,
    /// 写内存。仅 DEBUG 构建，绝不注册给 MCP。
    MemWrite = 0x0611,
    /// 设备 → 主机：触发完成事件。
    EventTrigger = 0x0682,
    /// 设备 → 主机：采集溢出。
    EventOverrun = 0x0683,
    /// 设备 → 主机：固件日志。
    EventLog = 0x0684,

    // ── 错误 0x07xx ──
    /// 带 ERROR flag 的错误响应。
    Error = 0x0701,
}

impl Cmd {
    /// 命令分类（高字节）。
    pub fn class(self) -> u8 {
        ((self as u16) >> 8) as u8
    }
}

impl From<Cmd> for u16 {
    fn from(c: Cmd) -> u16 {
        c as u16
    }
}

impl From<u16> for Cmd {
    fn from(v: u16) -> Cmd {
        use Cmd::*;
        match v {
            0x0101 => GetInfo,
            0x0102 => Ping,
            0x0103 => GetStatus,
            0x0104 => Reset,
            0x0200 => GetConfig,
            0x0201 => SetSampleRate,
            0x0202 => SetChannel,
            0x0203 => SetTrigger,
            0x0204 => SetAcq,
            0x0301 => Arm,
            0x0302 => Stop,
            0x0303 => ForceTrigger,
            0x0401 => ReadBuffer,
            0x0501 => Measure,
            0x0601 => Echo,
            0x0602 => SetLogLevel,
            0x0603 => GetLastError,
            0x0610 => MemRead,
            0x0611 => MemWrite,
            0x0682 => EventTrigger,
            0x0683 => EventOverrun,
            0x0684 => EventLog,
            0x0701 => Error,
            // 未知命令码：映射到 Error 占位。调用方必须先用
            // `Cmd::try_from_u16` 判断，才能真正区分"未知"。
            _ => Error,
        }
    }
}

impl Cmd {
    /// 严格解析命令码；未知码返回 `None`。
    ///
    /// 固件收到未知 CMD 时必须**按 LEN 吞掉整帧**再回 `UNKNOWN_CMD`。
    pub fn try_from_u16(v: u16) -> Option<Cmd> {
        let c = Cmd::from(v);
        if (c as u16) == v {
            Some(c)
        } else {
            None
        }
    }
}

// ══════════════════════════════════════════════════════════════════════
// 错误码
// ══════════════════════════════════════════════════════════════════════

/// 协议层错误码。编号一经发布永不复用。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u16)]
pub enum ErrorCode {
    /// 未定义的 CMD。
    UnknownCmd = 0x0001,
    /// LEN 越界或不匹配。
    BadLen = 0x0002,
    /// 参数越界。
    BadParam = 0x0003,
    /// 当前状态不允许（先 STOP）。
    Busy = 0x0004,
    /// 状态机不允许。
    BadState = 0x0005,
    /// capture_id 不符 / 无数据。
    NoData = 0x0006,
    /// 数据溢出。
    Overrun = 0x0007,
    /// 未触发 / 设备侧超时。
    Timeout = 0x0008,
    /// 主版本不符。
    VersionMismatch = 0x0009,
    /// 该功能此硬件不支持。
    Unsupported = 0x000A,
    /// Flash 操作失败。
    FlashErr = 0x000B,
    /// 内部错误。
    Internal = 0x000C,
}

/// 错误严重级别。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum Severity {
    /// 警告。
    Warn = 0,
    /// 错误。
    Err = 1,
    /// 致命。
    Fatal = 2,
}

// ══════════════════════════════════════════════════════════════════════
// 状态机
// ══════════════════════════════════════════════════════════════════════

/// 设备状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum State {
    /// 空闲。
    Idle = 0,
    /// 已武装，等待触发。
    Armed = 1,
    /// 连续流推送中。
    Streaming = 2,
    /// 采集完成，数据已冻结。
    Done = 3,
    /// 故障。
    Fault = 4,
}

impl Default for State {
    /// 默认为 [`State::Idle`] —— 设备上电后的状态。
    ///
    /// 有默认值是为了让 [`DeviceStatus`](scope_core) 之类「先 Default 再填」的
    /// 结构能用 `#[derive(Default)]`；**不要**拿它当「读不到就当作空闲」的
    /// 兜底 —— 那会把「设备回了个我们不认识的状态」伪装成「设备空闲」。
    fn default() -> State {
        State::Idle
    }
}

impl State {
    /// 从线上字节解析。
    pub fn from_u8(v: u8) -> Option<State> {
        match v {
            0 => Some(State::Idle),
            1 => Some(State::Armed),
            2 => Some(State::Streaming),
            3 => Some(State::Done),
            4 => Some(State::Fault),
            _ => None,
        }
    }
}

/// 该状态下是否允许执行该命令。
///
/// 对应 `proto/protocol.c` 的 `proto_cmd_allowed()`。
pub fn cmd_allowed(state: State, cmd: Cmd) -> bool {
    use Cmd::*;
    use State::*;

    // 基础命令在任何状态都允许
    if matches!(
        cmd,
        GetInfo | Ping | GetStatus | Stop | GetLastError | EventTrigger | EventOverrun | EventLog
    ) {
        return true;
    }

    match state {
        // IDLE / DONE 下**全部**允许 —— 包括 ARM。
        // 在 DONE 下再次 ARM 会丢弃当前采集的数据，放行的理由见
        // `proto/protocol.c` 里同一处的说明（与 C 侧逐字对应）。
        Idle | Done => true,
        // 采集进行中只允许强制触发，其余配置命令要回 BUSY
        Armed | Streaming => matches!(cmd, ForceTrigger),
        // 故障态只允许复位
        Fault => matches!(cmd, Reset),
    }
}

// ══════════════════════════════════════════════════════════════════════
// CRC-16/CCITT-FALSE
// ══════════════════════════════════════════════════════════════════════

/// CRC-16/CCITT-FALSE。
///
/// `poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000`。
/// 标准校验值：`"123456789"` → `0x29B1`。
///
/// 覆盖范围是 `VER..PAYLOAD` 末尾，**不含 SYNC**。
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ══════════════════════════════════════════════════════════════════════
// 帧
// ══════════════════════════════════════════════════════════════════════

/// 解析后的帧头。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// 协议版本字节。
    pub ver: u8,
    /// 标志位。
    pub flags: u8,
    /// 序号。
    pub seq: u16,
    /// 命令码（原始 u16，可能是未知命令）。
    pub cmd_raw: u16,
    /// payload 字节数。
    pub len: u16,
}

impl Header {
    /// 解析出的命令码；未知返回 `None`。
    pub fn cmd(&self) -> Option<Cmd> {
        Cmd::try_from_u16(self.cmd_raw)
    }

    /// 主版本是否与本端一致。
    pub fn ver_matches(&self) -> bool {
        (self.ver >> 4) == VER_MAJOR
    }

    /// 是否响应帧。
    pub fn is_response(&self) -> bool {
        self.flags & flags::RESP != 0
    }

    /// 是否错误帧。
    pub fn is_error(&self) -> bool {
        self.flags & flags::ERROR != 0
    }
}

/// 一帧：帧头 + payload。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// 帧头。
    pub header: Header,
    /// payload（长度 = `header.len`）。
    pub payload: Vec<u8>,
}

impl Frame {
    /// 构造一个请求帧（无 RESP / ERROR 标志）。
    pub fn request(seq: u16, cmd: Cmd, payload: Vec<u8>) -> Frame {
        Frame {
            header: Header {
                ver: VER,
                flags: 0,
                seq,
                cmd_raw: cmd as u16,
                len: payload.len() as u16,
            },
            payload,
        }
    }

    /// 编码为线上字节。
    ///
    /// # Panics
    /// payload 超过 [`MAX_PAYLOAD_TX`] 时 panic —— 这是编程错误，
    /// 上层必须在构造 payload 前就校验。
    pub fn encode(&self) -> Vec<u8> {
        self.try_encode()
            .expect("payload 超过 MAX_PAYLOAD_TX，应在上层校验")
    }

    /// 编码为线上字节；失败返回 `None`。
    pub fn try_encode(&self) -> Option<Vec<u8>> {
        if self.payload.len() > MAX_PAYLOAD_TX {
            return None;
        }
        let mut out = Vec::with_capacity(HEADER_LEN + self.payload.len() + CRC_LEN);
        out.push(SYNC0);
        out.push(SYNC1);
        out.push(self.header.ver);
        out.push(self.header.flags);
        out.extend_from_slice(&self.header.seq.to_le_bytes());
        out.extend_from_slice(&self.header.cmd_raw.to_le_bytes());
        out.extend_from_slice(&(self.payload.len() as u16).to_le_bytes());
        out.extend_from_slice(&self.payload);
        let crc = crc16(&out[2..]);
        out.extend_from_slice(&crc.to_le_bytes());
        Some(out)
    }
}

// ══════════════════════════════════════════════════════════════════════
// 增量解析器
// ══════════════════════════════════════════════════════════════════════

/// 解析结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseStatus {
    /// 解析出一个完整帧。
    Ok,
    /// 数据不足，等更多字节。
    NeedMore,
    /// CRC 校验失败（已跳过一个字节重同步）。
    CrcErr,
    /// LEN 越界（已跳过）。
    BadLen,
    /// 主版本不符（帧本身是完整的）。
    BadVer,
}

/// 增量字节流解析器。
///
/// 天然处理粘包/拆包：一次喂入可以是半个帧、一个帧或多个半帧。
/// 出错时以 **+1 字节步进**扫描 `AA 55` 重同步，而不是按帧长跳过 ——
/// 因为损坏的 LEN 字段本身就不可信。
#[derive(Debug, Clone)]
pub struct Parser {
    buf: Vec<u8>,
    buf_cap: usize,
    /// 上一次成功返回的帧长度：惰性消费，让调用者拿到数据后才移除。
    pending: usize,
    /// CRC 错误计数（对应设备侧 `rx_crc_err`）。
    pub crc_err_count: u32,
    /// 丢弃字节数（对应设备侧 `rx_dropped`）。
    pub dropped_count: u32,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    /// 创建主机侧解析器（缓冲上限 = [`MAX_FRAME_TX`]）。
    pub fn new() -> Self {
        Self::with_capacity(MAX_FRAME_TX)
    }

    /// 创建指定缓冲上限的解析器。固件侧应传 [`MAX_FRAME_RX`]。
    pub fn with_capacity(cap: usize) -> Self {
        Parser {
            buf: Vec::with_capacity(cap),
            buf_cap: cap,
            pending: 0,
            crc_err_count: 0,
            dropped_count: 0,
        }
    }

    /// 喂入字节并尝试解析。
    ///
    /// 一次调用最多返回一个帧。若一次喂入多帧，返回 `Ok` 后继续调用
    /// `feed(&[], &mut frame)` 排空缓冲。
    pub fn feed(&mut self, data: &[u8], frame: &mut Frame) -> ParseStatus {
        // 惰性消费上一次返回的帧
        if self.pending > 0 {
            self.consume(self.pending);
            self.pending = 0;
        }

        // 追加新数据；缓冲满则丢弃最旧字节（说明流已失步）
        let mut incoming = data;
        while incoming.len() > self.buf_cap.saturating_sub(self.buf.len()) {
            if self.buf.is_empty() {
                // 单次喂入超过整个缓冲：只保留尾部
                let skip = incoming.len() - self.buf_cap;
                incoming = &incoming[skip..];
                self.dropped_count += skip as u32;
                break;
            }
            self.consume(1);
            self.dropped_count += 1;
        }
        self.buf.extend_from_slice(incoming);

        loop {
            match self.try_parse(frame) {
                ParseStatus::Ok | ParseStatus::BadVer => {
                    self.pending = frame.header.len as usize + HEADER_LEN + CRC_LEN;
                    return match frame.header.ver_matches() {
                        true => ParseStatus::Ok,
                        false => ParseStatus::BadVer,
                    };
                }
                ParseStatus::NeedMore => return ParseStatus::NeedMore,
                // 已跳过一个字节，继续扫描
                ParseStatus::CrcErr | ParseStatus::BadLen => continue,
            }
        }
    }

    fn consume(&mut self, n: usize) {
        if n >= self.buf.len() {
            self.buf.clear();
        } else {
            self.buf.drain(..n);
        }
    }

    fn try_parse(&mut self, frame: &mut Frame) -> ParseStatus {
        // 找同步头：+1 字节步进
        while self.buf.len() >= 2 && !(self.buf[0] == SYNC0 && self.buf[1] == SYNC1) {
            self.consume(1);
            self.dropped_count += 1;
        }

        if self.buf.len() < HEADER_LEN {
            return ParseStatus::NeedMore;
        }

        let payload_len = u16::from_le_bytes([self.buf[8], self.buf[9]]) as usize;

        // 先验长度再读 payload —— 防越界
        if payload_len > MAX_PAYLOAD_TX {
            self.consume(1);
            self.dropped_count += 1;
            return ParseStatus::BadLen;
        }

        let need = HEADER_LEN + payload_len + CRC_LEN;
        if self.buf.len() < need {
            return ParseStatus::NeedMore;
        }

        let crc_calc = crc16(&self.buf[2..HEADER_LEN + payload_len]);
        let crc_wire = u16::from_le_bytes([
            self.buf[HEADER_LEN + payload_len],
            self.buf[HEADER_LEN + payload_len + 1],
        ]);

        if crc_calc != crc_wire {
            self.consume(1);
            self.crc_err_count += 1;
            return ParseStatus::CrcErr;
        }

        frame.header = Header {
            ver: self.buf[2],
            flags: self.buf[3],
            seq: u16::from_le_bytes([self.buf[4], self.buf[5]]),
            cmd_raw: u16::from_le_bytes([self.buf[6], self.buf[7]]),
            len: payload_len as u16,
        };
        frame.payload.clear();
        frame
            .payload
            .extend_from_slice(&self.buf[HEADER_LEN..HEADER_LEN + payload_len]);

        ParseStatus::Ok
    }
}

// ══════════════════════════════════════════════════════════════════════
// PACK12
// ══════════════════════════════════════════════════════════════════════

/// 两个 12-bit 样点 → 3 字节（省 25%）。
///
/// ```text
/// b0 = s0 低 8 位
/// b1 = (s0 >> 8) & 0x0F | (s1 & 0x0F) << 4
/// b2 = s1 >> 4
/// ```
pub fn pack12_pair(s0: u16, s1: u16) -> [u8; 3] {
    [
        (s0 & 0xFF) as u8,
        (((s0 >> 8) & 0x0F) | ((s1 & 0x0F) << 4)) as u8,
        ((s1 >> 4) & 0xFF) as u8,
    ]
}

/// PACK12 解码。
pub fn unpack12_pair(b: [u8; 3]) -> (u16, u16) {
    let s0 = ((b[0] as u16) | (((b[1] & 0x0F) as u16) << 8)) & 0x0FFF;
    let s1 = (((b[1] >> 4) as u16) | ((b[2] as u16) << 4)) & 0x0FFF;
    (s0, s1)
}

// ══════════════════════════════════════════════════════════════════════
// 分片头
// ══════════════════════════════════════════════════════════════════════

/// `READ_BUFFER` 响应 payload 的前 12 字节。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkHeader {
    /// 采集编号。
    pub capture_id: u16,
    /// 该片的绝对起始样点号。
    pub start_sample: u32,
    /// 本片样点数。
    pub count: u16,
    /// 抽点倍数。
    pub decimation: u16,
    /// 数据格式。
    pub format: u8,
    /// `CHUNK_FLAG_*`。
    pub flags: u8,
}

/// 分片头 flag：本采集最后一片。
pub const CHUNK_FLAG_LAST: u8 = 1 << 0;
/// 分片头 flag：数据无效 / 期间发生 overrun。
pub const CHUNK_FLAG_INVALID: u8 = 1 << 1;

impl ChunkHeader {
    /// 编码为 12 字节。
    pub fn encode(&self) -> [u8; CHUNK_HEADER_LEN] {
        let mut out = [0u8; CHUNK_HEADER_LEN];
        out[0..2].copy_from_slice(&self.capture_id.to_le_bytes());
        out[2..6].copy_from_slice(&self.start_sample.to_le_bytes());
        out[6..8].copy_from_slice(&self.count.to_le_bytes());
        out[8..10].copy_from_slice(&self.decimation.to_le_bytes());
        out[10] = self.format;
        out[11] = self.flags;
        out
    }

    /// 从 12 字节解析。
    pub fn decode(b: &[u8]) -> Option<ChunkHeader> {
        if b.len() < CHUNK_HEADER_LEN {
            return None;
        }
        Some(ChunkHeader {
            capture_id: u16::from_le_bytes([b[0], b[1]]),
            start_sample: u32::from_le_bytes([b[2], b[3], b[4], b[5]]),
            count: u16::from_le_bytes([b[6], b[7]]),
            decimation: u16::from_le_bytes([b[8], b[9]]),
            format: b[10],
            flags: b[11],
        })
    }

    /// 是否最后一片。
    pub fn is_last(&self) -> bool {
        self.flags & CHUNK_FLAG_LAST != 0
    }

    /// 数据是否有效。
    pub fn is_valid(&self) -> bool {
        self.flags & CHUNK_FLAG_INVALID == 0
    }
}

// ══════════════════════════════════════════════════════════════════════
// 测试：跑与 C 端同一份黄金向量
// ══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn vectors() -> Value {
        // host/crates/proto → ../../../proto/tests/vectors.json
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../proto/tests/vectors.json"
        );
        let raw = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("读不到黄金向量 {path}: {e}\n先跑 `make -C proto vectors`"));
        serde_json::from_str(&raw).expect("vectors.json 格式非法")
    }

    fn hex2bin(s: &str) -> Vec<u8> {
        assert!(s.len() % 2 == 0, "hex 长度必须是偶数: {s}");
        (0..s.len() / 2)
            .map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).expect("非法 hex"))
            .collect()
    }

    fn bin2hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{x:02x}")).collect()
    }

    #[test]
    fn crc_standard_check_value() {
        let v = vectors();
        let input = v["crc_check"]["input_ascii"].as_str().unwrap();
        let expected = v["crc_check"]["expected"].as_u64().unwrap() as u16;
        assert_eq!(crc16(input.as_bytes()), expected);
        assert_eq!(expected, 0x29B1);
        assert_eq!(crc16(&[]), 0xFFFF, "空输入应返回 init 值");
    }

    #[test]
    fn encode_matches_golden_frames() {
        let v = vectors();
        let frames = v["frames"].as_array().unwrap();
        assert!(!frames.is_empty());

        for f in frames {
            let name = f["name"].as_str().unwrap();
            let flags = f["flags"].as_u64().unwrap() as u8;
            let seq = f["seq"].as_u64().unwrap() as u16;
            let cmd = f["cmd"].as_u64().unwrap() as u16;
            let payload = hex2bin(f["payload_hex"].as_str().unwrap());
            let want = f["frame_hex"].as_str().unwrap();

            let frame = Frame {
                header: Header {
                    ver: VER,
                    flags,
                    seq,
                    cmd_raw: cmd,
                    len: payload.len() as u16,
                },
                payload,
            };
            let got = bin2hex(&frame.encode());
            assert_eq!(got, want, "帧编码不符: {name}");
        }
    }

    #[test]
    fn parser_handles_golden_streams() {
        let v = vectors();
        let cases = v["parser"].as_array().unwrap();
        assert!(!cases.is_empty());

        for c in cases {
            let name = c["name"].as_str().unwrap();
            let stream = hex2bin(c["stream_hex"].as_str().unwrap());
            let expect_ok = c["expect_ok"].as_array().unwrap();

            let mut parser = Parser::new();
            let mut got = Vec::new();
            let split_at = c["split_at"].as_u64().unwrap_or(0) as usize;

            let total = stream.len();
            let mut fed = 0usize;
            let mut frame = Frame {
                header: Header {
                    ver: 0,
                    flags: 0,
                    seq: 0,
                    cmd_raw: 0,
                    len: 0,
                },
                payload: Vec::new(),
            };

            loop {
                let status = if fed < total {
                    let n = if split_at > 0 {
                        split_at.min(total - fed)
                    } else {
                        total - fed
                    };
                    let s = parser.feed(&stream[fed..fed + n], &mut frame);
                    fed += n;
                    s
                } else {
                    parser.feed(&[], &mut frame)
                };

                match status {
                    ParseStatus::Ok | ParseStatus::BadVer => got.push(frame.clone()),
                    ParseStatus::NeedMore if fed >= total => break,
                    ParseStatus::NeedMore | ParseStatus::CrcErr | ParseStatus::BadLen => {}
                }
            }

            assert_eq!(got.len(), expect_ok.len(), "帧数不符: {name}");

            for (i, want) in expect_ok.iter().enumerate() {
                let w_cmd = want["cmd"].as_u64().unwrap() as u16;
                let w_seq = want["seq"].as_u64().unwrap() as u16;
                let w_flags = want["flags"].as_u64().unwrap() as u8;
                let w_payload = hex2bin(want["payload_hex"].as_str().unwrap());

                assert_eq!(got[i].header.cmd_raw, w_cmd, "{name}: 第 {i} 帧 cmd");
                assert_eq!(got[i].header.seq, w_seq, "{name}: 第 {i} 帧 seq");
                assert_eq!(got[i].header.flags, w_flags, "{name}: 第 {i} 帧 flags");
                assert_eq!(got[i].payload, w_payload, "{name}: 第 {i} 帧 payload");
            }

            let want_crc = c["expect_crc_err"].as_u64().unwrap() as u32;
            assert_eq!(parser.crc_err_count, want_crc, "{name}: crc_err 计数");

            let want_drop = c["expect_dropped"].as_u64().unwrap() as u32;
            if want_drop > 0 {
                assert_eq!(parser.dropped_count, want_drop, "{name}: dropped 计数");
            }
        }
    }

    #[test]
    fn pack12_matches_golden_and_roundtrips() {
        let v = vectors();
        for p in v["pack12"].as_array().unwrap() {
            let s0 = p["s0"].as_u64().unwrap() as u16;
            let s1 = p["s1"].as_u64().unwrap() as u16;
            let want = p["bytes_hex"].as_str().unwrap();

            let bytes = pack12_pair(s0, s1);
            assert_eq!(bin2hex(&bytes), want, "pack12({s0:03X},{s1:03X})");
            assert_eq!(unpack12_pair(bytes), (s0, s1), "pack12 往返失败");
        }
    }

    #[test]
    fn state_matrix_matches_golden() {
        let v = vectors();
        let matrix = v["state_matrix"].as_array().unwrap();
        for row in matrix {
            let state_num = row["state"].as_u64().unwrap() as u8;
            let state = State::from_u8(state_num).expect("非法 state");
            let allowed = row["allowed"].as_object().unwrap();

            for (cmd_str, want) in allowed {
                let cmd_raw: u16 = cmd_str.parse().unwrap();
                let cmd = Cmd::try_from_u16(cmd_raw)
                    .unwrap_or_else(|| panic!("向量的命令码 0x{cmd_raw:04X} 在 Cmd 枚举里不存在"));
                let got = cmd_allowed(state, cmd);
                assert_eq!(
                    got,
                    want.as_bool().unwrap(),
                    "state={state_num} cmd=0x{cmd_raw:04X}"
                );
            }
        }
    }

    #[test]
    fn unknown_cmd_is_detected() {
        assert!(
            Cmd::try_from_u16(0x9999).is_none(),
            "未知命令码必须返回 None"
        );
        assert!(Cmd::try_from_u16(0x0101).is_some());
    }

    #[test]
    fn chunk_header_roundtrip() {
        let ch = ChunkHeader {
            capture_id: 7,
            start_sample: 0xDEAD_BEEF,
            count: 1024,
            decimation: 16,
            format: 2,
            flags: CHUNK_FLAG_LAST,
        };
        let bytes = ch.encode();
        assert_eq!(bytes.len(), CHUNK_HEADER_LEN);
        assert_eq!(ChunkHeader::decode(&bytes), Some(ch));
        assert!(ch.is_last());
        assert!(ch.is_valid());
    }

    #[test]
    fn resync_uses_one_byte_step_not_frame_length() {
        // 构造：坏 LEN 的帧头之后紧跟一个合法帧。
        // 如果解析器按帧长跳过，就会错过后面那个合法帧。
        let good = Frame::request(2, Cmd::Ping, vec![1, 2, 3]).encode();

        let mut evil = vec![SYNC0, SYNC1, VER, 0];
        evil.extend_from_slice(&99u16.to_le_bytes()); // seq
        evil.extend_from_slice(&(Cmd::Echo as u16).to_le_bytes());
        evil.extend_from_slice(&4096u16.to_le_bytes()); // LEN 越界
        evil.extend_from_slice(&[0u8; 20]);

        let mut stream = evil.clone();
        stream.extend_from_slice(&good);

        let mut parser = Parser::new();
        let mut frame = Frame {
            header: Header {
                ver: 0,
                flags: 0,
                seq: 0,
                cmd_raw: 0,
                len: 0,
            },
            payload: Vec::new(),
        };

        assert_eq!(parser.feed(&stream, &mut frame), ParseStatus::Ok);
        assert_eq!(frame.header.cmd_raw, Cmd::Ping as u16);
        assert_eq!(frame.payload, vec![1, 2, 3]);
    }
}
