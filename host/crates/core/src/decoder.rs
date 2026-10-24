//! 协议解码器插件层 —— 唯一的解码入口。
//!
//! # 为什么是枚举分发而不是 trait 对象
//!
//! 核心 crate 不引 `dyn`：可 `Clone`、可 serde、可穷举匹配。加一种协议
//! 是加一个枚举变体 + 一个匹配分支，编译器保证所有调用点都处理了它。
//!
//! # I2C 只做适配，不重写
//!
//! [`Protocol::I2c`] 分支转调 [`crate::i2c_decode::decode_capture`]（既有
//! 测试含 `nack` 场景全部继续有效），本层只做类型转换。插件化的意义在
//! 出口统一：MCP / CLI / 证据包将来都消费 [`DecodedFrames`]，加协议不再
//! 改核心类型。
//!
//! # UART 的同步锁定态
//!
//! 采集窗可能正好落在起始位上（`uart_115k` 场景的 sample 0 就是低电平），
//! 此时没有 1→0 下降沿可找 —— 所以 sample 0 为低时允许它作为首个起始位
//! 候选。解完一帧后在「期望位置窗口」内找下一帧，失锁回全域搜索。
//! 没有锁定态会在采集窗起点解出 `0xD5` 一类假帧（探针实测）。

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::capture::Capture;
use crate::error::ScopeError;
use crate::i2c_decode::{self, I2cWarning, Transaction};

/// 解码协议。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Protocol {
    /// I²C 总线（需 ≥2 通道：SCL + SDA）。
    I2c,
    /// UART 串行异步（需 1 通道；8N1）。
    Uart,
}

impl Protocol {
    /// 需要几个通道。
    pub fn channels_needed(self) -> usize {
        match self {
            Protocol::I2c => 2,
            Protocol::Uart => 1,
        }
    }
}

/// 校验位。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Parity {
    /// 无校验位（8N1）。
    #[default]
    None,
    /// 偶校验 —— 当前未实现，传入会得到明确报错而不是静默错解。
    Even,
    /// 奇校验 —— 同上。
    Odd,
}

/// UART 解码参数。
///
/// 通道**不在**这里：[`DecodeRequest::channels`] 是唯一来源（空 = 自动取
/// 通道 0）。同一件事写两处，总有一天会打架。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UartConfig {
    /// 波特率。**0 不是「自动」**——自动估计是建议性功能，未实现；
    /// 填 0 会得到明确报错，不猜（猜错波特率会解出「看起来合理但全错」
    /// 的字节，属于红线）。
    pub baud: u32,
    /// 数据位数。当前仅支持 8。
    pub data_bits: u8,
    /// 校验位。当前仅支持 [`Parity::None`]。
    pub parity: Parity,
    /// 停止位数。当前仅支持 1。
    pub stop_bits: u8,
    /// 起始位是否为低电平（标准 UART 即如此）。
    pub start_bit_low: bool,
    /// 是否 MSB 先发。标准 UART 是 LSB 先（`false`）。
    pub msb_first: bool,
}

impl Default for UartConfig {
    fn default() -> Self {
        UartConfig {
            baud: 115_200,
            data_bits: 8,
            parity: Parity::None,
            stop_bits: 1,
            start_bit_low: true,
            msb_first: false,
        }
    }
}

/// 一次解码请求。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecodeRequest {
    /// 解码协议。
    pub protocol: Protocol,
    /// 显式通道。空 = 自动（UART 取通道 0；I2C 忽略本字段，
    /// 以 [`i2c_decode::I2cDecodeConfig`] 的 `scl/sda` 为准 —— 适配层不引入行为变化）。
    pub channels: Vec<usize>,
    /// 协议参数。
    pub params: DecodeParams,
}

/// 协议参数。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DecodeParams {
    /// I²C 参数（现有类型，不改）。
    I2c(i2c_decode::I2cDecodeConfig),
    /// UART 参数。
    Uart(UartConfig),
}

/// 一个 UART 字节帧。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UartFrame {
    /// 帧序号（1 起）。
    pub index: usize,
    /// 起始位所在样点号。
    pub start_sample: usize,
    /// 采样解出的字节。
    pub byte: u8,
    /// 停止位校验是否通过。`false` 的帧**保留**（丢弃会让错误率不可见），
    /// 字节值照给但不可信。
    pub ok: bool,
}

/// 解码出的一个帧（协议相关载荷）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Frame {
    /// 一次 START..STOP 传输。
    I2c(Transaction),
    /// 一个 UART 字节。
    Uart(UartFrame),
}

/// 解码告警。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DecodeWarning {
    /// I²C 解码器的告警（原文转发）。
    I2c(I2cWarning),
    /// 有 `ok = false` 的帧（UART 停止位校验失败）。
    FrameError {
        /// 坏帧数。
        count: usize,
    },
    /// 采集窗口在帧中间结束。
    Truncated,
}

impl DecodeWarning {
    /// 人读的一句话。I²C 告警转发其原文 —— 过滤会让「共 N 帧」被误读。
    pub fn text(&self) -> String {
        match self {
            DecodeWarning::I2c(w) => w.text(),
            DecodeWarning::FrameError { count } => format!(
                "{count} 个帧的停止位校验失败（波特率不匹配或线路噪声）；\
                 这些帧的字节值不可信"
            ),
            DecodeWarning::Truncated => "采集窗口在帧中间结束，最后一帧不完整".into(),
        }
    }
}

/// 解码质量。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DecodeQuality {
    /// **每比特**样点数 `rate_hz / baud`（UART）。低于 4 时解码被拒绝；
    /// I²C 无此概念，为 `None`。
    pub sample_margin: Option<f64>,
    /// 校验通过（`ok = true`）的帧数。I²C 为完整（正常 STOP 收尾）帧数。
    pub frame_ok: usize,
    /// 校验失败或被截断的帧数。I²C 为不完整帧数。
    pub frame_errors: usize,
}

/// 解码结果 —— 所有出口（MCP / CLI / 证据包）消费的统一类型。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DecodedFrames {
    /// 解码协议。
    pub protocol: Protocol,
    /// 采样率（Hz）—— 时间量的换算基准。
    pub rate_hz: u32,
    /// 帧序列（解码顺序）。
    pub frames: Vec<Frame>,
    /// 告警（未过滤原文）。
    pub warnings: Vec<DecodeWarning>,
    /// 质量统计。
    pub quality: DecodeQuality,
}

impl DecodedFrames {
    /// 帧数（「共 N 帧」里的 N）。
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    /// UART 的字节序列（含 `ok = false` 的帧，按帧序；I²C 返回 `None`）。
    pub fn uart_bytes(&self) -> Option<Vec<u8>> {
        if self.protocol != Protocol::Uart {
            return None;
        }
        Some(
            self.frames
                .iter()
                .map(|f| match f {
                    Frame::Uart(u) => u.byte,
                    Frame::I2c(_) => unreachable!("UART 结果里不会混进 I²C 帧"),
                })
                .collect(),
        )
    }

    /// UART 解码结果的人读文本（CLI / 证据包共用）。
    ///
    /// I²C 结果返回空串 —— I²C 的文本仍由 [`i2c_decode::I2cDecode::to_text`]
    /// 负责，两种协议不共用一套措辞。
    pub fn uart_text(&self) -> String {
        use std::fmt::Write as _;
        if self.protocol != Protocol::Uart {
            return String::new();
        }
        let mut s = String::new();
        let _ = writeln!(s, "UART 8N1 解码结果");
        let _ = writeln!(
            s,
            "共 {} 帧（{} 帧通过停止位校验）",
            self.frame_count(),
            self.quality.frame_ok
        );
        if let Some(m) = self.quality.sample_margin {
            let _ = writeln!(s, "每比特样点数: {m:.2}（低于 4 不出帧）");
        }
        let _ = writeln!(s);
        if self.frames.is_empty() {
            let _ = writeln!(s, "（没有解出任何帧）");
        }
        for f in &self.frames {
            if let Frame::Uart(u) = f {
                let _ = writeln!(
                    s,
                    "#{:<3} @ {:>6} 样点  0x{:02X}  {}",
                    u.index,
                    u.start_sample,
                    u.byte,
                    if u.ok {
                        "ok"
                    } else {
                        "✗ 停止位校验失败（字节不可信）"
                    }
                );
            }
        }
        if !self.warnings.is_empty() {
            let _ = writeln!(s);
            let _ = writeln!(s, "告警（原文，未过滤）:");
            for w in &self.warnings {
                let _ = writeln!(s, "  - {}", w.text());
            }
        }
        s
    }
}

/// 解码失败。
#[derive(Debug, Error)]
pub enum DecodeError {
    /// 每比特样点数低于 4，解码结果不可能可靠 —— 拒绝出帧。
    #[error("欠采样：每比特样点数 {margin:.2} < 4，无法可靠解码")]
    UnderSampled {
        /// 实际的每比特样点数。
        margin: f64,
    },
    /// 通道数不满足。
    #[error("{protocol:?} 解码需要 {need} 个通道，本次采集只有 {have} 个")]
    Channels {
        /// 解码协议。
        protocol: Protocol,
        /// 需要的通道数。
        need: usize,
        /// 实际通道数。
        have: usize,
    },
    /// 协议与参数不匹配（例如 `protocol=uart` 却给了 `DecodeParams::I2c`）。
    #[error("协议与参数不匹配：{protocol:?} 需要 {expected}")]
    Mismatch {
        /// 解码协议。
        protocol: Protocol,
        /// 需要的参数类型名。
        expected: &'static str,
    },
    /// 参数组合当前不支持（如实报，不静默错解）。
    #[error("不支持：{what}")]
    Unsupported {
        /// 不支持的内容与怎么办。
        what: String,
    },
    /// 线路全平坦，没有任何可解码的电平跳变。
    #[error("没有可解码的信号：线路全平坦")]
    NoSignal,
}

impl From<DecodeError> for ScopeError {
    fn from(e: DecodeError) -> Self {
        match e {
            DecodeError::UnderSampled { margin } => ScopeError::Unsupported(format!(
                "每比特样点数 {margin:.2} < 4，欠采样不出帧。\
                 降低波特率或提高采样率后重试"
            )),
            DecodeError::Channels {
                protocol,
                need,
                have,
            } => ScopeError::InvalidParam {
                field: "channels",
                value: format!("{have} 个通道"),
                reason: format!(
                    "{protocol:?} 解码需要 {need} 个通道。UART 需 1 通道，I²C 需 2 通道"
                ),
            },
            DecodeError::Mismatch { protocol, expected } => ScopeError::InvalidParam {
                field: "params",
                value: format!("{protocol:?}"),
                reason: format!("参数类型与协议不匹配，需要 {expected}"),
            },
            DecodeError::Unsupported { what } => ScopeError::Unsupported(what),
            DecodeError::NoSignal => {
                ScopeError::Unsupported("线路全平坦，没有可解码的跳变。检查信号线与共地".into())
            }
        }
    }
}

/// 欠采样保护门：每比特样点数低于此值拒绝解码。
const MIN_SAMPLES_PER_BIT: f64 = 4.0;

/// 解码一次采集。
///
/// 所有协议错误都在**本地**给出（不碰设备），且都带「怎么办」。
pub fn decode(cap: &Capture, req: &DecodeRequest) -> Result<DecodedFrames, DecodeError> {
    match (&req.protocol, &req.params) {
        (Protocol::I2c, DecodeParams::I2c(cfg)) => decode_i2c_adapter(cap, cfg),
        (Protocol::Uart, DecodeParams::Uart(cfg)) => {
            let ch = req.channels.first().copied().unwrap_or(0);
            decode_uart(cap, ch, cfg)
        }
        (protocol, _) => Err(DecodeError::Mismatch {
            protocol: *protocol,
            expected: match protocol {
                Protocol::I2c => "DecodeParams::I2c",
                Protocol::Uart => "DecodeParams::Uart",
            },
        }),
    }
}

/// I²C 适配层：转调既有 `decode_capture`，只做类型转换。
fn decode_i2c_adapter(
    cap: &Capture,
    cfg: &i2c_decode::I2cDecodeConfig,
) -> Result<DecodedFrames, DecodeError> {
    let need = Protocol::I2c.channels_needed();
    if cap.channels.len() < need {
        return Err(DecodeError::Channels {
            protocol: Protocol::I2c,
            need,
            have: cap.channels.len(),
        });
    }
    // 通道校验后，剩余错误（越界/门限非法）由 i2c_decode 自己报 —— 语义不变。
    let d = i2c_decode::decode_capture(cap, cfg)
        .map_err(|e| DecodeError::Unsupported { what: e.summary() })?;
    Ok(to_decoded(d))
}

/// 把既有 [`i2c_decode::I2cDecode`] 转成统一出口类型（公开给证据包第 5 步用）。
pub fn to_decoded(d: i2c_decode::I2cDecode) -> DecodedFrames {
    // 先取全部标量/借用，最后再 move `d.transactions` —— 部分移动后
    // 就不能再用 `d` 的方法了。
    let frame_errors = d.transactions.iter().filter(|t| !t.complete).count();
    let frame_ok = d.frame_count() - frame_errors;
    let warnings = d.warnings.iter().cloned().map(DecodeWarning::I2c).collect();
    DecodedFrames {
        protocol: Protocol::I2c,
        // 采样率不归解码器管；调用方在组装证据时用自己的 `cap.rate_hz`。
        // 这里填 0 并在文档注明，防止有人拿它当时间基准。
        rate_hz: 0,
        frames: d.transactions.into_iter().map(Frame::I2c).collect(),
        warnings,
        quality: DecodeQuality {
            sample_margin: None,
            frame_ok,
            frame_errors,
        },
    }
}

/// UART 8N1 解码器（探针验证过的算法，见模块头「同步锁定态」）。
fn decode_uart(cap: &Capture, ch: usize, cfg: &UartConfig) -> Result<DecodedFrames, DecodeError> {
    if cfg.data_bits != 8 {
        return Err(DecodeError::Unsupported {
            what: format!("data_bits = {}（当前仅支持 8，即 8N1）", cfg.data_bits),
        });
    }
    if cfg.parity != Parity::None {
        return Err(DecodeError::Unsupported {
            what: "parity ≠ None（当前仅支持 8N1；校验位解码未实现）".into(),
        });
    }
    if cfg.stop_bits != 1 {
        return Err(DecodeError::Unsupported {
            what: format!("stop_bits = {}（当前仅支持 1）", cfg.stop_bits),
        });
    }
    if cfg.baud == 0 {
        return Err(DecodeError::Unsupported {
            what: "baud 未指定。自动波特率估计是建议性功能（估错会解出看似合理".to_string()
                + "但全错的字节，属红线），尚未实现 —— 请显式给波特率",
        });
    }

    let have = cap.channels.len();
    if have == 0 || ch >= have {
        return Err(DecodeError::Channels {
            protocol: Protocol::Uart,
            need: 1,
            have,
        });
    }
    let s = cap.samples(ch).unwrap_or(&[]);
    if s.len() < 2 || cap.rate_hz == 0 {
        return Err(DecodeError::NoSignal);
    }

    // 每比特样点数。低于保护门拒绝出帧。
    let t = cap.rate_hz as f64 / cfg.baud as f64; // 样点/位
    if t < MIN_SAMPLES_PER_BIT {
        return Err(DecodeError::UnderSampled { margin: t });
    }

    // 中点二值化：实测窗内极值的中点（与 measure.rs 的 mid 同口径）。
    // 不用 Otsu —— UART 空闲态占绝大多数，直方图双峰不保证成立。
    let (mut lo, mut hi) = (u16::MAX, u16::MIN);
    for &v in s {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi <= lo {
        return Err(DecodeError::NoSignal);
    }
    let mid = (lo as f64 + hi as f64) / 2.0;
    let high = |i: usize| s[i] as f64 > mid;

    // 起始位候选：所有下降沿；sample 0 为低时补一个 0（采集窗可从起始位开始）。
    // 毛刺滤波：与前一候选间隔 < 0.5 位的丢弃前者（判为毛刺，docs/12 §3.3）。
    let mut starts: Vec<usize> = Vec::new();
    if cfg.start_bit_low && s[0] as f64 <= mid {
        starts.push(0);
    }
    for i in 1..s.len() {
        if s[i - 1] as f64 > mid && s[i] as f64 <= mid {
            if let Some(&last) = starts.last() {
                if ((i - last) as f64) < 0.5 * t && last != 0 {
                    starts.pop(); // 前一个是毛刺
                }
            }
            starts.push(i);
        }
    }
    if starts.is_empty() {
        return Err(DecodeError::NoSignal);
    }

    let n = s.len();
    let frame_bits = 1.0 + cfg.data_bits as f64 + cfg.stop_bits as f64; // 8N1 = 10
    let sample_at = |x: f64| -> Option<usize> {
        let i = x.floor() as i64;
        if i < 0 || i as usize as i64 != i || i as usize >= n {
            None
        } else {
            Some(i as usize)
        }
    };

    let mut frames: Vec<Frame> = Vec::new();
    let mut errors = 0usize;
    let mut truncated = false;
    let mut idx = 0usize;

    while idx < starts.len() {
        let start = starts[idx];

        // 起始位校验：0.5T 处应为低。
        let p_half = sample_at(start as f64 + 0.5 * t);
        match p_half {
            None => {
                truncated = true;
                break;
            }
            Some(p) if high(p) => {
                // 候选不是起始位（数据位内的下降沿）—— 下一候选。
                errors += 1;
                idx += 1;
                continue;
            }
            _ => {}
        }

        // 数据位：在 (1.5 + k)T 采样，k = 0..8。
        let mut byte = 0u8;
        let mut abort = false;
        for k in 0..cfg.data_bits {
            match sample_at(start as f64 + (1.5 + k as f64) * t) {
                None => {
                    truncated = true;
                    abort = true;
                    break;
                }
                Some(p) => {
                    let bit = high(p);
                    let shift = if cfg.msb_first {
                        cfg.data_bits - 1 - k
                    } else {
                        k
                    };
                    if bit {
                        byte |= 1 << shift;
                    }
                }
            }
        }
        if abort {
            break;
        }

        // 停止位：应为高。不为高 → 帧保留、标 ok = false（丢弃会让错误率不可见）。
        let p_stop = sample_at(start as f64 + (1.5 + cfg.data_bits as f64) * t);
        let ok = match p_stop {
            None => {
                truncated = true;
                break;
            }
            Some(p) => high(p),
        };
        if !ok {
            errors += 1;
        }
        frames.push(Frame::Uart(UartFrame {
            index: frames.len() + 1,
            start_sample: start,
            byte,
            ok,
        }));

        // 同步锁定：期望下一帧在 [start + (bits-0.5)T, start + (bits+2.5)T)。
        // 窗口覆盖 0..2.5 位空闲；命中则继续锁定，否则失锁回全域。
        let lo_w = start as f64 + (frame_bits - 0.5) * t;
        let hi_w = start as f64 + (frame_bits + 2.5) * t;
        let mut next = None;
        let mut j = idx + 1;
        while j < starts.len() && (starts[j] as f64) < hi_w {
            if starts[j] as f64 >= lo_w {
                next = Some(j);
                break;
            }
            j += 1;
        }
        match next {
            Some(j) => idx = j,
            None => {
                // 失锁：从窗口右端起全域搜索第一个候选。
                match starts[j.min(starts.len())..]
                    .iter()
                    .position(|&st| st as f64 >= hi_w)
                {
                    Some(off) => idx = j + off,
                    None => break, // 窗口右端之后再也没有下降沿
                }
            }
        }
    }

    let mut warnings = Vec::new();
    if errors > 0 {
        warnings.push(DecodeWarning::FrameError { count: errors });
    }
    if truncated {
        warnings.push(DecodeWarning::Truncated);
    }
    let frame_ok = frames.len() - errors;
    Ok(DecodedFrames {
        protocol: Protocol::Uart,
        rate_hz: cap.rate_hz,
        frames,
        warnings,
        quality: DecodeQuality {
            sample_margin: Some(t),
            frame_ok,
            frame_errors: errors,
        },
    })
}

// ══════════════════════════════════════════════════════════════════
// 测试
// ══════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Capture;

    /// 与模拟器 `uart_115k` 场景**同构**的波形生成器：整数位序号、
    /// 10 位帧 + 2 位空闲、LSB 先发。core 不依赖 sim（依赖方向反过来），
    /// 所以 B1 在 core 用同构生成器验，sim 侧另有真实场景对照测试。
    fn uart_wave(bytes: &[u8], baud: u32, rate: u32, n: usize, idle_bits: usize) -> Vec<u16> {
        let frame_bits = 10 + idle_bits;
        (0..n)
            .map(|i| {
                let bit = i as u64 * baud as u64 / rate as u64;
                let pat = (bytes.len() * frame_bits) as u64;
                let pos = (bit % pat) as usize;
                let frame = pos / frame_bits;
                let within = pos % frame_bits;
                let level = if within >= 10 {
                    true // 空闲位
                } else if within == 0 {
                    false // 起始位
                } else if within == 9 {
                    true // 停止位
                } else {
                    (bytes[frame] >> (within - 1)) & 1 == 1 // 数据位，LSB 先
                };
                if level {
                    4000
                } else {
                    0
                }
            })
            .collect()
    }

    fn uart_cap(s: Vec<u16>, rate: u32) -> Capture {
        let mut cap = Capture::new(1, rate, 1, s.len() as u32);
        cap.channels[0] = s;
        cap
    }

    fn uart_req(baud: u32) -> DecodeRequest {
        DecodeRequest {
            protocol: Protocol::Uart,
            channels: vec![0],
            params: DecodeParams::Uart(UartConfig {
                baud,
                ..UartConfig::default()
            }),
        }
    }

    const BYTES: [u8; 3] = [0x55, 0x0F, 0xA5];
    const RATE: u32 = 857_142;

    /// B1：正确波特率下解出真值序列。
    #[test]
    fn uart_decodes_the_reference_pattern() {
        let s = uart_wave(&BYTES, 115_200, RATE, 4096, 2);
        let cap = uart_cap(s, RATE);
        let out = decode(&cap, &uart_req(115_200)).expect("应能解码");
        assert_eq!(out.protocol, Protocol::Uart);
        // 4096 / (12 位 × 7.44 样点) ≈ 46 帧；全对、零告警。
        assert_eq!(out.frame_count(), 46, "帧数应约 46（12 位/帧）");
        assert_eq!(out.quality.frame_errors, 0);
        assert!(out.warnings.is_empty(), "{:?}", out.warnings);
        let bytes = out.uart_bytes().expect("UART 应有字节序列");
        // 前 12 字节是 [55, 0F, A5] × 4；尾帧是 46 = 15×3 + 1，只到 0x55。
        let want: Vec<u8> = (0..12).map(|i| BYTES[i % 3]).collect();
        assert_eq!(&bytes[..12], &want[..], "字节序列应循环真值");
        assert_eq!(bytes[45], 0x55);
        assert_eq!(out.quality.sample_margin, Some(RATE as f64 / 115_200.0));
    }

    /// B2：波特率偏 ±10% —— 字节序列不得等于真值，字节错误率必须过半，
    /// 且有告警。**不以帧错误数过半为判据**：锁定态对波特率偏差有一定
    /// 吸收力，坏帧数会低估错误（探针 +10% 实测仅 28% 帧错，但字节全错）。
    #[test]
    fn wrong_baud_produces_wrong_bytes_not_plausible_ones() {
        for baud in [115_200 - 11_520, 115_200 + 11_520] {
            let s = uart_wave(&BYTES, 115_200, RATE, 4096, 2);
            let cap = uart_cap(s, RATE);
            let out = decode(&cap, &uart_req(baud)).expect("欠采样门不会拦 ±10%");
            let bytes = out.uart_bytes().expect("UART 应有字节序列");

            // 负例第一判据：不等于真值序列（对齐 3 种相位里最好的那种）。
            let pattern = [0x55u8, 0x0F, 0xA5];
            let mismatch_rate = |off: usize| {
                bytes
                    .iter()
                    .enumerate()
                    .filter(|(i, b)| **b != pattern[(*i + off) % 3])
                    .count() as f64
                    / bytes.len() as f64
            };
            let best = (0..3).map(mismatch_rate).fold(f64::INFINITY, f64::min);
            assert!(
                best > 0.5,
                "baud={baud} 字节错误率 {best:.2} 未过半 —— 解码器「看起来对」了"
            );
            assert!(!out.warnings.is_empty(), "baud={baud} 有错帧必须有告警");
        }
    }

    /// B2 的另一面：正常波特率在 4096 点上字节错误率为 0。
    #[test]
    fn correct_baud_is_byte_perfect() {
        let s = uart_wave(&BYTES, 115_200, RATE, 4096, 2);
        let cap = uart_cap(s, RATE);
        let out = decode(&cap, &uart_req(115_200)).expect("应能解码");
        let bytes = out.uart_bytes().unwrap();
        let pattern = [0x55u8, 0x0F, 0xA5];
        let bad = bytes
            .iter()
            .enumerate()
            .filter(|(i, b)| **b != pattern[i % 3])
            .count();
        assert_eq!(bad, 0, "正确波特率下不应有错字节");
    }

    /// B5：欠采样保护 —— 每比特样点数 < 4 时拒绝出帧，不返回「看似合理」的结果。
    #[test]
    fn undersampled_is_refused() {
        let s = uart_wave(&BYTES, 115_200, 10_000, 4096, 2);
        let cap = uart_cap(s, 10_000);
        let err = decode(&cap, &uart_req(115_200)).unwrap_err();
        assert!(
            matches!(err, DecodeError::UnderSampled { margin } if (margin - 10_000.0 / 115_200.0).abs() < 1e-9),
            "应为 UnderSampled，实际 {err:?}"
        );
    }

    /// B4：通道数不满足时报错，不 panic。
    #[test]
    fn channel_shortfalls_are_reported() {
        // I²C 对单通道采集：报「需要 2 个通道」。
        let cap1 = uart_cap(vec![100u16; 16], RATE);
        let req = DecodeRequest {
            protocol: Protocol::I2c,
            channels: vec![],
            params: DecodeParams::I2c(crate::i2c_decode::I2cDecodeConfig::default()),
        };
        assert!(matches!(
            decode(&cap1, &req),
            Err(DecodeError::Channels {
                need: 2,
                have: 1,
                ..
            })
        ));

        // UART 通道越界：报「通道数不满足」。
        let s = uart_wave(&BYTES, 115_200, RATE, 256, 2);
        let cap = uart_cap(s, RATE);
        let mut req = uart_req(115_200);
        req.channels = vec![9];
        assert!(matches!(
            decode(&cap, &req),
            Err(DecodeError::Channels {
                need: 1,
                have: 1,
                ..
            })
        ));

        // UART 空 channels = 自动取通道 0：应正常解出。
        let mut req = uart_req(115_200);
        req.channels = vec![];
        assert!(decode(&cap, &req).is_ok(), "空通道应自动选 0");
    }

    /// 协议与参数不匹配：报错而不是 panic 或错解。
    #[test]
    fn mismatched_params_are_reported() {
        let cap = uart_cap(vec![100u16; 16], RATE);
        let req = DecodeRequest {
            protocol: Protocol::Uart,
            channels: vec![0],
            params: DecodeParams::I2c(crate::i2c_decode::I2cDecodeConfig::default()),
        };
        assert!(matches!(
            decode(&cap, &req),
            Err(DecodeError::Mismatch { .. })
        ));
    }

    /// 不支持的参数组合（校验位 / 数据位 / baud=0）：明确报错，不静默错解。
    #[test]
    fn unsupported_combinations_are_reported() {
        let s = uart_wave(&BYTES, 115_200, RATE, 256, 2);
        let cap = uart_cap(s, RATE);

        let mut base = uart_req(115_200);
        match &mut base.params {
            DecodeParams::Uart(u) => u.parity = Parity::Even,
            _ => unreachable!(),
        }
        assert!(matches!(
            decode(&cap, &base),
            Err(DecodeError::Unsupported { .. })
        ));

        let mut base = uart_req(0);
        match &mut base.params {
            DecodeParams::Uart(u) => u.baud = 0,
            _ => unreachable!(),
        }
        assert!(matches!(
            decode(&cap, &base),
            Err(DecodeError::Unsupported { .. })
        ));
    }

    /// 锁定态用例：采集窗从起始位开始（sample 0 为低）也能解，
    /// 且数据位内的下降沿不会被误判为起始位 —— 探针教训：没有这条
    /// 会在窗首解出 0xD5 一类假帧。
    #[test]
    fn sample_zero_start_bit_is_recognized() {
        let s = uart_wave(&BYTES, 115_200, RATE, 4096, 2);
        assert_eq!(s[0], 0, "生成器应从起始位开始（低电平）");
        let cap = uart_cap(s, RATE);
        let out = decode(&cap, &uart_req(115_200)).expect("应能解码");
        let bytes = out.uart_bytes().unwrap();
        assert_eq!(bytes[0], 0x55, "首帧必须是真值 0x55，不是假帧");
    }

    /// 坏停止位的帧保留且标 ok = false —— 丢弃会让错误率不可见。
    #[test]
    fn frame_error_is_kept_not_dropped() {
        let mut s = uart_wave(&BYTES, 115_200, RATE, 4096, 2);
        // 破坏第一帧的停止位：区间 [9T, 10T) ≈ [67, 75) 置低。
        for v in s.iter_mut().take(75).skip(67) {
            *v = 0;
        }
        let cap = uart_cap(s, RATE);
        let out = decode(&cap, &uart_req(115_200)).expect("应能解码");
        let first = match &out.frames[0] {
            Frame::Uart(u) => u.clone(),
            _ => unreachable!(),
        };
        assert!(!first.ok, "停止位被破坏的帧必须标 ok=false");
        assert_eq!(first.byte, 0x55, "字节仍保留（不可信，但不丢弃）");
        assert!(
            out.warnings
                .iter()
                .any(|w| matches!(w, DecodeWarning::FrameError { .. })),
            "有坏帧必须有 FrameError 告警"
        );
    }

    /// 全平坦线路：NoSignal，不是空帧列表。
    #[test]
    fn flat_line_is_no_signal() {
        let cap = uart_cap(vec![2048u16; 64], RATE);
        assert!(matches!(
            decode(&cap, &uart_req(115_200)),
            Err(DecodeError::NoSignal)
        ));
    }

    /// DecodeError → ScopeError 的转换链可用（MCP 的既有 From 依赖它）。
    #[test]
    fn decode_error_converts_into_scope_error() {
        let e: ScopeError = DecodeError::UnderSampled { margin: 0.087 }.into();
        // `Unsupported` 的 summary() 是写死的通用文案；具体信息在 Display 与 hint。
        assert!(e.to_string().contains("欠采样"), "{}", e);
        assert!(e.hint().is_some(), "必须带「怎么办」");
    }
}
