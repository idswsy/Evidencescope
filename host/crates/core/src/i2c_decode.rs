//! I2C 协议解码器 —— 数字通路的解释器。
//!
//! # 为什么是边沿驱动
//!
//! 采样率解不了 I2C（见 [`docs/00-origin.md`] 的四条约束）：857 kSPS 的 ADC 要解
//! 400 kHz 需要 3.33 MSPS，还伴随采样相位拍频。但**只要把「等间隔采样」换成
//! 「事件驱动计时」**，问题就消失了 —— 所以本模块刻意**不假定采样率足够高**，
//! 它只依赖「SCL 的上升沿发生了」和「此刻 SDA 是多少」这两个事实。
//!
//! 由此得到一条重要性质：**解码结果与采样率无关**，只与边沿的时间戳有关。
//! 采集是 857 kSPS 还是 10 kSPS，只要每个 SCL 上升沿都被采到，解码结果完全一致。
//!
//! # 判决门限：三态，不是二值
//!
//! 电平不硬判 0/1，而是分三态：
//!
//! ```text
//!   0 ──── VIL ──────── 判决带 ──────── VIH ──── 4095
//!   │  Low  │         Unknown         │  High  │
//! ```
//!
//! 落在判决带内的样点**不做分类判定**，计入 [`SignalQuality::unknown_scl`] /
//! [`SignalQuality::unknown_sda`] 并产生 [`I2cWarning::DecisionBandSamples`]。
//! 这条纪律的来源是项目定位 —— **无法给出可信结论时必须显式拒答，不得输出看似精确的无依据数值**。
//!
//! # 与 MCP 的关系
//!
//! [`I2cDecode`] 是 `scope_i2c_decode` 工具的返回体，必须**紧凑**：
//! 它只含帧序列与统计量，不含任何原始样点。这是三层 token 防护的第二层。
//!
//! # 适用范围：这是 **ADC 通路**的解码器（重要）
//!
//! 本模块吃的是 ADC 采样序列，所以它受采样率约束 —— 这与项目的双通路设计
//! 直接相关（见 [`docs/00-origin.md`] 与 ADR-005）：
//!
//! | I2C 速率 | SCL 周期 | 857 kSPS 下的样点/比特 | 本解码器 |
//! |---|---|---|---|
//! | 100 kHz | 10 µs | 8.6 | ✅ 舒适 |
//! | 400 kHz（50% 占空比） | 2.5 µs | 2.1 | ⚠️ 勉强，贴着理论上限 |
//! | 400 kHz（fast-mode 最坏 tHIGH=0.6 µs） | — | 0.51 | ❌ 高电平期不足一个采样点，信息已经丢失 |
//! | 1 MHz | — | 0.86 | ❌ 不可能 |
//!
//! **400 kHz / 1 MHz 的权威解码走的是数字通路**（LM393 → TIM 输入捕获，
//! 13.9 ns 边沿时间戳），那条路径是事件驱动的，与采样率无关。
//! 本模块的职责是：100 kHz 档可靠解码 + 从模拟视角评估信号质量。
//!
//! 注意模拟器 `i2c_400k` 场景用的是 **50% 占空比**（tHIGH = 1.25 µs），
//! 比真实 fast-mode 总线宽松 —— 它上面跑通不代表真总线上跑得通。
//!
//! [`docs/00-origin.md`]: ../../../../docs/00-origin.md

use crate::capture::Capture;
use crate::error::{Result, ScopeError};
use serde::{Deserialize, Serialize};

/// 12-bit 满量程。
const FULL_SCALE: u16 = 4095;

/// 默认判决门限比例：0.3 / 0.7 · VDD。
///
/// 与模拟器自检用的门限一致（`host/crates/sim/src/waveform.rs`），
/// 这样「模拟器产出的波形」与「真机采到的波形」走同一条判定路径。
pub const DEFAULT_VIL_RATIO: f32 = 0.30;

/// 默认判决门限比例：0.7 · VDD。见 [`DEFAULT_VIL_RATIO`]。
pub const DEFAULT_VIH_RATIO: f32 = 0.70;

// ══════════════════════════════════════════════════════════════════
// 电平判定
// ══════════════════════════════════════════════════════════════════

/// 一个样点的三态判定结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Level {
    /// 低于或等于 `VIL`。
    Low,
    /// 高于或等于 `VIH`。
    High,
    /// 落在判决带内 —— **不做分类判定**，记录并报警。
    Unknown,
}

/// 判决门限（ADC LSB）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Levels {
    /// 判为高电平的下限（含）。
    pub vih_lsb: u16,
    /// 判为低电平的上限（含）。
    pub vil_lsb: u16,
}

impl Levels {
    /// 按比例构造（`vil_ratio < vih_ratio`，取值 0..1）。
    pub fn from_ratios(vil_ratio: f32, vih_ratio: f32) -> Levels {
        let to_lsb = |r: f32| (FULL_SCALE as f32 * r.clamp(0.0, 1.0)).round() as u16;
        Levels {
            vil_lsb: to_lsb(vil_ratio),
            vih_lsb: to_lsb(vih_ratio),
        }
    }

    /// 项目默认门限：0.3 / 0.7 · VDD。
    pub fn default_ratio() -> Levels {
        Levels::from_ratios(DEFAULT_VIL_RATIO, DEFAULT_VIH_RATIO)
    }

    /// 门限本身是否自洽（`vil < vih`）。
    ///
    /// 不合法时**解码仍会继续**，但会给出 [`I2cWarning::LevelsInvalid`] ——
    /// 让 Agent 拿到「解不出来是因为你门限设反了」，而不是一个空结果。
    pub fn is_valid(&self) -> bool {
        self.vil_lsb < self.vih_lsb
    }

    /// 单样点判定。
    pub fn classify(&self, value: u16) -> Level {
        if value >= self.vih_lsb {
            Level::High
        } else if value <= self.vil_lsb {
            Level::Low
        } else {
            Level::Unknown
        }
    }
}

impl Default for Levels {
    fn default() -> Self {
        Levels::default_ratio()
    }
}

// ══════════════════════════════════════════════════════════════════
// 配置
// ══════════════════════════════════════════════════════════════════

/// 一次 I2C 解码的输入参数。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct I2cDecodeConfig {
    /// SCL 所在通道（0 起）。
    pub scl_channel: usize,
    /// SDA 所在通道（0 起）。
    pub sda_channel: usize,
    /// 判决门限。
    pub levels: Levels,
    /// 去抖时间（ns）。小于一个采样周期时等价于「不去抖」。
    pub debounce_ns: u32,
}

impl I2cDecodeConfig {
    /// 默认：CH0 = SCL、CH1 = SDA，0.3/0.7 门限，50 ns 去抖。
    pub fn new(scl_channel: usize, sda_channel: usize) -> I2cDecodeConfig {
        I2cDecodeConfig {
            scl_channel,
            sda_channel,
            levels: Levels::default_ratio(),
            debounce_ns: 50,
        }
    }
}

impl Default for I2cDecodeConfig {
    fn default() -> Self {
        I2cDecodeConfig::new(0, 1)
    }
}

// ══════════════════════════════════════════════════════════════════
// 输出类型
// ══════════════════════════════════════════════════════════════════

/// 一次地址传输。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Address {
    /// 地址值（7 位时为 0..=127，10 位时为 0..=1023）。
    pub value: u16,
    /// 是否为 10 位地址。
    pub ten_bit: bool,
    /// 读（`true`）还是写（`false`）。
    pub read: bool,
    /// 从机是否应答（`false` = NACK）。
    pub acked: bool,
}

/// 帧内的一个事件。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EventKind {
    /// 起始位（总线由空闲进入传输）。
    Start,
    /// 重复起始位（一次传输中途重新开始，常见于「写寄存器 → 读回」）。
    RepeatedStart,
    /// 地址字节，含其应答位。
    Address(Address),
    /// 数据字节，含其应答位。
    Data {
        /// 字节值。
        value: u8,
        /// 从机是否应答（`false` = NACK）。
        acked: bool,
    },
    /// 停止位。
    Stop,
    /// 采集窗口在帧中间结束 —— 这一帧**不完整**，不能当作正常收尾。
    Truncated,
}

/// 带位置的事件。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Event {
    /// 事件发生的样点号。
    ///
    /// 地址/数据事件取**采样该比特的 SCL 上升沿**；START/STOP 取 SDA 跳变点。
    pub sample: u32,
    /// 换算成时间（µs，相对采集起点）。
    pub time_us: u32,
    /// 事件内容。
    pub kind: EventKind,
}

/// 一次 START..STOP 传输。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Transaction {
    /// 起始样点。
    pub start_sample: u32,
    /// 起始时间（µs）。
    pub start_time_us: u32,
    /// 是否由重复起始位开始（`true` 表示它接在上一帧之后，属于同一次操作）。
    pub repeated: bool,
    /// 首个地址字节解出的地址；解不出为 `None`。
    pub address: Option<Address>,
    /// 数据字节（**不含**地址字节）。
    pub bytes: Vec<u8>,
    /// 帧内事件序列。
    pub events: Vec<Event>,
    /// 是否有正常 STOP 收尾。`false` 说明被采集窗口截断了。
    pub complete: bool,
}

impl Transaction {
    /// 地址的常见写法：`0x44 (W)` / `0x44 (R)`；无地址返回 `"—"`。
    pub fn address_str(&self) -> String {
        match &self.address {
            Some(a) => format!("0x{:02X} ({})", a.value, if a.read { "R" } else { "W" }),
            None => "—".to_string(),
        }
    }

    /// 有没有**从机拒绝**（地址或数据没被应答）。
    ///
    /// ⚠ 读事务里，主机在**最后一个字节**回 NACK 是**正常的收尾** ——
    /// 它表达的意思正是「我读够了，不用再给下一个字节了」。那不是故障。
    ///
    /// 回归：从前这里把所有 `!acked` 一视同仁，于是「写寄存器 → 读回」
    /// 这种最常见的 I2C 时序里，**读帧永远显示 `nack: true`**。
    /// 一份完全健康的读数看起来像出了错，一个 Agent 几乎必然会据此
    /// 报出一个不存在的问题。
    pub fn has_nack(&self) -> bool {
        self.events.iter().enumerate().any(|(i, e)| match &e.kind {
            EventKind::Address(a) => !a.acked,
            EventKind::Data { acked, .. } => !*acked && !self.is_master_terminating_nack(i),
            _ => false,
        })
    }

    /// 第 `i` 个事件是不是「读事务末尾由**主机**发出的收尾 NACK」。
    ///
    /// 读操作的最后一个字节，主机回 NACK 的意思是「我读够了」—— 那是**正常收尾**，
    /// 不是从机故障。所以它不该和「从机没应答」共用一个记号。
    ///
    /// **这条规则只在这里定义一次。** [`Self::has_nack`] 与 `report.rs` 的证据包
    /// 都调它 —— 两边各写一份判断，迟早只改一处（本项目最熟的那类缺陷）。
    pub fn is_master_terminating_nack(&self, i: usize) -> bool {
        // 只有读事务才可能由主机发收尾 NACK
        let reading = matches!(&self.address, Some(a) if a.read);
        if !reading {
            return false;
        }
        let last_data = self
            .events
            .iter()
            .rposition(|e| matches!(&e.kind, EventKind::Data { .. }));
        Some(i) == last_data
            && matches!(
                self.events.get(i).map(|e| &e.kind),
                Some(EventKind::Data { acked: false, .. })
            )
    }
}

/// 一条线卡住时停在哪个电平。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StuckLevel {
    /// 停在高电平。
    High,
    /// 停在低电平。
    Low,
}

impl std::fmt::Display for StuckLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            StuckLevel::High => "高",
            StuckLevel::Low => "低",
        })
    }
}

/// 解码过程中的告警。
///
/// 这些**不是错误** —— 解码会继续，但 Agent 必须知道结果的可信度。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum I2cWarning {
    /// 两条线在整个采集窗内没有任何跳变。
    NoSignal,
    /// `VIL >= VIH`，门限设反了。
    LevelsInvalid {
        /// 配置的高电平门限。
        vih_lsb: u16,
        /// 配置的低电平门限。
        vil_lsb: u16,
    },
    /// SCL 全程停在同一电平，没有时钟。
    SclStuck {
        /// 停在哪。
        level: StuckLevel,
    },
    /// 有 SCL 时钟，但 SDA 全程不变 —— 通常是 SDA 没接或从机没上电。
    SdaStuck {
        /// 停在哪。
        level: StuckLevel,
    },
    /// 有样点落在判决带内。占比偏高说明门限没设对。
    DecisionBandSamples {
        /// 哪条线。
        line: I2cLine,
        /// 落在带内的样点数。
        count: u32,
        /// 首个落在带内的样点号。
        first_sample: u32,
    },
    /// 实测高电平只比 `VIH` 高一点点 —— 上拉偏弱或驱动不足。
    MarginalHigh {
        /// 哪条线。
        line: I2cLine,
        /// 实测高电平。
        high_lsb: u16,
        /// 配置的高电平门限。
        vih_lsb: u16,
    },
    /// 采集窗口在帧中间结束，最后一帧不完整。
    TruncatedFrame,
}

/// 告警针对的是哪条线。
///
/// # 为什么是枚举而不是 `u8`
///
/// 这里原本是 `channel: u8`，值取 0（SCL 轨）/ 1（SDA 轨），文案却写成
/// `"CH{channel}"` —— 于是**一条轨的内部序号被当成通道号印了出来**。
///
/// 后果：用户在界面上把 SCL 选在 CH2、SDA 选在 CH3，告警照样说「CH0」「CH1」，
/// 与任何真实通道都对不上。而这类「一个含义被读错的整数」正是枚举能根除的 ——
/// 叫 `line` 就只能填 [`I2cLine::Scl`] 或 [`I2cLine::Sda`]，填不成通道号。
///
/// ⚠ 注意它与**采集通道号**是两回事：通道号取决于用户在界面上把 SCL/SDA
/// 选到了哪一路，只有 `I2cDecodeConfig` 知道。所以这里根本不该印通道号。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum I2cLine {
    /// 时钟线。
    Scl,
    /// 数据线。
    Sda,
}

impl I2cLine {
    /// 中文名，用于告警文案。
    pub fn name(self) -> &'static str {
        match self {
            I2cLine::Scl => "SCL",
            I2cLine::Sda => "SDA",
        }
    }
}

/// 信号质量统计 —— 从数字通路能看出来的部分。
///
/// 注意：**上拉强度、振铃、边沿形状要靠模拟通路**（ADC）判断，
/// 本结构只给出数字侧能确定的事实。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignalQuality {
    /// 实测 SCL 高电平（采集窗内 SCL 为高时的最大样点值）。
    pub scl_high_level: u16,
    /// 实测 SDA 高电平。
    pub sda_high_level: u16,
    /// SCL 上升沿个数。
    pub scl_edges: u32,
    /// SCL 频率（Hz）；无法测定为 `None`。
    pub scl_freq_hz: Option<u32>,
    /// SCL 高电平占空比（0..1）；无法测定为 `None`。
    pub scl_duty: Option<f32>,
    /// 落在判决带内的 SCL 样点数。
    pub unknown_scl: u32,
    /// 落在判决带内的 SDA 样点数。
    pub unknown_sda: u32,
    /// 参与解码的总样点数。
    pub total_samples: u32,
}

impl SignalQuality {
    /// 判决带内的样点占比（0..1）。超过 1% 就值得怀疑门限。
    pub fn decision_band_ratio(&self) -> f32 {
        if self.total_samples == 0 {
            return 0.0;
        }
        (self.unknown_scl + self.unknown_sda) as f32 / (self.total_samples as f32 * 2.0)
    }
}

/// 一次完整解码的结果 —— `scope_i2c_decode` 的返回体。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct I2cDecode {
    /// 帧序列。
    pub transactions: Vec<Transaction>,
    /// 告警。
    pub warnings: Vec<I2cWarning>,
    /// 信号质量统计。
    pub quality: SignalQuality,
}

impl I2cDecode {
    /// 一帧都没解出来。
    pub fn is_empty(&self) -> bool {
        self.transactions.is_empty()
    }

    /// 帧数（「共 N 帧」里的 N）。
    pub fn frame_count(&self) -> usize {
        self.transactions.len()
    }

    /// 全部数据字节（跨帧拼接，**不含地址**）。
    pub fn all_bytes(&self) -> Vec<u8> {
        self.transactions
            .iter()
            .flat_map(|t| t.bytes.iter().copied())
            .collect()
    }

    /// 有没有任何一条告警属于「结果不可信」级别。
    pub fn is_untrustworthy(&self) -> bool {
        self.warnings.iter().any(|w| {
            matches!(
                w,
                I2cWarning::NoSignal
                    | I2cWarning::LevelsInvalid { .. }
                    | I2cWarning::SclStuck { .. }
            )
        })
    }

    /// 渲染成 `i2c_decode.txt` 的格式。
    ///
    /// 这是**给人看**的出口 —— Agent 拿的是结构化的 [`I2cDecode`] 本身。
    pub fn to_text(&self) -> String {
        let mut out = String::new();
        out.push_str(&format!("I2C 解码结果\n共 {} 帧\n\n", self.frame_count()));

        if self.transactions.is_empty() {
            out.push_str("（没有解出任何帧）\n");
        }

        for (i, tx) in self.transactions.iter().enumerate() {
            out.push_str(&format!(
                "帧 {}  @ {:.3} ms{}\n",
                i + 1,
                tx.start_time_us as f64 / 1000.0,
                if tx.repeated { "  (重复起始)" } else { "" }
            ));
            for ev in &tx.events {
                let line = match &ev.kind {
                    EventKind::Start => "  START".to_string(),
                    EventKind::RepeatedStart => "  Sr    (重复起始)".to_string(),
                    EventKind::Address(a) => format!(
                        "  地址  {:<4} {}   {}",
                        format!("0x{:02X}", a.value),
                        if a.read { "(R)" } else { "(W)" },
                        if a.acked { "ACK" } else { "NACK ←" }
                    ),
                    EventKind::Data { value, acked } => format!(
                        "  数据  0x{value:02X}      {}",
                        if *acked { "ACK" } else { "NACK ←" }
                    ),
                    EventKind::Stop => "  STOP".to_string(),
                    EventKind::Truncated => "  —— 采集窗口在此截断 ——".to_string(),
                };
                out.push_str(&format!("{line}\n"));
            }
            if !tx.bytes.is_empty() {
                let hex: Vec<String> = tx.bytes.iter().map(|b| format!("{b:02X}")).collect();
                out.push_str(&format!("  ⇒ 数据字节: {}\n", hex.join(" ")));
            }
            out.push('\n');
        }

        out.push_str("信号质量\n");
        out.push_str(&format!("  SCL 边沿   {}\n", self.quality.scl_edges));
        if let Some(f) = self.quality.scl_freq_hz {
            out.push_str(&format!("  SCL 频率   {:.1} kHz\n", f as f64 / 1000.0));
        }
        if let Some(d) = self.quality.scl_duty {
            out.push_str(&format!("  SCL 占空比 {:.1}%\n", d * 100.0));
        }
        out.push_str(&format!(
            "  实测高电平 SCL={} SDA={} LSB\n",
            self.quality.scl_high_level, self.quality.sda_high_level
        ));
        out.push_str(&format!(
            "  判决带样点 SCL={} SDA={} ({:.2}%)\n",
            self.quality.unknown_scl,
            self.quality.unknown_sda,
            self.quality.decision_band_ratio() * 100.0
        ));

        if !self.warnings.is_empty() {
            out.push_str("\n告警\n");
            for w in &self.warnings {
                out.push_str(&format!("  ⚠ {}\n", warning_text(w)));
            }
        }
        out
    }
}

impl I2cWarning {
    /// 给人看的一句话说明。
    ///
    /// 放在方法里而不是 pub 自由函数：可发现性更好，也不往 crate 根命名空间里
    /// 再塞一个名字（那里已经 re-export 了十几个）。CLI / GUI / MCP 共用这一份，
    /// 避免文案在三处漂移 —— 跟「协议四处同步」是同一条纪律。
    pub fn text(&self) -> String {
        warning_text(self)
    }
}

fn warning_text(w: &I2cWarning) -> String {
    match w {
        I2cWarning::NoSignal => "两条线都没有跳变，没有信号".to_string(),
        I2cWarning::LevelsInvalid { vih_lsb, vil_lsb } => {
            format!("门限非法：VIH={vih_lsb} 不大于 VIL={vil_lsb}")
        }
        I2cWarning::SclStuck { level } => format!("SCL 全程停在{level}，没有时钟"),
        I2cWarning::SdaStuck { level } => {
            format!("SDA 全程停在{level}；有 SCL 但无数据（SDA 未接？从机未上电？）")
        }
        I2cWarning::DecisionBandSamples {
            line,
            count,
            first_sample,
        } => format!(
            "{} 有 {count} 个样点落在判决带内（首个 @{first_sample}），门限可能没设对",
            line.name()
        ),
        I2cWarning::MarginalHigh {
            line,
            high_lsb,
            vih_lsb,
        } => {
            format!(
                "{} 实测高电平仅 {high_lsb} LSB（门限 {vih_lsb}），上拉偏弱或驱动不足",
                line.name()
            )
        }
        I2cWarning::TruncatedFrame => "采集窗口在帧中间结束，最后一帧不完整".to_string(),
    }
}

// ══════════════════════════════════════════════════════════════════
// 解码主体
// ══════════════════════════════════════════════════════════════════

/// 单通道的稳定电平跟踪器。
///
/// **判决带内的样点不改变稳定电平**（保持上一次确定的值），只计数。
/// 这样一次毛刺或一段缓慢的边沿不会伪造出一个假的跳变。
struct StableLevel {
    stable: Level,
    candidate: Level,
    run: u32,
    unknown: u32,
    first_unknown: Option<u32>,
    high_max: u16,
}

impl StableLevel {
    fn new() -> StableLevel {
        StableLevel {
            stable: Level::High, // I2C 空闲态：两线都是高
            candidate: Level::High,
            run: 0,
            unknown: 0,
            first_unknown: None,
            high_max: 0,
        }
    }

    /// 喂入一个样点，返回 `Some(new_level)` 表示稳定电平刚刚发生了跳变。
    fn feed(&mut self, idx: u32, value: u16, levels: &Levels, need: u32) -> Option<Level> {
        let raw = levels.classify(value);
        if raw == Level::High {
            self.high_max = self.high_max.max(value);
        }

        match raw {
            Level::Unknown => {
                self.unknown += 1;
                if self.first_unknown.is_none() {
                    self.first_unknown = Some(idx);
                }
                // 候选作废，但**稳定电平不动** —— 判决带内不下结论
                self.candidate = Level::Unknown;
                self.run = 0;
                None
            }
            _ => {
                if raw == self.candidate {
                    self.run = self.run.saturating_add(1);
                } else {
                    self.candidate = raw;
                    self.run = 1;
                }
                if self.run >= need && raw != self.stable {
                    self.stable = raw;
                    Some(raw)
                } else {
                    None
                }
            }
        }
    }

    /// 用第一个可判定的样点作为初始稳定电平。
    ///
    /// 采集可能从中途开始（总线不一定处于空闲态），所以不能硬编码初值。
    fn prime(&mut self, values: &[u16], levels: &Levels) {
        for &v in values {
            let lv = levels.classify(v);
            if lv != Level::Unknown {
                self.stable = lv;
                self.candidate = lv;
                self.run = 1;
                return;
            }
        }
    }
}

/// 去抖窗口换算成样点数。至少 1（一个样点即「不去抖」）。
fn debounce_samples(debounce_ns: u32, rate_hz: u32) -> u32 {
    if rate_hz == 0 || debounce_ns == 0 {
        return 1;
    }
    let n = (debounce_ns as u64 * rate_hz as u64) / 1_000_000_000;
    (n.max(1)) as u32
}

/// 一次进行中的传输。
struct PendingTx {
    start_sample: u32,
    repeated: bool,
    address: Option<Address>,
    bytes: Vec<u8>,
    events: Vec<Event>,
}

/// 解码核心 —— 纯函数，不依赖 [`Capture`]，便于单测。
///
/// `scl` 与 `sda` 必须等长且同起点；多余的部分按较短的一条截断。
pub fn decode(
    scl: &[u16],
    sda: &[u16],
    rate_hz: u32,
    levels: Levels,
    debounce_ns: u32,
) -> I2cDecode {
    let n = scl.len().min(sda.len());
    let need = debounce_samples(debounce_ns, rate_hz);
    let to_us = |sample: u32| -> u32 {
        if rate_hz == 0 {
            0
        } else {
            ((sample as u64 * 1_000_000) / rate_hz as u64) as u32
        }
    };

    let mut warnings: Vec<I2cWarning> = Vec::new();
    if !levels.is_valid() {
        warnings.push(I2cWarning::LevelsInvalid {
            vih_lsb: levels.vih_lsb,
            vil_lsb: levels.vil_lsb,
        });
    }

    let mut quality = SignalQuality {
        scl_high_level: 0,
        sda_high_level: 0,
        scl_edges: 0,
        scl_freq_hz: None,
        scl_duty: None,
        unknown_scl: 0,
        unknown_sda: 0,
        total_samples: n as u32,
    };

    if n < 2 {
        warnings.push(I2cWarning::NoSignal);
        return I2cDecode {
            transactions: Vec::new(),
            warnings,
            quality,
        };
    }

    let mut scl_track = StableLevel::new();
    let mut sda_track = StableLevel::new();
    scl_track.prime(&scl[..n], &levels);
    sda_track.prime(&sda[..n], &levels);

    let mut transactions: Vec<Transaction> = Vec::new();
    let mut cur: Option<PendingTx> = None;

    // 位组装
    let mut shift: u32 = 0;
    let mut nbits: u32 = 0;
    // 0 = 等第一个地址字节；1 = 等 10 位地址的第二字节；2 = 地址已完整
    let mut addr_stage: u8 = 0;
    let mut ten_bit_hi: u16 = 0;
    let mut ten_bit_read = false;

    // SCL 时序统计
    let mut rise_idx: Vec<u32> = Vec::new();
    let mut scl_transitions: u32 = 0;

    let mut sda_transitions: u32 = 0;

    for i in 0..n {
        let idx = i as u32;
        let prev_scl = scl_track.stable;
        let prev_sda = sda_track.stable;

        let scl_changed = scl_track.feed(idx, scl[i], &levels, need).map(|lv| {
            scl_transitions += 1;
            lv
        });
        let sda_changed = sda_track.feed(idx, sda[i], &levels, need).map(|lv| {
            sda_transitions += 1;
            lv
        });

        let scl_hi = scl_track.stable == Level::High;
        let sda_hi = sda_track.stable == Level::High;

        // ── 1) SDA 跳变：只在 SCL 为高时才有 START / STOP 语义 ──
        if sda_changed.is_some() && scl_hi && prev_scl == Level::High {
            if prev_sda == Level::High && !sda_hi {
                // 高→低 = 起始位。
                //
                // 「重复起始」的判据是**上一帧还没收尾就重新开始**，
                // 不是「之前有过别的帧」—— 紧挨着跑的两笔独立事务
                // （STOP 之后立刻 START）是两帧，不是重复起始。
                // 回归：这里曾经写成 `!transactions.is_empty()`，
                // 于是背靠背的每一笔事务都被误标成了重复起始。
                let repeated = cur.is_some();
                if let Some(tx) = cur.take() {
                    // `complete = true` —— 走到这里说明**上一帧没有 STOP 就
                    // 重新起始**，那正是重复起始（`repeated` 的判据就是它）。
                    // 以 Sr 收尾的帧是**正常结束**的，只是它的后半截在下一帧里。
                    //
                    // 回归：这里从前传的是 `false`，而 `complete` 的语义是
                    // 「有没有被采集窗口截断」。于是「写寄存器 → 读回」这种
                    // 最常见的 I2C 时序里，**写帧永远显示 complete=false** ——
                    // 一个 Agent 会把它读成「这一帧没读完」，从而报出一个
                    // 根本不存在的问题。（`false` 只该留给窗口截断，
                    // 见下面 ── 3) 收尾 ── 那一处。）
                    transactions.push(finish(tx, true, to_us));
                }
                cur = Some(PendingTx {
                    start_sample: idx,
                    repeated,
                    address: None,
                    bytes: Vec::new(),
                    events: vec![Event {
                        sample: idx,
                        time_us: to_us(idx),
                        kind: if repeated {
                            EventKind::RepeatedStart
                        } else {
                            EventKind::Start
                        },
                    }],
                });
                shift = 0;
                nbits = 0;
                addr_stage = 0;
            } else if prev_sda == Level::Low && sda_hi {
                // 低→高 = 停止位
                if let Some(mut tx) = cur.take() {
                    tx.events.push(Event {
                        sample: idx,
                        time_us: to_us(idx),
                        kind: EventKind::Stop,
                    });
                    transactions.push(finish(tx, true, to_us));
                }
                shift = 0;
                nbits = 0;
                addr_stage = 0;
            }
        }

        // ── 2) SCL 上升沿：采样数据位 ──
        if let Some(Level::High) = scl_changed {
            rise_idx.push(idx);

            if let Some(tx) = cur.as_mut() {
                shift = (shift << 1) | u32::from(sda_hi);
                nbits += 1;

                if nbits == 9 {
                    let value = (shift >> 1) as u8;
                    let acked = (shift & 1) == 0; // 第 9 位为 0 = ACK

                    match addr_stage {
                        0 => {
                            // 第一个字节 = 地址（7 位）或 10 位地址的高字节
                            if (value & 0xF8) == 0xF0 {
                                // 10 位地址的首字节：A9:A8 在 bit2:1，R/W 在 bit0。
                                // 此刻地址还不完整，**先不产生 Address 事件** ——
                                // 半个地址报出去比不报更糟。
                                ten_bit_hi = u16::from((value >> 1) & 0b11);
                                ten_bit_read = (value & 1) != 0;
                                addr_stage = 1;
                            } else {
                                let a = Address {
                                    value: u16::from(value >> 1),
                                    ten_bit: false,
                                    read: (value & 1) != 0,
                                    acked,
                                };
                                tx.address = Some(a.clone());
                                addr_stage = 2;
                                tx.events.push(Event {
                                    sample: idx,
                                    time_us: to_us(idx),
                                    kind: EventKind::Address(a),
                                });
                            }
                        }
                        1 => {
                            // 10 位地址的低字节
                            let a = Address {
                                value: (ten_bit_hi << 8) | u16::from(value),
                                ten_bit: true,
                                read: ten_bit_read,
                                acked,
                            };
                            tx.address = Some(a.clone());
                            addr_stage = 2;
                            tx.events.push(Event {
                                sample: idx,
                                time_us: to_us(idx),
                                kind: EventKind::Address(a),
                            });
                        }
                        _ => {
                            tx.bytes.push(value);
                            tx.events.push(Event {
                                sample: idx,
                                time_us: to_us(idx),
                                kind: EventKind::Data { value, acked },
                            });
                        }
                    }

                    shift = 0;
                    nbits = 0;
                }
            }
        }
    }

    // ── 3) 收尾 ──
    if let Some(mut tx) = cur.take() {
        tx.events.push(Event {
            sample: (n - 1) as u32,
            time_us: to_us((n - 1) as u32),
            kind: EventKind::Truncated,
        });
        transactions.push(finish(tx, false, to_us));
        warnings.push(I2cWarning::TruncatedFrame);
    }

    // ── 4) 信号质量 ──
    quality.unknown_scl = scl_track.unknown;
    quality.unknown_sda = sda_track.unknown;
    quality.scl_edges = rise_idx.len() as u32;
    quality.scl_high_level = scl_track.high_max;
    quality.sda_high_level = sda_track.high_max;

    // ── SCL 频率与占空比 ──
    //
    // 两个坑：
    //
    // 1. **空闲段必须排除。** 事务之间的总线空闲会形成一个超长「周期」
    //    （SCL 全程为高）。把它算进去，频率会偏低、占空比会偏高 ——
    //    200 个样点的空闲就足以把占空比从 50% 抬到 70% 以上。
    //    判据：周期不超过中位数的 1.5 倍。时钟抖动远小于这个数，
    //    而空闲段通常是它的几十倍，分得很开。
    //
    // 2. **频率要用均值不是中位数。** 857142 Hz 下 100 kHz 的周期是
    //    8.57 个采样点，实际周期在 8 和 9 之间交替；取中位数会落到
    //    8 或 9，得到 107.1 / 95.2 kHz（误差 5%）。均值才收敛到真值。
    if rise_idx.len() >= 2 {
        let mut periods: Vec<u32> = rise_idx.windows(2).map(|w| w[1] - w[0]).collect();
        periods.sort_unstable();
        let median = periods[periods.len() / 2];
        let cutoff = median + median / 2;

        let clock: Vec<u32> = periods
            .iter()
            .copied()
            .filter(|&p| p > 0 && p <= cutoff)
            .collect();
        if !clock.is_empty() && rate_hz > 0 {
            let mean = clock.iter().map(|&p| p as f64).sum::<f64>() / clock.len() as f64;
            if mean > 0.0 {
                quality.scl_freq_hz = Some((rate_hz as f64 / mean) as u32);
            }
        }

        let mut sum = 0.0f32;
        let mut cnt = 0u32;
        for w in rise_idx.windows(2) {
            let p = w[1] - w[0];
            if p == 0 || p > cutoff {
                continue;
            }
            let win = &scl[w[0] as usize..w[1] as usize];
            let high = win
                .iter()
                .filter(|&&v| levels.classify(v) == Level::High)
                .count();
            sum += high as f32 / win.len() as f32;
            cnt += 1;
        }
        if cnt > 0 {
            quality.scl_duty = Some(sum / cnt as f32);
        }
    }

    // ── 5) 告警汇总 ──
    if scl_transitions == 0 && sda_transitions == 0 {
        warnings.push(I2cWarning::NoSignal);
    } else {
        let as_stuck = |lv: Level| {
            if lv == Level::High {
                StuckLevel::High
            } else {
                StuckLevel::Low
            }
        };
        if scl_transitions == 0 {
            warnings.push(I2cWarning::SclStuck {
                level: as_stuck(scl_track.stable),
            });
        }
        // 有 SCL 时钟但 SDA 一次没动 —— 数据线的问题，不是协议的问题
        if sda_transitions == 0 && scl_transitions > 0 {
            warnings.push(I2cWarning::SdaStuck {
                level: as_stuck(sda_track.stable),
            });
        }
    }

    if let Some(first) = scl_track.first_unknown {
        warnings.push(I2cWarning::DecisionBandSamples {
            line: I2cLine::Scl,
            count: scl_track.unknown,
            first_sample: first,
        });
    }
    if let Some(first) = sda_track.first_unknown {
        warnings.push(I2cWarning::DecisionBandSamples {
            line: I2cLine::Sda,
            count: sda_track.unknown,
            first_sample: first,
        });
    }

    // 高电平只比门限高一点点 → 上拉偏弱
    for (line, high) in [
        (I2cLine::Scl, scl_track.high_max),
        (I2cLine::Sda, sda_track.high_max),
    ] {
        if high > 0 && high >= levels.vih_lsb {
            let margin = high.saturating_sub(levels.vih_lsb);
            let headroom = FULL_SCALE.saturating_sub(levels.vih_lsb);
            if headroom > 0 && u32::from(margin) * 10 < u32::from(headroom) {
                warnings.push(I2cWarning::MarginalHigh {
                    line,
                    high_lsb: high,
                    vih_lsb: levels.vih_lsb,
                });
            }
        }
    }

    I2cDecode {
        transactions,
        warnings,
        quality,
    }
}

/// 把进行中的传输收尾成 [`Transaction`]。
fn finish(tx: PendingTx, complete: bool, to_us: impl Fn(u32) -> u32) -> Transaction {
    Transaction {
        start_sample: tx.start_sample,
        start_time_us: to_us(tx.start_sample),
        repeated: tx.repeated,
        address: tx.address,
        bytes: tx.bytes,
        events: tx.events,
        complete,
    }
}

/// 从一次采集里解码 I2C。
///
/// 通道号越界返回 [`ScopeError::InvalidParam`] —— 在本地拦下，不发到线上。
pub fn decode_capture(cap: &Capture, cfg: &I2cDecodeConfig) -> Result<I2cDecode> {
    let scl = cap
        .samples(cfg.scl_channel)
        .ok_or_else(|| channel_err(cfg.scl_channel, cap.channels.len()))?;
    let sda = cap
        .samples(cfg.sda_channel)
        .ok_or_else(|| channel_err(cfg.sda_channel, cap.channels.len()))?;
    Ok(decode(scl, sda, cap.rate_hz, cfg.levels, cfg.debounce_ns))
}

fn channel_err(ch: usize, have: usize) -> ScopeError {
    ScopeError::InvalidParam {
        field: "i2c_channel",
        value: ch.to_string(),
        reason: format!("该采集只有 {have} 个通道"),
    }
}

/// 自动判定哪条通道是 SCL、哪条是 SDA。
///
/// 判据：**SCL 一定有时钟**（边沿多、占空比接近 50%），而 SDA 在传输间隙是平的。
/// 两个方向都试一遍，取「解出的完整帧更多」的那个；打平则取 SCL 边沿更多的。
///
/// 返回 `(scl_channel, sda_channel)`；通道不足或两条都没信号时返回 `None`。
pub fn detect_channels(cap: &Capture, levels: Levels, debounce_ns: u32) -> Option<(usize, usize)> {
    if cap.channels.len() < 2 {
        return None;
    }

    // 判据是**边沿数**，不是解出多少帧。
    //
    // I2C 里 SCL 每个比特时钟一次，SDA 一个比特至多变一次 —— 所以
    // 「哪条线的边沿多，哪条就是 SCL」是硬性质。
    //
    // 回归：这里曾经以「解出的完整帧数」为首要判据，结果**反向接法反而赢**。
    // 把 SDA 当 SCL 时，真正的 SCL（高频跳变）被当成数据线，那些跳变被
    // 误读成 START/STOP，于是解出一堆「被伪 STOP 正常收尾」的空壳帧，
    // 完整帧数比正确方向还多一个 —— 自动判定选反，62 个 address=null 的
    // 空帧还被报成 trustworthy=true。
    let edges = |scl_ch: usize, sda_ch: usize| -> u32 {
        let (Some(a), Some(b)) = (cap.samples(scl_ch), cap.samples(sda_ch)) else {
            return 0;
        };
        decode(a, b, cap.rate_hz, levels, debounce_ns)
            .quality
            .scl_edges
    };

    // 只在前两个通道之间试正反 —— 更多通道的组合需要人工指定
    let forward = edges(0, 1);
    let reverse = edges(1, 0);

    if forward == 0 && reverse == 0 {
        return None; // 两个方向都没有边沿 —— 没信号
    }
    // 打平按约定取默认
    Some(if forward >= reverse { (0, 1) } else { (1, 0) })
}

// ══════════════════════════════════════════════════════════════════
// 测试
// ══════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    const VIH: u16 = 2866;
    const VIL: u16 = 1228;

    fn levels() -> Levels {
        Levels {
            vih_lsb: VIH,
            vil_lsb: VIL,
        }
    }

    /// 手工搭一条 I2C 波形 —— 不依赖模拟器，这样解码器的 bug 不会被
    /// 模拟器的 bug 掩盖。`bits_per_sample` 控制采样密度，用来验证
    /// 「解码与采样率无关」这条性质。
    struct Builder {
        scl: Vec<u16>,
        sda: Vec<u16>,
    }

    impl Builder {
        fn new() -> Builder {
            Builder {
                scl: Vec::new(),
                sda: Vec::new(),
            }
        }

        /// 保持 `hold` 个采样周期。
        fn hold(&mut self, scl_hi: bool, sda_hi: bool, hold: usize) {
            for _ in 0..hold {
                self.scl.push(if scl_hi { 4095 } else { 0 });
                self.sda.push(if sda_hi { 4095 } else { 0 });
            }
        }

        /// 一个比特：SCL 低半周期 → SCL 高半周期，数据在低电平期间就位。
        fn bit(&mut self, sda_hi: bool, hold: usize) {
            self.hold(false, sda_hi, hold);
            self.hold(true, sda_hi, hold);
        }

        /// 发送一个字节 + 其应答位。
        fn byte(&mut self, b: u8, acked: bool, hold: usize) {
            for i in (0..8).rev() {
                self.bit((b >> i) & 1 == 1, hold);
            }
            self.bit(!acked, hold); // ACK = SDA 低
        }

        fn start(&mut self, hold: usize) {
            // 空闲：两线高 → SCL 高时 SDA 拉低
            self.hold(true, true, hold);
            self.hold(true, false, hold);
            self.hold(false, false, hold);
        }

        /// 重复起始。
        ///
        /// 关键：释放 SDA **必须在 SCL 为低时做**。若在 SCL 为高时把 SDA
        /// 拉高，那是一个 STOP —— 生成的就是「STOP 后跟 START」，
        /// 而不是重复起始。
        fn repeated_start(&mut self, hold: usize) {
            self.hold(false, false, hold); // SCL 拉低（此时 SDA 仍被拉低）
            self.hold(false, true, hold); // 趁 SCL 为低释放 SDA —— 不构成 STOP
            self.hold(true, true, hold); // SCL 拉高
            self.hold(true, false, hold); // SCL 高时拉低 SDA = 重复起始
            self.hold(false, false, hold);
        }

        fn stop(&mut self, hold: usize) {
            self.hold(false, false, hold);
            self.hold(true, false, hold);
            self.hold(true, true, hold); // SCL 高时 SDA 拉高 = 停止位
        }
    }

    #[test]
    fn decodes_a_simple_register_write() {
        // 经典场景：写 0x44 的寄存器 0x00 = 0x1A（模拟器的默认事务）
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4); // 地址 0x44 + W
        b.byte(0x00, true, 4);
        b.byte(0x1A, true, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        assert_eq!(r.frame_count(), 1, "应解出 1 帧");
        let tx = &r.transactions[0];
        assert!(tx.complete);
        assert_eq!(tx.bytes, vec![0x00, 0x1A]);
        let addr = tx.address.as_ref().expect("应有地址");
        assert_eq!(addr.value, 0x44);
        assert!(!addr.read);
        assert!(addr.acked);
        assert!(!tx.has_nack());
    }

    #[test]
    fn decodes_write_then_read_with_repeated_start() {
        // 真实用法：写寄存器地址 → 重复起始 → 读回
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4); // 写 0x44
        b.byte(0x00, true, 4); // 寄存器 0x00
        b.repeated_start(4);
        b.byte(0x89, true, 4); // 读 0x44
        b.byte(0x1A, true, 4);
        b.byte(0x2B, false, 4); // 最后一字节主机 NACK
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        assert_eq!(r.frame_count(), 2, "重复起始应切成两帧");
        assert!(r.transactions[0].bytes == vec![0x00]);
        assert!(r.transactions[1].repeated, "第二帧应标记为重复起始");
        assert_eq!(r.transactions[1].bytes, vec![0x1A, 0x2B]);
        assert!(r.transactions[1].address.as_ref().unwrap().read);

        // 最后一字节的应答位确实是 NACK（主机发的）……
        let last_acked = r.transactions[1]
            .events
            .iter()
            .rev()
            .find_map(|e| match &e.kind {
                EventKind::Data { acked, .. } => Some(*acked),
                _ => None,
            });
        assert_eq!(last_acked, Some(false), "最后一字节的应答位应当是 NACK");

        // ……但那是**正常的读收尾**（「我读够了」），不是故障。
        //
        // 回归：这条断言从前写的是 `assert!(has_nack())` —— 它把
        // 「最后一字节的 ack 位」和「这一帧出了错」当成了同一件事。
        // 于是每一笔「写寄存器 → 读回」的读帧都被报成 `nack: true`，
        // 一份健康的读数看起来像坏了。
        assert!(
            !r.transactions[1].has_nack(),
            "主机在读事务末字节回 NACK 是正常收尾，不该报成故障"
        );
    }

    #[test]
    fn a_slave_nack_is_still_reported() {
        // 上一条把「主机的读收尾 NACK」排除掉了 —— 这里守住另一边：
        // **从机**拒绝应答必须照实报出来，不能一起被滤掉。
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4); // 地址被应答：器件在总线上
        b.byte(0x00, false, 4); // 这一笔写被从机 NACK
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);
        assert_eq!(r.frame_count(), 1);
        assert!(r.transactions[0].has_nack(), "从机拒绝应答必须被报出来");
        assert!(
            r.transactions[0].address.as_ref().unwrap().acked,
            "地址是通过的"
        );
    }

    #[test]
    fn a_frame_ended_by_a_repeated_start_is_complete() {
        // `complete` 的语义是「没被采集窗口截断」，**不是**「以 STOP 收尾」。
        // 以重复起始结束的写帧是正常结束的，它的后半截就在下一帧里。
        //
        // 回归：从前这条路径传 `false`，于是「写寄存器 → 读回」里
        // 写帧永远显示 `complete: false`，看起来像被截断了。
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x00, true, 4);
        b.repeated_start(4);
        b.byte(0x89, true, 4);
        b.byte(0x2B, false, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);
        assert!(
            r.transactions[0].complete,
            "以重复起始结束的帧是完整的，只是后半截在下一帧"
        );

        // 对照组：真的被窗口截断的那种，`complete` 必须是 false
        let mut b2 = Builder::new();
        b2.start(4);
        b2.byte(0x88, true, 4);
        let r2 = decode(&b2.scl, &b2.sda, 857_142, levels(), 50);
        assert!(!r2.transactions[0].complete, "被截断的帧不该算完整");
    }

    #[test]
    fn result_is_independent_of_sample_rate() {
        // 这是整个模块最重要的一条性质：边沿驱动 → 与采样密度无关。
        // 每个半周期 2 个样点（勉强够）到 20 个样点（很奢侈），结果必须一致。
        let mut results = Vec::new();
        for hold in [2usize, 3, 8, 20] {
            let mut b = Builder::new();
            b.start(hold);
            b.byte(0x88, true, hold);
            b.byte(0x00, true, hold);
            b.byte(0x1A, true, hold);
            b.stop(hold);

            let r = decode(&b.scl, &b.sda, 857_142, levels(), 0);
            assert_eq!(r.frame_count(), 1, "hold={hold} 时应解出 1 帧");
            assert_eq!(
                r.transactions[0].bytes,
                vec![0x00, 0x1A],
                "hold={hold} 时数据字节不符"
            );
            results.push(r.transactions[0].bytes.clone());
        }
        assert!(
            results.windows(2).all(|w| w[0] == w[1]),
            "不同采样密度解出的内容必须一致"
        );
    }

    #[test]
    fn decodes_ten_bit_address() {
        // 10 位地址 0x0A3：首字节 1111 0 A9 A8 R/W = 0xF0（A9:A8 = 00，写），
        // 第二字节才是低 8 位 0xA3
        let mut b = Builder::new();
        b.start(4);
        b.byte(0b1111_0000, true, 4); // A9:A8 = 00, R/W = 0
        b.byte(0xA3, true, 4); // 低 8 位
        b.byte(0x55, true, 4); // 数据
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        let tx = &r.transactions[0];
        let addr = tx.address.as_ref().expect("应有地址");
        assert!(addr.ten_bit, "应识别为 10 位地址");
        assert_eq!(addr.value, 0x0A3);
        assert_eq!(tx.bytes, vec![0x55], "地址两字节不应混进数据");
    }

    #[test]
    fn reports_nack_on_missing_slave() {
        // 从机不存在：地址字节后 SDA 保持高 = NACK
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, false, 4); // NACK
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        let tx = &r.transactions[0];
        assert!(tx.has_nack(), "应报告 NACK");
        assert!(!tx.address.as_ref().unwrap().acked);
        assert!(tx.bytes.is_empty(), "NACK 后没有数据字节");
    }

    #[test]
    fn no_signal_is_reported_not_silently_empty() {
        // 两线都是直流 —— 必须明确说「没信号」，而不是返回一个空结果让 Agent 猜
        let flat: Vec<u16> = vec![0; 512];
        let r = decode(&flat, &flat, 857_142, levels(), 50);

        assert!(r.is_empty());
        assert!(r.warnings.contains(&I2cWarning::NoSignal));
        assert!(r.is_untrustworthy());
    }

    #[test]
    fn sda_stuck_is_distinguished_from_no_signal() {
        // 有 SCL 时钟、SDA 不动 —— 这是「数据线没接」，不是「没信号」
        let mut b = Builder::new();
        for _ in 0..20 {
            b.hold(false, true, 4);
            b.hold(true, true, 4);
        }
        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        assert!(r.warnings.contains(&I2cWarning::SdaStuck {
            level: StuckLevel::High
        }));
        assert!(!r.warnings.contains(&I2cWarning::NoSignal));
        assert!(r.quality.scl_edges > 0);
    }

    #[test]
    fn samples_in_decision_band_are_counted_not_guessed() {
        // 把 SDA 的高电平压到判决带里 —— 不应被硬判成 High，
        // 而应被计数并告警
        let band = (VIH + VIL) / 2;
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x1A, true, 4);
        b.stop(4);

        // 把 SDA 所有高电平样点改到判决带内
        for v in b.sda.iter_mut() {
            if *v == 4095 {
                *v = band;
            }
        }

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);
        assert!(
            r.quality.unknown_sda > 0,
            "判决带内的样点必须被计数，而不是被猜成 0 或 1"
        );
        assert!(r.warnings.iter().any(|w| matches!(
            w,
            I2cWarning::DecisionBandSamples {
                line: I2cLine::Sda,
                ..
            }
        )));
    }

    /// 告警文案必须说 **SCL / SDA**，不能印通道号。
    ///
    /// 回归：文案曾经是 `"CH{channel}"`，而那个 `channel` 装的是**轨序号**
    /// （0=SCL 轨、1=SDA 轨），不是采集通道号。于是用户在界面上把 SCL 选在
    /// CH2、SDA 选在 CH3，告警照样说「CH0」「CH1」—— 与任何真实通道都对不上。
    ///
    /// 判据：文案里出现 SCL / SDA，且**不出现任何 `CH` + 数字**。
    #[test]
    fn warnings_name_the_line_not_a_channel_number() {
        for line in [I2cLine::Scl, I2cLine::Sda] {
            let w = I2cWarning::MarginalHigh {
                line,
                high_lsb: 2900,
                vih_lsb: 2866,
            };
            let t = w.text();
            assert!(t.contains(line.name()), "文案应点名 {}：{t}", line.name());
            assert!(
                !t.contains("CH"),
                "文案里不得出现通道号写法（那个下标不是通道号）：{t}"
            );
        }

        let t = I2cWarning::DecisionBandSamples {
            line: I2cLine::Scl,
            count: 3,
            first_sample: 12,
        }
        .text();
        assert!(t.contains("SCL"), "应点名 SCL：{t}");
        assert!(!t.contains("CH"), "不得出现通道号写法：{t}");
    }

    #[test]
    fn invalid_thresholds_are_reported() {
        let bad = Levels {
            vih_lsb: 1000,
            vil_lsb: 3000, // 设反了
        };
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, bad, 50);
        assert!(r
            .warnings
            .iter()
            .any(|w| matches!(w, I2cWarning::LevelsInvalid { .. })));
    }

    #[test]
    fn truncated_frame_is_marked_incomplete() {
        // 采集窗口在事务中间被切断：该帧不得按完整帧处理
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x00, true, 4);
        // 这里就结束了，没有 STOP

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        assert_eq!(r.frame_count(), 1);
        assert!(!r.transactions[0].complete, "被截断的帧不能标为完整");
        assert!(r.warnings.contains(&I2cWarning::TruncatedFrame));
        assert!(matches!(
            r.transactions[0].events.last().unwrap().kind,
            EventKind::Truncated
        ));
    }

    #[test]
    fn scl_frequency_is_measured() {
        // SCL 周期 = 8 个采样周期（4 低 + 4 高）
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.stop(4);

        let rate = 800_000u32; // 周期 8 采样 @800kHz = 10 µs → 100 kHz
        let r = decode(&b.scl, &b.sda, rate, levels(), 50);

        let f = r.quality.scl_freq_hz.expect("应测出 SCL 频率");
        assert!(
            (95_000..=105_000).contains(&f),
            "SCL 频率应在 100 kHz 附近，实测 {f} Hz"
        );
    }

    #[test]
    fn debounce_rejects_a_single_sample_glitch() {
        // SCL 稳定为高、SDA 走一次起始位（其实没产生 SCL 时钟），
        // 然后在 SCL 低电平段里插一个**孤立的高电平采样点**。
        //
        // 这个毛刺会伪造出一个上升沿。开启去抖后它必须被丢掉 —— 否则
        // 解码器会在一个根本不存在的时刻去采样 SDA。
        let mut b = Builder::new();
        b.start(8); // SCL 全程为高，SDA 高→低

        let idx = b.scl.len() - 2; // 落在最后那段 SCL 低电平里
        b.scl[idx] = 4095;

        let no_debounce = decode(&b.scl, &b.sda, 857_142, levels(), 0);
        // 20 µs @ 857 kHz ≈ 17 个采样周期，远大于 1 个毛刺
        let debounced = decode(&b.scl, &b.sda, 857_142, levels(), 20_000);

        assert_eq!(
            no_debounce.quality.scl_edges, 1,
            "不去抖时应看到毛刺伪造出的那一个上升沿"
        );
        assert_eq!(
            debounced.quality.scl_edges, 0,
            "去抖后毛刺应被丢弃，真实上升沿数为 0"
        );
    }

    #[test]
    fn back_to_back_transactions_are_not_repeated_starts() {
        // 回归：曾经把 `repeated` 写成「之前有过别的帧」，于是背靠背的
        // 每一笔独立事务（STOP 之后立刻 START）都被误标成重复起始。
        // 真机上总线连续跑时，这会让 Agent 以为所有事务都是「写寄存器→读回」。
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x1A, true, 4);
        b.stop(4);
        b.start(4); // 第二笔独立事务
        b.byte(0x88, true, 4);
        b.byte(0x2B, true, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);

        assert_eq!(r.frame_count(), 2);
        assert!(
            !r.transactions[0].repeated && !r.transactions[1].repeated,
            "两笔独立事务都不该被标成重复起始"
        );
        assert!(r.transactions[0].complete && r.transactions[1].complete);
    }

    #[test]
    fn scl_duty_ignores_idle_high_between_transactions() {
        // 两笔事务之间让总线空闲一大段（SDA/SCL 都是高）——
        // 空占比不该把这段空闲算进去，否则会得出一个偏大的数
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.stop(4);
        b.hold(true, true, 200); // 长空闲
        b.start(4);
        b.byte(0x88, true, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);
        let duty = r.quality.scl_duty.expect("应测出占空比");
        assert!(
            (0.4..=0.6).contains(&duty),
            "SCL 是方波，占空比应接近 50%，实测 {:.1}%",
            duty * 100.0
        );
    }

    #[test]
    fn channel_detection_uses_edge_count_not_frame_count() {
        // 回归：判据曾经是「解出的完整帧数」，而反向接法解出的空壳帧**更多**
        // （真 SCL 的跳变被当成数据线后误读成 START/STOP，每帧都被伪 STOP
        // 正常收尾），于是自动判定稳定选反。
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x00, true, 4);
        b.byte(0x1A, true, 4);
        b.stop(4);

        let n = b.scl.len() as u32;
        let mut normal = Capture::new(1, 857_142, 2, n);
        normal.channels = vec![b.scl.clone(), b.sda.clone()];
        assert_eq!(
            detect_channels(&normal, levels(), 50),
            Some((0, 1)),
            "正接（CH0=SCL）应判为 (0, 1)"
        );

        // 反过来接：CH0 = SDA、CH1 = SCL
        let mut swapped = Capture::new(1, 857_142, 2, n);
        swapped.channels = vec![b.sda, b.scl];
        assert_eq!(
            detect_channels(&swapped, levels(), 50),
            Some((1, 0)),
            "反接（CH1=SCL）应判为 (1, 0)，而不是被空壳帧骗过去"
        );
    }

    #[test]
    fn text_rendering_contains_the_essentials() {
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x00, true, 4);
        b.byte(0x1A, true, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);
        let text = r.to_text();

        assert!(text.contains("共 1 帧"), "应显示帧数");
        assert!(text.contains("0x44"), "应显示地址");
        assert!(text.contains("0x1A"), "应显示数据字节");
        assert!(text.contains("START"));
        assert!(text.contains("STOP"));
    }

    #[test]
    fn all_bytes_flattens_across_frames() {
        let mut b = Builder::new();
        b.start(4);
        b.byte(0x88, true, 4);
        b.byte(0x00, true, 4);
        b.repeated_start(4);
        b.byte(0x89, true, 4);
        b.byte(0x1A, true, 4);
        b.byte(0x2B, false, 4);
        b.stop(4);

        let r = decode(&b.scl, &b.sda, 857_142, levels(), 50);
        assert_eq!(r.all_bytes(), vec![0x00, 0x1A, 0x2B]);
    }
}
