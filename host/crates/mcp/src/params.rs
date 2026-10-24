//! 工具参数类型 —— 每个工具 `inputSchema` 的**唯一来源**。
//!
//! 这里的每个结构体同时干两件事，用的**是同一个类型**：
//!
//! 1. [`JsonSchema`] 生成 `tools/list` 里的 `inputSchema`
//! 2. `Deserialize` 把 `tools/call` 的 `arguments` 解成它
//!
//! 于是「schema 声明的参数」与「实现读的参数」在类型上被绑死 ——
//! 改字段名时编译器直接报错，而不是让两边悄悄分家还能编译通过。
//! `docs/03-protocol.md` 里「Rust 类型 → serde/schemars → MCP JSON Schema」
//! 那句话，说的是这件事。
//!
//! # 为什么一律 `deny_unknown_fields`
//!
//! 从前的实现是 `p.get("字段名").and_then(|v| v.as_u64())`：字段名拼错、
//! 类型给错，`.get()` 一律返回 `None`，于是**静默走默认值**，工具照常
//! 返回成功。Agent 拿到的是「它没要求的配置下采的数据」，还以为是自己的
//! 那次调用 —— 这类错误顺路径测试永远发现不了，因为测试用的字段名是对的。
//!
//! 现在这种输入会得到一条指明字段的错误，而 `deny_unknown_fields` 同时
//! 让 schema 带上 `additionalProperties: false`，客户端提前就能看见。
//!
//! # 整数一律用 `u64`，收窄在实现里做
//!
//! 这条规则是**统一**的，没有例外：schema 里所有整数参数都是 `u64`，
//! 真实边界由 `#[schemars(range(..))]` 告诉客户端，越界值则由 `session.rs`
//! 收窄时报一句带「怎么办」的中文错误。
//!
//! 为什么不按字段挑最窄的类型？因为**收窄发生在 serde，而 serde 只会说
//! 「invalid value: integer 65537, expected u16」** —— 没有边界、没有
//! 「怎么办」。同一次重构里，`capture_samples` 留了宽类型，于是
//! `65537` 得到的是「超出范围（1..=4096）」；而 `count` 用了 `u32`，
//! `5000000000` 只得到一句类型不符。同一个错误，两种体验。
//!
//! # 默认值
//!
//! 有固定默认值的字段用 `#[serde(default = "...")]`，schemars 会把那个
//! 函数的返回值写进 schema 的 `default` —— 客户端因此知道「省略它会怎样」。
//! 函数与 `session.rs` 的兜底都指向 [`defaults`] 里的同一个常量。
//!
//! ⚠ **`count` / `max_points` 故意没有 `default`**：`count` 缺省时用的是
//! `max_points`（旧名），两个都缺省才是 512。给它一个 serde 默认值会让
//! `count` 永远有值，`max_points` 这条兼容路径就再也走不到了。

use schemars::{json_schema, JsonSchema, Schema, SchemaGenerator};
use scope_core::acquire::MAX_TIMEOUT_MS;
use scope_core::f103::{MAX_CAPTURE_SAMPLES, MAX_DECIMATION, MAX_SAMPLE_RATE_HZ};
use scope_sim::{FaultInjection, Scenario};
use serde::de::{self, Deserializer};
use serde::Deserialize;
use std::borrow::Cow;

// ══════════════════════════════════════════════════════════════
// 默认值
// ══════════════════════════════════════════════════════════════

/// 缺省值集中在这里，**每一条都有两个使用者**：serde 的 `default` 函数
/// （它决定 schema 里写什么）与 `session.rs` 的兜底（它决定显式传 `null`
/// 时怎么走）。两边指向同一个常量，改一处不会只改一半。
pub mod defaults {
    /// `scope_connect.baud`
    pub const BAUD: u32 = 921_600;
    /// `scope_capture.timeout_ms` —— 也是 `scope_acquire` 的等待上限默认。
    pub const CAPTURE_TIMEOUT_MS: u64 = 2_000;
    /// 预览点数上限（三层 token 防护第一层的硬顶）。
    pub const PREVIEW_POINTS: u64 = 256;
    /// `scope_read_waveform.count` 与 `max_points` 都缺省时的点数。
    pub const READ_COUNT: u64 = 512;
    /// `scope_read_waveform.start_sample` / `channel`。
    pub const ZERO: u64 = 0;
    /// `scope_i2c_decode.debounce_ns`
    pub const DEBOUNCE_NS: u64 = 50;
    /// `scope_decode` 的 UART 缺省波特率（8N1）。
    pub const DECODE_BAUD: u64 = 115_200;
    /// `scope_watch.duration_ms`
    pub const WATCH_MS: u64 = 1_000;
    /// `scope_watch.max_points`
    pub const WATCH_POINTS: u64 = 128;
    /// 去抖窗口的上限（ns）。
    ///
    /// I2C 最快 400 kHz，半个时钟周期是 1250 ns。去抖窗口一旦接近它，
    /// **真实的时钟边沿就会被当成毛刺滤掉**，而解码器仍会返回一份
    /// 看起来正常的结果（`trustworthy` 还是 true）。
    pub const MAX_DEBOUNCE_NS: u64 = 1_000;
    /// 故障注入里单次「延迟尖峰」的时长上限（ms）。
    ///
    /// 和 `MAX_TIMEOUT_MS` 是**同一类防线**：这个值会被原样交给
    /// `std::thread::sleep`，而 `serve()` 是单线程 —— 给一个天文数字
    /// 就能让整个 server 永久不响应任何请求（实测 20 秒零响应，
    /// 只能杀进程）。5 秒已经够模拟「链路卡了一下」，再长就不是尖峰了。
    pub const MAX_LATENCY_SPIKE_MS: u64 = 5_000;

    // ── 下面这些只给 serde 用（见模块头「默认值」一节）──
    pub(super) fn baud() -> Option<u32> {
        Some(BAUD)
    }
    pub(super) fn timeout_ms() -> Option<u64> {
        Some(CAPTURE_TIMEOUT_MS)
    }
    pub(super) fn preview_points() -> Option<u64> {
        Some(PREVIEW_POINTS)
    }
    pub(super) fn zero() -> Option<u64> {
        Some(ZERO)
    }
    pub(super) fn debounce_ns() -> Option<u64> {
        Some(DEBOUNCE_NS)
    }
    pub(super) fn decode_baud() -> Option<u64> {
        Some(DECODE_BAUD)
    }
    pub(super) fn watch_ms() -> Option<u64> {
        Some(WATCH_MS)
    }
    pub(super) fn watch_points() -> Option<u64> {
        Some(WATCH_POINTS)
    }
}

// ══════════════════════════════════════════════════════════════
// 通用件
// ══════════════════════════════════════════════════════════════

/// 不吃参数的工具用它。
///
/// 特意不用 `()`：`()` 的反序列化对 JSON 对象是**宽松**的，传一堆陌生参数
/// 进去也照常成功。这个空结构体配 `deny_unknown_fields` 才能把多余参数挡下来。
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NoArgs {}

/// 连接方式。
///
/// 只认 schema 里那两个名字。`scope_device::TransportKind::parse` 还接受
/// `simulator` / `uart` / `port` 这些别名（给人用的 CLI 走那条路），
/// 但 MCP 这层要严格对齐 schema —— 否则 schema 又成了一句空话。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum TransportArg {
    /// 串口真机
    Serial,
    /// 内置模拟器（不需要硬件）
    Sim,
}

/// 模拟器波形场景。
///
/// 合法值直接来自 [`scope_sim::Scenario`] —— schema 的 `enum`、
/// 反序列化、错误提示里的清单全是同一份，加场景时不可能只改一边。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioArg(pub Scenario);

impl From<ScenarioArg> for Scenario {
    fn from(s: ScenarioArg) -> Scenario {
        s.0
    }
}

impl<'de> Deserialize<'de> for ScenarioArg {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        Scenario::parse(&s).map(ScenarioArg).ok_or_else(|| {
            de::Error::custom(format!(
                "未知场景 {s:?}；可选：{}",
                Scenario::all_names().collect::<Vec<_>>().join(" / ")
            ))
        })
    }
}

impl JsonSchema for ScenarioArg {
    fn schema_name() -> Cow<'static, str> {
        "SimScenario".into()
    }

    fn schema_id() -> Cow<'static, str> {
        concat!(module_path!(), "::ScenarioArg").into()
    }

    fn json_schema(_gen: &mut SchemaGenerator) -> Schema {
        json_schema!({
            "type": "string",
            "enum": Scenario::all_names().collect::<Vec<_>>(),
            "description": "模拟器波形场景。清单与 scope_sim::Scenario::parse 同源",
        })
    }
}

/// 采集方式。
///
/// `stream` 在 schema 里出现是**有意的**：固件侧 STREAMING 还没实现，
/// 与其在 schema 里假装它不存在（客户端传了会撞上一句生硬的结构错误），
/// 不如收下来，然后给一句「尚未实现，要连续观察请用 scope_watch」。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum AcqMode {
    /// 单次采集
    Single,
    /// 连续流 —— **尚未实现**
    Stream,
}

/// 采样格式。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum SampleFormat {
    /// 原始 12-bit 样点，每点 2 字节
    Raw,
    /// 12-bit 打包，每点 1.5 字节
    Packed12,
    /// 每桶最大值/最小值
    Minmax,
}

/// 触发模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum TriggerMode {
    /// 约 200 ms 无触发也强制完成一次 —— 否则 Agent 会永久阻塞
    Auto,
    /// 一直等到真正触发
    Normal,
    /// 触发一次后回到空闲
    Single,
}

/// 触发边沿。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum TriggerEdge {
    /// 上升沿
    Rising,
    /// 下降沿
    Falling,
}

/// 通道耦合（对齐 `proto/protocol.h` 的 `coupling_t`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Coupling {
    /// 直流耦合
    Dc,
    /// 交流耦合
    Ac,
}

impl Coupling {
    /// 协议里的取值（`proto/protocol.h` 的 `coupling_t`）。
    ///
    /// 第一版没有它 —— 当时 `SET_CHANNEL` 还没接线，留着会是死代码。
    /// 现在接了，它就有了使用者。
    pub fn as_u8(self) -> u8 {
        match self {
            Coupling::Dc => 0,
            Coupling::Ac => 1,
        }
    }

    /// JSON 里的名字。
    pub fn name(self) -> &'static str {
        match self {
            Coupling::Dc => "dc",
            Coupling::Ac => "ac",
        }
    }
}

impl ReadFormat {
    /// 实际返回的格式名。
    pub fn name(self) -> &'static str {
        match self {
            ReadFormat::Raw => "raw",
        }
    }
}

impl TriggerMode {
    /// JSON / 协议文档里的名字。穷尽 match —— 加变体编译不过。
    pub fn name(self) -> &'static str {
        match self {
            TriggerMode::Auto => "auto",
            TriggerMode::Normal => "normal",
            TriggerMode::Single => "single",
        }
    }
}

impl TriggerEdge {
    /// JSON / 协议文档里的名字。
    pub fn name(self) -> &'static str {
        match self {
            TriggerEdge::Rising => "rising",
            TriggerEdge::Falling => "falling",
        }
    }
}

impl SaveFormat {
    /// 实际写出的格式名。
    pub fn name(self) -> &'static str {
        match self {
            SaveFormat::Csv => "csv",
        }
    }
}

/// 可选的测量指标。
///
/// 从前是一组字符串加手工比对 —— 那时 schema 与实现各写一份清单。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum MetricKind {
    /// 峰峰值（V）
    Vpp,
    /// 最小值（V）
    Min,
    /// 最大值（V）
    Max,
    /// 平均值（V）
    Mean,
    /// 交流有效值（V）
    Rms,
    /// 频率（Hz），已排除事务间空闲
    Freq,
    /// 占空比（%）
    Duty,
    /// 上升时间（ns）
    Rise,
    /// 下降时间（ns）—— 与上升时间对称
    Fall,
    /// 过冲（%）。参考电平是**平台均值**，不是全窗最大值；非双电平信号测不出
    Overshoot,
    /// 下冲（%）。同过冲
    Undershoot,
    /// 平均高电平脉宽（ns）。周期测不出时（孤立脉冲）仍有值
    High,
    /// 跃变总数（实测中点处的上升沿 + 下降沿）。
    /// ⚠ 与 `scope_capture` 摘要里的 `rising_edges` **不同源**：那个固定在
    /// 2048 LSB 判定，这个用实测中点 —— 信号有直流偏置时两者差很多
    Edges,
}

// ══════════════════════════════════════════════════════════════
// 各工具的参数
// ══════════════════════════════════════════════════════════════

/// `scope_connect`
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConnectArgs {
    /// 串口号，例如 "COM7"。给了它而没给 transport 时按串口处理。
    /// **transport 显式写了 "serial" 时这个字段必填**（schema 表达不了这条联合约束）
    pub port: Option<String>,
    /// 串口波特率，默认 921600
    #[serde(default = "defaults::baud")]
    #[schemars(range(min = 1))]
    pub baud: Option<u32>,
    /// 连接方式。省略时：给了 port 当串口，否则用内置模拟器
    pub transport: Option<TransportArg>,
    /// 模拟器波形场景，**省略则用 i2c_100k**（2 通道 —— 这是唯一能验证
    /// I2C 解码的场景，所以选它当默认）
    pub sim_scenario: Option<ScenarioArg>,
}

/// 触发配置。
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TriggerArgs {
    /// 触发模式。**省略则沿用设备当前值**
    pub mode: Option<TriggerMode>,
    /// 触发边沿。**省略则沿用设备当前值**
    pub edge: Option<TriggerEdge>,
    /// 触发电平，ADC LSB 整数（12-bit）。与 level_v 二选一，都给我时以本字段为准。
    /// **省略则沿用设备当前值**
    #[schemars(range(min = 0, max = 4095))]
    pub level_lsb: Option<u64>,
    /// 触发电平，伏特。仅在没给 level_lsb 时使用
    pub level_v: Option<f64>,
    /// 触发源。**省略则沿用设备当前值**
    ///
    /// 取值来自 `proto/protocol.h` 的 `trig_src_t`：`0`=CH1 / `1`=CH2 /
    /// `2`=软件触发。**不是 0..=255** —— 从前用 u8，`3` 与 `255` 会被
    /// 静默收下并原样下发，设备收到一个未定义的值。
    #[schemars(range(min = 0, max = 2))]
    pub source: Option<u64>,
}

/// 通道配置。
///
/// ⚠ **尚未接线上层**：字段按 `SET_CHANNEL` 的 payload 定义，但
/// `scope_configure` 目前只把它们回显在警告里。定义出来是为了让 schema
/// 说真话 —— 之前这里是裸的 `{"type":"object"}`，客户端根本不知道能填什么。
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ChannelArgs {
    /// 通道号，从 0 起算，默认 0
    #[schemars(range(min = 0, max = 255))]
    pub ch: Option<u64>,
    /// 是否使能该通道
    pub enable: Option<bool>,
    /// 量程档位索引
    #[schemars(range(min = 0, max = 255))]
    pub range_idx: Option<u64>,
    /// 耦合方式
    pub coupling: Option<Coupling>,
    /// 垂直偏移，ADC LSB 整数（i16，负值合法 —— 把波形挪进屏幕）
    pub offset_lsb: Option<i16>,
}

/// `scope_configure`
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConfigureArgs {
    /// 请求采样率（Hz）。会被量化到定时器能达到的档位，**以回显的实际值为准**
    #[schemars(range(min = 1, max = MAX_SAMPLE_RATE_HZ))]
    pub sample_rate_hz: Option<u64>,
    /// 采集点数。上限是 8 KB 环按 u16 折半的结果
    #[schemars(range(min = 1, max = MAX_CAPTURE_SAMPLES))]
    pub capture_samples: Option<u64>,
    /// 采集方式
    pub mode: Option<AcqMode>,
    /// 采样格式
    pub format: Option<SampleFormat>,
    /// 抽点倍数（MINMAX 的桶大小）
    #[schemars(range(min = 1, max = MAX_DECIMATION))]
    pub decimation: Option<u64>,
    /// 触发配置。字段是 `SET_TRIGGER` 的子集，**省略的子项沿用设备当前值**
    /// （与下面的采集三件套同一套语义）
    pub trigger: Option<TriggerArgs>,
    /// 通道配置（尚未实现，见 [`ChannelArgs`]）
    pub channel: Option<ChannelArgs>,
}

/// `scope_capture`
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CaptureArgs {
    /// 采集方式。`stream` 尚未实现，传了会给你一句解释而不是静默按单次跑
    pub mode: Option<AcqMode>,
    /// 等待触发的超时（ms），默认 2000，**上限 [`MAX_TIMEOUT_MS`]**。
    /// 超过上限会被拒绝 —— 给一个天文数字会让调用永久挂住而不是报错
    #[serde(default = "defaults::timeout_ms")]
    #[schemars(range(min = 1, max = MAX_TIMEOUT_MS))]
    pub timeout_ms: Option<u64>,
    /// 预览点数上限，默认 256，**硬顶也是 256**（三层 token 防护第一层）
    #[serde(default = "defaults::preview_points")]
    #[schemars(range(min = 1, max = 256))]
    pub max_preview_points: Option<u64>,
}

/// `scope_read_waveform` 的采样格式。
///
/// 只有一个变体是**故意的**：`READ_BUFFER` 回的就是原始 12-bit 样点。
/// 要落盘成别的格式去用 `scope_save_capture`。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ReadFormat {
    /// 原始 12-bit 样点
    Raw,
}

/// `scope_read_waveform`
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ReadWaveformArgs {
    /// 采集 id，来自 scope_capture / scope_list_captures
    #[schemars(range(min = 0, max = 65535))]
    pub capture_id: u64,
    /// 起始样点下标，从 0 起算，默认 0。**超出采集长度不是错误** ——
    /// 会返回一个空页（这样分页循环不必自己判断结尾）
    #[serde(default = "defaults::zero")]
    pub start_sample: Option<u64>,
    /// 本次取多少点。硬顶 4096；超出**拒绝**而不是静默截断。
    /// 省略时：若给了 `max_points` 就用它，两个都没给才是 512
    #[schemars(range(min = 1, max = 4096))]
    pub count: Option<u64>,
    /// `count` 的旧名，仍然兼容。两个都给时以 `count` 为准
    #[schemars(range(min = 1, max = 4096))]
    pub max_points: Option<u64>,
    /// 采样格式，目前只支持 raw
    pub format: Option<ReadFormat>,
    /// 通道号，默认 0
    #[serde(default = "defaults::zero")]
    pub channel: Option<u64>,
}

/// `scope_measure`
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeasureArgs {
    /// 采集 id
    #[schemars(range(min = 0, max = 65535))]
    pub capture_id: u64,
    /// 只算这些基础指标，省略则全算。瞬态与 TIE 分析不受此过滤，始终附带。
    /// **名字拼错会报错**，不会静默少算一项
    pub metrics: Option<Vec<MetricKind>>,
    /// 只测这个通道，省略则每个通道都测。
    /// **给一个不存在的通道会报错**，不是回一份空测量结果
    #[schemars(range(min = 0, max = 255))]
    pub channel: Option<u64>,
}

/// `scope_i2c_decode`
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct I2cDecodeArgs {
    /// 采集 id
    #[schemars(range(min = 0, max = 65535))]
    pub capture_id: u64,
    /// SCL 通道号。`scl_channel` / `sda_channel` 要**么都给要么都不给**，
    /// 都不给时按边沿密度自动判定（SCL 的边沿更密）
    #[schemars(range(min = 0, max = 255))]
    pub scl_channel: Option<u64>,
    /// SDA 通道号
    #[schemars(range(min = 0, max = 255))]
    pub sda_channel: Option<u64>,
    /// 判为「高」的门限，ADC LSB 整数。与 vil_lsb 可以单独给
    #[schemars(range(min = 0, max = 4095))]
    pub vih_lsb: Option<u64>,
    /// 判为「低」的门限，ADC LSB 整数
    #[schemars(range(min = 0, max = 4095))]
    pub vil_lsb: Option<u64>,
    /// 去抖窗口（ns），默认 50。**0 是合法的** —— 表示不去抖。
    /// 上限见 [`defaults::MAX_DEBOUNCE_NS`]：窗口开到跟时钟周期一个量级，
    /// 真实边沿就会被当毛刺滤掉，而解码器仍会报 `trustworthy`
    #[serde(default = "defaults::debounce_ns")]
    #[schemars(range(min = 0, max = defaults::MAX_DEBOUNCE_NS))]
    pub debounce_ns: Option<u64>,
}

/// `scope_decode` 的协议。
///
/// 与 core 的 `scope_core::decoder::Protocol` 对应，但 schema 生成在
/// 本 crate（core 不依赖 schemars）——枚举值是小写字符串，两个名字
/// 一一对应，改一处必须改另一处（有测试锁住合法值）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ProtocolArg {
    /// I²C 总线（需 2 通道；参数在 `i2c` 对象里）
    I2c,
    /// UART 串行异步，8N1（需 1 通道；参数在 `uart` 对象里）
    Uart,
}

/// `scope_decode` 的 UART 参数（`protocol = "uart"` 时生效）。
///
/// 只暴露 `baud` —— 帧格式固定 8N1，未实现的组合不该出现在 schema 里
/// 让 Agent 以为可用。
#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UartParams {
    /// 波特率，默认 115200。**没有「自动」**：估错波特率会解出看似合理
    /// 但全错的字节，自动估计未实现也不猜 —— 客户端必须显式确认波特率
    #[serde(default = "defaults::decode_baud")]
    #[schemars(range(min = 1, max = 4_000_000))]
    pub baud: Option<u64>,
}

/// `scope_decode` 的 I²C 参数（`protocol = "i2c"` 时生效）。
///
/// 语义与 [`I2cDecodeArgs`] 同一套（那是 `scope_i2c_decode` 的别名路径），
/// 只是去掉了 `capture_id` —— 它在顶层。
#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct I2cParams {
    /// SCL 通道号。与 `sda_channel` 要**么都给要么都不给**，
    /// 都不给时按边沿密度自动判定（SCL 的边沿更密）
    #[schemars(range(min = 0, max = 255))]
    pub scl_channel: Option<u64>,
    /// SDA 通道号
    #[schemars(range(min = 0, max = 255))]
    pub sda_channel: Option<u64>,
    /// 判为「高」的门限，ADC LSB 整数。与 `vil_lsb` 可以单独给
    #[schemars(range(min = 0, max = 4095))]
    pub vih_lsb: Option<u64>,
    /// 判为「低」的门限，ADC LSB 整数
    #[schemars(range(min = 0, max = 4095))]
    pub vil_lsb: Option<u64>,
    /// 去抖窗口（ns），默认 50。**0 是合法的** —— 表示不去抖
    #[serde(default = "defaults::debounce_ns")]
    #[schemars(range(min = 0, max = defaults::MAX_DEBOUNCE_NS))]
    pub debounce_ns: Option<u64>,
}

/// `scope_decode`
///
/// `protocol = "i2c"` 时给 `i2c` 对象（给了 `uart` 会被拒，反之亦然 ——
/// 互斥参数显式报错，比静默忽略更能暴露 Agent 的调用错误）。
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DecodeArgs {
    /// 采集 id，来自 scope_capture / scope_list_captures
    #[schemars(range(min = 0, max = 65535))]
    pub capture_id: u64,
    /// 解码协议
    pub protocol: ProtocolArg,
    /// 显式通道号（0 起）。省略时：UART 用通道 0；I²C 忽略本字段
    /// （用 `i2c.scl_channel` / `i2c.sda_channel`）
    pub channels: Option<Vec<u64>>,
    /// UART 参数（`protocol = "uart"` 时生效）
    pub uart: Option<UartParams>,
    /// I²C 参数（`protocol = "i2c"` 时生效）
    pub i2c: Option<I2cParams>,
}

/// `scope_import_capture`
///
/// 从 CSV 导入一次采集到本会话（**不连设备**）。导入后可直接用
/// `scope_measure` / `scope_decode` 引用返回的 `capture_id`。
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportCaptureArgs {
    /// CSV 路径，例如 "./captures/wave.csv"
    pub path: String,
    /// 显式采样率（Hz）。**首行 `# rate_hz=` 优先**，其次本字段，
    /// 最后才由时间列反推（且只在能精确反推时才接受）
    #[schemars(range(min = 1u64, max = 4_294_967_295u64))]
    pub rate_hz: Option<u64>,
    /// 满量程电压（V），默认 3.3。与 `offset_v` 都不给时，按当前标定窗口的
    /// 换算解释电压列（本项目导出文件是双极性的，0 V 在码值 2048）
    #[schemars(range(min = 0.001, max = 100.0))]
    pub full_scale_v: Option<f64>,
    /// 码值 0 对应的输入电压（V），默认 0（单极性 0..3.3 V）
    pub offset_v: Option<f64>,
    /// 电压列也按原始码值解析（`CH1V` 不换算）
    pub raw: Option<bool>,
    /// 时间原点不为零时归一化（否则拒绝）
    pub normalize_t0: Option<bool>,
}

/// `scope_save_capture` 的导出格式。
///
/// 从前 schema 宣称支持 npy / bin 而实现一律写 CSV —— 静默落错格式比报错
/// 危险得多，Agent 会拿一个 CSV 当二进制去解析。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum SaveFormat {
    /// 逗号分隔文本
    Csv,
}

/// `scope_save_capture`
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SaveCaptureArgs {
    /// 采集 id
    #[schemars(range(min = 0, max = 65535))]
    pub capture_id: u64,
    /// 写到哪，例如 "./captures/cap1.csv"
    pub path: String,
    /// 导出格式，目前只有 csv
    pub format: Option<SaveFormat>,
}

/// `scope_watch`
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WatchArgs {
    /// 观察多久（ms），默认 1000，**上限 [`MAX_TIMEOUT_MS`]**。
    /// 超过上限会被拒绝 —— 给一个天文数字会让调用永久挂住而不是报错
    #[serde(default = "defaults::watch_ms")]
    #[schemars(range(min = 1, max = MAX_TIMEOUT_MS))]
    pub duration_ms: Option<u64>,
    /// 第一次采集的预览点数上限，默认 128，硬顶 256
    #[serde(default = "defaults::watch_points")]
    #[schemars(range(min = 1, max = 256))]
    pub max_points: Option<u64>,
}

/// 故障注入配置。
///
/// 字段与 [`scope_sim::FaultInjection`] 一一对应。**整体替换**当前配置 ——
/// 省略的字段按关闭处理，所以 `inject: {}` 就是「清除全部故障」。
#[derive(Debug, Clone, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InjectArgs {
    /// 每 N 个帧丢弃一个（0 = 关闭）。验证命令层的重试
    pub drop_every_n_frames: Option<u32>,
    /// 每 N 个帧破坏一个的 CRC（0 = 关闭）
    pub crc_err_every_n_frames: Option<u32>,
    /// 随机延迟尖峰的概率
    #[schemars(range(min = 0.0, max = 1.0))]
    pub latency_spike_probability: Option<f64>,
    /// 延迟尖峰的时长（ms），上限 [`defaults::MAX_LATENCY_SPIKE_MS`]。
    /// **超过上限会被拒绝** —— 这个值会原样交给 `thread::sleep`，
    /// 而 MCP server 是单线程，一个天文数字就能把它永久睡死
    #[schemars(range(min = 0, max = defaults::MAX_LATENCY_SPIKE_MS))]
    pub latency_spike_ms: Option<u64>,
    /// 永不触发 —— 验证超时路径不会卡死
    pub no_trigger: Option<bool>,
    /// 制造一次采集溢出
    pub force_overrun: Option<bool>,
}

impl InjectArgs {
    /// 转成模拟器的故障配置。
    ///
    /// **整体替换**：没给的字段一律按关闭处理。不做合并是有意的 ——
    /// 合并会让「我到底还留着哪些故障」变成一个要翻历史才知道的问题，
    /// 而排查一个被注入过故障的采集时，这正是最不该靠猜的东西。
    ///
    /// 概率越界会被**拒绝**。回归：schema 里写着 `0.0..=1.0`，但
    /// `range()` 只是注解、serde 不校验，于是 `-3.0` 一路走到
    /// `roll < probability` —— 恒假，**等于把故障静默清掉**，而响应还说
    /// `faults_clean: true`，看不出是「你没注入」还是「你的参数被吃掉了」。
    pub fn into_faults(self) -> Result<FaultInjection, String> {
        let p = self.latency_spike_probability.unwrap_or(0.0);
        if !(0.0..=1.0).contains(&p) {
            return Err(format!(
                "latency_spike_probability={p} 超出范围（0.0..=1.0）"
            ));
        }
        // `latency_spike_ms` 与概率一样要**封顶**。它会被原样交给
        // `sim/device.rs` 的 `std::thread::sleep`，而 `serve()` 是单线程 ——
        // u64::MAX 毫秒＝永久睡死，且之后每一次工具调用都会再睡一次。
        // 这是和 `MAX_TIMEOUT_MS` 完全同类的一个洞，从故障注入这个门漏回来。
        let ms = self.latency_spike_ms.unwrap_or(0);
        if ms > defaults::MAX_LATENCY_SPIKE_MS {
            return Err(format!(
                "latency_spike_ms={ms} 超出范围（0..={}）",
                defaults::MAX_LATENCY_SPIKE_MS
            ));
        }
        Ok(FaultInjection {
            drop_every_n_frames: self.drop_every_n_frames.unwrap_or(0),
            crc_err_every_n_frames: self.crc_err_every_n_frames.unwrap_or(0),
            latency_spike_probability: p,
            latency_spike_ms: ms,
            no_trigger: self.no_trigger.unwrap_or(false),
            force_overrun: self.force_overrun.unwrap_or(false),
        })
    }
}

/// `scope_sim_set_scenario`
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SimSetScenarioArgs {
    /// 要切到的场景
    pub scenario: ScenarioArg,
    /// 波形随机种子。**只对 `noise` 有影响** —— 其余场景要么是纯解析式，
    /// 要么（`pulse_glitch`）刻意做成确定性的，好让 minmax 预览里那个
    /// 尖峰落在可断言的位置。省略则保持当前种子，而默认种子是固定的，
    /// 所以同一场景反复跑结果一致
    pub seed: Option<u64>,
    /// 故障注入配置。**整体替换**，省略的字段按关闭处理
    pub inject: Option<InjectArgs>,
}

/// `scope_debug_raw`
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DebugRawArgs {
    /// 命令码，见 proto/protocol.h 的命令表
    #[schemars(range(min = 0, max = 65535))]
    pub cmd: u64,
    /// 十六进制字节，空格分隔，例如 "11 22 33"。省略即空 payload。
    /// 每个 token 接受 1~2 位、可带 `0x` 前缀（"F" 与 "0x0F" 等价）
    pub payload_hex: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    /// 走**和 `tools/list` 完全同一条**生成路径，不另开小灶。
    fn schema_json<T: JsonSchema>() -> Value {
        schemars::schema_for!(T).into()
    }

    #[test]
    fn scenario_enum_comes_from_the_simulator() {
        // 这条卡住「schema 给的场景清单」与「解析认的场景」是同一份。
        let s = schema_json::<ScenarioArg>();
        let listed: Vec<&str> = s["enum"]
            .as_array()
            .expect("ScenarioArg 的 schema 应当有 enum")
            .iter()
            .filter_map(|v| v.as_str())
            .collect();
        assert_eq!(
            listed,
            Scenario::all_names().collect::<Vec<_>>(),
            "MCP 的 sim_scenario 清单与 scope_sim::Scenario 分家了"
        );
        // 而且列出的每一个都真能解析
        for name in &listed {
            let got: ScenarioArg = serde_json::from_value(json!(name)).unwrap_or_else(|e| {
                panic!("schema 列了 {name:?}，但反序列化不认：{e}");
            });
            assert_eq!(Scenario::from(got).name(), *name);
        }
    }

    #[test]
    fn enums_reject_values_the_schema_does_not_list() {
        for bad in ["i2c_999k", "I2C_100K", ""] {
            assert!(
                serde_json::from_value::<ScenarioArg>(json!(bad)).is_err(),
                "{bad:?} 不该被接受"
            );
        }
        assert!(serde_json::from_value::<TransportArg>(json!("bogus")).is_err());
        assert!(serde_json::from_value::<TriggerEdge>(json!("both")).is_err());
        assert!(serde_json::from_value::<MetricKind>(json!("freqency")).is_err());
    }

    #[test]
    fn unknown_fields_are_rejected_not_ignored() {
        // 这是整个重构要防的那一类：字段名拼错时**从前是静默走默认值**，
        // 工具照常返回成功，Agent 拿到的是它没要求的配置下的数据。
        let e = serde_json::from_value::<I2cDecodeArgs>(json!({
            "capture_id": 1,
            "scl_chanel": 0
        }))
        .unwrap_err();
        assert!(
            e.to_string().contains("scl_chanel"),
            "错误要点名拼错的字段，实测 {e}"
        );

        // 不吃参数的工具也不能对多余参数照单全收
        assert!(serde_json::from_value::<NoArgs>(json!({ "whatever": 1 })).is_err());
        // 但空对象要能过
        assert!(serde_json::from_value::<NoArgs>(json!({})).is_ok());
    }

    #[test]
    fn required_fields_are_required() {
        let e = serde_json::from_value::<ReadWaveformArgs>(json!({ "count": 8 })).unwrap_err();
        assert!(e.to_string().contains("capture_id"), "实测 {e}");

        let e = serde_json::from_value::<SimSetScenarioArgs>(json!({ "seed": 3 })).unwrap_err();
        assert!(e.to_string().contains("scenario"), "实测 {e}");
    }

    #[test]
    fn deny_unknown_fields_shows_up_in_the_schema() {
        // schema 与实现必须说同一件事：`deny_unknown_fields` 让反序列化拒绝
        // 陌生字段，那么 schema 里就该有 additionalProperties: false，
        // 好让客户端提前知道，而不是发了才被拒。
        assert_eq!(
            schema_json::<ConnectArgs>()["additionalProperties"],
            json!(false),
            "deny_unknown_fields 应当反映到 schema 上"
        );
        assert_eq!(
            schema_json::<NoArgs>()["additionalProperties"],
            json!(false)
        );
    }

    #[test]
    fn ranges_are_advertised_in_the_schema() {
        // 上界写进 schema，客户端提前就能看见；实现里仍会再校验一次。
        //
        // ⚠ 断言里**故意写死字面量**，而不是 `json!(MAX_CAPTURE_SAMPLES)` ——
        // 拿常量跟自己比，常量改成什么测试都过（断言两侧是同一个符号），
        // 等于没测。写成字面量之后，改常量会让这条红，逼你确认「物理上限
        // 真的变了吗，还是只是改了名字」。
        let t = schema_json::<TriggerArgs>();
        assert_eq!(t["properties"]["level_lsb"]["minimum"], json!(0));
        assert_eq!(t["properties"]["level_lsb"]["maximum"], json!(4095));

        let c = schema_json::<ConfigureArgs>();
        assert_eq!(c["properties"]["capture_samples"]["minimum"], json!(1));
        assert_eq!(c["properties"]["capture_samples"]["maximum"], json!(4096));
        assert_eq!(c["properties"]["decimation"]["maximum"], json!(256));
        assert_eq!(c["properties"]["sample_rate_hz"]["maximum"], json!(857_142));
        // 这两个常量是 schema 与实现的共同来源，顺手确认它们没被改成别的
        assert_eq!(MAX_CAPTURE_SAMPLES, 4096);
        assert_eq!(MAX_SAMPLE_RATE_HZ, 857_142);

        // 触发源不是 0..=255 —— 它来自 `trig_src_t`，只有三个合法值。
        // 注释删掉不会红，但**上界改了会**。
        assert_eq!(t["properties"]["source"]["maximum"], json!(2));

        // 去抖窗口的上界：接近时钟周期会把真实边沿当毛刺滤掉
        assert_eq!(
            schema_json::<I2cDecodeArgs>()["properties"]["debounce_ns"]["maximum"],
            json!(1_000)
        );

        // 延迟尖峰时长：会被原样交给 thread::sleep，不封顶就睡死单线程 server
        let inj = &schema_json::<SimSetScenarioArgs>()["$defs"]["InjectArgs"]["properties"];
        assert_eq!(inj["latency_spike_ms"]["maximum"], json!(5_000));
        assert_eq!(inj["latency_spike_probability"]["maximum"], json!(1.0));
    }

    #[test]
    fn doc_comments_become_descriptions() {
        // 字段说明是 LLM 判断怎么填参数的主要依据，不能悄悄丢掉。
        let c = schema_json::<ConfigureArgs>();
        let d = c["properties"]["sample_rate_hz"]["description"]
            .as_str()
            .unwrap_or("");
        assert!(d.contains("量化"), "字段说明没进 schema，实测 {d:?}");
    }

    #[test]
    fn inject_defaults_to_a_clean_slate() {
        // `inject: {}` 的语义是「清除全部故障」，不是「什么都不做」——
        // 之前这个字段**根本没人读**，传了也返回成功，工具说明却写着「注入故障」。
        let got: InjectArgs = serde_json::from_value(json!({})).unwrap();
        let clean = got.into_faults().expect("空 inject 不该报错");
        assert!(clean.is_clean(), "空 inject 应当等于清除全部故障");

        let got: InjectArgs = serde_json::from_value(json!({ "no_trigger": true })).unwrap();
        let f = got.into_faults().unwrap();
        assert!(f.no_trigger);
        assert!(!f.is_clean());
        // 没给的字段一律按关闭处理 —— 整体替换，不做合并
        assert_eq!(f.drop_every_n_frames, 0);
    }

    #[test]
    fn inject_rejects_a_probability_outside_the_declared_range() {
        // 回归：schema 里写着 `0.0..=1.0`，但 `range()` 只是注解、serde 不校验，
        // 于是 `-3.0` 一路走到 `roll < probability` —— 恒假，**等于把故障
        // 静默清掉**，而响应还说 `faults_clean: true`，看不出是「你没注入」
        // 还是「你的参数被吃掉了」。`2.5` 则是「每 7 帧必延迟」，也不是
        // 它字面上的意思。
        // 只测有限值：JSON 里没有 NaN/Infinity，`json!` 会把它们写成 `null`，
        // 而 `null` 在 `Option<f64>` 上意味着「没给」—— 这是**构造不出**的输入，
        // 不是漏测。
        for bad in [-3.0f64, 2.5, 1.000_001, -0.000_001, f64::MAX] {
            let got: InjectArgs =
                serde_json::from_value(json!({ "latency_spike_probability": bad })).unwrap();
            assert!(
                got.into_faults().is_err(),
                "latency_spike_probability={bad} 应当被拒绝"
            );
        }
        for ok in [0.0f64, 0.5, 1.0] {
            let got: InjectArgs =
                serde_json::from_value(json!({ "latency_spike_probability": ok })).unwrap();
            assert!(got.into_faults().is_ok(), "{ok} 在范围内，应当被接受");
        }
    }

    #[test]
    fn timeouts_are_bounded_in_the_schema() {
        // 上界必须写进 schema，客户端提前就能看见 —— 这不是「越大越灵活」，
        // 超过上限会让调用永久挂住（详见 session.rs 的 check_timeout）。
        let c = schema_json::<CaptureArgs>();
        assert_eq!(
            c["properties"]["timeout_ms"]["maximum"],
            json!(MAX_TIMEOUT_MS),
            "timeout_ms 的上界就是 core 里那个常量，不是另抄一个数字"
        );
        let w = schema_json::<WatchArgs>();
        assert_eq!(
            w["properties"]["duration_ms"]["maximum"],
            json!(MAX_TIMEOUT_MS)
        );
        assert_eq!(w["properties"]["duration_ms"]["minimum"], json!(1));
    }

    #[test]
    fn fixed_defaults_show_up_in_the_schema() {
        // 回归：改成类型化参数时**丢掉了手写 schema 里全部 7 处 `default`**。
        // 客户端/LLM 主要靠这个关键字知道「省略参数会发生什么」，
        // 只剩中文散文兜底是不够的。
        //
        // ⚠ 期望值写**字面量**，不写 `json!(defaults::XXX)`：
        // 拿常量跟自己比的话，把常量的值改掉测试照样绿，等于没测。
        // 写死之后改常量会让这条红，逼你确认「默认值真的该变吗」。
        let cases: [(Value, &str, Value); 8] = [
            (schema_json::<ConnectArgs>(), "baud", json!(921_600)),
            (schema_json::<CaptureArgs>(), "timeout_ms", json!(2_000)),
            (
                schema_json::<CaptureArgs>(),
                "max_preview_points",
                json!(256),
            ),
            (schema_json::<I2cDecodeArgs>(), "debounce_ns", json!(50)),
            (schema_json::<WatchArgs>(), "duration_ms", json!(1_000)),
            (schema_json::<WatchArgs>(), "max_points", json!(128)),
            // 这两个是第一版漏掉的（「漏掉两处」正是被验证抓出来的）
            (schema_json::<ReadWaveformArgs>(), "start_sample", json!(0)),
            (schema_json::<ReadWaveformArgs>(), "channel", json!(0)),
        ];
        for (schema, field, want) in cases {
            assert_eq!(
                schema["properties"][field]["default"], want,
                "{field} 的 default 没进 schema"
            );
        }
    }

    #[test]
    fn read_waveform_count_deliberately_has_no_serde_default() {
        // `count` 缺省时用的是 `max_points`（旧名），两个都缺省才是 512。
        // 给它一个 serde 默认值会让 `count` 永远有值，
        // `session.rs` 里 `p.count.or(p.max_points)` 那条兼容路径就再也走不到 ——
        // 这是「补 default」时最容易踩的坑。
        assert!(schema_json::<ReadWaveformArgs>()["properties"]["count"]
            .get("default")
            .is_none());
        // 而它确实有默认值，只是由实现决定，所以必须写在说明里
        let d = schema_json::<ReadWaveformArgs>()["properties"]["count"]["description"]
            .as_str()
            .unwrap_or("")
            .to_string();
        assert!(
            d.contains("512"),
            "count 的说明里要说清缺省点数，实测 {d:?}"
        );
    }
}
