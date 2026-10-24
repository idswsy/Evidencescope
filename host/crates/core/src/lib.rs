//! # scope-core —— 命令层
//!
//! CLI / GUI / MCP 都只依赖这个 crate，不关心背后是真硬件还是模拟器。
//!
//! ```no_run
//! use scope_core::{CommandBus, DevicePort, CaptureStore};
//!
//! # fn demo<P: DevicePort>(port: P) -> Result<(), Box<dyn std::error::Error>> {
//! let mut bus = CommandBus::new(port);
//! let info = bus.connect()?;                       // GET_INFO + GET_CONFIG
//! eprintln!("设备: {} 通道, 上限 {} Hz", info.ch_count, info.rate_max_hz);
//!
//! let actual = bus.set_sample_rate(857_143)?;      // 设备回显量化后的值
//! bus.set_trigger(1, 0, 0, 2048, 2048, 1000)?;     // normal / ch0 / 上升沿
//! bus.set_acq(0, 4096, 0, 1)?;                     // 单次 / 4096 点 / RAW16
//! bus.arm()?;
//! # Ok(())
//! # }
//! ```
//!
//! ## 分层
//!
//! ```text
//! CLI / MCP / GUI            ← 只依赖本 crate
//!      ↓
//! CommandBus                 ← 命令层：编码 / seq / 重试 / 状态缓存
//!      ↓  DevicePort (trait)  ← 唯一的解耦点
//! UART │ USB CDC │ 模拟器 │ TCP
//! ```

#![deny(clippy::all)]
#![warn(missing_docs)]

pub mod acquire;
pub mod analysis;
pub mod calib;
pub mod capture;
pub mod command;
pub mod decoder;
pub mod device;
mod edges;
pub mod error;
pub mod i2c_decode;
pub mod import;
pub mod measure;
pub mod persist;
pub mod report;
pub mod signal;
pub mod tie;
pub mod transient;

pub use acquire::{acquire, acquire_cancellable, AcquireParams, TriggerEvent};
pub use analysis::{
    quantity_label, AnalysisReport, CalibrationState, ChannelAnalysis, EvidenceContext, Provenance,
    Quality, RefuseReason, Source, Verdict, ANALYSIS_SCHEMA,
};
pub use calib::{
    decode_uid, encode_uid, is_placeholder_uid, CalibrationFile, CalibrationStore, DeviceCalib,
    ScaleSet, CALIB_VERSION,
};
pub use capture::{
    Capture, CaptureSource, CaptureStore, ChannelScale, ChannelSummary, MinMaxPreview,
    DEFAULT_HISTORY,
};
pub use command::{
    state_name, CommandBus, DeviceConfig, DeviceInfo, Response, RetryPolicy, TIMEOUT_CONTROL,
    TIMEOUT_PING, TIMEOUT_TRIGGER,
};
// ⚠ 这里不重导出 `decoder::decode` 与 `decoder::Frame`：`i2c_decode::decode` 与
// `scope_proto::Frame` 已占用根上的名字（E0252）。解码入口用全路径
// `scope_core::decoder::decode`，解码帧类型用 `scope_core::decoder::Frame`。
pub use decoder::{
    to_decoded, DecodeError, DecodeParams, DecodeQuality, DecodeRequest, DecodeWarning,
    DecodedFrames, Parity, Protocol, UartConfig, UartFrame,
};
pub use device::{DevicePort, FrameReader};
pub use error::{DeviceError, LinkError, Result, ScopeError};
pub use i2c_decode::{
    decode, decode_capture, detect_channels, Address, Event, EventKind, I2cDecode, I2cDecodeConfig,
    I2cWarning, Levels, SignalQuality, Transaction,
};
pub use import::{
    from_csv, ImportError, ImportOptions, ImportedCapture, RateSource, DEFAULT_FULL_SCALE_V,
    IMPORT_ID_BASE, IMPORT_ID_LIMIT, MAX_IMPORT_CHANNELS,
};
pub use measure::{measure, Measurements};
pub use report::{build_evidence, EvidenceInput, MAX_CHARS, MAX_FRAMES, PREVIEW_POINTS};
pub use signal::{
    detect_levels, Classification, LevelStats, LevelVerdict, NotLevelReason, SignalKind,
    UncertainReason,
};
pub use tie::{TieAnalysis, TieMethod, TieReason, TieStats};
pub use transient::{TransientAnalysis, TransientReason, TransientStats};

// 重导出协议类型，让上层不必直接依赖 scope-proto
pub use scope_proto::{
    cmd_allowed, ChunkHeader, Cmd, ErrorCode, Frame, Header, Severity, State, CAPTURE_MAX_SAMPLES,
    CHUNK_FLAG_INVALID, CHUNK_FLAG_LAST, PREFERRED_CHUNK,
};

/// 上位机版本。
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// F103 立创板的能力常量 —— 写在一处，供 UI 与文档引用。
///
/// 这些数字来自 [`docs/04-performance.md`]，不是猜的。
///
/// [`docs/04-performance.md`]: ../../../../docs/04-performance.md
pub mod f103 {
    /// 72 MHz 系统 + USB 下 ADCCLK 只能取 12 MHz → 单 ADC 上限。
    pub const MAX_SAMPLE_RATE_HZ: u32 = 857_142;
    /// 双 ADC 快速交织的理论上限（**仅用于显示**，不可作测量依据）。
    pub const MAX_INTERLEAVED_HZ: u32 = 1_714_285;
    /// 单次采集上限。
    pub const MAX_CAPTURE_SAMPLES: u32 = 4096;
    /// 抽点倍数上限（`SET_ACQ.decimation: 1..256`，见 `proto/protocol.h`）。
    ///
    /// 提成常量是因为它此前散在三处：命令层校验、MCP 校验、以及要做成
    /// 工具 schema 的上界 —— 三份写死的 256 迟早只改一处。
    pub const MAX_DECIMATION: u32 = 256;
    /// 采集环缓冲字节数。
    pub const RING_BYTES: u32 = 8 * 1024;
    /// ADC 位数。
    pub const ADC_BITS: u8 = 12;
    /// 数字通路（LM393 + 输入捕获）的时间分辨率（ns）。
    pub const DIGITAL_PATH_RESOLUTION_NS: u32 = 14;
    /// 触发比较的迟滞带宽（ADC LSB）。
    pub const TRIGGER_HYSTERESIS_LSB: u16 = 16;

    /// 一条采样率是否在硬件的物理上限内。
    ///
    /// UI 与 MCP 在发出去之前先用它拦下非法请求 —— 不要指望设备替你兜底。
    pub fn rate_is_achievable(hz: u32) -> bool {
        hz > 0 && hz <= MAX_INTERLEAVED_HZ
    }

    /// 把请求的采样率吸附到最近的定时器分频档位。
    ///
    /// 设备最终会回显 `actual_hz`，这个函数只给 UI 做「档位菜单」用，
    /// **不**负责拒绝超出 ADC 上限的请求（那是 [`rate_is_achievable`] 的事）。
    ///
    /// 定时器挂在 72 MHz 时钟上（APB1 预分频不为 1 时定时器时钟 ×2），取 `PSC = 0`，则采样周期是
    /// `ARR + 1` 个 tick，范围 `1..=65536`，可达频率集合为
    /// `{ 72_000_000 / period }`。
    ///
    /// 注意不能简单地取 `period = round(72e6 / requested)` 再相除：
    /// 末尾的整数除法会再引入一次向下取整，导致某些请求吸附到**次近**的档位
    /// （例如请求 1100 Hz 会落到 1099 而不是 1100）。
    /// 所以在 `period` 附近取三个候选，挑真正最接近的那个。
    ///
    /// **平局规则**：两个档位到请求值的距离完全相同时，取**更快**的那个
    /// （优先向过采样一侧取值，避免因欠采样而混叠）。
    /// 例：8546 Hz 到 `8545` 与 `8547` 都是 1 Hz → 返回 `8547`。
    ///
    /// 例：`requested = 857143` → `period = 84` → `857142 Hz`（F103 的默认档）
    pub fn nearest_achievable_rate(requested_hz: u32) -> u32 {
        const TIMER_HZ: u64 = 72_000_000;
        const PERIOD_MAX: u64 = 65_536;

        if requested_hz == 0 {
            return 0;
        }

        let ideal = TIMER_HZ as f64 / requested_hz as f64;
        let base = (ideal.round() as u64).clamp(1, PERIOD_MAX);

        let mut best_rate = (TIMER_HZ / base) as u32;
        let mut best_err = (best_rate as f64 - requested_hz as f64).abs();

        // 只扫 ±1 就够了：理想周期取整后，最近的两档一定在相邻周期里
        for p in [base.saturating_sub(1).max(1), (base + 1).min(PERIOD_MAX)] {
            let rate = (TIMER_HZ / p) as u32;
            let err = (rate as f64 - requested_hz as f64).abs();
            let better = err < best_err || (err == best_err && rate > best_rate);
            if better {
                best_err = err;
                best_rate = rate;
            }
        }
        best_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn f103_rate_table_is_sane() {
        // 默认档：PSC=0, ARR=83 → 72e6 / 84
        assert_eq!(f103::nearest_achievable_rate(857_143), 857_142);
        assert_eq!(f103::nearest_achievable_rate(857_142), 857_142);
        // 整数档位应精确落回自己
        assert_eq!(f103::nearest_achievable_rate(1_000_000), 1_000_000);
        assert_eq!(f103::nearest_achievable_rate(600_000), 600_000);
        assert_eq!(f103::nearest_achievable_rate(500_000), 500_000);

        assert!(f103::rate_is_achievable(857_142));
        assert!(!f103::rate_is_achievable(0));
    }

    /// 穷举 oracle：直接按「可达频率集合」的定义找最近档位。
    ///
    /// 这是**独立于实现的**参考解 —— 扫描整个周期窗口，而不是照抄
    /// `nearest_achievable_rate` 的 ±1 三候选启发式，所以在真出偏差时
    /// 能真的抓到（回归 1 的 867469 Hz 就是被它抓出来的）。
    ///
    /// 平局规则与实现一致：距离相同时取更快的那档。
    fn brute_force_nearest(req: u32) -> u32 {
        const TIMER_HZ: u64 = 72_000_000;
        let ideal = TIMER_HZ as f64 / req as f64;
        let lo = (ideal - 8.0).floor().max(1.0) as u64;
        let hi = (ideal + 8.0).ceil().min(65_536.0) as u64;

        let mut best = (TIMER_HZ / lo) as u32;
        let mut best_err = (best as f64 - req as f64).abs();
        for p in lo..=hi {
            let rate = (TIMER_HZ / p) as u32;
            let err = (rate as f64 - req as f64).abs();
            if err < best_err || (err == best_err && rate > best) {
                best_err = err;
                best = rate;
            }
        }
        best
    }

    #[test]
    fn rate_quantization_matches_brute_force_ground_truth() {
        // 回归 1：曾写成 `72e6/req - 1`，整数除法把 857143 落到 period=83，
        //         得出 867469 Hz 这种硬件上根本不存在的档位（误差 10326 Hz）。
        // 回归 2：曾写成 `72e6 / round(72e6/req)`，末尾的向下取整让 1100 Hz
        //         吸附到 1099 Hz，而不是真正的最近档 1100 Hz。
        for req in 1_100u32..=900_000 {
            let got = f103::nearest_achievable_rate(req);
            let want = brute_force_nearest(req);
            assert_eq!(got, want, "req={req}: 吸附结果与穷举解不符");
        }
    }

    #[test]
    fn tie_prefers_the_faster_rate() {
        // 8546 到 8545 和 8547 距离都是 1 —— 取更快的那档，避免欠采样混叠
        assert_eq!(f103::nearest_achievable_rate(8_546), 8_547);
    }

    #[test]
    fn rate_quantization_is_idempotent() {
        for req in 1_100u32..=900_000 {
            let once = f103::nearest_achievable_rate(req);
            let twice = f103::nearest_achievable_rate(once);
            assert_eq!(once, twice, "req={req}: 对已是档位的值再吸附应不变");
        }
    }

    #[test]
    fn rates_below_the_slowest_step_clamp_to_it() {
        // PSC=0 时 ARR+1 最大 65536 → 最低触发率 72e6/65536 ≈ 1098.6 Hz。
        // 比这更慢的请求只能钳到这一档（要更慢就得动 PSC，规格里写明了）。
        let slowest = f103::nearest_achievable_rate(1);
        assert_eq!(slowest, 1_098);
        assert_eq!(f103::nearest_achievable_rate(500), slowest);
        assert_eq!(f103::nearest_achievable_rate(1_098), slowest);
    }

    #[test]
    fn zero_request_does_not_panic() {
        assert_eq!(f103::nearest_achievable_rate(0), 0);
    }
}
