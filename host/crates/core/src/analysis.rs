//! 对外唯一的分析结果对象。
//!
//! # 为什么需要这一层
//!
//! 在它之前，同一批数字至少有四套表示：内部 [`Measurements`]、MCP 手抠的
//! JSON 键、证据包文本、GUI 视图。加一个字段要改四处，而且没有任何一处能
//! 表达「这个数可不可信」。本模块把「数值」与「判决」绑在一起：
//!
//! - [`Verdict::Ok`]：有数值不确定度；
//! - [`Verdict::Bounded`]：只能给下界（例如亚样点边沿）；
//! - [`Verdict::Refused`]：不出数，且必须说明原因。
//!
//! # 边界
//!
//! - [`Measurements`] **不改**。它是内部计算类型，`None` 的「测不出」语义
//!   全部保留；本模块只做一层转换。
//! - 文本渲染仍复用 [`crate::report`] 的八个段落函数；本模块的
//!   [`render_text`] 是**唯一对外入口**，`report::build_evidence` 只负责
//!   组装 [`AnalysisReport`] 与 [`EvidenceContext`]。

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::calib::ScaleSet;
use crate::capture::Capture;
use crate::command::DeviceConfig;
use crate::decoder::{DecodeRequest, DecodedFrames};
use crate::edges::{edges_hysteretic, sigma_hat};
use crate::measure::{filter_gaps, measure, Measurements};
use crate::signal::{detect_levels, UncertainReason};

/// 对外 schema 名。
pub const ANALYSIS_SCHEMA: &str = "analysis/v1";

/// 一次采集的完整分析结果。
///
/// `decoded` 不在本阶段落地：D2 的解码器类型尚不存在，D2 以**加性字段**补入。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct AnalysisReport {
    /// 契约名，当前恒为 [`ANALYSIS_SCHEMA`]。
    pub schema: String,
    /// 采集编号。
    pub capture_id: u16,
    /// 实际采样率（Hz）。
    pub rate_hz: u32,
    /// 数据从哪来、标没标定。
    pub provenance: Provenance,
    /// 逐通道结果。没有数据的通道不会出现在这里。
    pub channels: Vec<ChannelAnalysis>,
    /// 必须原样透出的说明（模拟器、未标定等）。
    #[serde(default)]
    pub notes: Vec<String>,
}

/// 数据来源与标定状态。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Provenance {
    /// 来源。模拟器也走 `Device`，靠 `simulated` 区分。
    pub source: Source,
    /// 标定状态。
    pub calibration: CalibrationState,
    /// 采集墙钟时间（Unix 秒）。
    #[serde(default)]
    pub captured_at_unix: u64,
}

/// 采集来源。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Source {
    /// 设备（含模拟器）。
    Device {
        /// 链路描述，例如 `sim(i2c_100k)` 或 `COM7 @ 921600`。
        link: String,
        /// 是否模拟器产生。
        simulated: bool,
    },
    /// 文件导入（D3 落地后使用）。
    File {
        /// 文件路径。
        path: String,
    },
}

/// 标定状态 —— 只描述「敢不敢把伏特值当准数」，不携带换算参数本身。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum CalibrationState {
    /// 没有该 uid 的记录，全部按占位换算。
    Uncalibrated,
    /// 有记录，但缺部分通道。
    PartiallyCalibrated {
        /// 设备 uid 的规范十六进制形式。
        uid: Option<String>,
    },
    /// 全部通道都有标定。
    Calibrated {
        /// 设备 uid 的规范十六进制形式。
        uid: Option<String>,
        /// 满量程（V），来自 `volts_per_lsb × 4096`。
        full_scale_v: f64,
    },
}

/// 一个通道的分析结果。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ChannelAnalysis {
    /// 通道号（0 起）。
    pub channel: usize,
    /// 内部测量结果，字段与语义完全不变。
    pub measurements: Measurements,
    /// 每个量的可信度判决。
    pub quality: Quality,
}

/// 一个通道的质量载体。
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct Quality {
    /// 噪声底 σ̂（ADC LSB）。P0 落地前为 `None`。
    pub noise_lsb: Option<f64>,
    /// 迟滞带宽 h（ADC LSB）。`None` 表示 σ̂ 估不出、退化为单阈值。
    #[serde(default)]
    pub hysteresis_lsb: Option<f64>,
    /// 键与 [`Measurements`] 字段同名。`BTreeMap` 保证 JSON 键序稳定。
    #[serde(default)]
    pub verdicts: BTreeMap<String, Verdict>,
}

/// 单个量的可信度。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Verdict {
    /// 有数值不确定度。
    Ok {
        /// 标准不确定度。
        u: f64,
    },
    /// 只能给下界。
    Bounded {
        /// 下界。
        lower: f64,
    },
    /// 不出数，并说明原因。
    Refused {
        /// 拒答原因。
        reason: RefuseReason,
    },
}

/// 拒答原因。
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum RefuseReason {
    /// 样点太少。
    TooFewSamples,
    /// 幅度太小。
    TooSmallAmplitude,
    /// 没有周期。
    NoPeriodicity,
    /// 间隔不稳定。
    IntervalUnstable,
    /// 窗内频率漂移。
    FrequencyDrift,
    /// 采样密度不足（混叠）。
    Aliased,
    /// 形状不在可判范围。
    ShapeUnrecognized,
    /// 需要标定的量没有标定数据（**只用于电压派生量**）。
    CalibrationMissing,
    /// 该量对本波形没有定义。
    NotApplicable,
    /// 该量没有不确定度模型（不是没标定）。
    NoUncertaintyModel,
    /// 模型存在但未通过自校验（例如占空比，见 `docs/10` §5.2）。
    ModelUnverified,
}

impl RefuseReason {
    /// 给人读的中文原因。措辞与 MCP 的枚举值分开：那边是机器契约
    /// （`no_uncertainty_model`），这里是给模型/用户看的句子。
    pub fn text(self) -> &'static str {
        match self {
            RefuseReason::TooFewSamples => "样点太少",
            RefuseReason::TooSmallAmplitude => "幅度太小",
            RefuseReason::NoPeriodicity => "无周期",
            RefuseReason::IntervalUnstable => "间隔不稳",
            RefuseReason::FrequencyDrift => "频率漂移",
            RefuseReason::Aliased => "采样密度不足",
            RefuseReason::ShapeUnrecognized => "形状不在可判范围",
            RefuseReason::CalibrationMissing => "未标定",
            RefuseReason::NotApplicable => "对本波形无定义",
            RefuseReason::NoUncertaintyModel => "无不确定度模型",
            RefuseReason::ModelUnverified => "模型未验证",
        }
    }
}

impl From<UncertainReason> for RefuseReason {
    fn from(r: UncertainReason) -> Self {
        match r {
            UncertainReason::TooFewSamples => Self::TooFewSamples,
            UncertainReason::TooSmallAmplitude => Self::TooSmallAmplitude,
            UncertainReason::NoPeriodicity => Self::NoPeriodicity,
            UncertainReason::IntervalUnstable => Self::IntervalUnstable,
            UncertainReason::FrequencyDrift => Self::FrequencyDrift,
            UncertainReason::Aliased => Self::Aliased,
            UncertainReason::ShapeUnrecognized => Self::ShapeUnrecognized,
        }
    }
}

/// 证据包需要、但 [`AnalysisReport`] 自己不携带的上下文。
///
/// 这些字段一半是「采集上下文」而不是「分析结果」：链路描述、设备回显配置、
/// 用户问题。把它们塞进 `AnalysisReport` 会让对外契约混入进程内状态。
#[derive(Debug, Clone)]
pub struct EvidenceContext<'a> {
    /// 采集本身（渲染采集元数据、形状、包络用）。
    pub capture: &'a Capture,
    /// 逐通道换算与「标没标定」的来源。
    pub scale: &'a ScaleSet,
    /// 链路描述。
    pub link: &'a str,
    /// 是否模拟器数据。
    pub simulated: bool,
    /// 设备回显配置。
    pub config: Option<&'a DeviceConfig>,
    /// 解码结果（任意协议）。
    pub decode: Option<&'a DecodedFrames>,
    /// 解码请求（用于在证据包里回显通道/门限/波特率）。
    pub decode_req: Option<&'a DecodeRequest>,
    /// 用户额外问题。
    pub question: Option<&'a str>,
}

impl AnalysisReport {
    /// 从一次采集构造对外结果。
    ///
    /// 所有测量值仍来自 [`measure()`]（全项目唯一实现），本函数不做任何重算。
    pub fn from_capture(
        capture: &Capture,
        scale: &ScaleSet,
        link: &str,
        simulated: bool,
    ) -> AnalysisReport {
        let calibration = calibration_state(scale);
        let mut channels = Vec::with_capacity(capture.channels.len());
        for ch in 0..capture.channels.len() {
            let Some(m) = measure(capture, ch, &scale.get(ch)) else {
                continue;
            };
            let quality = Quality::empty(ch, capture, scale, &m);
            channels.push(ChannelAnalysis {
                channel: ch,
                measurements: m,
                quality,
            });
        }

        let mut notes = Vec::new();
        if simulated {
            notes.push("本采集来自模拟器，结论不能用于判断真实硬件".to_string());
        }
        match &calibration {
            CalibrationState::Uncalibrated => {
                notes.push("电压未标定：按占位参数换算，只看相对关系，不要当成校准值".to_string())
            }
            CalibrationState::PartiallyCalibrated { .. } => notes
                .push("电压部分未标定：缺通道按占位参数换算，标定过的通道才可当准数".to_string()),
            CalibrationState::Calibrated { .. } => {}
        }

        AnalysisReport {
            schema: ANALYSIS_SCHEMA.to_string(),
            capture_id: capture.id,
            rate_hz: capture.rate_hz,
            provenance: Provenance {
                source: Source::Device {
                    link: link.to_string(),
                    simulated,
                },
                calibration,
                captured_at_unix: capture.wall_time,
            },
            channels,
            notes,
        }
    }

    /// 按通道号取结果。
    pub fn channel(&self, ch: usize) -> Option<&ChannelAnalysis> {
        self.channels.iter().find(|c| c.channel == ch)
    }
}

impl Quality {
    /// 第一版判决：**没有模型就拒答**，不编造不确定度。
    ///
    /// P0/P2 落地后，频率、σ̂ 等会从 `Refused(NoUncertaintyModel)` 升级为
    /// `Ok{u}` 或 `Bounded`；本函数是那一层的唯一入口。
    pub fn empty(ch: usize, cap: &Capture, scale: &ScaleSet, m: &Measurements) -> Quality {
        let mut verdicts = BTreeMap::new();
        let s = cap.samples(ch).unwrap_or(&[]);
        let summary = cap.summary(ch);
        let levels = detect_levels(s);
        let sep = if levels.is_two_level() {
            levels.sep_lsb
        } else {
            summary.as_ref().map(|x| x.pp_lsb as f64).unwrap_or(0.0)
        };
        let noise = sigma_hat(s);
        let h = noise
            .map(|sigma| (3.0 * sigma).min(0.25 * sep))
            .unwrap_or(0.0);

        // 电压派生量：未标定 → CalibrationMissing；已标定 → 仍无模型。
        let voltage = if scale.is_calibrated(ch) {
            Verdict::Refused {
                reason: RefuseReason::NoUncertaintyModel,
            }
        } else {
            Verdict::Refused {
                reason: RefuseReason::CalibrationMissing,
            }
        };
        for key in ["vpp", "min", "max", "mean", "rms", "ac_rms"] {
            verdicts.insert(key.to_string(), voltage.clone());
        }

        let freq_verdict = match m.freq_hz {
            None => refused(RefuseReason::NoPeriodicity),
            Some(_) => {
                // P2 的第一道门：边沿集合被假跃变污染时，频率不可信。
                // 阈值与 signal::classify 同一口径：max(0.10, 0.5/mean_gap)。
                let mid = summary
                    .as_ref()
                    .map(|x| (x.min_lsb as f64 + x.max_lsb as f64) / 2.0)
                    .unwrap_or(0.0);
                let (rising, _) = edges_hysteretic(s, mid, h);
                match filter_gaps(&rising).and_then(|f| f.cv().map(|cv| (cv, f))) {
                    Some((cv, f)) if !f.kept.is_empty() => {
                        let mean_gap = f.kept.iter().sum::<f64>() / f.kept.len() as f64;
                        let threshold =
                            0.10f64.max(if mean_gap > 0.0 { 0.5 / mean_gap } else { 0.10 });
                        if cv > threshold {
                            refused(RefuseReason::IntervalUnstable)
                        } else {
                            refused(RefuseReason::NoUncertaintyModel)
                        }
                    }
                    _ => refused(RefuseReason::NoUncertaintyModel),
                }
            }
        };
        verdicts.insert("freq_hz".into(), freq_verdict);

        verdicts.insert(
            "duty_pct".into(),
            match m.duty_pct {
                Some(_) => refused(RefuseReason::ModelUnverified),
                None => refused(RefuseReason::NotApplicable),
            },
        );

        // 亚样点边沿：单次采集里没有中间样点，只能给下界 0.8·Ts。
        let lower_ns = if cap.rate_hz > 0 {
            0.8 * 1e9 / cap.rate_hz as f64
        } else {
            0.0
        };
        for (key, value) in [("rise_ns", m.rise_ns), ("fall_ns", m.fall_ns)] {
            verdicts.insert(
                key.into(),
                match value {
                    Some(_) => Verdict::Bounded { lower: lower_ns },
                    None => refused(RefuseReason::NotApplicable),
                },
            );
        }

        for (key, value) in [
            ("overshoot_pct", m.overshoot_pct),
            ("undershoot_pct", m.undershoot_pct),
            ("high_ns", m.high_ns),
        ] {
            verdicts.insert(
                key.into(),
                match value {
                    Some(_) => refused(RefuseReason::NoUncertaintyModel),
                    None => refused(RefuseReason::NotApplicable),
                },
            );
        }

        // 计数没有「不确定度」语义：数出来的就是数出来的。
        verdicts.insert("edges".into(), Verdict::Ok { u: 0.0 });

        Quality {
            noise_lsb: noise,
            hysteresis_lsb: noise.map(|_| h),
            verdicts,
        }
    }
}

fn refused(reason: RefuseReason) -> Verdict {
    Verdict::Refused { reason }
}

fn calibration_state(scale: &ScaleSet) -> CalibrationState {
    if scale.all_calibrated() {
        CalibrationState::Calibrated {
            uid: scale.uid_hex(),
            full_scale_v: scale.get(0).volts_per_lsb * 4096.0,
        }
    } else if scale.record_found() {
        CalibrationState::PartiallyCalibrated {
            uid: scale.uid_hex(),
        }
    } else {
        CalibrationState::Uncalibrated
    }
}

/// 证据包里每个量的显示名与单位。顺序即渲染顺序（与测量行一致）。
///
/// 表里没有的量不会消失 —— [`verdict_line`] 会把它们按原始键名列出，
/// 提醒作者补显示名，而不是让一个量从模型眼里静默消失。
const VERDICT_KEYS: [(&str, &str, &str); 14] = [
    ("vpp", "Vpp", "V"),
    ("min", "min", "V"),
    ("max", "max", "V"),
    ("mean", "mean", "V"),
    ("rms", "RMS", "V"),
    ("ac_rms", "AC-RMS", "V"),
    ("freq_hz", "频率", "Hz"),
    ("duty_pct", "占空比", "%"),
    ("rise_ns", "上升时间", "ns"),
    ("fall_ns", "下降时间", "ns"),
    ("overshoot_pct", "过冲", "%"),
    ("undershoot_pct", "下冲", "%"),
    ("high_ns", "高电平脉宽", "ns"),
    ("edges", "跃变数", "次"),
];

/// 证据包里某个量的显示名与单位。
///
/// `None` 表示这是一个还没进 [`VERDICT_KEYS`] 的新量 —— 证据包会退回用
/// 原始键名，**跨出口测试**则应当报错提醒补名字。
pub fn quantity_label(key: &str) -> Option<(&'static str, &'static str)> {
    VERDICT_KEYS
        .iter()
        .find(|(k, _, _)| *k == key)
        .map(|(_, label, unit)| (*label, *unit))
}

/// 一个通道的可信度行。
///
/// 证据包从前只印数值，不印判决 —— 模型看到 `频率=207046 Hz` 时没有任何
/// 线索知道这个量其实是 `Refused`。MCP 的 JSON 出口一直带着 `quality`，
/// 文本出口缺这一行就破了「同一份结论给人和给模型」。
///
/// 输出按「判决类别」分组，而不是逐量罗列：14 个字段逐个写成
/// `Vpp=拒答(未标定)` 会占掉半屏，且同类原因重复到无法阅读。
fn verdict_line(q: &Quality) -> Option<String> {
    if q.verdicts.is_empty() {
        return None;
    }

    let mut exact: Vec<&str> = Vec::new();
    let mut uncertain: Vec<String> = Vec::new();
    let mut bounds: Vec<String> = Vec::new();
    // 原因 → 该原因下的量。用 Vec 而非 map：保持 VERDICT_KEYS 的顺序。
    let mut refused: Vec<(&'static str, Vec<String>)> = Vec::new();
    let mut seen: Vec<&str> = Vec::new();

    let mut push_refused = |reason: &'static str, label: String| match refused
        .iter_mut()
        .find(|(r, _)| *r == reason)
    {
        Some((_, list)) => list.push(label),
        None => refused.push((reason, vec![label])),
    };

    for (key, label, unit) in VERDICT_KEYS {
        seen.push(key);
        match q.verdicts.get(key) {
            None => {}
            Some(Verdict::Ok { u }) if *u == 0.0 => exact.push(label),
            Some(Verdict::Ok { u }) => uncertain.push(format!("{label}±{u:.3} {unit}")),
            Some(Verdict::Bounded { lower }) => {
                bounds.push(format!("{label}≥{lower:.1} {unit}"));
            }
            Some(Verdict::Refused { reason }) => {
                push_refused(reason.text(), label.to_string());
            }
        }
    }

    // 表外的新量：以键名出现，不静默丢弃。
    for (key, verdict) in &q.verdicts {
        if seen.contains(&key.as_str()) {
            continue;
        }
        match verdict {
            Verdict::Ok { u } if *u == 0.0 => uncertain.push(key.clone()),
            Verdict::Ok { u } => uncertain.push(format!("{key}±{u:.3}")),
            Verdict::Bounded { lower } => bounds.push(format!("{key}≥{lower:.1}")),
            Verdict::Refused { reason } => push_refused(reason.text(), key.clone()),
        }
    }

    let mut parts = Vec::new();
    if !exact.is_empty() {
        parts.push(format!("可信（精确）: {}", exact.join("/")));
    }
    if !uncertain.is_empty() {
        parts.push(format!("可信: {}", uncertain.join("、")));
    }
    if !bounds.is_empty() {
        parts.push(format!("下界: {}", bounds.join("、")));
    }
    for (reason, labels) in refused {
        parts.push(format!("拒答（{reason}）: {}", labels.join("/")));
    }
    Some(parts.join("；"))
}

/// 把一次采集渲染成证据包文本 —— **唯一对外入口**。
pub fn render_text(report: &AnalysisReport, ctx: &EvidenceContext<'_>) -> String {
    crate::report::render_sections(report, ctx)
}

/// 通道测量段。放在这里是为了让「怎么解释 Measurements」只有一处。
pub(crate) fn channels_text(s: &mut String, report: &AnalysisReport, cap: &Capture) {
    use std::fmt::Write as _;

    let _ = writeln!(s);
    let _ = writeln!(s, "== 通道测量（供你判断用；界面上已有，不必复述）==");

    let mut any = false;
    for ch in 0..cap.channels.len() {
        let name = crate::report::channel_label(ch);
        let Some(ca) = report.channel(ch) else {
            let _ = writeln!(s, "{name}: 测不出（通道无数据）");
            continue;
        };
        let m = &ca.measurements;
        any = true;
        let _ = writeln!(
            s,
            "{name}: Vpp={:.4} V  min={:.4} V  max={:.4} V  mean={:.4} V  RMS={:.4} V  AC-RMS={:.4} V",
            m.vpp, m.min, m.max, m.mean, m.rms, m.ac_rms
        );
        let freq = match m.freq_hz {
            Some(f) => format!("{f:.1} Hz"),
            None => "测不出（无明显周期）".to_string(),
        };
        let duty = match m.duty_pct {
            Some(d) => format!("{d:.1} %"),
            None => "测不出".to_string(),
        };
        let rise = match m.rise_ns {
            Some(r) => format!("{r:.1} ns"),
            None => "测不出（没有干净的边沿）".to_string(),
        };
        let fall = match m.fall_ns {
            Some(f) => format!("{f:.1} ns"),
            None => "测不出（没有干净的边沿）".to_string(),
        };
        let over = match m.overshoot_pct {
            Some(p) => format!("{p:.1} %"),
            None => "测不出（无双电平参考）".to_string(),
        };
        let under = match m.undershoot_pct {
            Some(p) => format!("{p:.1} %"),
            None => "测不出（无双电平参考）".to_string(),
        };
        let high = match m.high_ns {
            Some(h) => format!("{h:.1} ns"),
            None => "测不出（没有高电平段）".to_string(),
        };
        let _ = writeln!(
            s,
            "     频率={freq}  占空比={duty}  上升/下降={rise}/{fall}",
        );
        let _ = writeln!(
            s,
            "     过冲/下冲={over}/{under}  高电平脉宽={high}  跃变数={}",
            m.edges
        );
        if let Some(v) = verdict_line(&ca.quality) {
            let _ = writeln!(s, "     可信度: {v}");
        }
    }
    if !any {
        let _ = writeln!(s, "（所有通道都测不出 —— 采集可能是空的）");
    }
    let _ = writeln!(
        s,
        "注：频率与占空比已排除空闲段；上升/下降时间受采样周期限制，分辨率约 1 个采样周期。\
         过冲的参考电平是平台均值，不是全窗最大值。"
    );
    let _ = writeln!(
        s,
        "注 2：「可信（精确）」表示值确定但没有不确定度模型（如整数计数）；\
         「可信」带 ± 的才是含不确定度的数；「下界」只保证不小于该值；\
         「拒答」的量没有可信区间，上面印出的数只用于定位，**不要引用**。"
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Capture;
    use crate::measure::measure;

    fn square_capture() -> Capture {
        let rate = 1_000_000u32;
        let mut cap = Capture::new(7, rate, 1, 4096);
        let s: Vec<u16> = (0..4096)
            .map(|i| if (i / 50) % 2 == 0 { 0 } else { 4000 })
            .collect();
        cap.channels[0] = s;
        cap
    }

    #[test]
    fn report_round_trips_through_json() {
        let cap = square_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let json = serde_json::to_string(&report).expect("序列化失败");
        let back: AnalysisReport = serde_json::from_str(&json).expect("反序列化失败");
        assert_eq!(report, back);
        assert_eq!(back.schema, ANALYSIS_SCHEMA);
    }

    #[test]
    fn empty_quality_never_claims_ok_for_model_less_quantities() {
        let cap = square_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let q = &report.channels[0].quality;
        for (key, verdict) in &q.verdicts {
            if key == "edges" {
                assert!(matches!(verdict, Verdict::Ok { u } if *u == 0.0));
                continue;
            }
            assert!(
                !matches!(verdict, Verdict::Ok { .. }),
                "{key} 没有模型却给了 Ok：{verdict:?}"
            );
        }
        assert_eq!(
            q.verdicts.get("duty_pct"),
            Some(&Verdict::Refused {
                reason: RefuseReason::ModelUnverified
            })
        );
        assert_eq!(
            q.verdicts.get("vpp"),
            Some(&Verdict::Refused {
                reason: RefuseReason::CalibrationMissing
            })
        );
    }

    #[test]
    fn report_uses_the_same_measurements_as_the_old_path() {
        let cap = square_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let direct = measure(&cap, 0, &scale.get(0)).expect("应测得出");
        assert_eq!(report.channels[0].measurements, direct);
    }

    #[test]
    fn text_and_json_agree_on_the_same_capture() {
        let cap = square_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let ctx = EvidenceContext {
            capture: &cap,
            scale: &scale,
            link: "sim(square)",
            simulated: true,
            config: None,
            decode: None,
            decode_req: None,
            question: None,
        };
        let text = render_text(&report, &ctx);
        let m = &report.channels[0].measurements;
        assert!(
            text.contains(&format!("Vpp={:.4} V", m.vpp)),
            "证据包与 AnalysisReport 的 Vpp 不一致：{text}"
        );
        assert!(
            text.contains(&format!("频率={:.1} Hz", m.freq_hz.expect("方波应有频率"))),
            "证据包与 AnalysisReport 的频率不一致：{text}"
        );
    }

    #[test]
    fn verdict_line_groups_by_verdict_and_keeps_units() {
        let cap = square_capture();
        let report = AnalysisReport::from_capture(&cap, &ScaleSet::uncalibrated(1), "sim", true);
        let line = verdict_line(&report.channels[0].quality).expect("应有可信度行");

        assert!(
            line.contains("拒答（未标定）: Vpp/min/max/mean/RMS/AC-RMS"),
            "电压量应作为一组拒答：{line}"
        );
        assert!(
            line.contains("下界: 上升时间≥") && line.contains(" ns"),
            "边沿时间应给下界并带单位：{line}"
        );
        assert!(
            line.contains("可信（精确）: 跃变数"),
            "计数是精确值：{line}"
        );
        assert!(
            line.contains("拒答（模型未验证）: 占空比"),
            "占空比尚无自校验模型：{line}"
        );
    }

    #[test]
    fn verdict_line_does_not_drop_keys_missing_from_the_label_table() {
        let mut q = Quality::default();
        q.verdicts.insert(
            "future_metric".into(),
            Verdict::Refused {
                reason: RefuseReason::NotApplicable,
            },
        );
        let line = verdict_line(&q).expect("非空判决应渲染");
        assert!(line.contains("future_metric"), "新量不能静默消失：{line}");
    }

    #[test]
    fn irregular_gaps_are_refused_as_interval_unstable() {
        // 大部分周期 50 样点，每第 10 个周期缩成 10 样点：
        // 中位数仍能定出 50，短间隔不会被 filter_gaps 丢掉，cv 必然升高。
        let mut cap = Capture::new(9, 1_000_000, 1, 4096);
        let mut s = vec![0u16; 4096];
        let mut i = 0usize;
        let mut n = 0usize;
        while i < 4096 {
            let period = if n % 10 == 9 { 10 } else { 50 };
            for j in 0..period / 2 {
                if i + j < 4096 {
                    s[i + j] = 4000;
                }
            }
            i += period;
            n += 1;
        }
        cap.channels[0] = s;
        let report = AnalysisReport::from_capture(&cap, &ScaleSet::uncalibrated(1), "sim", true);
        let q = &report.channels[0].quality;
        assert_eq!(
            q.verdicts.get("freq_hz"),
            Some(&Verdict::Refused {
                reason: RefuseReason::IntervalUnstable
            }),
            "污染边沿应拒答频率，实际 {:?}",
            q.verdicts.get("freq_hz")
        );
    }

    #[test]
    fn clean_square_freq_is_not_refused_as_unstable() {
        let cap = square_capture();
        let report = AnalysisReport::from_capture(&cap, &ScaleSet::uncalibrated(1), "sim", true);
        let q = &report.channels[0].quality;
        assert_eq!(
            q.verdicts.get("freq_hz"),
            Some(&Verdict::Refused {
                reason: RefuseReason::NoUncertaintyModel
            }),
            "干净方波不应被当成间隔不稳定：{:?}",
            q.verdicts.get("freq_hz")
        );
    }
}
