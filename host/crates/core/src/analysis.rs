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
//! - [`Verdict::BoundedAbove`]：只能给上界（例如单峰振铃的频率上界）；
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
use crate::tie::{self, TieAnalysis, TieReason, TieStats};
use crate::transient::{self, TransientAnalysis, TransientReason, TransientStats};

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

/// 标定状态 —— 只描述「伏特换算值是否可信」，不携带换算参数本身。
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
    /// 瞬态特征。逐量 `Option`：测不出的量在 `quality.verdicts` 里带原因。
    #[serde(default)]
    pub transient: TransientStats,
    /// TIE / 周期抖动统计量。拒答时为 `None`，原因在 `quality.verdicts`。
    #[serde(default)]
    pub tie: Option<TieStats>,
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
    /// 只能给上界。
    BoundedAbove {
        /// 上界。
        upper: f64,
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
    /// 同向边沿太少（TIE 至少需要 30 条）。
    TooFewEdges,
    /// 边沿快于采样栅格，插值时刻不反映真实边沿（TIE）。
    EdgeTooFastForTie,
    /// 间隔不稳定。
    IntervalUnstable,
    /// 窗内频率漂移。
    FrequencyDrift,
    /// 单次采集内周期明显漂移（TIE 非平稳）。
    NonStationary,
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
            RefuseReason::TooFewEdges => "边沿太少",
            RefuseReason::EdgeTooFastForTie => "边沿快于采样栅格",
            RefuseReason::IntervalUnstable => "间隔不稳",
            RefuseReason::FrequencyDrift => "频率漂移",
            RefuseReason::NonStationary => "周期漂移",
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

impl From<TieReason> for RefuseReason {
    fn from(r: TieReason) -> Self {
        match r {
            TieReason::TooFewSamples => Self::TooFewSamples,
            TieReason::TooFewEdges => Self::TooFewEdges,
            TieReason::EdgeTooFastForTie => Self::EdgeTooFastForTie,
            TieReason::Aliased => Self::Aliased,
            TieReason::IntervalUnstable => Self::IntervalUnstable,
            TieReason::NonStationary => Self::NonStationary,
            TieReason::ShapeUnrecognized => Self::ShapeUnrecognized,
        }
    }
}

impl From<TransientReason> for RefuseReason {
    fn from(r: TransientReason) -> Self {
        match r {
            TransientReason::TooFewSamples => Self::TooFewSamples,
            TransientReason::NotApplicable => Self::NotApplicable,
            TransientReason::ShapeUnrecognized => Self::ShapeUnrecognized,
            TransientReason::Aliased => Self::Aliased,
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
            let samples = capture.samples(ch).unwrap_or(&[]);
            let transient = transient::analyze(samples, capture.rate_hz);
            let tie = tie::analyze(samples, capture.rate_hz);
            let quality = Quality::with_analyses(ch, capture, scale, &m, &transient, &tie);
            channels.push(ChannelAnalysis {
                channel: ch,
                measurements: m,
                transient: transient.stats,
                tie: tie.stats,
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
    /// 第一版判决：**没有模型就拒答**，不生成无依据的不确定度值。
    ///
    /// P0/P2 落地后，频率、σ̂ 等会从 `Refused(NoUncertaintyModel)` 升级为
    /// `Ok{u}` 或 `Bounded`；本函数是那一层的唯一入口。
    pub fn empty(ch: usize, cap: &Capture, scale: &ScaleSet, m: &Measurements) -> Quality {
        let samples = cap.samples(ch).unwrap_or(&[]);
        let transient = transient::analyze(samples, cap.rate_hz);
        let tie = tie::analyze(samples, cap.rate_hz);
        Self::with_analyses(ch, cap, scale, m, &transient, &tie)
    }

    /// 与 [`Self::empty`] 相同，但复用调用方已经算好的瞬态 / TIE。
    fn with_analyses(
        ch: usize,
        cap: &Capture,
        scale: &ScaleSet,
        m: &Measurements,
        transient: &TransientAnalysis,
        tie: &TieAnalysis,
    ) -> Quality {
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

        transient_verdicts(transient, &mut verdicts);
        tie_verdicts(tie, &mut verdicts);

        Quality {
            noise_lsb: noise,
            hysteresis_lsb: noise.map(|_| h),
            verdicts,
        }
    }
}

/// 瞬态量的判决。
///
/// 只有整数 runt 计数是精确值；其余量有数但**没有不确定度模型**，
/// 一律 `Refused(NoUncertaintyModel)` —— 不拿 `Ok{u:0}` 冒充。
fn transient_verdicts(tr: &TransientAnalysis, verdicts: &mut BTreeMap<String, Verdict>) {
    let reason = |key: &str| {
        refused(RefuseReason::from(
            *tr.reasons
                .get(key)
                .unwrap_or(&TransientReason::ShapeUnrecognized),
        ))
    };

    verdicts.insert(
        "ring_freq_hz".into(),
        if tr.stats.ring_freq_hz.is_some() {
            refused(RefuseReason::NoUncertaintyModel)
        } else {
            reason("ring_freq_hz")
        },
    );
    verdicts.insert(
        "ring_freq_upper_hz".into(),
        match (tr.stats.ring_freq_upper_hz, tr.stats.ring_freq_hz) {
            (Some(upper), _) => Verdict::BoundedAbove { upper },
            (None, Some(_)) => refused(RefuseReason::NotApplicable),
            (None, None) => reason("ring_freq_hz"),
        },
    );

    for (key, present) in [
        ("damping_ratio", tr.stats.damping_ratio.is_some()),
        ("settle_2_ns", tr.stats.settle_2_ns.is_some()),
        ("settle_5_ns", tr.stats.settle_5_ns.is_some()),
        ("rc_tau_ns", tr.stats.rc_tau_ns.is_some()),
        (
            "slew_max_lsb_per_us",
            tr.stats.slew_max_lsb_per_us.is_some(),
        ),
        (
            "slew_avg_lsb_per_us",
            tr.stats.slew_avg_lsb_per_us.is_some(),
        ),
    ] {
        verdicts.insert(
            key.into(),
            if present {
                refused(RefuseReason::NoUncertaintyModel)
            } else {
                reason(key)
            },
        );
    }

    verdicts.insert(
        "runts".into(),
        match tr.stats.runts {
            Some(_) => Verdict::Ok { u: 0.0 },
            None => reason("runts"),
        },
    );
}

/// TIE 量的判决。
///
/// 只有边沿数（整数计数）与 σ_TIE 有模型：docs/11 §4.4 给出 σ 的相对
/// 不确定度 `1/√(2(n−1))`（n = 30 时约 13%）。其余量（均值、峰峰值、
/// 周期抖动、ρ、单沿地板）没有不确定度模型，拒答；卡方区间本身随
/// `TieStats` 导出，作为 σ 的区间，不再单独当成一个量。
fn tie_verdicts(tie: &TieAnalysis, verdicts: &mut BTreeMap<String, Verdict>) {
    const KEYS: [&str; 8] = [
        "tie_n_edges",
        "tie_rho",
        "tie_sigma_ns",
        "tie_mean_ns",
        "tie_pk_pk_ns",
        "tie_pj_rms_ns",
        "tie_pj_pk_pk_ns",
        "tie_resolution_floor_ns",
    ];

    let Some(t) = &tie.stats else {
        let reason = refused(RefuseReason::from(
            tie.reason.unwrap_or(TieReason::ShapeUnrecognized),
        ));
        for key in KEYS {
            verdicts.insert(key.into(), reason.clone());
        }
        return;
    };

    verdicts.insert("tie_n_edges".into(), Verdict::Ok { u: 0.0 });
    let df = t.n_edges.saturating_sub(1) as f64;
    let u = if df > 0.0 {
        t.sigma_tie_ns / (2.0 * df).sqrt()
    } else {
        0.0
    };
    verdicts.insert("tie_sigma_ns".into(), Verdict::Ok { u });
    for key in [
        "tie_rho",
        "tie_mean_ns",
        "tie_pk_pk_ns",
        "tie_pj_rms_ns",
        "tie_pj_pk_pk_ns",
        "tie_resolution_floor_ns",
    ] {
        verdicts.insert(key.into(), refused(RefuseReason::NoUncertaintyModel));
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
const VERDICT_KEYS: [(&str, &str, &str); 31] = [
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
    ("ring_freq_hz", "振铃频率", "Hz"),
    ("ring_freq_upper_hz", "振铃频率上界", "Hz"),
    ("damping_ratio", "阻尼比", "无量纲"),
    ("settle_2_ns", "调节时间(±2%)", "ns"),
    ("settle_5_ns", "调节时间(±5%)", "ns"),
    ("rc_tau_ns", "RC τ", "ns"),
    ("slew_max_lsb_per_us", "最大压摆率", "LSB/µs"),
    ("slew_avg_lsb_per_us", "平均压摆率", "LSB/µs"),
    ("runts", "runt 数", "个"),
    ("tie_n_edges", "TIE 边沿数", "个"),
    ("tie_rho", "ρ 比值", "无量纲"),
    ("tie_sigma_ns", "TIE σ", "ns"),
    ("tie_mean_ns", "TIE 均值", "ns"),
    ("tie_pk_pk_ns", "TIE 峰峰值", "ns"),
    ("tie_pj_rms_ns", "周期抖动 RMS", "ns"),
    ("tie_pj_pk_pk_ns", "周期抖动峰峰值", "ns"),
    ("tie_resolution_floor_ns", "单沿分辨率地板", "ns"),
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
/// 输出按「判决类别」分组，而不是逐量罗列：31 个字段逐个写成
/// `Vpp=拒答(未标定)` 会占掉半屏，且同类原因重复到无法阅读。
/// 把一个通道的判决表渲染成一行文本 —— 证据包、CLI 与 GUI 共用的**唯一来源**
/// （调用方各加自己的缩进/前缀，但措辞只有这一份）。
pub fn verdict_line(q: &Quality) -> Option<String> {
    if q.verdicts.is_empty() {
        return None;
    }

    let mut exact: Vec<&str> = Vec::new();
    let mut uncertain: Vec<String> = Vec::new();
    let mut bounds: Vec<String> = Vec::new();
    let mut bounds_above: Vec<String> = Vec::new();
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
            Some(Verdict::Ok { u }) => uncertain.push(value_with_unit(label, *u, unit)),
            Some(Verdict::Bounded { lower }) => {
                bounds.push(bound_with_unit(label, '≥', *lower, unit));
            }
            Some(Verdict::BoundedAbove { upper }) => {
                bounds_above.push(bound_with_unit(label, '≤', *upper, unit));
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
            Verdict::Ok { u } if *u == 0.0 => exact.push(key.as_str()),
            Verdict::Ok { u } => uncertain.push(format!("{key}±{u:.3}")),
            Verdict::Bounded { lower } => bounds.push(format!("{key}≥{lower:.1}")),
            Verdict::BoundedAbove { upper } => bounds_above.push(format!("{key}≤{upper:.1}")),
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
    if !bounds_above.is_empty() {
        parts.push(format!("上界: {}", bounds_above.join("、")));
    }
    for (reason, labels) in refused {
        parts.push(format!("拒答（{reason}）: {}", labels.join("/")));
    }
    Some(parts.join("；"))
}

/// 带单位的值。无量纲量的单位为空串，不留下多余空格。
fn value_with_unit(label: &str, u: f64, unit: &str) -> String {
    if unit.is_empty() {
        format!("{label}±{u:.3}")
    } else {
        format!("{label}±{u:.3} {unit}")
    }
}

fn bound_with_unit(label: &str, sign: char, value: f64, unit: &str) -> String {
    if unit.is_empty() {
        format!("{label}{sign}{value:.1}")
    } else {
        format!("{label}{sign}{value:.1} {unit}")
    }
}

/// 把一次采集渲染成证据包文本 —— **唯一对外入口**。
pub fn render_text(report: &AnalysisReport, ctx: &EvidenceContext<'_>) -> String {
    crate::report::render_sections(report, ctx)
}

fn opt_ns(v: Option<f64>) -> String {
    match v {
        Some(x) => format!("{x:.1} ns"),
        None => "测不出".to_string(),
    }
}

/// 瞬态与 TIE 两段的文本。**证据包与 CLI 共用这一份**，避免两处措辞漂移。
///
/// 返回的每一行**不带外层缩进**：首行以「瞬态:」「TIE:」开始，续行以
/// 6 个空格对齐到首行内容之后。调用方给每行加同样的前缀即可。
pub fn transient_tie_lines(ca: &ChannelAnalysis) -> Vec<String> {
    let tr = &ca.transient;
    // 上界用「≤」而不是「=≤」：只有上界时读作「振铃≤…」，与可信度行的
    // 「振铃频率上界≤…」同一口径。
    let ring = match (tr.ring_freq_hz, tr.ring_freq_upper_hz) {
        (Some(f), _) => format!("={f:.1} Hz"),
        (None, Some(u)) => format!("≤{u:.1} Hz"),
        (None, None) => "=测不出".to_string(),
    };
    let zeta = match tr.damping_ratio {
        Some(z) => format!("{z:.3}"),
        None => "测不出".to_string(),
    };
    let slew = match (tr.slew_max_lsb_per_us, tr.slew_avg_lsb_per_us) {
        (Some(a), Some(b)) => format!("{a:.1}/{b:.1} LSB/µs"),
        _ => "测不出".to_string(),
    };
    let runts = match tr.runts {
        Some(n) => n.to_string(),
        None => "测不出".to_string(),
    };

    let mut lines = vec![
        format!(
            "瞬态: 振铃{ring}  阻尼比={zeta}  调节(±2%)={}  调节(±5%)={}",
            opt_ns(tr.settle_2_ns),
            opt_ns(tr.settle_5_ns)
        ),
        format!(
            "      RC τ={}  压摆(max/avg)={slew}  runt={runts}",
            opt_ns(tr.rc_tau_ns)
        ),
    ];

    match &ca.tie {
        Some(t) => {
            let grid = if t.grid_limited { "是" } else { "否" };
            lines.push(format!(
                "TIE: 边沿数={} ρ={:.2} σ={:.1} ns [CI95 {:.1}, {:.1}] 均值={:.1} ns 峰峰={:.1} ns",
                t.n_edges,
                t.rho,
                t.sigma_tie_ns,
                t.ci95_low_ns,
                t.ci95_high_ns,
                t.mean_tie_ns,
                t.pk_pk_ns
            ));
            lines.push(format!(
                "     周期抖动 RMS={:.1} ns 峰峰={:.1} ns 单沿地板={:.1} ns 网格受限={grid}",
                t.period_jitter_rms_ns, t.period_jitter_pk_pk_ns, t.resolution_floor_ns
            ));
        }
        None => {
            let reason = match ca.quality.verdicts.get("tie_sigma_ns") {
                Some(Verdict::Refused { reason }) => reason.text(),
                _ => "测不出",
            };
            lines.push(format!("TIE: 测不出（{reason}）"));
        }
    }
    lines
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

        for line in transient_tie_lines(ca) {
            let _ = writeln!(s, "     {line}");
        }

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
         「上界」只保证不大于该值；\
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
            // 整数计数：数出来的就是数出来的；测不出时可以是 Refused。
            let exact_count = matches!(key.as_str(), "edges" | "runts" | "tie_n_edges");
            if exact_count {
                assert!(
                    matches!(verdict, Verdict::Ok { u } if *u == 0.0)
                        || matches!(verdict, Verdict::Refused { .. }),
                    "{key} 只能是精确计数或带原因的拒答：{verdict:?}"
                );
                continue;
            }
            // σ_TIE 是唯一有不确定度模型的新量（docs/11 §4.4）。
            if key == "tie_sigma_ns" {
                match verdict {
                    Verdict::Ok { u } => assert!(*u >= 0.0),
                    Verdict::Refused { .. } => {}
                    other => panic!("tie_sigma_ns 不应是 {other:?}"),
                }
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

    /// A4 的第二半：**已标定**只换电压口径，不换「没有模型就拒答」。
    ///
    /// 回归：若把电压判决写成 `calibrated ? Ok{u:0} : Refused(CalibrationMissing)`，
    /// 这条立刻红。标定解决的是「LSB 到伏特的换算」，不是「这个数的
    /// 不确定度是多少」——后者仍然没有模型。
    #[test]
    fn calibrated_voltages_still_refuse_without_a_model() {
        use crate::calib::CalibrationStore;
        use crate::capture::ChannelScale as CalScale;

        let cap = square_capture();
        let uid = [0x5e; 12];
        let mut store = CalibrationStore::empty();
        store
            .set_channel(uid, 0, CalScale::default(), None, 0)
            .expect("写入标定");
        let scale = store.scales_for(Some(&uid), 1);
        assert!(scale.all_calibrated(), "前置条件：这一窗应已标定");

        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let q = &report.channels[0].quality;

        for key in ["vpp", "min", "max", "mean", "rms", "ac_rms"] {
            assert_eq!(
                q.verdicts.get(key),
                Some(&Verdict::Refused {
                    reason: RefuseReason::NoUncertaintyModel
                }),
                "{key} 已标定但没有不确定度模型，必须拒答"
            );
        }
        for key in ["freq_hz", "overshoot_pct", "undershoot_pct", "high_ns"] {
            assert_eq!(
                q.verdicts.get(key),
                Some(&Verdict::Refused {
                    reason: RefuseReason::NoUncertaintyModel
                }),
                "{key} 没有不确定度模型，必须拒答"
            );
        }
        assert_eq!(
            q.verdicts.get("duty_pct"),
            Some(&Verdict::Refused {
                reason: RefuseReason::ModelUnverified
            })
        );
        for key in ["rise_ns", "fall_ns"] {
            assert!(
                matches!(q.verdicts.get(key), Some(Verdict::Bounded { .. })),
                "{key} 只能给下界（亚样点边沿）：{:?}",
                q.verdicts.get(key)
            );
        }
        // 唯一允许 Ok 的量：整数计数，值就是数出来的。
        assert_eq!(q.verdicts.get("edges"), Some(&Verdict::Ok { u: 0.0 }));
    }

    #[test]
    fn report_uses_the_same_measurements_as_the_old_path() {
        let cap = square_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let direct = measure(&cap, 0, &scale.get(0)).expect("应测得出");
        assert_eq!(report.channels[0].measurements, direct);
    }

    /// 1 MHz、周期 32 样点、10–90% = 2.4 样点的方波：TIE 可用
    /// （ρ = 2.4 ≥ 2），叠加近似高斯噪声让 σ_TIE > 0。
    fn slow_clock_capture() -> Capture {
        const LO: f64 = 500.0;
        const HI: f64 = 3500.0;
        const PERIOD: f64 = 32.0;
        const WIDTH: f64 = 3.0;
        let mut cap = Capture::new(11, 1_000_000, 1, 4096);
        let mut seed = 0x5EED_1234u32;
        cap.channels[0] = (0..4096)
            .map(|i| {
                let t = i as f64;
                let k = ((t - 4.0) / PERIOD).floor();
                let rise = 4.0 + k * PERIOD;
                let fall = rise + PERIOD / 2.0;
                let mut v = if t >= rise - WIDTH / 2.0 && t <= rise + WIDTH / 2.0 {
                    LO + (HI - LO) * (t - (rise - WIDTH / 2.0)) / WIDTH
                } else if t >= fall - WIDTH / 2.0 && t <= fall + WIDTH / 2.0 {
                    HI - (HI - LO) * (t - (fall - WIDTH / 2.0)) / WIDTH
                } else if (t - 4.0).rem_euclid(PERIOD) < PERIOD / 2.0 {
                    HI
                } else {
                    LO
                };
                let mut draw = || {
                    seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                    ((seed >> 16) % 2001) as f64 / 1000.0 - 1.0
                };
                v += 5.0 * (draw() + draw() + draw());
                v.clamp(0.0, 4095.0).round() as u16
            })
            .collect();
        cap
    }

    /// 单次阶跃 + 一个过冲峰：瞬态只能给振铃频率的**上界**。
    fn single_overshoot_capture() -> Capture {
        let mut cap = Capture::new(13, 1_000_000, 1, 4096);
        let mut s = vec![500u16; 4096];
        // 4 样点的上升段，然后从 3800 指数回落并稳定到 3500。
        for i in 0..4 {
            s[100 + i] = 500 + 825 * (i as u16 + 1);
        }
        for (i, v) in s.iter_mut().enumerate().skip(104) {
            *v = (3500.0 + 300.0 * (-((i - 104) as f64) / 30.0).exp()).round() as u16;
        }
        cap.channels[0] = s;
        cap
    }

    #[test]
    fn tie_and_transient_are_exported_with_verdicts() {
        let cap = slow_clock_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(slow_clock)", true);
        let ca = &report.channels[0];
        let q = &ca.quality;

        let t = ca.tie.as_ref().expect("慢边沿时钟应报 TIE");
        assert!(t.n_edges >= 30, "边沿数 {} 不足", t.n_edges);
        assert!(t.sigma_tie_ns > 0.0, "含噪时钟的 σ_TIE 应大于 0");

        // σ 的不确定度模型来自 docs/11 §4.4：σ 的相对不确定度 1/√(2(n−1))。
        match q.verdicts.get("tie_sigma_ns") {
            Some(Verdict::Ok { u }) => {
                let want = t.sigma_tie_ns / (2.0 * (t.n_edges - 1) as f64).sqrt();
                assert!(
                    (u - want).abs() < 1e-9,
                    "σ 的不确定度 {u} 与 1/√(2(n−1)) 模型 {want} 不符"
                );
            }
            other => panic!("tie_sigma_ns 应有模型并给 Ok，实测 {other:?}"),
        }
        assert_eq!(q.verdicts.get("tie_n_edges"), Some(&Verdict::Ok { u: 0.0 }));
        for key in [
            "tie_rho",
            "tie_mean_ns",
            "tie_pk_pk_ns",
            "tie_pj_rms_ns",
            "tie_pj_pk_pk_ns",
            "tie_resolution_floor_ns",
        ] {
            assert_eq!(
                q.verdicts.get(key),
                Some(&Verdict::Refused {
                    reason: RefuseReason::NoUncertaintyModel
                }),
                "{key} 没有不确定度模型，必须拒答"
            );
        }

        // 瞬态：有值的量都进了出口；整数计数是精确值。
        assert!(
            ca.transient.slew_max_lsb_per_us.is_some(),
            "线性边沿应能给出压摆率"
        );
        // 这个合成时钟的慢边沿会把 k-means 平台中心拉向中位，±5% 容差带
        // 不再覆盖平台 —— `detect_levels` 判不出双电平，runt 只能「不适用」。
        // 这里**不能**编一个 0：双电平的精确计数由下面的干净方波覆盖。
        assert_eq!(
            q.verdicts.get("runts"),
            Some(&Verdict::Refused {
                reason: RefuseReason::NotApplicable
            }),
            "双电平判不出时 runt 必须不适用"
        );
        assert!(
            matches!(
                q.verdicts.get("slew_max_lsb_per_us"),
                Some(Verdict::Refused {
                    reason: RefuseReason::NoUncertaintyModel
                })
            ),
            "压摆率没有不确定度模型：{:?}",
            q.verdicts.get("slew_max_lsb_per_us")
        );

        // 所有出口量都必须有显示名，否则 A3 会在缺名时报错。
        for key in q.verdicts.keys() {
            assert!(quantity_label(key).is_some(), "{key} 没有补进 VERDICT_KEYS");
        }

        // 序列化合同：新字段可往返。
        let json = serde_json::to_string(&report).expect("序列化失败");
        let back: AnalysisReport = serde_json::from_str(&json).expect("反序列化失败");
        assert_eq!(report, back);
    }

    #[test]
    fn a_fast_edge_capture_refuses_tie_with_a_reason() {
        let cap = square_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(square)", true);
        let ca = &report.channels[0];
        assert!(ca.tie.is_none(), "瞬时边沿不能报 TIE");
        // 干净方波是双电平 —— runt 计数是精确值，可以 Ok。
        assert_eq!(
            ca.quality.verdicts.get("runts"),
            Some(&Verdict::Ok { u: 0.0 }),
            "双电平方波应给出精确的 runt 计数"
        );
        for key in [
            "tie_n_edges",
            "tie_rho",
            "tie_sigma_ns",
            "tie_mean_ns",
            "tie_pk_pk_ns",
            "tie_pj_rms_ns",
            "tie_pj_pk_pk_ns",
            "tie_resolution_floor_ns",
        ] {
            assert_eq!(
                ca.quality.verdicts.get(key),
                Some(&Verdict::Refused {
                    reason: RefuseReason::EdgeTooFastForTie
                }),
                "{key} 应带「边沿快于栅格」的拒答原因"
            );
        }
    }

    #[test]
    fn an_upper_bound_is_exported_as_bounded_above() {
        let cap = single_overshoot_capture();
        let scale = ScaleSet::uncalibrated(1);
        let report = AnalysisReport::from_capture(&cap, &scale, "sim(step)", true);
        let ca = &report.channels[0];
        assert_eq!(ca.transient.ring_freq_hz, None);
        let upper = ca.transient.ring_freq_upper_hz.expect("单峰应给上界");
        assert_eq!(
            ca.quality.verdicts.get("ring_freq_upper_hz"),
            Some(&Verdict::BoundedAbove { upper }),
            "上界不能写成下界"
        );

        let ctx = EvidenceContext {
            capture: &cap,
            scale: &scale,
            link: "sim(step)",
            simulated: true,
            config: None,
            decode: None,
            decode_req: None,
            question: None,
        };
        let text = render_text(&report, &ctx);
        let line = verdict_line(&ca.quality).expect("应有可信度行");
        assert!(
            line.contains(&format!("上界: 振铃频率上界≤{upper:.1} Hz")),
            "上界没进可信度行：{line}"
        );
        assert!(
            text.contains(&format!("振铃≤{upper:.1} Hz")),
            "上界没进瞬态数值行：{text}"
        );
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
    fn table_external_exact_verdict_is_grouped_as_exact() {
        let mut q = Quality::default();
        q.verdicts
            .insert("future_count".into(), Verdict::Ok { u: 0.0 });
        let line = verdict_line(&q).expect("非空判决应渲染");
        assert!(
            line.contains("可信（精确）: future_count"),
            "u=0 的表外计数应归入「可信（精确）」：{line}"
        );
        assert!(
            !line.contains("可信: future_count"),
            "u=0 的表外计数不应带进「可信」组：{line}"
        );
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
