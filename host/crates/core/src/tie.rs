//! 单次采集内的 TIE 与周期抖动。
//!
//! TIE（Time Interval Error）是逐沿时刻相对**整段采集**的理想格点的偏差。
//! 本模块只做几何估计：不碰标定、不做跨采集累积；没有不确定度模型的量
//! 在出口处为 `Refused`，不生成数值。
//!
//! # 与 `measure` 的关系
//!
//! 边沿判定、迟滞带宽、10–90% 过渡时间全部复用既有实现（[`crate::edges`]
//! 与 [`crate::measure`]），本模块不另造一套口径。TIE 的输入是**同向**
//! （上升）边沿的亚样点时刻；下降沿用同一套插值，只是不参与统计。
//!
//! # 采样密度门
//!
//! 线性插值能给出亚样点时刻，但**无法恢复比一个采样间隔更快的边沿**：
//! 一条物理宽度小于一个采样间隔的边沿，在 10% 与 90% 之间**最多只能落进
//! 一个样点**；此时插值给出的过渡时间只反映采样相位，与真实边沿快慢无关。
//! `docs/11` §4.4 用 `ρ = Tr/Ts ≤ 0.35` 表达 T2 的栅格拒答门，但 0.35 对应
//! 的物理边沿在 857 kSPS 下根本无法被解析 —— 测出来的 Tr 会饱和在
//! 0.8–1.6 个样点。因此本模块在采样数据上用「全段没有任何一条沿含两个
//! 10–90% 内部样点」作为栅格拒答条件（等价于物理 `ρ ≲ 1`，比 0.35 宽，
//! 但它是可实现的），并保留 `ρ ≤ 0.35` 作为额外的保守门。
//!
//! 因此本模块的拒答条件是：
//! - 所有上升沿的 10–90% 内部样点数都 < 2（边沿未被栅格分辨）→
//!   [`TieReason::EdgeTooFastForTie`]；
//! - 否则按测得的 `ρ`：`ρ ≤ 0.35` 拒答，`0.35 < ρ < 2` 报数但标
//!   [`TieStats::grid_limited`]。

use crate::edges::{edges_hysteretic, sigma_hat};
use crate::signal::detect_levels;

/// 同向边沿数下限（docs/11 §10 Q5=A）。
pub const MIN_TIE_EDGES: usize = 30;

/// T2 的栅格拒答门：`ρ = Tr/Ts ≤ 0.35` 时不报 TIE。
pub const MIN_EDGE_RATIO: f64 = 0.35;

/// `ρ ≥ 2` 才进入 T2 的适用区间；`0.35 < ρ < 2` 只报数并标 `grid_limited`。
pub const TRUSTED_EDGE_RATIO: f64 = 2.0;

/// 相位均匀时两点线性插值的误差系数（docs/11 §4.4 的蒙特卡洛核验值）。
pub const KAPPA: f64 = 1.021;

/// 格点周期短于这么多样点就按混叠拒答（半周期 < 2 样点）。
const MIN_GRID_PERIOD_SAMPLES: f64 = 4.0;

/// 间隔稳定性的量化感知门限，与 `signal::MAX_GAP_CV` 同口径。
const MAX_GAP_CV: f64 = 0.10;

/// 单次采集内的非平稳门：前后半段平均间隔的相对差。
///
/// `docs/11` §4.8 的 0.1% 是**跨采集**的 T̂ 比较；单次采集内间隔本身含
/// 抖动，用 0.1% 会把正常抖动误判成漂移。这里取 10%，只拦明显扫频/变频。
const MAX_GAP_DRIFT: f64 = 0.10;

/// 边沿时刻的估计方法。
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TieMethod {
    /// 迟滞边沿判定 + 中点线性插值。
    LinearInterpolation,
}

/// TIE 统计量。全部时间量单位 ns。
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TieStats {
    /// 参与统计的同向边沿数。
    pub n_edges: usize,
    /// 测得的 `ρ = Tr/Ts`。
    pub rho: f64,
    /// TIE 的样本标准差（n−1 分母）。
    pub sigma_tie_ns: f64,
    /// TIE 的峰峰值。
    pub pk_pk_ns: f64,
    /// TIE 的均值。
    pub mean_tie_ns: f64,
    /// σ 的 95% 置信区间下限（卡方）。
    pub ci95_low_ns: f64,
    /// σ 的 95% 置信区间上限（卡方）。
    pub ci95_high_ns: f64,
    /// 本次条件下由 `σ_v`、峰峰值与过渡时间算出的单沿不确定度。
    ///
    /// `κ·(σ_v/pp)·Tr`，`κ = 1.021`；干净信号上为 0 —— 那是数据的性质，
    /// 不是「测不出」。
    pub resolution_floor_ns: f64,
    /// 周期抖动 RMS（相对 T̂ 的偏差，总体 RMS）。
    pub period_jitter_rms_ns: f64,
    /// 周期抖动峰峰值。
    pub period_jitter_pk_pk_ns: f64,
    /// `0.35 < ρ < 2`：数值给出，但插值口径未被 T2 的适用区间覆盖。
    pub grid_limited: bool,
    /// 估计方法。
    pub method: TieMethod,
}

/// TIE 拒答原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TieReason {
    /// 样点太少。
    TooFewSamples,
    /// 同向边沿不足 [`MIN_TIE_EDGES`]。
    TooFewEdges,
    /// 边沿未被采样栅格分辨（或 `ρ ≤ 0.35`）。
    EdgeTooFastForTie,
    /// 格点周期接近奈奎斯特。
    Aliased,
    /// 间隔集合被空闲段或假跃变污染。
    IntervalUnstable,
    /// 单次采集内前后半段周期明显漂移。
    NonStationary,
    /// 找不到可用的 10–90% 过渡。
    ShapeUnrecognized,
}

/// 一次 TIE 分析的全部结果。
///
/// 拒答时 [`TieAnalysis::stats`] 为 `None`、[`TieAnalysis::reason`] 为
/// `Some(..)`；报数时相反。边沿时刻与序列在能算出时照常返回，便于调用方
/// 展示「为什么被拒」（例如边沿数不足时的实际个数）。
#[derive(Debug, Clone, PartialEq)]
pub struct TieAnalysis {
    /// 检测到的同向（上升）边沿数。
    pub n_edges: usize,
    /// 测得的 `ρ = Tr/Ts`（过渡时间不可测时为 `None`）。
    pub rho: Option<f64>,
    /// 插值后的上升沿时刻（ns）。
    pub edge_times_ns: Vec<f64>,
    /// TIE 序列（ns）。拒答时为空。
    pub tie_ns: Vec<f64>,
    /// 周期抖动序列（ns）。拒答时为空。
    pub period_jitter_ns: Vec<f64>,
    /// 统计量；拒答时为 `None`。
    pub stats: Option<TieStats>,
    /// 拒答原因；报数时为 `None`。
    pub reason: Option<TieReason>,
}

/// 对一个通道的样点做 TIE 分析。
///
/// 只使用**上升沿**：时钟的相邻同向沿间隔就是一个周期，正负沿混用会把
/// 占空比误差混进周期抖动。门限、迟滞带宽、过渡时间与 [`crate::measure`]
/// 同一口径。
pub fn analyze(s: &[u16], rate_hz: u32) -> TieAnalysis {
    let mut out = TieAnalysis {
        n_edges: 0,
        rho: None,
        edge_times_ns: Vec::new(),
        tie_ns: Vec::new(),
        period_jitter_ns: Vec::new(),
        stats: None,
        reason: None,
    };

    if s.len() < 16 || rate_hz == 0 {
        out.reason = Some(TieReason::TooFewSamples);
        return out;
    }

    let (mut lo, mut hi) = (u16::MAX, u16::MIN);
    for &v in s {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi <= lo {
        out.reason = Some(TieReason::TooFewEdges);
        return out;
    }

    let mid = (lo as f64 + hi as f64) / 2.0;
    let levels = detect_levels(s);
    let sep = if levels.is_two_level() {
        levels.sep_lsb
    } else {
        (hi - lo) as f64
    };
    let h = sigma_hat(s)
        .map(|sigma| (3.0 * sigma).min(0.25 * sep))
        .unwrap_or(0.0);
    let (rising, _) = edges_hysteretic(s, mid, h);

    out.n_edges = rising.len();
    let edge_times: Vec<f64> = rising
        .iter()
        .filter_map(|&i| {
            let a = s[i - 1] as f64;
            let b = s[i] as f64;
            if b == a {
                None
            } else {
                Some((i - 1) as f64 + (mid - a) / (b - a))
            }
        })
        .collect();
    out.edge_times_ns = edge_times
        .iter()
        .map(|t| t * 1e9 / rate_hz as f64)
        .collect();

    if edge_times.len() < MIN_TIE_EDGES {
        out.reason = Some(TieReason::TooFewEdges);
        return out;
    }

    let gaps: Vec<f64> = edge_times.windows(2).map(|w| w[1] - w[0]).collect();
    let Some(filtered) = filter_gaps_f64(&gaps) else {
        out.reason = Some(TieReason::TooFewEdges);
        return out;
    };
    let period = if filtered.kept.is_empty() {
        filtered.median
    } else {
        filtered.kept.iter().sum::<f64>() / filtered.kept.len() as f64
    };
    if period < MIN_GRID_PERIOD_SAMPLES {
        out.reason = Some(TieReason::Aliased);
        return out;
    }

    // 逐沿的 10–90% 过渡（与 `measure::transition_ns` 同一两点插值形式，
    // 但那个函数只返回全窗第一条边沿；TIE 需要每一条沿的几何）。
    let geometry: Vec<(f64, usize)> = rising
        .iter()
        .filter_map(|&i| edge_geometry(s, lo, hi, i))
        .collect();
    if geometry.len() < MIN_TIE_EDGES {
        out.reason = Some(TieReason::ShapeUnrecognized);
        return out;
    }
    let median_tr = median(&geometry.iter().map(|(tr, _)| *tr).collect::<Vec<_>>());
    let max_interior = geometry.iter().map(|(_, n)| *n).max().unwrap_or(0);
    // Tr 的单位是样点，所以 Tr/Ts 就是它本身 —— 不要再乘一次速率。
    let rho = median_tr;
    out.rho = Some(rho);

    // 栅格分辨：一条物理上短于一个采样间隔的边沿，10–90% 之间**最多只能
    // 落进一个样点**。全段没有任何一条沿含两个内部样点时，插值给出的 Tr
    // 只反映采样相位，不反映真实边沿 —— 这时 ρ 数字没有意义，按拒答处理。
    // 这比 `docs/11` 的 ρ ≤ 0.35 更宽，但它是采样数据上唯一可实现的判据：
    // 0.35 对应的物理边沿在 857 kSPS 下根本不可能被解析出来。
    if max_interior < 2 || rho <= MIN_EDGE_RATIO {
        out.reason = Some(TieReason::EdgeTooFastForTie);
        return out;
    }

    // 空闲段会让「格点」失去意义；假跃变会让短间隔混进来。
    if filtered.dropped > 0 {
        out.reason = Some(TieReason::IntervalUnstable);
        return out;
    }
    if filtered.cv() > MAX_GAP_CV.max(0.5 / period) {
        out.reason = Some(TieReason::IntervalUnstable);
        return out;
    }
    if filtered.drift() > MAX_GAP_DRIFT {
        out.reason = Some(TieReason::NonStationary);
        return out;
    }

    let n = edge_times.len();
    let t0 = edge_times[0];
    let tie: Vec<f64> = edge_times
        .iter()
        .enumerate()
        .map(|(k, &t)| t - (t0 + k as f64 * period))
        .collect();
    let period_jitter: Vec<f64> = gaps.iter().map(|g| g - period).collect();

    let mean = tie.iter().sum::<f64>() / n as f64;
    let df = (n - 1) as f64;
    let var = tie.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / df;
    let sigma = var.sqrt();
    let (tie_min, tie_max) = min_max(&tie);
    let (pj_min, pj_max) = min_max(&period_jitter);
    let m = period_jitter.len() as f64;

    let chi2_hi = chi2_quantile(1.959_963_984_540_054, df);
    let chi2_lo = chi2_quantile(-1.959_963_984_540_054, df);
    let to_ns = 1e9 / rate_hz as f64;
    let sigma_v = sigma_hat(s).unwrap_or(0.0);
    let pp = (hi - lo) as f64;

    out.tie_ns = tie.iter().map(|x| x * to_ns).collect();
    out.period_jitter_ns = period_jitter.iter().map(|x| x * to_ns).collect();
    out.stats = Some(TieStats {
        n_edges: n,
        rho,
        sigma_tie_ns: sigma * to_ns,
        pk_pk_ns: (tie_max - tie_min) * to_ns,
        mean_tie_ns: mean * to_ns,
        ci95_low_ns: (df * var / chi2_hi).sqrt() * to_ns,
        ci95_high_ns: (df * var / chi2_lo).sqrt() * to_ns,
        resolution_floor_ns: KAPPA * (sigma_v / pp) * median_tr * to_ns,
        period_jitter_rms_ns: (period_jitter.iter().map(|x| x * x).sum::<f64>() / m).sqrt() * to_ns,
        period_jitter_pk_pk_ns: (pj_max - pj_min) * to_ns,
        grid_limited: rho < TRUSTED_EDGE_RATIO,
        method: TieMethod::LinearInterpolation,
    });
    out
}

/// 插值后的间隔过滤 —— 与 [`crate::measure::filter_gaps`] 同一口径
/// （中位数 ×1.5），只是输入从整数索引换成亚样点间隔。
struct GapFilterF64 {
    median: f64,
    dropped: usize,
    kept: Vec<f64>,
}

impl GapFilterF64 {
    fn mean(&self) -> Option<f64> {
        if self.kept.is_empty() {
            return None;
        }
        let m = self.kept.iter().sum::<f64>() / self.kept.len() as f64;
        (m > 0.0).then_some(m)
    }

    fn cv(&self) -> f64 {
        let Some(mean) = self.mean() else {
            return f64::INFINITY;
        };
        let var = self
            .kept
            .iter()
            .map(|g| (g - mean) * (g - mean))
            .sum::<f64>()
            / self.kept.len() as f64;
        var.sqrt() / mean
    }

    fn drift(&self) -> f64 {
        if self.kept.len() < 4 {
            return 0.0;
        }
        let Some(mean) = self.mean() else {
            return 0.0;
        };
        let half = self.kept.len() / 2;
        let first = self.kept[..half].iter().sum::<f64>() / half as f64;
        let second = self.kept[half..].iter().sum::<f64>() / (self.kept.len() - half) as f64;
        ((second - first).abs() / mean).min(1.0)
    }
}

fn filter_gaps_f64(gaps: &[f64]) -> Option<GapFilterF64> {
    if gaps.len() < 2 {
        return None;
    }
    let mut sorted = gaps.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = sorted[sorted.len() / 2];
    let cutoff = median * 1.5;
    let kept: Vec<f64> = gaps.iter().copied().filter(|g| *g <= cutoff).collect();
    Some(GapFilterF64 {
        median,
        dropped: gaps.len() - kept.len(),
        kept,
    })
}

/// 一条上升沿的 10–90% 过渡时间（样点）与严格落在两个门限内的样点数。
///
/// 内部样点数是「边沿是否被采样栅格解析」的可观测量：物理宽度小于一个
/// 采样间隔的边沿，内部样点数**不可能**达到 2。
fn edge_geometry(s: &[u16], lo: u16, hi: u16, i_mid: usize) -> Option<(f64, usize)> {
    let (lo_f, hi_f) = (lo as f64, hi as f64);
    let t10 = lo_f + 0.1 * (hi_f - lo_f);
    let t90 = lo_f + 0.9 * (hi_f - lo_f);

    let j10 = (1..=i_mid)
        .rev()
        .find(|&j| (s[j - 1] as f64) < t10 && (s[j] as f64) >= t10)?;
    let j90 = (i_mid..s.len()).find(|&j| (s[j - 1] as f64) < t90 && (s[j] as f64) >= t90)?;
    if j90 < j10 {
        return None;
    }

    let cross = |j: usize, t: f64| -> f64 {
        let a = s[j - 1] as f64;
        let b = s[j] as f64;
        (j - 1) as f64 + (t - a) / (b - a)
    };
    let interior = (j10..j90)
        .filter(|&k| {
            let v = s[k] as f64;
            v > t10 && v < t90
        })
        .count();
    Some((cross(j90, t90) - cross(j10, t10), interior))
}

fn median(v: &[f64]) -> f64 {
    let mut sorted = v.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = sorted.len();
    if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    }
}

fn min_max(v: &[f64]) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &x in v {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    (lo, hi)
}

/// 卡方分位数的 Wilson–Hilferty 近似（df = n−1 ≥ 29，误差 < 0.3%）。
///
/// `z` 是标准正态分位数：传 ±1.959964 得到 95% 区间的两端。
fn chi2_quantile(z: f64, df: f64) -> f64 {
    let t = 1.0 - 2.0 / (9.0 * df) + z * (2.0 / (9.0 * df)).sqrt();
    df * t * t * t
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: u32 = 1_000_000;

    /// 分段线性梯形时钟。`rises[k] = 4 + Σ gaps[..k]`。
    ///
    /// 边沿是严格的直线段，所以中点穿越的线性插值与解析时刻**逐位一致**；
    /// 测试断言的 TIE 由构造式直接给出，不依赖被测实现。
    fn clock_from_gaps(len: usize, gaps: &[f64], width: f64, noise_seed: Option<u32>) -> Vec<u16> {
        const LO: f64 = 500.0;
        const HI: f64 = 3500.0;

        let mut rises = vec![4.0];
        while *rises.last().unwrap() < len as f64 + 2.0 * width {
            let k = rises.len() - 1;
            rises.push(rises[k] + gaps[k % gaps.len()]);
        }
        let mut edges: Vec<(f64, bool)> = Vec::with_capacity(rises.len() * 2);
        for &r in &rises {
            edges.push((r, true));
            edges.push((r + 16.0, false));
        }

        let mut seed = noise_seed;
        (0..len)
            .map(|i| {
                let t = i as f64;
                let mut v = LO;
                let mut level: Option<bool> = None;
                for &(e, rising) in &edges {
                    if (t - e).abs() <= width / 2.0 {
                        let frac = (t - (e - width / 2.0)) / width;
                        v = if rising {
                            LO + (HI - LO) * frac
                        } else {
                            HI - (HI - LO) * frac
                        };
                        level = None;
                        break;
                    }
                    if e <= t {
                        level = Some(rising);
                    }
                }
                if let Some(rising) = level {
                    v = if rising { HI } else { LO };
                }
                if let Some(x) = seed.as_mut() {
                    v += 5.0 * approx_gaussian(x);
                }
                v.clamp(0.0, 4095.0).round() as u16
            })
            .collect()
    }

    /// 三个独立均匀分布之和：σ = 1（近似高斯，避免均匀噪声把 MAD 估计器
    /// 带偏）。调用方乘上目标 σ_v。
    fn approx_gaussian(seed: &mut u32) -> f64 {
        let mut draw = || {
            *seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            ((*seed >> 16) % 2001) as f64 / 1000.0 - 1.0
        };
        draw() + draw() + draw()
    }

    fn clock(len: usize, period: f64, width: f64, jitter: &[f64], noise: Option<u32>) -> Vec<u16> {
        // 直接按上升沿偏移构造，避免累积误差进入解析真值。
        const LO: f64 = 500.0;
        const HI: f64 = 3500.0;
        let mut rises = Vec::new();
        let mut k = 0usize;
        loop {
            let r = 4.0 + k as f64 * period + jitter[k % jitter.len().max(1)];
            if r > len as f64 + width {
                break;
            }
            rises.push(r);
            k += 1;
        }
        let mut edges: Vec<(f64, bool)> = Vec::with_capacity(rises.len() * 2);
        for &r in &rises {
            edges.push((r, true));
            edges.push((r + period / 2.0, false));
        }
        let mut seed = noise;
        (0..len)
            .map(|i| {
                let t = i as f64;
                let mut v = LO;
                let mut level: Option<bool> = None;
                for &(e, rising) in &edges {
                    if (t - e).abs() <= width / 2.0 {
                        let frac = (t - (e - width / 2.0)) / width;
                        v = if rising {
                            LO + (HI - LO) * frac
                        } else {
                            HI - (HI - LO) * frac
                        };
                        level = None;
                        break;
                    }
                    if e <= t {
                        level = Some(rising);
                    }
                }
                if let Some(rising) = level {
                    v = if rising { HI } else { LO };
                }
                if let Some(x) = seed.as_mut() {
                    v += 5.0 * approx_gaussian(x);
                }
                v.clamp(0.0, 4095.0).round() as u16
            })
            .collect()
    }

    /// 与模拟器 `uart_115k` 同构的 0x55/0x0F/0xA5 帧流。
    fn uart_115k(len: usize) -> Vec<u16> {
        const BAUD: u64 = 115_200;
        const RATE: u32 = 857_142;
        const BYTES: [u8; 3] = [0x55, 0x0F, 0xA5];
        const FRAME_BITS: u64 = 12;

        (0..len)
            .map(|i| {
                let pos = (i as u64 * BAUD / RATE as u64) % (BYTES.len() as u64 * FRAME_BITS);
                let byte = BYTES[(pos / FRAME_BITS) as usize];
                let within = pos % FRAME_BITS;
                let high = match within {
                    0 => false,
                    1..=8 => byte & (1 << (within - 1)) != 0,
                    _ => true,
                };
                if high {
                    3500
                } else {
                    500
                }
            })
            .collect()
    }

    #[test]
    fn tie_matches_the_analytic_truth_of_a_jittered_clock() {
        // 62.5 kHz、10–90% = 2.4 样点 → ρ = 2.4；周期抖动模式
        // [0, +0.5, 0, −0.5] 样点。
        //
        // 解析真值必须用**实际估出的格点周期** T̂ 展开：128 个沿只有 127 个
        // 间隔，而抖动模式在这 127 个间隔上并不零和 ——
        // T̂ = (e_127 − e_0)/127 = 31.99606 样点，不是 32。差值 0.00394 样点
        // 会让 TIE 端到端累积半个样点的斜坡，直接改变 σ 与均值。
        const PATTERN: [f64; 4] = [0.0, 0.5, 0.0, -0.5];
        let s = clock(4096, 32.0, 3.0, &PATTERN, None);
        let a = analyze(&s, RATE);
        assert_eq!(a.n_edges, 128, "解析构型应有 128 个上升沿");
        assert!(a.reason.is_none(), "可测构型不应拒答：{:?}", a.reason);
        let t = a.stats.expect("应有统计量");
        assert_eq!(t.n_edges, 128);
        assert!(!t.grid_limited);
        assert_eq!(t.method, TieMethod::LinearInterpolation);
        let rho = a.rho.unwrap();
        assert!((2.0..3.5).contains(&rho), "ρ={rho} 应落在 T2 可用区");
        assert!((t.rho - rho).abs() < 1e-12, "统计量里的 ρ 应与顶层一致");

        // 独立的构造式真值：只由生成器的解析沿位置导出，不读实现内部量。
        let edges: Vec<f64> = (0..128)
            .map(|k| 4.0 + 32.0 * k as f64 + PATTERN[k % PATTERN.len()])
            .collect();
        let that = (edges[127] - edges[0]) / 127.0;
        let expected_tie: Vec<f64> = edges
            .iter()
            .enumerate()
            .map(|(k, &e)| e - (edges[0] + k as f64 * that))
            .collect();
        let expected_pj: Vec<f64> = edges.windows(2).map(|w| w[1] - w[0] - that).collect();

        let sample_ns = 1e9 / RATE as f64;
        let want_sigma = sample_sigma(&expected_tie) * sample_ns;
        let want_mean = mean(&expected_tie) * sample_ns;
        let want_pk_pk = (max_of(&expected_tie) - min_of(&expected_tie)) * sample_ns;
        let want_pj_rms =
            (expected_pj.iter().map(|x| x * x).sum::<f64>() / expected_pj.len() as f64).sqrt()
                * sample_ns;
        let want_pj_pk_pk = (max_of(&expected_pj) - min_of(&expected_pj)) * sample_ns;

        assert!(
            (t.sigma_tie_ns - want_sigma).abs() < 1e-6,
            "σ_TIE={} ns，解析值 {} ns",
            t.sigma_tie_ns,
            want_sigma
        );
        assert!(
            (t.mean_tie_ns - want_mean).abs() < 1e-6,
            "均值 {} ns，解析值 {} ns（T̂ 偏 0.0039 样点会累积半个样点的斜坡）",
            t.mean_tie_ns,
            want_mean
        );
        assert!(
            (t.pk_pk_ns - want_pk_pk).abs() < 1e-6,
            "峰峰值 {} ns，解析值 {} ns",
            t.pk_pk_ns,
            want_pk_pk
        );
        assert!(t.ci95_low_ns < t.sigma_tie_ns && t.sigma_tie_ns < t.ci95_high_ns);
        assert!(
            (t.period_jitter_rms_ns - want_pj_rms).abs() < 1e-6,
            "周期抖动 RMS={} ns，解析值 {} ns",
            t.period_jitter_rms_ns,
            want_pj_rms
        );
        assert!(
            (t.period_jitter_pk_pk_ns - want_pj_pk_pk).abs() < 1e-6,
            "周期抖动峰峰值={} ns，解析值 {} ns",
            t.period_jitter_pk_pk_ns,
            want_pj_pk_pk
        );
        assert_eq!(t.resolution_floor_ns, 0.0, "无噪声时单沿地板应为 0");

        // TIE 序列逐点对账：线性斜坡上的中点插值是精确的，误差只来自浮点。
        assert_eq!(a.tie_ns.len(), expected_tie.len());
        for (k, (got, want)) in a.tie_ns.iter().zip(&expected_tie).enumerate() {
            assert!(
                (got - want * sample_ns).abs() < 1e-6,
                "第 {k} 个 TIE={got} ns，解析值 {} ns",
                want * sample_ns
            );
        }
        assert_eq!(a.period_jitter_ns.len(), expected_pj.len());
        for (k, (got, want)) in a.period_jitter_ns.iter().zip(&expected_pj).enumerate() {
            assert!(
                (got - want * sample_ns).abs() < 1e-6,
                "第 {k} 个周期抖动={got} ns，解析值 {} ns",
                want * sample_ns
            );
        }
    }

    fn mean(v: &[f64]) -> f64 {
        v.iter().sum::<f64>() / v.len() as f64
    }

    fn sample_sigma(v: &[f64]) -> f64 {
        let m = mean(v);
        (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (v.len() - 1) as f64).sqrt()
    }

    fn max_of(v: &[f64]) -> f64 {
        v.iter().copied().fold(f64::NEG_INFINITY, f64::max)
    }

    fn min_of(v: &[f64]) -> f64 {
        v.iter().copied().fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn thirty_edges_is_the_minimum_for_statistics() {
        let make = |cycles: usize| {
            // 最后一个上升沿之后再留 20 样点，让最后一个下降沿完整。
            let len = 4 + (cycles - 1) * 32 + 20;
            clock(len, 32.0, 3.0, &[0.0], None)
        };

        let a29 = analyze(&make(29), RATE);
        assert_eq!(a29.n_edges, 29);
        assert!(a29.stats.is_none());
        assert_eq!(a29.reason, Some(TieReason::TooFewEdges));

        for cycles in [30, 31] {
            let a = analyze(&make(cycles), RATE);
            assert_eq!(a.n_edges, cycles, "{cycles} 沿构型边沿数不符");
            assert!(a.reason.is_none(), "{cycles} 沿应报数：{:?}", a.reason);
            assert_eq!(a.stats.unwrap().n_edges, cycles);
        }
    }

    #[test]
    fn a_square_like_edge_faster_than_the_grid_is_refused() {
        // 50 kHz、全摆幅 400 ns（10–90% 320 ns → ρ ≈ 0.27）。
        // 边沿只占 0.34 样点，10%/90% 必然落在同一采样间隔内。
        let period = 857_142.0 / 50_000.0;
        let s = clock(4096, period, 400.0e-9 * 857_142.0, &[0.0], None);
        let a = analyze(&s, 857_142);
        assert!(a.n_edges >= MIN_TIE_EDGES);
        assert_eq!(a.reason, Some(TieReason::EdgeTooFastForTie));
        assert!(a.stats.is_none());
    }

    #[test]
    fn uart_edges_are_refused_as_too_fast_for_tie() {
        let a = analyze(&uart_115k(4096), 857_142);
        assert!(a.n_edges >= MIN_TIE_EDGES);
        assert_eq!(
            a.reason,
            Some(TieReason::EdgeTooFastForTie),
            "UART 的位沿是瞬时跳变，应走栅格拒答"
        );
        assert!(a.stats.is_none());
    }

    #[test]
    fn a_gapped_clock_is_refused_as_interval_unstable() {
        // 每 5 个周期插入一段 20 样点的空闲：长间隔被中位数 ×1.5 过滤掉。
        let gaps: Vec<f64> = (0..128)
            .map(|k| if k % 5 == 4 { 52.0 } else { 32.0 })
            .collect();
        let s = clock_from_gaps(4096, &gaps, 3.0, None);
        let a = analyze(&s, RATE);
        assert_eq!(a.reason, Some(TieReason::IntervalUnstable));
        assert!(a.stats.is_none());
    }

    #[test]
    fn a_drifting_clock_is_refused_as_non_stationary() {
        // 间隔从 32 线性增到约 42 样点：cv ≈ 8% 通过稳定性门，
        // 但前后半段均值差 ≈ 14% 触发非平稳门。
        let gaps: Vec<f64> = (0..128).map(|k| 32.0 + 0.08 * k as f64).collect();
        let s = clock_from_gaps(4096, &gaps, 3.0, None);
        let a = analyze(&s, RATE);
        assert_eq!(a.reason, Some(TieReason::NonStationary));
        assert!(a.stats.is_none());
    }

    #[test]
    fn a_clock_between_the_two_rho_gates_is_reported_as_grid_limited() {
        // 全摆幅 2 样点、相位 4.25：10–90% 内含两个内部样点（可解析），
        // 但插值 Tr = 1.93 样点 → 0.35 < ρ < 2，报数并标网格受限。
        let s = clock(4096, 32.0, 2.0, &[0.25], None);
        let a = analyze(&s, RATE);
        let t = a.stats.expect("网格受限区仍应报数");
        let rho = a.rho.unwrap();
        assert!((0.35..2.0).contains(&rho), "ρ={rho} 应落在网格受限区");
        assert!(t.grid_limited, "rho={rho} 应标 grid_limited");
        assert!(t.sigma_tie_ns >= 0.0);
    }

    /// Q7-B 的替代证据：网格受限区（0.35 < ρ < 2）也拿构造式真值对一次账。
    ///
    /// Q7-B 原计划用 `slow_edge_noisy` 实测这一区间，但该场景 1 kHz、4096 点
    /// 内只有约 4 条同向沿，达不到 N_min = 30 —— TIE 会先按 `TooFewEdges`
    /// 拒答，根本走不到 ρ 门。这里用同构合成时钟（2 样点线性沿、已知逐沿
    /// 偏移、无噪声）验证：实现没有在 ρ < 2 时引入额外偏差。
    ///
    /// 它**不**证明真实边沿可用 —— 真实边沿在跨界两点间偏离线性，而那正是
    /// 保留 `grid_limited` 标记与拒答门的理由（docs/11 §11.3 冲突 3）。
    #[test]
    fn a_grid_limited_clock_still_matches_the_constructed_truth() {
        const PATTERN: [f64; 4] = [0.0, 0.5, 0.0, -0.5];
        let s = clock(4096, 32.0, 2.0, &PATTERN, None);
        let a = analyze(&s, RATE);
        let t = a.stats.expect("网格受限区仍应报数");
        let rho = a.rho.unwrap();
        assert!((0.35..2.0).contains(&rho), "ρ={rho} 应落在网格受限区");
        assert!(t.grid_limited, "rho={rho} 应标 grid_limited");

        let edges: Vec<f64> = (0..128)
            .map(|k| 4.0 + 32.0 * k as f64 + PATTERN[k % PATTERN.len()])
            .collect();
        let that = (edges[127] - edges[0]) / 127.0;
        let sample_ns = 1e9 / RATE as f64;
        let expected_tie: Vec<f64> = edges
            .iter()
            .enumerate()
            .map(|(k, &e)| (e - (edges[0] + k as f64 * that)) * sample_ns)
            .collect();
        let expected_pj: Vec<f64> = edges
            .windows(2)
            .map(|w| (w[1] - w[0] - that) * sample_ns)
            .collect();

        let want_sigma = sample_sigma(&expected_tie);
        assert!(
            (t.sigma_tie_ns - want_sigma).abs() < 1e-6,
            "σ_TIE={} ns，解析值 {want_sigma} ns",
            t.sigma_tie_ns
        );
        assert_eq!(a.tie_ns.len(), expected_tie.len());
        for (k, (got, want)) in a.tie_ns.iter().zip(&expected_tie).enumerate() {
            assert!(
                (got - want).abs() < 1e-6,
                "第 {k} 个 TIE={got} ns，解析值 {want} ns"
            );
        }
        assert_eq!(a.period_jitter_ns.len(), expected_pj.len());
        for (k, (got, want)) in a.period_jitter_ns.iter().zip(&expected_pj).enumerate() {
            assert!(
                (got - want).abs() < 1e-6,
                "第 {k} 个周期抖动={got} ns，解析值 {want} ns"
            );
        }
    }

    #[test]
    fn resolution_floor_follows_kappa_times_noise_over_slope() {
        // 无周期抖动、白噪声 σ ≈ 5 LSB、10–90% = 2.4 样点：
        // floor = 1.021 × (5/4000) × 2400 ns ≈ 3.06 ns。
        let s = clock(4096, 32.0, 3.0, &[0.0], Some(0x5EED_1234));
        let a = analyze(&s, RATE);
        let t = a.stats.expect("含噪慢边沿应可测");
        assert!(
            (3.0..=8.0).contains(&t.resolution_floor_ns),
            "单沿地板 {} ns 不在预期区间",
            t.resolution_floor_ns
        );
        assert!(
            t.sigma_tie_ns > 1.0 && t.sigma_tie_ns < 30.0,
            "含噪 TIE σ={} ns 不在预期量级",
            t.sigma_tie_ns
        );

        // 钉住 κ 本身（docs/11 §10 冻结裁决取 1.021，对外承诺 ≥90× 改善）。
        // 期望值在**测试里**用字面量 1.021 独立复算，不读 `KAPPA` 常量 ——
        // 否则常量改成任何值测试都绿（审查变异实测：1.021 → 0.884 时旧的
        // 区间断言 (3.0..=8.0) 抓不住）。
        let sigma_v = sigma_hat(&s).expect("含噪场景应能估出 σ̂");
        let (lo, hi) = s
            .iter()
            .fold((u16::MAX, u16::MIN), |(l, h), &v| (l.min(v), h.max(v)));
        let expected = 1.021 * (sigma_v / (hi - lo) as f64) * t.rho * (1e9 / RATE as f64);
        assert!(
            (t.resolution_floor_ns - expected).abs() < 1e-9,
            "单沿地板 {} ns 与 κ=1.021 的复算值 {} ns 不一致 —— \
             KAPPA 常量偏离了冻结裁决",
            t.resolution_floor_ns,
            expected
        );
    }

    #[test]
    fn tie_is_deterministic() {
        let s = clock(4096, 32.0, 3.0, &[0.0, 0.5, 0.0, -0.5], None);
        assert_eq!(analyze(&s, RATE), analyze(&s, RATE));
    }

    #[test]
    fn a_short_capture_refuses_everything_with_a_reason() {
        let a = analyze(&[0, 4000, 0, 4000, 0, 4000, 0, 4000], RATE);
        assert_eq!(a.reason, Some(TieReason::TooFewSamples));
        assert!(a.stats.is_none());
        assert!(a.tie_ns.is_empty());
    }
}
