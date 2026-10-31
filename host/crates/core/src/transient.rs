//! 瞬态特征 —— 振铃、阻尼、调节时间、RC 时间常数、压摆率、runt。
//!
//! 这一层只做**形状估计**，不碰标定，也不把「拟合成功」直接当成
//! 「可信」：数值由本模块给出，可信度由 [`crate::analysis::Quality`] 按
//! 每个量的模型决定。没有不确定度模型的量在出口处是 `Refused`，
//! 而不是 `Ok { u: 0.0 }`。
//!
//! # 边界
//!
//! - 输入是**原始样点 + 采样率**，与 [`crate::measure`] 同源；
//! - 振铃 / 阻尼 / 调节时间 / runt 需要双电平参考，判不出双电平时
//!   这些量是 `NotApplicable`；
//! - RC τ 与压摆率不要求双电平，它们在边沿 / 单调段上定义；
//! - 所有时间量输出 ns，压摆率输出 LSB/µs（未标定时保持 LSB 域，
//!   不做未经标定的电压换算）。

use std::collections::BTreeMap;

use crate::edges::{edges_hysteretic, sigma_hat};
use crate::measure::{transition_ns, EdgeDir};
use crate::signal::detect_levels;

/// 少于这么多样点，瞬态量一律判不出。
const MIN_SAMPLES: usize = 16;

/// 振铃峰值的最小突出量（相对平台间隔）。
const MIN_RING_PROMINENCE_FRAC: f64 = 0.02;

/// 调节时间带的宽度（相对平台间隔）。
const SETTLE_BANDS: [f64; 2] = [0.02, 0.05];

/// RC τ 至少需要的单调段长度（样点）。
const MIN_TAU_RUN: usize = 8;

/// RC τ 的多个采样间距（样点）。
const TAU_STRIDES: [usize; 4] = [4, 8, 16, 32];

/// RC τ 估计的一致性上限（MAD / 中位数）。
const MAX_TAU_REL_SPREAD: f64 = 0.20;

/// 三点闭式里每个差分至少要这么大（LSB）。
///
/// 12-bit 量化让每个样点带 ±0.5 LSB，差分误差 ±1 LSB。指数尾部的差分只有
/// 几个 LSB，此时「比值」几乎全是量化噪声。实测（`rc_charge`，τ = 128.57
/// 样点）：门限 8 LSB 时 d=4 的估计中位数只有 82 样点，d=8 是 122；
/// 门限 16 LSB 时 d=8/16/32 都收敛到 128.6–128.8（误差 < 0.2%）。
/// d=4 即使 16 LSB 仍偏低，但它只占四个步长之一，中位数由其余三个拉住。
const MIN_TAU_DELTA_LSB: f64 = 16.0;

/// 压摆率最大值受噪声支配时，噪声底与单样点边沿增量的比值上限。
const MAX_SLEW_NOISE_FRAC: f64 = 0.50;

/// 一个通道的瞬态估计值。
///
/// `None` 表示**测不出**；具体原因在 [`TransientAnalysis::reasons`] 里。
/// `ring_freq_upper_hz` 只在「有峰但半周期不足 4 样点」或「只有一个峰」时
/// 给上界，此时 `ring_freq_hz` 为 `None`。
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TransientStats {
    /// 振铃频率（Hz）。峰值间隔的倒数。
    pub ring_freq_hz: Option<f64>,
    /// 振铃频率上界（Hz）。只给上界时使用。
    pub ring_freq_upper_hz: Option<f64>,
    /// 阻尼比 ζ。由首个过冲峰值反解。
    pub damping_ratio: Option<f64>,
    /// 调节时间（ns，±2% 带）。
    pub settle_2_ns: Option<f64>,
    /// 调节时间（ns，±5% 带）。
    pub settle_5_ns: Option<f64>,
    /// RC 时间常数 τ（ns）。
    pub rc_tau_ns: Option<f64>,
    /// 边沿区间内最大 |Δv/Δt|（LSB/µs）。
    pub slew_max_lsb_per_us: Option<f64>,
    /// 10%→90% 平均斜率（LSB/µs）。
    pub slew_avg_lsb_per_us: Option<f64>,
    /// 越过中点但未达到第二门限的脉冲段总数（高 + 低）。
    pub runts: Option<u32>,
}

/// 瞬态量测不出的原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransientReason {
    /// 样点太少。
    TooFewSamples,
    /// 该量对这种波形没有定义（例如非双电平信号的振铃）。
    NotApplicable,
    /// 没有找到可拟合的形状。
    ShapeUnrecognized,
    /// 采样密度不足（混叠）。
    Aliased,
}

/// [`analyze`] 的完整结果：数值 + 每个量的失败原因。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TransientAnalysis {
    /// 数值。
    pub stats: TransientStats,
    /// 键与字段同名；只记录**失败**的量。
    pub reasons: BTreeMap<&'static str, TransientReason>,
}

const RING_KEY: &str = "ring_freq_hz";
const DAMPING_KEY: &str = "damping_ratio";
const SETTLE_2_KEY: &str = "settle_2_ns";
const SETTLE_5_KEY: &str = "settle_5_ns";
const TAU_KEY: &str = "rc_tau_ns";
const SLEW_MAX_KEY: &str = "slew_max_lsb_per_us";
const SLEW_AVG_KEY: &str = "slew_avg_lsb_per_us";
const RUNT_KEY: &str = "runts";

const ALL_KEYS: [&str; 8] = [
    RING_KEY,
    DAMPING_KEY,
    SETTLE_2_KEY,
    SETTLE_5_KEY,
    TAU_KEY,
    SLEW_MAX_KEY,
    SLEW_AVG_KEY,
    RUNT_KEY,
];

fn fail(out: &mut TransientAnalysis, key: &'static str, reason: TransientReason) {
    out.reasons.insert(key, reason);
}

fn fail_all(out: &mut TransientAnalysis, reason: TransientReason) {
    for key in ALL_KEYS {
        fail(out, key, reason);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    Rising,
    Falling,
}

#[derive(Debug, Clone, Copy)]
struct Step {
    start: usize,
    end: usize,
    target: f64,
    dir: Dir,
}

fn min_max(s: &[u16]) -> (u16, u16) {
    let mut lo = u16::MAX;
    let mut hi = u16::MIN;
    for &v in s {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    (lo, hi)
}

fn residual(v: u16, target: f64, dir: Dir) -> f64 {
    match dir {
        Dir::Rising => v as f64 - target,
        Dir::Falling => target - v as f64,
    }
}

/// 一次跃变之后到下一个反向沿之前的片段。目标电平取平台的均值。
fn steps(
    rising: &[usize],
    falling: &[usize],
    n: usize,
    levels: &crate::signal::LevelStats,
) -> Vec<Step> {
    let mut out = Vec::new();
    for &r in rising {
        let end = falling.iter().copied().find(|&f| f > r).unwrap_or(n);
        if end > r + 1 {
            out.push(Step {
                start: r,
                end,
                target: levels.vhi_lsb,
                dir: Dir::Rising,
            });
        }
    }
    for &f in falling {
        let end = rising.iter().copied().find(|&r| r > f).unwrap_or(n);
        if end > f + 1 {
            out.push(Step {
                start: f,
                end,
                target: levels.vlo_lsb,
                dir: Dir::Falling,
            });
        }
    }
    out.sort_by_key(|s| s.start);
    out
}

/// 片段内残差的局部极大值（上升段的过冲峰 / 下降段的下冲峰）。
fn segment_peaks(s: &[u16], seg: &Step, min_prominence: f64) -> Vec<usize> {
    let mut peaks = Vec::new();
    if seg.end <= seg.start + 2 {
        return peaks;
    }
    for i in (seg.start + 1)..(seg.end - 1) {
        let cur = residual(s[i], seg.target, seg.dir);
        if cur < min_prominence {
            continue;
        }
        let left = residual(s[i - 1], seg.target, seg.dir);
        let right = residual(s[i + 1], seg.target, seg.dir);
        if cur >= left && cur > right {
            peaks.push(i);
        }
    }
    peaks
}

fn mean(v: &[f64]) -> Option<f64> {
    if v.is_empty() {
        None
    } else {
        Some(v.iter().sum::<f64>() / v.len() as f64)
    }
}

fn median(v: &[f64]) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    let mut sorted = v.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = sorted.len();
    Some(if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    })
}

fn estimate_ring(
    a: &mut TransientAnalysis,
    s: &[u16],
    rate_hz: u32,
    steps: &[Step],
    sep: f64,
    sigma: f64,
) {
    let min_prominence = (MIN_RING_PROMINENCE_FRAC * sep).max(3.0 * sigma).max(2.0);
    let mut intervals = Vec::new();
    let mut peaks_total = 0usize;
    for seg in steps {
        let peaks = segment_peaks(s, seg, min_prominence);
        peaks_total += peaks.len();
        for w in peaks.windows(2) {
            intervals.push((w[1] - w[0]) as f64 / rate_hz as f64);
        }
    }

    let Some(mean_interval) = mean(&intervals) else {
        if peaks_total == 1 {
            // 只看到一个峰：给不出间隔，只能给上界。
            a.stats.ring_freq_upper_hz = Some(rate_hz as f64 / 8.0);
        } else {
            fail(a, RING_KEY, TransientReason::ShapeUnrecognized);
        }
        return;
    };
    if !(mean_interval.is_finite() && mean_interval > 0.0) {
        fail(a, RING_KEY, TransientReason::ShapeUnrecognized);
        return;
    }

    let f = 1.0 / mean_interval;
    let half_samples = rate_hz as f64 / (2.0 * f);
    if half_samples < 2.0 {
        fail(a, RING_KEY, TransientReason::Aliased);
    } else if half_samples < 4.0 {
        a.stats.ring_freq_upper_hz = Some(rate_hz as f64 / 8.0);
    } else {
        a.stats.ring_freq_hz = Some(f);
    }
}

fn estimate_damping(a: &mut TransientAnalysis, s: &[u16], steps: &[Step], sep: f64, sigma: f64) {
    let min_prominence = (MIN_RING_PROMINENCE_FRAC * sep).max(3.0 * sigma).max(2.0);
    let mut zetas = Vec::new();
    for seg in steps {
        let peaks = segment_peaks(s, seg, min_prominence);
        let Some(&first) = peaks.first() else {
            continue;
        };
        let mp = residual(s[first], seg.target, seg.dir) / sep;
        if !(mp.is_finite() && mp > 0.0) {
            continue;
        }
        let ln = mp.ln();
        let denom = (std::f64::consts::PI.powi(2) + ln * ln).sqrt();
        if denom > 0.0 {
            let z = ln.abs() / denom;
            if z.is_finite() && z >= 0.0 {
                zetas.push(z);
            }
        }
    }
    match mean(&zetas) {
        Some(z) => a.stats.damping_ratio = Some(z),
        None => fail(a, DAMPING_KEY, TransientReason::ShapeUnrecognized),
    }
}

fn estimate_settling(
    a: &mut TransientAnalysis,
    s: &[u16],
    rate_hz: u32,
    steps: &[Step],
    sep: f64,
    band_frac: f64,
    key: &'static str,
) {
    let band = band_frac * sep;
    let mut values = Vec::new();
    for seg in steps {
        let mut last_outside = None;
        let mut entered = false;
        for (i, &v) in s.iter().enumerate().take(seg.end).skip(seg.start) {
            if (v as f64 - seg.target).abs() > band {
                last_outside = Some(i);
            } else {
                entered = true;
            }
        }
        let Some(last) = last_outside else {
            // 一开始就在带内：跃变快于采样栅格，片段内没有可分辨的
            // 调节过程。记 0 ns 会伪装成「瞬间稳定」的精确测量，与
            // 「直到片段结束都没进入带内」同样按无有效过程跳过；
            // 全部片段都跳过时由 mean 的 None 走拒答。
            continue;
        };
        if !entered {
            // 直到片段结束都没进入带内 —— 这次跃变没有调节时间。
            continue;
        }
        // 结尾仍在外侧：下一个沿（或窗尾）打断了调节过程，不能当作已稳定。
        if last + 1 >= seg.end.saturating_sub(1) {
            continue;
        }
        values.push((last + 1 - seg.start) as f64 * 1e9 / rate_hz as f64);
    }
    match mean(&values) {
        Some(v) => match key {
            SETTLE_2_KEY => a.stats.settle_2_ns = Some(v),
            SETTLE_5_KEY => a.stats.settle_5_ns = Some(v),
            _ => unreachable!("调节时间只有两个键"),
        },
        None => fail(a, key, TransientReason::ShapeUnrecognized),
    }
}

fn count_runts(s: &[u16], mid: f64, vlo: f64, vhi: f64, sep: f64) -> u32 {
    let high_limit = vhi - 0.1 * sep;
    let low_limit = vlo + 0.1 * sep;
    let mut count = 0u32;
    let mut i = 0usize;
    while i < s.len() {
        if (s[i] as f64) > mid {
            let start = i;
            let mut max = u16::MIN;
            while i < s.len() && (s[i] as f64) > mid {
                max = max.max(s[i]);
                i += 1;
            }
            if i - start >= 2 && (max as f64) < high_limit {
                count += 1;
            }
        } else if (s[i] as f64) < mid {
            let start = i;
            let mut min = u16::MAX;
            while i < s.len() && (s[i] as f64) < mid {
                min = min.min(s[i]);
                i += 1;
            }
            if i - start >= 2 && (min as f64) > low_limit {
                count += 1;
            }
        } else {
            i += 1;
        }
    }
    count
}

fn monotonic_runs(s: &[u16], start: usize, end: usize, dir: Dir) -> Vec<(usize, usize)> {
    let mut runs = Vec::new();
    let mut i = start;
    while i < end {
        let mut j = i + 1;
        while j < end {
            let ok = match dir {
                Dir::Rising => s[j] >= s[j - 1],
                Dir::Falling => s[j] <= s[j - 1],
            };
            if !ok {
                break;
            }
            j += 1;
        }
        if j - i >= MIN_TAU_RUN {
            runs.push((i, j));
        }
        i = j;
    }
    runs
}

/// 三点闭式 τ 估计：在单调段上用多个采样间距各算一组，要求各组中位数一致。
fn estimate_tau(a: &mut TransientAnalysis, s: &[u16], rate_hz: u32, steps: &[Step], sep: f64) {
    let mut per_stride_medians = Vec::new();
    let mut max_run_len = 0usize;

    for seg in steps {
        for (start, end) in monotonic_runs(s, seg.start, seg.end, seg.dir) {
            let total_change = (s[end - 1] as f64 - s[start] as f64).abs();
            if total_change < 0.2 * sep {
                continue;
            }
            max_run_len = max_run_len.max(end - start);
            for d in TAU_STRIDES {
                if end - start < 2 * d + 1 {
                    continue;
                }
                let mut estimates = Vec::new();
                let step = (d / 2).max(1);
                let mut i = start;
                while i + 2 * d < end {
                    let y1 = s[i] as f64;
                    let y2 = s[i + d] as f64;
                    let y3 = s[i + 2 * d] as f64;
                    let (denom, num) = (y2 - y1, y3 - y2);
                    if denom.abs() >= MIN_TAU_DELTA_LSB && num.abs() >= MIN_TAU_DELTA_LSB {
                        let q = num / denom;
                        // q≈1 是直线，q≤0 是噪声/非单调；两者都不是指数。
                        if q > 0.02 && q < 0.98 {
                            let tau = -(d as f64) / q.ln();
                            if tau.is_finite() && tau > 0.0 {
                                estimates.push(tau);
                            }
                        }
                    }
                    i += step;
                }
                if estimates.len() >= 3 {
                    if let Some(m) = median(&estimates) {
                        per_stride_medians.push(m);
                    }
                }
            }
        }
    }

    if per_stride_medians.len() < 2 || max_run_len == 0 {
        fail(a, TAU_KEY, TransientReason::ShapeUnrecognized);
        return;
    }
    let Some(m) = median(&per_stride_medians) else {
        fail(a, TAU_KEY, TransientReason::ShapeUnrecognized);
        return;
    };
    let deviations: Vec<f64> = per_stride_medians.iter().map(|x| (x - m).abs()).collect();
    let spread = median(&deviations).unwrap_or(f64::INFINITY);
    if !(m.is_finite() && m > 0.0)
        || spread / m > MAX_TAU_REL_SPREAD
        || m > 2.0 * max_run_len as f64
    {
        fail(a, TAU_KEY, TransientReason::ShapeUnrecognized);
        return;
    }
    a.stats.rc_tau_ns = Some(m * 1e9 / rate_hz as f64);
}

/// 3 点中值平滑，只用于最大压摆率；不让单点噪声决定结果。
fn median3(s: &[u16]) -> Vec<f64> {
    if s.len() < 3 {
        return s.iter().map(|&v| v as f64).collect();
    }
    let mut out = Vec::with_capacity(s.len());
    out.push(s[0] as f64);
    for w in s.windows(3) {
        let mut a = [w[0], w[1], w[2]];
        a.sort_unstable();
        out.push(a[1] as f64);
    }
    out.push(s[s.len() - 1] as f64);
    out
}

fn estimate_slew(
    a: &mut TransientAnalysis,
    s: &[u16],
    rate_hz: u32,
    lo: u16,
    hi: u16,
    edges: (&[usize], &[usize]),
    sigma: f64,
) {
    let (rising, falling) = edges;
    let sep = (hi as f64 - lo as f64).max(0.0);
    if sep <= 0.0 {
        fail(a, SLEW_MAX_KEY, TransientReason::NotApplicable);
        fail(a, SLEW_AVG_KEY, TransientReason::NotApplicable);
        return;
    }

    let rise_ns = transition_ns(s, rate_hz, lo, hi, EdgeDir::Rising, None);
    let fall_ns = transition_ns(s, rate_hz, lo, hi, EdgeDir::Falling, None);
    let avg_slew = {
        let mut values = Vec::new();
        for tr in [rise_ns, fall_ns].into_iter().flatten() {
            if tr > 0.0 {
                values.push(0.8 * sep * 1000.0 / tr);
            }
        }
        mean(&values)
    };
    match avg_slew {
        Some(v) => a.stats.slew_avg_lsb_per_us = Some(v),
        None => fail(a, SLEW_AVG_KEY, TransientReason::ShapeUnrecognized),
    }

    let tr_ns =
        mean(&[rise_ns, fall_ns].into_iter().flatten().collect::<Vec<_>>()).filter(|v| *v > 0.0);
    let Some(tr_ns) = tr_ns else {
        fail(a, SLEW_MAX_KEY, TransientReason::ShapeUnrecognized);
        return;
    };
    let tr_samples = tr_ns * rate_hz as f64 / 1e9;
    if !(tr_samples.is_finite() && tr_samples > 0.0)
        || sigma > MAX_SLEW_NOISE_FRAC * sep / tr_samples
    {
        fail(a, SLEW_MAX_KEY, TransientReason::ShapeUnrecognized);
        return;
    }

    let half_window = (2.0 * tr_samples).max(4.0) as usize;
    let mut max_diff = 0.0f64;
    for (edges, opposite) in [(rising, falling), (falling, rising)] {
        for &edge in edges {
            let next = opposite
                .iter()
                .copied()
                .find(|&i| i > edge)
                .unwrap_or(s.len());
            let end = next.min(edge.saturating_add(half_window)).min(s.len());
            if end <= edge + 1 {
                continue;
            }
            let smoothed = median3(&s[edge..end]);
            for w in smoothed.windows(2) {
                max_diff = max_diff.max((w[1] - w[0]).abs());
            }
        }
    }
    if max_diff <= 0.0 {
        fail(a, SLEW_MAX_KEY, TransientReason::ShapeUnrecognized);
    } else {
        a.stats.slew_max_lsb_per_us = Some(max_diff * rate_hz as f64 / 1e6);
    }
}

/// 估算一个通道的瞬态特征。
pub fn analyze(s: &[u16], rate_hz: u32) -> TransientAnalysis {
    let mut out = TransientAnalysis::default();
    if rate_hz == 0 || s.len() < MIN_SAMPLES {
        fail_all(&mut out, TransientReason::TooFewSamples);
        return out;
    }

    let (lo, hi) = min_max(s);
    if hi <= lo {
        fail_all(&mut out, TransientReason::NotApplicable);
        return out;
    }

    let levels = detect_levels(s);
    let two_level = levels.is_two_level() && levels.sep_lsb > 0.0;
    let sep = if two_level {
        levels.sep_lsb
    } else {
        (hi - lo) as f64
    };
    let mid = if two_level {
        (levels.vlo_lsb + levels.vhi_lsb) / 2.0
    } else {
        (lo as f64 + hi as f64) / 2.0
    };
    let sigma = sigma_hat(s).unwrap_or(0.0);
    let h = (3.0 * sigma).min(0.25 * sep);
    let (rising, falling) = edges_hysteretic(s, mid, h);

    if two_level {
        let step_list = steps(&rising, &falling, s.len(), &levels);
        if step_list.is_empty() {
            for key in [RING_KEY, DAMPING_KEY, SETTLE_2_KEY, SETTLE_5_KEY] {
                fail(&mut out, key, TransientReason::ShapeUnrecognized);
            }
        } else {
            estimate_ring(&mut out, s, rate_hz, &step_list, sep, sigma);
            estimate_damping(&mut out, s, &step_list, sep, sigma);
            estimate_settling(
                &mut out,
                s,
                rate_hz,
                &step_list,
                sep,
                SETTLE_BANDS[0],
                SETTLE_2_KEY,
            );
            estimate_settling(
                &mut out,
                s,
                rate_hz,
                &step_list,
                sep,
                SETTLE_BANDS[1],
                SETTLE_5_KEY,
            );
        }
        out.stats.runts = Some(count_runts(s, mid, levels.vlo_lsb, levels.vhi_lsb, sep));
    } else {
        for key in [RING_KEY, DAMPING_KEY, SETTLE_2_KEY, SETTLE_5_KEY, RUNT_KEY] {
            fail(&mut out, key, TransientReason::NotApplicable);
        }
    }

    let slew_lo = if two_level {
        levels.vlo_lsb.round().clamp(0.0, 4095.0) as u16
    } else {
        lo
    };
    let slew_hi = if two_level {
        levels.vhi_lsb.round().clamp(0.0, 4095.0) as u16
    } else {
        hi
    };
    estimate_slew(
        &mut out,
        s,
        rate_hz,
        slew_lo,
        slew_hi,
        (&rising, &falling),
        sigma,
    );
    estimate_tau(
        &mut out,
        s,
        rate_hz,
        &steps(&rising, &falling, s.len(), &levels),
        sep,
    );

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: u32 = 857_142;

    fn sample(f: impl Fn(f64) -> f64, n: usize) -> Vec<u16> {
        (0..n)
            .map(|i| f(i as f64 / RATE as f64).round().clamp(0.0, 4095.0) as u16)
            .collect()
    }

    /// 与 `sim/src/waveform.rs` 的 `pwm_at` 同一解析式（真值场景的测试替身）。
    fn pwm_at(t: f64, duty: f64) -> f64 {
        const EDGE: f64 = 0.01;
        const HZ: f64 = 1000.0;
        let phase = (HZ * t) % 1.0;
        let half = EDGE / 2.0;
        let frac = if phase < half {
            phase / half
        } else if phase < duty - half {
            1.0
        } else if phase < duty + half {
            (duty + half - phase) / EDGE
        } else {
            0.0
        };
        2048.0 - 1800.0 + 3600.0 * frac.clamp(0.0, 1.0)
    }

    /// 与 `sim/src/waveform.rs` 的 `step_ring_at` 同一解析式。
    fn step_ring_at(t: f64) -> f64 {
        const ZETA: f64 = 0.35;
        const FD_HZ: f64 = 12_000.0;
        const PERIOD: f64 = 2.0e-3;
        const LO: f64 = 800.0;
        const HI: f64 = 2800.0;

        let phase = (t % PERIOD) / PERIOD;
        let (v0, v1, s) = if phase < 0.5 {
            (LO, HI, phase * PERIOD)
        } else {
            (HI, LO, (phase - 0.5) * PERIOD)
        };
        let root = (1.0 - ZETA * ZETA).sqrt();
        let wd = 2.0 * std::f64::consts::PI * FD_HZ;
        let wn = wd / root;
        let decay = (-ZETA * wn * s).exp();
        let osc = (wd * s).cos() + (ZETA / root) * (wd * s).sin();
        v1 + (v0 - v1) * decay * osc
    }

    /// 与 `sim/src/waveform.rs` 的 `rc_charge_at` 同一解析式。
    fn rc_charge_at(t: f64) -> f64 {
        const PERIOD: f64 = 1.0e-3;
        const TAU: f64 = 150.0e-6;
        const LO: f64 = 800.0;
        const HI: f64 = 2800.0;
        let phase = (t % PERIOD) / PERIOD;
        if phase < 0.5 {
            let s = phase * PERIOD;
            LO + (HI - LO) * (1.0 - (-s / TAU).exp())
        } else {
            let v_top = LO + (HI - LO) * (1.0 - (-(PERIOD / 2.0) / TAU).exp());
            let s = (phase - 0.5) * PERIOD;
            LO + (v_top - LO) * (-s / TAU).exp()
        }
    }

    fn pwm_with_runts() -> Vec<u16> {
        // 1 kHz、25% 占空比，正常高电平 3400 / 低电平 800。
        // 第 3/5/7 个（下标 2/4/6）高电平段换成 2200（高 runt），
        // 第 4/6 个（下标 3/5）低电平段换成 1600（低 runt）——
        // 共 5 个，两个方向都有。10 个周期保证正常电平仍是多数，
        // `detect_levels` 不会因为 runt 太多而判不出双电平。
        // 40 个周期：runt 只占 3%，两座主平台仍是压倒性多数。
        // 10 个周期时 runt 占 22%，平台中心被拉到中间、占用率跌到 0.7 以下，
        // `detect_levels` 会（正确地）判不出双电平。
        const PERIOD: usize = 857;
        let mut v = Vec::with_capacity(PERIOD * 40);
        for n in 0..40 {
            let high_level = if matches!(n, 2 | 4 | 6) { 2200 } else { 3400 };
            let low_level = if matches!(n, 3 | 5) { 1600 } else { 800 };
            for i in 0..PERIOD {
                let phase = i as f64 / PERIOD as f64;
                v.push(if phase < 0.25 { high_level } else { low_level });
            }
        }
        v
    }

    #[test]
    fn step_ring_matches_the_analytic_truth() {
        let s = sample(step_ring_at, 4096);
        let a = analyze(&s, RATE);

        let f = a.stats.ring_freq_hz.expect("step_ring 应能测出振铃频率");
        assert!(
            (f - 12_000.0).abs() / 12_000.0 < 0.05,
            "振铃频率 {f} Hz 与解析值 12 kHz 差得太多"
        );

        let z = a.stats.damping_ratio.expect("step_ring 应能测出阻尼比");
        assert!((z - 0.35).abs() < 0.03, "阻尼比 {z} 与解析值 0.35 差得太多");

        let ts2 = a.stats.settle_2_ns.expect("应能测出 ±2% 调节时间");
        assert!(
            (99_500.0..=141_200.0).contains(&ts2),
            "±2% 调节时间 {ts2} ns 不在峰值离散化允许区间 [99.5, 141.2] µs"
        );
        let ts5 = a.stats.settle_5_ns.expect("应能测出 ±5% 调节时间");
        assert!(
            (67_000.0..=108_700.0).contains(&ts5),
            "±5% 调节时间 {ts5} ns 不在峰值离散化允许区间 [67.0, 108.7] µs"
        );
        assert_eq!(a.stats.runts, Some(0), "step_ring 没有 runt");
    }

    #[test]
    fn fast_edge_square_settling_is_refused_not_zero() {
        // 快边沿方波（跃变在相邻两样点间完成）：跃变片段内全部样点都
        // 在阈值带内，没有可分辨的调节过程。调节时间必须拒答
        // （ShapeUnrecognized），不得输出 0.0 ns 的退化值。
        let s = pwm_with_runts();
        let a = analyze(&s, RATE);

        assert!(
            a.stats.settle_2_ns.is_none(),
            "快边沿方波的 ±2% 调节时间应拒答，实际 {:?}",
            a.stats.settle_2_ns
        );
        assert!(
            a.stats.settle_5_ns.is_none(),
            "快边沿方波的 ±5% 调节时间应拒答，实际 {:?}",
            a.stats.settle_5_ns
        );
        assert_eq!(
            a.reasons.get(SETTLE_2_KEY),
            Some(&TransientReason::ShapeUnrecognized)
        );
        assert_eq!(
            a.reasons.get(SETTLE_5_KEY),
            Some(&TransientReason::ShapeUnrecognized)
        );
    }

    #[test]
    fn rc_charge_matches_the_analytic_tau() {
        let s = sample(rc_charge_at, 4096);
        let a = analyze(&s, RATE);
        let tau = a.stats.rc_tau_ns.expect("rc_charge 应能测出 τ");
        assert!(
            (tau - 150_000.0).abs() / 150_000.0 < 0.02,
            "RC τ = {tau} ns，与解析值 150 µs 相差超过 2%"
        );
        assert_eq!(a.stats.runts, None, "非双电平的 RC 曲线没有 runt 定义");
        assert_eq!(
            a.reasons.get("runts"),
            Some(&TransientReason::NotApplicable),
            "RC 曲线的 runt 应明确标为不适用"
        );
    }

    #[test]
    fn narrow_pwm_has_no_runt_and_an_analytic_slew() {
        let s = sample(|t| pwm_at(t, 0.05), 4096);
        let a = analyze(&s, RATE);

        assert_eq!(a.stats.runts, Some(0), "正常窄脉冲不是 runt");
        // 这个场景的上升沿是 5 µs 满沿（10→90% = 4 µs → 720 LSB/µs），
        // 下降沿是 10 µs 满沿（→ 360 LSB/µs）。平均口径对两个方向取均值，
        // 因此解析值是 (720 + 360) / 2 = 540；最大口径取更快的那一边 = 720。
        let avg = a
            .stats
            .slew_avg_lsb_per_us
            .expect("应能测出 10-90% 平均压摆率");
        assert!(
            (avg - 540.0).abs() < 54.0,
            "平均压摆率 {avg} LSB/µs，理论值 540"
        );
        let max = a.stats.slew_max_lsb_per_us.expect("应能测出最大压摆率");
        assert!(
            (max - 720.0).abs() < 72.0,
            "最大压摆率 {max} LSB/µs，理论值 720（上升沿 3600 LSB / 5 µs 满沿）"
        );
        assert!(avg < max, "不对称边沿上平均与最大不应相等");
        assert!(
            a.stats.rc_tau_ns.is_none(),
            "线性边沿 + 平台不应被当成指数曲线"
        );
    }

    #[test]
    fn runt_pulses_are_counted_in_both_directions() {
        let s = pwm_with_runts();
        let a = analyze(&s, RATE);
        assert_eq!(
            a.stats.runts,
            Some(5),
            "应数出 3 个高 runt + 2 个低 runt，实际 {:?}",
            a.stats.runts
        );
    }

    #[test]
    fn a_ring_above_the_sampling_density_is_refused_as_aliased() {
        // 1 kHz 方波 + 每次跳变后叠加 400 kHz 衰减振铃（半周期约 1.07 个样点）。
        // 振铃只占极少数样点，不影响双电平判定。
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / RATE as f64;
                let phase = (1000.0 * t) % 1.0;
                let base = if phase < 0.5 { 2800.0 } else { 800.0 };
                // `phase` 是周期数，振铃公式要**秒**。
                let since_edge = if phase < 0.5 { phase } else { phase - 0.5 } * 1e-3;
                let ring = 300.0
                    * (-since_edge / 5.0e-6).exp()
                    * (2.0 * std::f64::consts::PI * 400_000.0 * since_edge).sin();
                (base + ring).round().clamp(0.0, 4095.0) as u16
            })
            .collect();
        let a = analyze(&s, RATE);
        assert!(a.stats.ring_freq_hz.is_none(), "混叠时不得给出振铃频率数字");
        assert_eq!(
            a.reasons.get("ring_freq_hz"),
            Some(&TransientReason::Aliased),
            "半周期 < 2 样点必须拒答为 Aliased"
        );
    }

    #[test]
    fn a_short_capture_refuses_every_transient_quantity() {
        let a = analyze(&[0, 4000, 0, 4000, 0, 4000, 0, 4000], RATE);
        assert!(a.stats == TransientStats::default());
        for key in [
            "ring_freq_hz",
            "damping_ratio",
            "settle_2_ns",
            "settle_5_ns",
            "rc_tau_ns",
            "slew_max_lsb_per_us",
            "slew_avg_lsb_per_us",
            "runts",
        ] {
            assert_eq!(
                a.reasons.get(key),
                Some(&TransientReason::TooFewSamples),
                "{key} 应因样点不足拒答"
            );
        }
    }

    #[test]
    fn a_sine_has_no_two_level_transient_quantities() {
        let s = sample(
            |t| 2048.0 + 1800.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin(),
            4096,
        );
        let a = analyze(&s, RATE);
        for key in [
            "ring_freq_hz",
            "damping_ratio",
            "settle_2_ns",
            "settle_5_ns",
        ] {
            assert!(a.stats_field(key).is_none(), "{key} 对正弦必须是 None");
            assert_eq!(
                a.reasons.get(key),
                Some(&TransientReason::NotApplicable),
                "{key} 对正弦应标为不适用"
            );
        }
    }

    /// 每个量要么有数值、要么有拒答原因 —— 不能两者都没有，也不能都有。
    ///
    /// 这是模块对出口的契约：`Quality` 会把 `Some` 映射成判决、把 `reasons`
    /// 映射成 `Refused`。漏掉一个键，出口就会静默地什么都不说。
    #[test]
    fn every_transient_quantity_is_either_measured_or_refused_with_a_reason() {
        let noise: Vec<u16> = {
            let mut x = 0x5EED_1234u32;
            (0..4096)
                .map(|_| {
                    x = x.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                    ((x >> 16) % 4001) as u16
                })
                .collect()
        };
        let cases: [(&str, Vec<u16>); 6] = [
            ("step_ring", sample(step_ring_at, 4096)),
            ("rc_charge", sample(rc_charge_at, 4096)),
            ("pwm_1k_5", sample(|t| pwm_at(t, 0.05), 4096)),
            (
                "sine",
                sample(
                    |t| 2048.0 + 1800.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin(),
                    4096,
                ),
            ),
            ("noise", noise),
            ("too_short", vec![0, 4000, 0, 4000, 0, 4000, 0, 4000]),
        ];
        for (name, s) in cases {
            let a = analyze(&s, RATE);
            for &key in &ALL_KEYS {
                let has_value = a.stats_field(key).is_some();
                let has_reason = a.reasons.contains_key(key);
                assert!(
                    has_value != has_reason,
                    "{name} 的 {key}：数值 {has_value}，原因 {has_reason} —— 必须二选一"
                );
            }
        }
    }

    impl TransientAnalysis {
        fn stats_field(&self, key: &str) -> Option<f64> {
            match key {
                "ring_freq_hz" => self.stats.ring_freq_hz,
                "damping_ratio" => self.stats.damping_ratio,
                "settle_2_ns" => self.stats.settle_2_ns,
                "settle_5_ns" => self.stats.settle_5_ns,
                "rc_tau_ns" => self.stats.rc_tau_ns,
                "slew_max_lsb_per_us" => self.stats.slew_max_lsb_per_us,
                "slew_avg_lsb_per_us" => self.stats.slew_avg_lsb_per_us,
                "runts" => self.stats.runts.map(|v| v as f64),
                _ => panic!("未知字段 {key}"),
            }
        }
    }
}
