//! 统一的边沿判定原语：迟滞状态机 + 噪声底 σ̂。
//!
//! 主机侧此前只有单阈值比较（`s[i-1] < mid && s[i] >= mid`）。边沿缓慢或
//! 噪声较大时，一次物理边沿会在中点附近来回穿越，被计成多次跃变，
//! `edges` 与 `freq_hz` 一起失真。本模块把固件侧 `trigger.c` 的施密特迟滞
//! 搬到主机侧，并按实测噪声自标定带宽。
//!
//! **不变量**：`h = 0` 时逐字退化为旧实现；干净信号的边沿索引逐位不变。

use crate::measure::{falling_edges, rising_edges};
use crate::signal::detect_levels;

/// 噪声底 σ̂（ADC LSB）。
///
/// 算法：平台带内取相邻差分，`σ̂ = MAD(Δ)/0.6745/√2`。白噪声差分的方差是
/// 原噪声的两倍，故除以 √2；高斯分布下 `MAD = 0.6745σ`。
///
/// 没有双电平平台时退化为全样点差分。样点不足或全平坦返回 `None`。
pub(crate) fn sigma_hat(s: &[u16]) -> Option<f64> {
    if s.len() < 16 {
        return None;
    }
    let levels = detect_levels(s);
    let in_band = |v: u16| -> bool {
        if !levels.is_two_level() {
            return true;
        }
        let band = 0.02 * levels.sep_lsb;
        (v as f64 - levels.vlo_lsb).abs() <= band || (v as f64 - levels.vhi_lsb).abs() <= band
    };

    let diffs: Vec<f64> = s
        .windows(2)
        .filter(|w| in_band(w[0]) && in_band(w[1]))
        .map(|w| w[1] as f64 - w[0] as f64)
        .collect();
    if diffs.len() < 16 {
        return None;
    }

    let med = median(&diffs);
    let deviations: Vec<f64> = diffs.iter().map(|d| (d - med).abs()).collect();
    let mad = median(&deviations);
    Some(mad / 0.6745 / std::f64::consts::SQRT_2)
}

/// 迟滞边沿判定。
///
/// 返回 `(上升沿索引, 下降沿索引)`。索引口径与旧实现一致：
/// 上升沿是**本段内首次穿过中点**的索引 `j`（`s[j-1] < mid <= s[j]`），
/// 下降沿对称。迟滞只决定「一次跃变是否成立」，不改变已经成立的索引口径。
pub(crate) fn edges_hysteretic(s: &[u16], mid: f64, h: f64) -> (Vec<usize>, Vec<usize>) {
    if h <= 0.0 || !h.is_finite() {
        return (rising_edges(s, mid), falling_edges(s, mid));
    }

    let up = mid + h / 2.0;
    let dn = mid - h / 2.0;
    let mut rising = Vec::new();
    let mut falling = Vec::new();
    let mut state_high = s.first().is_some_and(|&v| (v as f64) >= mid);
    let mut seg_start = 0usize;

    for i in 1..s.len() {
        let v = s[i] as f64;
        if !state_high && v >= up {
            // 本段内第一次穿过中点
            if let Some(j) =
                (seg_start.max(1)..=i).find(|&j| (s[j - 1] as f64) < mid && (s[j] as f64) >= mid)
            {
                rising.push(j);
            }
            state_high = true;
            seg_start = i;
        } else if state_high && v <= dn {
            if let Some(j) =
                (seg_start.max(1)..=i).find(|&j| (s[j - 1] as f64) >= mid && (s[j] as f64) < mid)
            {
                falling.push(j);
            }
            state_high = false;
            seg_start = i;
        }
    }

    (rising, falling)
}

/// 中位数。调用方保证 `v` 非空。
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

#[cfg(test)]
mod tests {
    use super::*;

    fn square(rate: u32, n: usize) -> Vec<u16> {
        (0..n)
            .map(|i| {
                if (i / (rate as usize / 1000 / 2)) % 2 == 0 {
                    0
                } else {
                    4000
                }
            })
            .collect()
    }

    #[test]
    fn h_zero_is_exactly_the_old_path() {
        let s = square(1_000_000, 4096);
        let mid = 2000.0;
        assert_eq!(
            edges_hysteretic(&s, mid, 0.0),
            (rising_edges(&s, mid), falling_edges(&s, mid))
        );
    }

    #[test]
    fn clean_square_indices_do_not_move() {
        let s = square(1_000_000, 4096);
        let mid = 2000.0;
        let old = (rising_edges(&s, mid), falling_edges(&s, mid));
        let h = sigma_hat(&s).map(|sg| 3.0 * sg).unwrap_or(0.0);
        assert_eq!(edges_hysteretic(&s, mid, h), old);
    }

    #[test]
    fn hysteresis_kills_false_edges_on_a_noisy_slow_edge() {
        // 慢边沿在中点附近缓慢爬升，叠加 ±20 LSB 抖动：
        // 单阈值会把一次物理跃变数成很多次；迟滞带宽 60 LSB 应只算一次。
        let mut s = Vec::new();
        for i in 0..200 {
            let base = if i < 50 {
                1000.0
            } else if i < 150 {
                2000.0 + (i as f64 - 100.0) * 2.0
            } else {
                3000.0
            };
            let noise = if i % 2 == 0 { 20.0 } else { -20.0 };
            s.push((base + noise).clamp(0.0, 4095.0) as u16);
        }
        let mid = 2000.0;
        let old = rising_edges(&s, mid).len();
        let h = 60.0;
        let (new_rising, _) = edges_hysteretic(&s, mid, h);
        assert!(old > 1, "构造应能触发旧实现的多次跃变，实际 {old}");
        assert_eq!(new_rising.len(), 1, "迟滞后一次物理边沿只应算一次");
    }

    #[test]
    fn sigma_hat_is_zero_on_a_clean_square() {
        let s = square(1_000_000, 4096);
        assert_eq!(sigma_hat(&s), Some(0.0));
    }

    #[test]
    fn sigma_hat_is_positive_on_noise() {
        // 确定性 LCG 造 2048 ± 20 LSB 的零均值噪声（不要用 +/- 交替：
        // 双峰序列的差分中位数会落在某一侧，MAD 被压成 0）。
        let mut x = 0x5EED_1234u32;
        let s: Vec<u16> = (0..4096)
            .map(|_| {
                x = x.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                let noise = ((x >> 16) % 41) as i32 - 20;
                (2048 + noise).clamp(0, 4095) as u16
            })
            .collect();
        let sg = sigma_hat(&s).expect("应能估出噪声");
        assert!(sg > 0.0, "噪声底应为正，实际 {sg}");
    }
}
