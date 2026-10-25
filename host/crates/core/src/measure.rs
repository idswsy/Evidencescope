//! 定点测量 —— 对一个通道算出峰峰值、频率、占空比、上升时间。
//!
//! 主统计量直接用 [`Capture::summary`] 已经算好的结果，这里只补三样它没有的：
//! 平均频率、占空比、上升时间。
//!
//! # 三个容易算错的地方（都踩过）
//!
//! 1. **幅值与电平的换算不一样**。峰峰值 / RMS 是**幅值**，只乘 V/LSB；
//!    最小值 / 最大值 / 均值是**电平**，要先减零点再乘。混用会让
//!    2.31 V 的峰峰值显示成 0.66 V（甚至把 RMS 算成负数）。
//! 2. **频率不能用「边沿数 ÷ 总时长」**。I2C 在两次事务之间 SCL 保持高电平，
//!    那段时间没有边沿却占着时长 —— 100 kHz 会被算成 87 kHz。
//! 3. **周期估计要「先筛后平均」**。用中位数筛掉空闲段，再对保留下来的间隔
//!    取均值。直接用中位数当周期也不行：8.57 个采样点的周期会让 8/9 交替的
//!    采样抖动把中位数带到 9 上，报出 95.2 kHz。

use crate::capture::{Capture, ChannelScale};
use crate::signal::detect_levels;

/// 一个通道的测量结果。
///
/// 可选字段为 `None` 表示**测不出来**（直流没有周期、没有边沿就没有上升时间），
/// 而不是 0 —— 项目纪律：无可信结论的量必须显式拒答，不得输出看似精确的无依据数值。
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Measurements {
    /// 峰峰值（V）。
    pub vpp: f64,
    /// 最小值（V）。
    pub min: f64,
    /// 最大值（V）。
    pub max: f64,
    /// 均值（V）。
    pub mean: f64,
    /// 扣除直流后的 RMS（波动），也就是标准差 —— 交流信号的有效值看这个。
    pub ac_rms: f64,
    /// 含直流的真 RMS。直流信号也有值，别拿它当"有效值"理解。
    pub rms: f64,
    /// 平均频率（Hz）。没有上升沿则为 `None`。
    pub freq_hz: Option<f64>,
    /// 高电平占比（%）。没有边沿则为 `None`。
    pub duty_pct: Option<f64>,
    /// 10%→90% 上升时间（ns）。找不到干净的边沿则为 `None`。
    pub rise_ns: Option<f64>,
    /// 90%→10% 下降时间（ns）。与 [`Measurements::rise_ns`] 完全对称。
    ///
    /// 与设备侧 `MEASURE` 响应的 `fall_time_samples`（`proto/protocol.h`）是
    /// **同一个量**，只是单位不同 —— 那个报样点数，这个报纳秒。别再造第三个定义。
    pub fall_ns: Option<f64>,
    /// 过冲（%）：`(全窗最高样点 − 高平台均值) / 平台间隔 × 100`。
    ///
    /// **参考电平取平台的均值，不取全窗最大值** —— 后者会让这个量恒等于 0。
    /// 平台由 [`crate::signal::detect_levels`] 判定；判不出双电平（正弦、噪声）
    /// 时为 `None`，因为那时根本不存在「稳态电平」这个概念。
    ///
    /// 没有过冲时是 `Some(0.0)`，**不是 `None`** —— 「测了，是 0」与「测不出」
    /// 是两件事。噪声会让它出现几个百分点的正值，那是物理事实，不钳。
    pub overshoot_pct: Option<f64>,
    /// 下冲（%）：`(低平台均值 − 全窗最低样点) / 平台间隔 × 100`。同过冲。
    pub undershoot_pct: Option<f64>,
    /// 平均高电平持续时间（ns）—— 也就是脉宽。
    ///
    /// # 与 `duty_pct × 周期` 的关系（别把它想复杂了）
    ///
    /// 干净周期信号上两者**代数等价**：占空比按窗口长度加权、这个按脉冲个数
    /// 平均，而每个窗口恰好含一个高电平段时，两个平均是同一个。**不要**在
    /// 文档或提示词里声称它们必然不同。
    ///
    /// 它单独存在的理由是**周期测不出来的那些波形**：孤立脉冲、阶跃响应 ——
    /// 那时 `duty_pct` 是 `None`（上升沿不足 3 个），而脉宽仍然是个有意义的数。
    ///
    /// 已知周期时**长于一个周期的高电平段按空闲丢弃**（与占空比排除事务间
    /// 空闲同一条规矩，I2C 的 SCL 空闲就停在低电平）；测不出周期时全部计入。
    pub high_ns: Option<f64>,
    /// 本次测量中点处的跃变总数（上升 + 下降）。
    ///
    /// ⚠ **与 `ChannelSummary::rising_edges` 不同源**：那个固定在 2048 LSB
    /// 判定（`capture.rs`），这个用实测中点 `(lo+hi)/2`。信号有直流偏置时
    /// 两者可以差出很多 —— 那正是它值得单独存在的理由。
    pub edges: u32,
}

/// 对一个通道做测量。
///
/// 主统计量直接用 core 已经算好的 [`scope_core::ChannelSummary`]，
/// 这里只补三样它没有的：平均频率、占空比、上升时间。
pub fn measure(cap: &Capture, ch: usize, scale: &ChannelScale) -> Option<Measurements> {
    let sum = cap.summary(ch)?;
    let s = cap.samples(ch)?;

    // ⚠ 两种换算不能混：
    //   电平（min/max/mean）→ 减零点再乘 V/LSB
    //   幅值（Vpp/RMS/AC-RMS）→ **只乘 V/LSB**，减零点会把幅值算成电平
    // 回归：这里曾经一律用 lsb_to_volts，于是 Vpp 2.31 V 被报成 0.66 V，
    // 而 AC-RMS 直接变成了负数。
    let vpl = scale.volts_per_lsb;
    let level = |lsb: f64| scale.lsb_to_volts(lsb.round().clamp(0.0, 4095.0) as u16);
    let magnitude = |lsb: f64| lsb * vpl;

    let (mut lo, mut hi) = (u16::MAX, u16::MIN);
    for &v in s {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi <= lo {
        // 直流：有电平，但没有周期/占空比/边沿/过冲可言。
        // `edges` 是 0 而不是 None —— 它是计数，计数就是 0，没有「测不出」这回事。
        return Some(Measurements {
            vpp: 0.0,
            min: level(lo as f64),
            max: level(hi as f64),
            mean: level(sum.mean_lsb),
            ac_rms: magnitude(sum.ac_rms_lsb),
            rms: magnitude(sum.rms_lsb),
            freq_hz: None,
            duty_pct: None,
            rise_ns: None,
            fall_ns: None,
            overshoot_pct: None,
            undershoot_pct: None,
            high_ns: None,
            edges: 0,
        });
    }

    let mid = (lo as f64 + hi as f64) / 2.0;
    // 迟滞带宽：h = min(3σ̂, 0.25·sep)。σ̂ 是数据自标定的噪声底，
    // 不是固定常数；σ̂ 估不出（样点太少/全平坦）时退化为 h = 0（旧行为）。
    let levels = detect_levels(s);
    let sep = if levels.is_two_level() {
        levels.sep_lsb
    } else {
        (hi - lo) as f64
    };
    let h = crate::edges::sigma_hat(s)
        .map(|sigma| (3.0 * sigma).min(0.25 * sep))
        .unwrap_or(0.0);
    let (rising, falling) = crate::edges::edges_hysteretic(s, mid, h);

    // ── 周期：取相邻上升沿间隔的**中位数** ──
    //
    // 不能拿「边沿数 ÷ 总时长」：I2C 在两次事务之间 SCL 保持高电平，
    // 那段时间没有边沿却占着时长，于是 100 kHz 的 SCL 会被算成 87 kHz。
    // 空闲间隔是时钟周期的几十倍，落在中位数之外，自然被排除。
    // 先用中位数定出「真实周期」的量级（它对空闲段的几十倍间隔免疫），
    // 再对**通过筛选的间隔取均值**。
    //
    // 为什么最终用均值而不是中位数：采样周期 8.57 个点时，相邻间隔是 8/9
    // 交替抖动，中位数会落在 9 上 → 报 95.2 kHz；均值才回到 8.57 → 100.0 kHz。
    // 回归：这里曾经直接用中位数当周期。
    let period = filter_gaps(&rising).map(|f| {
        // kept 不可能为空（中位数本身就是一条间隔），这个回退只是不假设
        if f.kept.is_empty() {
            f.median
        } else {
            f.kept.iter().sum::<f64>() / f.kept.len() as f64
        }
    });
    let freq_hz = period.map(|p| cap.rate_hz as f64 / p);

    // ── 占空比：只统计「真实时钟周期」内的高电平占比 ──
    let duty_pct = period.map(|p| {
        let cutoff = p * 1.6;
        let (mut high, mut total) = (0usize, 0usize);
        for w in rising.windows(2) {
            let gap = (w[1] - w[0]) as f64;
            if gap > cutoff {
                continue; // 空闲段，跳过
            }
            for &v in &s[w[0]..w[1]] {
                if v as f64 > mid {
                    high += 1;
                }
                total += 1;
            }
        }
        if total == 0 {
            return None;
        }
        Some(high as f64 * 100.0 / total as f64)
    });

    // ── 边沿时间的搜索窗口 ──
    //
    // 有周期时取**半个周期**：10%→90% 的过渡，正弦占 0.295 个周期、三角占
    // 0.4 个周期，半周期都容得下；而一个只升到幅度 60% 就被打断的毛刺沿
    // 找不到 90% 穿越，于是如实返回「测不出」，而不是一路扫到后面的边沿
    // 凑一个横跨谷底的荒谬值（那是本函数加窗口前的实际行为）。
    //
    // 无周期时（阶跃响应、孤立脉冲）**不加窗口** —— 那种信号本来就该在
    // 整个窗内找它的 10%/90%。
    let window = period.map(|p| (p * 0.5).max(MIN_EDGE_WINDOW_SAMPLES));

    // ── 过冲 / 下冲：参考电平是**平台的均值**，不是全窗最大值 ──
    let levels = detect_levels(s);
    let (overshoot_pct, undershoot_pct) = if levels.is_two_level() && levels.sep_lsb > 0.0 {
        let sep = levels.sep_lsb;
        (
            Some((((hi as f64) - levels.vhi_lsb) / sep * 100.0).max(0.0)),
            Some(((levels.vlo_lsb - lo as f64) / sep * 100.0).max(0.0)),
        )
    } else {
        // 不是双电平就没有「稳态电平」这个参考 —— 正弦、噪声、调幅一律测不出。
        // 拿中心硬算会给正弦造出几十个百分点的假过冲。
        (None, None)
    };

    // ── 边沿时间的门限：双电平用平台电平，非双电平用全局极值 ──
    //
    // t10/t90 取全局 min/max 时，过冲/下冲 ≥ 约 10% pp 的波形会把 90% 线抬进
    // 过冲峰、10% 线压进下冲谷：`step_ring`（过冲 30.8%）实测 fall_ns =
    // 972 749.8 ns —— 那是平台宽度，不是边沿时间，且 rise 恒 None
    // （docs/11 §10.3 F7 裁决）。IEEE 181 的 10%/90% 参考本就是**稳态电平**；
    // 双电平判定的 vlo/vhi 恰是平台中心（过冲/下冲是少数派样点，不挪动
    // 桶中心）。非双电平（正弦/噪声/调幅）没有平台可言，维持全局极值与
    // 既有拒答路径。
    let (edge_lo, edge_hi) = if levels.is_two_level() && levels.sep_lsb > 0.0 {
        (
            levels.vlo_lsb.round().clamp(0.0, 4095.0) as u16,
            levels.vhi_lsb.round().clamp(0.0, 4095.0) as u16,
        )
    } else {
        (lo, hi)
    };

    Some(Measurements {
        vpp: magnitude(sum.pp_lsb as f64),
        min: level(lo as f64),
        max: level(hi as f64),
        mean: level(sum.mean_lsb),
        ac_rms: magnitude(sum.ac_rms_lsb),
        rms: magnitude(sum.rms_lsb),
        freq_hz,
        duty_pct: duty_pct.flatten(),
        rise_ns: transition_ns(s, cap.rate_hz, edge_lo, edge_hi, EdgeDir::Rising, window),
        fall_ns: transition_ns(s, cap.rate_hz, edge_lo, edge_hi, EdgeDir::Falling, window),
        overshoot_pct,
        undershoot_pct,
        high_ns: high_time_ns(s, cap.rate_hz, mid, period),
        edges: (rising.len() + falling.len()) as u32,
    })
}

/// 边沿时间窗口的下限（样点）。再短的窗口连一个采样点都放不下。
const MIN_EDGE_WINDOW_SAMPLES: f64 = 4.0;

/// 上升沿所在的样点索引（用中点判定）。
///
/// `pub(crate)` 是因为 [`crate::signal::classify`] 的边沿必须与这里**同一口径**
/// —— 那个固定 2048 LSB 的 `ChannelSummary::rising_edges` 会把直流偏置的信号
/// 数成 0 个沿（见下方 `edge_count_follows_the_measured_midpoint` 那条测试）。
pub(crate) fn rising_edges(s: &[u16], mid: f64) -> Vec<usize> {
    (1..s.len())
        .filter(|&i| (s[i - 1] as f64) < mid && (s[i] as f64) >= mid)
        .collect()
}

/// 下降沿所在的样点索引。
///
/// 与 [`rising_edges`] **严格互补**：`s[i-1] >= mid && s[i] < mid`。
/// 两边都把「恰好等于中点」归给上升沿，所以每个跃变恰好算作一个方向 ——
/// 不会漏掉一个，也不会被数两遍。
pub(crate) fn falling_edges(s: &[u16], mid: f64) -> Vec<usize> {
    (1..s.len())
        .filter(|&i| (s[i - 1] as f64) >= mid && (s[i] as f64) < mid)
        .collect()
}

/// 边沿方向。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EdgeDir {
    Rising,
    Falling,
}

/// 10%→90%（上升）/ 90%→10%（下降）的过渡时间，单位 ns。
///
/// **`rise_ns` 与 `fall_ns` 共用这一个函数** —— 镜像由构造保证，不靠复制粘贴。
/// 复制一份再把不等号写反是这类代码最常见的错法，而且**在对称波形上测不出来**。
///
/// `window` 是搜索窗口（样点）。`None` = 不设窗口。
pub(crate) fn transition_ns(
    s: &[u16],
    rate_hz: u32,
    lo: u16,
    hi: u16,
    dir: EdgeDir,
    window: Option<f64>,
) -> Option<f64> {
    if rate_hz == 0 || s.len() < 4 || hi <= lo {
        return None;
    }
    let (lo_f, hi_f) = (lo as f64, hi as f64);
    let mid = (lo_f + hi_f) / 2.0;
    let t10 = lo_f + (hi_f - lo_f) * 0.1;
    let t90 = lo_f + (hi_f - lo_f) * 0.9;

    // 朝这个方向的**第一个跨中点边沿**，以及先后要穿过的两个门限。
    // 上升先穿 10% 再穿 90%；下降反过来。
    let (start, first_t, second_t) = match dir {
        EdgeDir::Rising => (
            (1..s.len()).find(|&i| (s[i - 1] as f64) < mid && (s[i] as f64) >= mid)?,
            t10,
            t90,
        ),
        EdgeDir::Falling => (
            (1..s.len()).find(|&i| (s[i - 1] as f64) >= mid && (s[i] as f64) < mid)?,
            t90,
            t10,
        ),
    };

    // ── 起点：退到「上一个电平还没离开」的位置 ──
    //
    // ⚠ **只从中点穿越点往后搜是不够的。** 慢边沿（正弦、RC 充电、控制环响应）
    // 的 10% 穿越点在中点**之前**，往后永远找不到。2026-10-06 实测确认：
    //   • 1 kHz 正弦 @ 857142  → rise_ns = None
    //   • 10 步线性上升（10 µs）→ rise_ns = None
    // 而本函数的文档一直写着「1 kHz 正弦的上升沿本身就要几百微秒，窗口开小了会漏」——
    // **代码做不到它声称的事**。快边沿之所以能过，是因为整个 10–90% 都落在
    // 中点穿越的那一个采样间隔里。
    //
    // 退到「离开前一档门限」的那一点再往前搜，两个方向、快慢边沿就都成立了。
    // 对快边沿结果不变（退一格后那一格是平的，不产生穿越点）。
    let anchor = match dir {
        EdgeDir::Rising => (0..start).rev().find(|&i| (s[i] as f64) <= t10),
        EdgeDir::Falling => (0..start).rev().find(|&i| (s[i] as f64) >= t90),
    }
    .unwrap_or(start);

    // 窗口从**中点穿越点**量起：它限制的是「跨过去之后还找了多远」，
    // 而 anchor 只是把起点提前，不改变窗口的含义。
    let end = match window {
        Some(w) => s.len().min(start + w as usize + 1),
        None => s.len(),
    };

    // 线性插值定位穿越点 —— 采样间隔就是分辨率下限。
    let cross = |i: usize, t: f64| -> Option<f64> {
        let (a, b) = (s[i - 1] as f64, s[i] as f64);
        if a == b {
            return None;
        }
        let hit = match dir {
            EdgeDir::Rising => a < t && b >= t,
            EdgeDir::Falling => a >= t && b < t,
        };
        if !hit {
            return None;
        }
        Some((i - 1) as f64 + (t - a) / (b - a))
    };

    let mut c_first: Option<f64> = None;
    let mut c_second: Option<f64> = None;
    for i in (anchor.max(1))..end {
        if c_first.is_none() {
            c_first = cross(i, first_t);
        }
        // ⚠ 两个门限在**同一次迭代**里都要看：一个采样步就跨完 10% 与 90% 的
        // 快边沿（真机上很常见），错过这一次就再也找不到了。
        if c_first.is_some() {
            c_second = cross(i, second_t);
            if c_second.is_some() {
                break;
            }
        }
    }
    match (c_first, c_second) {
        // 下降时 c_first 是 90% 穿越点（早），c_second 是 10% 穿越点（晚），
        // 所以这一个 `b > a` 对两个方向都成立。
        (Some(a), Some(b)) if b > a => Some((b - a) * 1e9 / rate_hz as f64),
        _ => None,
    }
}

/// 平均高电平持续时间（ns）—— 脉宽。
///
/// # 空闲怎么处理
///
/// 已知周期时，**长于一个周期的高电平段按空闲丢弃**。I2C 的 SCL 在两次
/// 事务之间停在低电平、SDA 停在低或高 —— 那段不是脉冲，算进去会把均值
/// 抬到毫无意义的量级。测不出周期时全部计入（孤立脉冲、阶跃响应）。
///
/// 与 `duty_pct × 周期` 的关系见 [`Measurements::high_ns`] 的说明。
fn high_time_ns(s: &[u16], rate_hz: u32, mid: f64, period: Option<f64>) -> Option<f64> {
    if rate_hz == 0 {
        return None;
    }
    let mut widths: Vec<f64> = Vec::new();
    let mut i = 0usize;
    while i < s.len() {
        if (s[i] as f64) > mid {
            let start = i;
            while i < s.len() && (s[i] as f64) > mid {
                i += 1;
            }
            let len = (i - start) as f64;
            if !matches!(period, Some(p) if len > p) {
                widths.push(len);
            }
        } else {
            i += 1;
        }
    }
    if widths.is_empty() {
        return None;
    }
    let mean_samples = widths.iter().sum::<f64>() / widths.len() as f64;
    Some(mean_samples * 1e9 / rate_hz as f64)
}

/// 「真实周期」的过滤结果：中位间隔 + 通过 1.5× 筛选的间隔集。
///
/// ⚠ **这是唯一的间隔过滤实现。** [`crate::signal::classify`] 的间隔稳定性
/// （cv / 漂移）必须在**同一份过滤后的集合**上计算 —— 用未过滤的原始间隔，
/// I2C 事务间的空闲段会把干净的 SCL 判成「间隔不固定」（实测原始 cv=0.534，
/// 过滤后 0.058）。改这里的规则，两个调用方一起变。
pub(crate) fn filter_gaps(edges: &[usize]) -> Option<GapFilter> {
    if edges.len() < 3 {
        return None; // 少于 3 个边沿，中位数没有意义
    }
    let mut gaps: Vec<f64> = edges.windows(2).map(|w| (w[1] - w[0]) as f64).collect();
    let median = {
        gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        gaps[gaps.len() / 2]
    };
    let cutoff = median * 1.5;
    let total = gaps.len();
    let kept: Vec<f64> = gaps.into_iter().filter(|g| *g <= cutoff).collect();
    Some(GapFilter {
        median,
        dropped: total - kept.len(),
        kept,
    })
}

/// [`filter_gaps`] 的结果。
pub(crate) struct GapFilter {
    /// 中位间隔（样点）。
    pub median: f64,
    /// 被当空闲丢掉的间隔数。
    pub dropped: usize,
    /// 通过筛选的间隔（样点）。
    pub kept: Vec<f64>,
}

impl GapFilter {
    /// 过滤后间隔的变异系数 `cv = σ/μ`。
    ///
    /// 与 [`crate::signal::classify`] 的 `MAX_GAP_CV` 同一口径：假跃变会产生
    /// 额外的短间隔，而中位数 ×1.5 过滤器只丢长间隔、不丢短间隔，所以 cv
    /// 是「边沿集合被污染」的可观测量。
    pub(crate) fn cv(&self) -> Option<f64> {
        if self.kept.is_empty() {
            return None;
        }
        let mean = self.kept.iter().sum::<f64>() / self.kept.len() as f64;
        if mean <= 0.0 {
            return None;
        }
        let var = self
            .kept
            .iter()
            .map(|g| (g - mean) * (g - mean))
            .sum::<f64>()
            / self.kept.len() as f64;
        Some(var.sqrt() / mean)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1 MHz 采样 —— 一个样点正好 1000 ns，断言可以写整数。
    const RATE: u32 = 1_000_000;

    fn cap_of(s: Vec<u16>, rate: u32) -> Capture {
        let mut c = Capture::new(1, rate, 1, s.len() as u32);
        c.channels[0] = s;
        c
    }

    fn m(s: Vec<u16>, rate: u32) -> Measurements {
        measure(&cap_of(s, rate), 0, &ChannelScale::default()).expect("单通道捕获必须测得出")
    }

    /// 周期方波：每周期 `high` 个高电平 + `low` 个低电平。
    fn pwm(high: usize, low: usize, periods: usize) -> Vec<u16> {
        let mut v = Vec::with_capacity((high + low) * periods);
        for _ in 0..periods {
            v.resize(v.len() + high, 1000);
            v.resize(v.len() + low, 0);
        }
        v
    }

    /// 线性边沿：`up` 步从 0 爬到 1000，保持，`down` 步落回 0。
    fn ramp(up: usize, hold: usize, down: usize, tail: usize) -> Vec<u16> {
        let mut v: Vec<u16> = (0..=up).map(|i| (i * 1000 / up) as u16).collect();
        v.resize(v.len() + hold, 1000);
        for i in 1..=down {
            v.push((1000 - i * 1000 / down) as u16);
        }
        v.resize(v.len() + tail, 0);
        v
    }

    // ── 边沿时间 ─────────────────────────────────────────────────

    /// **下降时间用下降沿自己的陡度，不是上升沿的。**
    ///
    /// 非对称是刻意的：对称波形上「fall 直接复用 rise 的实现」这个变异体
    /// 根本杀不掉 —— 而那正是这类代码最常见的错法。
    ///
    /// 上升 10 个采样间隔 → 10%→90% 占 8 个间隔 = 8000 ns；
    /// 下降 2 个采样间隔 → 占 1.6 个间隔 = 1600 ns。
    ///
    /// 变异：`fall_ns` 复用 `rise_ns` 的值 —— 这条红。
    #[test]
    fn fall_time_uses_the_falling_slopes_own_steepness() {
        let x = m(ramp(10, 5, 2, 5), RATE);
        let (r, f) = (x.rise_ns.expect("上升时间"), x.fall_ns.expect("下降时间"));
        assert!((r - 8000.0).abs() < 100.0, "上升时间实测 {r} ns，应为 8000");
        assert!((f - 1600.0).abs() < 100.0, "下降时间实测 {f} ns，应为 1600");
        assert!(f < r / 3.0, "两者差得太小 —— 镜像多半写成了复制粘贴");
    }

    /// **慢边沿必须测得出。**
    ///
    /// 回归：加「退到离开上一档门限的位置再往前搜」这个锚点之前，从**中点
    /// 穿越点**往后搜永远找不到 10% 穿越点（它在中点之前），于是慢边沿
    /// 一律返回 `None` —— 而本函数的文档一直声称支持它。2026-10-06 实测确认。
    ///
    /// 变异：把 `anchor` 换回 `start` —— 这条红。
    #[test]
    fn a_slow_edge_is_measurable_not_silently_none() {
        let x = m(ramp(10, 5, 2, 5), RATE);
        assert!(
            x.rise_ns.is_some(),
            "10 个采样间隔的上升沿必须测得出，实测 None"
        );
    }

    /// 正弦的 10%→90% 过渡是**解析值**：`2·asin(0.8)/(2π)·T = 0.2952·T`。
    ///
    /// 1 kHz → 295.2 µs。这不是从实现里抄出来的数，是算出来的 ——
    /// 所以它能同时钉住锚点、门限比例和插值三件事。
    ///
    /// 变异：门限从 10%/90% 改成 20%/80% → 变成 0.2048·T，这条红。
    #[test]
    fn a_sine_edge_time_matches_the_analytic_value() {
        let rate = 857_142u32;
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / rate as f64;
                (2048.0 + 1700.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin())
                    .round()
                    .clamp(0.0, 4095.0) as u16
            })
            .collect();
        let x = m(s, rate);
        let expect = 0.29517 * 1e9 / 1000.0; // 295 170 ns
        let got = x.rise_ns.expect("正弦的过渡时间");
        assert!(
            (got - expect).abs() < expect * 0.01,
            "实测 {got} ns，解析值 {expect} ns"
        );
    }

    /// **被截断的边沿要如实说「测不出」，不许拿后面那个完整边沿凑一个数。**
    ///
    /// 加窗口之前，10% 找到之后可以一路扫到几百个样点之外，把 90% 从
    /// **下一个**边沿上取下来 —— 返回一个横跨谷底的荒谬值（本用例里是
    /// 100 733 ns，而真实过渡只有几个采样的量级）。
    ///
    /// 构造：第一个上升沿只到 60%（够跨中点、够不着 90%），后面接 3 个完整
    /// 周期让周期估得出来，窗口才生效（半周期 = 50 样点，够不到 100 之外）。
    ///
    /// 变异：`window` 一律传 `None` —— 这条红。
    #[test]
    fn a_truncated_edge_reports_none_instead_of_borrowing_the_next_one() {
        let mut s = vec![0u16; 60];
        s.resize(100, 600); // 只升到 60%
        s.resize(160, 0);
        s.extend_from_slice(&pwm(50, 50, 3));

        let x = m(s, RATE);
        assert!(x.freq_hz.is_some(), "构造必须有可测的周期，否则窗口不生效");
        assert!(
            x.rise_ns.is_none(),
            "只升到 60% 的边沿应报「测不出」，实测 {:?}",
            x.rise_ns
        );
    }

    // ── 过冲 / 下冲 ──────────────────────────────────────────────

    /// 阶跃响应：基线 0，稳态 1000，冲顶 1200 → 过冲 20%。
    ///
    /// 分母是**平台间隔**（1000），不是全窗峰峰值（1200）；
    /// 参考是**平台均值**（1000），不是全窗最大值（1200）。
    ///
    /// 变异：分母写成全窗峰峰值 → 16.7%，这条红；
    ///       参考写成全窗最大值 → 0%，这条红。
    #[test]
    fn overshoot_is_relative_to_the_plateau_not_the_window_extremes() {
        let mut s = vec![0u16; 300];
        s.resize(890, 1000); // 稳态平台
        s.push(1200); // 冲顶一个样点
        s.resize(1000, 1000); // 再补稳态，让平台占绝对多数

        let x = m(s, RATE);
        let over = x.overshoot_pct.expect("阶跃响应应给出过冲");
        assert!(
            (over - 20.0).abs() < 1.5,
            "过冲实测 {over}%，应为 20%（分母是平台间隔 1000，不是峰峰值 1200）"
        );
        assert!(
            x.undershoot_pct.expect("下冲").abs() < 1e-9,
            "基线没有被冲低，下冲应为 0"
        );
    }

    /// **没有过冲时是 `Some(0.0)`，不是 `None`。**
    ///
    /// 「测了，是 0」与「测不出」是两件不同的事 —— 前者是结论，后者是缺失。
    /// 混起来读者就分不清「这个信号很干净」与「这个信号根本没法测」。
    ///
    /// 变异：算出 0 时返回 `None` —— 这条红。
    #[test]
    fn zero_overshoot_is_a_measurement_not_a_missing_value() {
        let x = m(pwm(10, 10, 20), RATE);
        assert_eq!(
            x.overshoot_pct,
            Some(0.0),
            "干净方波的过冲应如实报 0，而不是 None"
        );
        assert_eq!(x.undershoot_pct, Some(0.0));
    }

    /// 正弦**没有**稳态电平，过冲必须是 `None`。
    ///
    /// 拿双电平中心硬算会给正弦造出几十个百分点的假过冲 —— 而那个数字会被
    /// 语言模型当成事实引用。
    ///
    /// 变异：去掉 `is_two_level()` 门限 —— 这条红。
    #[test]
    fn overshoot_is_none_for_a_signal_without_steady_levels() {
        let rate = 857_142u32;
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / rate as f64;
                (2048.0 + 1700.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin())
                    .round()
                    .clamp(0.0, 4095.0) as u16
            })
            .collect();
        let x = m(s, rate);
        assert!(
            x.overshoot_pct.is_none(),
            "正弦不该有「过冲」这个量，实测 {:?}",
            x.overshoot_pct
        );
        assert!(x.undershoot_pct.is_none());
    }

    // ── 脉宽 ─────────────────────────────────────────────────────

    /// 25% 占空比 @ 1 MHz、周期 20 样点 → 高电平 5 个样点 = 5000 ns。
    ///
    /// 断言写**精确值**而不是「差不多」：差一个样点就是 +1000 ns，
    /// 那正是「高电平段多算一格」这类 off-by-one 的症状。
    ///
    /// 变异：数高电平段时边界写成闭区间（多算一个样点）→ 6000 ns，这条红。
    #[test]
    fn high_time_is_the_pulse_width_in_nanoseconds() {
        let x = m(pwm(5, 15, 20), RATE);
        let h = x.high_ns.expect("脉宽");
        assert!((h - 5000.0).abs() < 1.0, "脉宽实测 {h} ns，应为 5000");
    }

    /// **周期测不出来时脉宽仍然有意义** —— 这是 `high_ns` 单独存在的理由。
    ///
    /// 孤立脉冲：只有 1 个上升沿，而 `median_gap` 要 3 个才给周期 ——
    /// `duty_pct` 是 `None`，脉宽却是个实打实的数。
    ///
    /// 变异：`high_ns` 也要求周期（照抄 duty 的 `period.map`）—— 这条红。
    #[test]
    fn pulse_width_survives_without_a_measurable_period() {
        let mut s = vec![0u16; 50];
        s.resize(57, 1000); // 7 个样点的脉冲
        s.resize(200, 0);
        let x = m(s, RATE);
        assert!(x.duty_pct.is_none(), "只有一个上升沿，占空比应测不出");
        let h = x.high_ns.expect("孤立脉冲的脉宽必须给得出");
        assert!((h - 7000.0).abs() < 1.0, "脉宽实测 {h} ns，应为 7000");
    }

    // ── 边沿计数 ─────────────────────────────────────────────────

    /// 边沿数用**实测中点**，不是固定的 2048 LSB。
    ///
    /// 信号在 3000..4000 之间摆动：中点 3500 处有 39 个跃变，而
    /// `ChannelSummary` 那个固定 2048 的计数器一个都数不到。
    ///
    /// 变异：`edges` 改成读 `summary.rising_edges` —— 前半段红。
    #[test]
    fn edge_count_follows_the_measured_midpoint() {
        let mut s = pwm(5, 5, 20);
        for v in s.iter_mut() {
            *v += 3000; // 整体抬到 3000..4000
        }
        let cap = cap_of(s, RATE);
        let x = measure(&cap, 0, &ChannelScale::default()).unwrap();

        assert_eq!(x.edges, 39, "3000..4000 之间应有 39 个跃变");
        assert_eq!(
            cap.summary(0).map(|s| s.rising_edges).unwrap_or(0),
            0,
            "固定 2048 门限的计数器一个边沿都数不到 —— 两者不同源，这是有意的"
        );
    }

    /// 直流：计数是 0（计数没有「测不出」这回事），其余新字段一律 `None`。
    #[test]
    fn a_flat_line_has_no_edges_and_no_new_measurements() {
        let x = m(vec![2048u16; 500], RATE);
        assert_eq!(x.edges, 0);
        assert!(x.rise_ns.is_none());
        assert!(x.fall_ns.is_none());
        assert!(x.overshoot_pct.is_none());
        assert!(x.undershoot_pct.is_none());
        assert!(x.high_ns.is_none());
    }
}
