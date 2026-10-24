//! 信号形状判定 —— 「这是什么波形」。
//!
//! # 它为什么存在
//!
//! 在它之前，主机侧对波形的全部理解只有 9 个标量（Vpp / 频率 / 占空比 / 上升时间…）。
//! 它们**没有一个是「形状」**：一个 40% 占空比的方波与一个方波状的数据线，
//! 在这些数字上长得一模一样。于是分析面板只能把判断形状这件事整个交给语言模型 ——
//! 而模型手上也只有那 9 个数，只能猜。
//!
//! 2026-10-06 真机：屏幕上给一个 2 kHz、占空比 50.3% 的方波，回答里三段有两段在讲
//! 「缺 SDA，I2C 解码做不了」。那个跑偏的入口正是这里 —— 证据包里只有
//! 「有解码 / 无解码」两个分支，非 I2C 的波形没有任何名字。
//!
//! # 两个刻意的设计
//!
//! 1. **判决全部在原始 LSB 域完成**，伏特只在出口换算（与 [`crate::measure`] 同一条纪律）。
//! 2. **返回证据，不返回布尔**。分类结果会作为「事实」进语言模型的提示词，
//!    按项目纪律，一条结论必须带上支撑它的数字 —— 判错了，读者要能一眼看出来。
//!
//! # 常量怎么读
//!
//! 下面每个数值常量都标了它是**拍脑袋的**还是**从公式来的**，拍脑袋的还要写出
//! 它的失效方向。这是本仓库对「魔法数字」的一贯要求：不许有一个数字说不清来历。

use crate::capture::{Capture, ChannelSummary};
use crate::f103;

// ── 直方图 ────────────────────────────────────────────────────────

/// 直方图桶数。4096 / 256 = 每桶 16 LSB。
///
/// **16 不是随便挑的**：它正好等于硬件触发用的迟滞带宽
/// （[`f103::TRIGGER_HYSTERESIS_LSB`]）。16 LSB 以内的电平差，连采集链自己
/// 都当成同一个电平；再细分只会把噪声切成碎块。
const HIST_BINS: usize = 256;

/// 一个样点落到哪个桶 —— 右移 4 位即除以 16。
const BIN_SHIFT: u32 = 4;

// ── 门限 ──────────────────────────────────────────────────────────

/// 少于这么多样点，判不出任何东西。
///
/// **拍脑袋的。** 失效方向：调大 → 短采集（协议下限 1 点）一律判不出，
/// 保守但会漏；调小 → 2 个样点也能凑出「完美的双电平」（`[0, 4095]`），
/// 那是纯噪声。测试里 `&[0, 4095]` 走的是这一条。
const MIN_SAMPLES: usize = 16;

/// 两个平台的最小间隔（LSB）。
///
/// **拍脑袋的**，参考量级：3.3 V 满量程下 256 LSB ≈ 0.2 V。
/// 真实的逻辑电平（3V3、1V8 经电平转换）远大于它。
/// 失效方向：调小 → 噪声里也能「找到」两个平台；调大 → 低摆幅的逻辑信号判不出。
const MIN_SEP_LSB: f64 = 256.0;

/// 平台带宽 = 平台间隔的这个比例。
///
/// **拍脑袋的**，但它是「占用率」这个判据的核心，而占用率是区分方波与正弦的
/// 唯一依据（见 [`MIN_OCCUPANCY`]）。
/// 失效方向：调大 → 正弦的绝大部分样点也被算作「在平台上」，方波与正弦不再可分；
/// 调小 → 平台上的噪声样点被踢出去，占用率虚低，真方波判不出。
const PLATEAU_TOL_FRAC: f64 = 0.05;

/// 平台带宽的下限（LSB）。与桶宽同源 —— 16 LSB 以内的差异是同一个电平。
const PLATEAU_TOL_MIN_LSB: f64 = f103::TRIGGER_HYSTERESIS_LSB as f64;

/// 落在两个平台内的样点占比下限 —— **这就是置信度**。
///
/// **拍脑袋的**，但有余量依据：50% 方波与 I2C 的 SCL 实测在 0.95 以上；
/// 而正弦、三角、均匀噪声都落在 0.1 量级（正弦的双电平中心在 ±0.637A，
/// 落在 ±5%·间隔 带内的时间只有百分之几）。门限取 0.70，与两者都差着
/// 一个数量级 —— 不是卡在边界上的数。
///
/// ⚠ **具体数值由测试钉死，不要把这个「0.1 量级」当成实测值到处引用。**
///
/// 失效方向：调低 → 正弦被当成方波；调高 → 边沿慢的真方波（边沿样点占比大）判不出。
const MIN_OCCUPANCY: f64 = 0.70;

/// 少数派平台至少要有这么多个样点。
///
/// **拍脑袋的。** 与 `measure::median_gap` 的「少于 3 个边沿没有意义」同量级：
/// 1 个样点是一次毛刺，3 个才够算一个电平。
/// 失效方向：调 1 → 全窗一个尖峰也算「高平台」，直方图从此不可信。
const MIN_MINORITY_COUNT: usize = 3;

/// 少数派平台的占比下限。
///
/// **拍脑袋的，唯一目的是挡住「大窗口里的偶发尖峰」**：4096 点里少于 0.5%
/// 就是不到 21 个样点。它和上一条**必须同时满足**，因为两条挡的不是一件事 ——
/// 这条挡「占比太小」，上一条挡「绝对数太小」。100 点的窗口里 5% 占空比只有
/// 5 个样点，靠的是上一条而不是这一条。
const MIN_MINORITY_FRAC: f64 = 0.005;

// ── 结果 ──────────────────────────────────────────────────────────

/// 双电平判定的结果 —— **永远返回结构，不返回 `bool`**。
///
/// 判不出来时，各个数值字段仍然带着尽力估计的值，供调用方放进证据里 ——
/// 但那时它们**不可当结论引用**（`verdict` 已经说了判不出来）。
#[derive(Debug, Clone, PartialEq)]
pub struct LevelStats {
    /// 参与判定的样点数。
    pub n: usize,
    /// 低平台的样点均值（LSB）。
    pub vlo_lsb: f64,
    /// 高平台的样点均值（LSB）。
    pub vhi_lsb: f64,
    /// `vhi_lsb - vlo_lsb`。
    pub sep_lsb: f64,
    /// 落在两个平台带宽内的样点占比（0..1）。**这是置信度**。
    pub occupancy: f64,
    /// 两个平台中较少的那一个的样点占比（0..1）。
    pub minority_frac: f64,
    /// 判定了什么。
    pub verdict: LevelVerdict,
}

/// 双电平判决的结论。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LevelVerdict {
    /// 是一个干净的双电平（逻辑）信号。
    TwoLevel,
    /// 不是。原因是第一个没通过的门限。
    NotTwoLevel(NotLevelReason),
}

/// 判不出双电平的原因。
///
/// 按**固定顺序**取第一个不满足的门限 —— 顺序写死是为了让「同一个输入永远
/// 得到同一个原因」，否则证据包会随实现细节抖动。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotLevelReason {
    /// 样点太少。
    TooShort,
    /// 只有一座峰 —— 直流，或者整段落在同一个电平里。
    SinglePeak,
    /// 两个平台的间隔小于 [`MIN_SEP_LSB`]。
    SeparationTooSmall,
    /// 平台占用率低于 [`MIN_OCCUPANCY`]，形状不是矩形的（正弦、三角、噪声）。
    OccupancyTooLow,
    /// 少数派平台的样点太少 —— 那是一次毛刺，不是一个电平。
    MinorityTooSmall,
}

impl LevelStats {
    /// 是不是一个干净的双电平信号。
    pub fn is_two_level(&self) -> bool {
        self.verdict == LevelVerdict::TwoLevel
    }
}

/// 桶 `i` 代表的电平（LSB）—— 取桶中心。
fn bin_center(i: usize) -> f64 {
    (i as f64 + 0.5) * (1u32 << BIN_SHIFT) as f64
}

/// 对一批原始样点做双电平判定。
///
/// 见模块头的说明。这是纯函数：不碰标定、不碰设备、不做换算。
pub fn detect_levels(s: &[u16]) -> LevelStats {
    let n = s.len();

    // ── 1. 样点太少 ──
    if n < MIN_SAMPLES {
        let m = mean_of(s);
        return LevelStats {
            n,
            vlo_lsb: m,
            vhi_lsb: m,
            sep_lsb: 0.0,
            occupancy: 0.0,
            minority_frac: 0.0,
            verdict: LevelVerdict::NotTwoLevel(NotLevelReason::TooShort),
        };
    }

    // ── 2. 直方图 ──
    let mut hist = [0u16; HIST_BINS];
    for &v in s {
        let b = (v as u32 >> BIN_SHIFT) as usize;
        hist[b] = hist[b].saturating_add(1);
    }

    // ── 3. Otsu 定初始分界 ──
    //
    // **不能用全局均值去初始化 k-means。** 95/5 的窄脉冲（`pwm_1k_5` 那种）
    // 会让两个中心双双落进 95% 那一侧，整个判定崩掉。Otsu 找的是
    // 「类间方差最大」的分界，它保证初始分界落在两类**之间**。
    let total = n as f64;
    let Some(t) = otsu_split(&hist, total) else {
        // 全体样点挤在同一侧 —— 直流，或者整段同一个电平
        let m = mean_of(s);
        return LevelStats {
            n,
            vlo_lsb: m,
            vhi_lsb: m,
            sep_lsb: 0.0,
            occupancy: 0.0,
            minority_frac: 0.0,
            verdict: LevelVerdict::NotTwoLevel(NotLevelReason::SinglePeak),
        };
    };

    // ── 4. 桶上迭代两次 2-均值 ──
    //
    // 在**桶**上迭代而不是在样点上：桶最多 256 个，一次迭代几百次运算，
    // 而一维的两类问题收敛极快，两次足够。
    let (mut c0, mut c1) = match (bin_mean(&hist, 0, t), bin_mean(&hist, t + 1, HIST_BINS - 1)) {
        (Some(a), Some(b)) => (a, b),
        // 分界两侧有一侧是空的 —— 与「只有一座峰」是同一件事
        _ => {
            let m = mean_of(s);
            return LevelStats {
                n,
                vlo_lsb: m,
                vhi_lsb: m,
                sep_lsb: 0.0,
                occupancy: 0.0,
                minority_frac: 0.0,
                verdict: LevelVerdict::NotTwoLevel(NotLevelReason::SinglePeak),
            };
        }
    };
    for _ in 0..2 {
        let boundary = (c0 + c1) / 2.0;
        let (a, b) = rebalance(&hist, boundary);
        match (a, b) {
            (Some(x), Some(y)) => {
                c0 = x;
                c1 = y;
            }
            // 迭代把一侧清空了 —— 退回上一轮的中心，别再动
            _ => break,
        }
    }
    if c1 < c0 {
        std::mem::swap(&mut c0, &mut c1);
    }
    let sep_bin = c1 - c0;
    if sep_bin < MIN_SEP_LSB {
        return LevelStats {
            n,
            vlo_lsb: c0,
            vhi_lsb: c1,
            sep_lsb: sep_bin,
            occupancy: 0.0,
            minority_frac: 0.0,
            verdict: LevelVerdict::NotTwoLevel(NotLevelReason::SeparationTooSmall),
        };
    }

    // ── 5. 回到原始样点：精修平台均值 / 占用率 / 少数派 ──
    //
    // 桶中心只有 16 LSB 的分辨率，直接拿它当电平会引入最多 8 LSB 的偏差；
    // 而「平台均值」是这个判定的最终产物（过冲要拿它当参考），必须精确。
    let tol = (PLATEAU_TOL_FRAC * sep_bin).max(PLATEAU_TOL_MIN_LSB);
    let (mut nlo, mut nhi) = (0usize, 0usize);
    let (mut slo, mut shi) = (0.0f64, 0.0f64);
    for &v in s {
        let vf = v as f64;
        let dlo = (vf - c0).abs();
        let dhi = (vf - c1).abs();
        // 平局归低平台 —— 任选一个都行，但必须**确定**，否则同一输入会给出不同结果
        if dlo <= tol && dlo <= dhi {
            nlo += 1;
            slo += vf;
        } else if dhi <= tol {
            nhi += 1;
            shi += vf;
        }
    }
    if nlo == 0 || nhi == 0 {
        return LevelStats {
            n,
            vlo_lsb: c0,
            vhi_lsb: c1,
            sep_lsb: sep_bin,
            occupancy: 0.0,
            minority_frac: 0.0,
            verdict: LevelVerdict::NotTwoLevel(NotLevelReason::OccupancyTooLow),
        };
    }

    let vlo = slo / nlo as f64;
    let vhi = shi / nhi as f64;
    let sep = vhi - vlo;
    let occupancy = (nlo + nhi) as f64 / total;
    let minority = nlo.min(nhi);

    // ── 6. 门限，固定顺序 ──
    let reason = if sep < MIN_SEP_LSB {
        Some(NotLevelReason::SeparationTooSmall)
    } else if occupancy < MIN_OCCUPANCY {
        Some(NotLevelReason::OccupancyTooLow)
    } else if minority < MIN_MINORITY_COUNT || (minority as f64) < MIN_MINORITY_FRAC * total {
        Some(NotLevelReason::MinorityTooSmall)
    } else {
        None
    };

    LevelStats {
        n,
        vlo_lsb: vlo,
        vhi_lsb: vhi,
        sep_lsb: sep,
        occupancy,
        minority_frac: minority as f64 / total,
        verdict: match reason {
            Some(r) => LevelVerdict::NotTwoLevel(r),
            None => LevelVerdict::TwoLevel,
        },
    }
}

/// Otsu 分界：使类间方差最大的那个桶。
///
/// 返回 `None` 表示**分不开**（一侧恒空）—— 那就是「只有一座峰」。
fn otsu_split(hist: &[u16; HIST_BINS], total: f64) -> Option<usize> {
    let sum_all: f64 = hist
        .iter()
        .enumerate()
        .map(|(i, &c)| bin_center(i) * c as f64)
        .sum();

    let (mut w0, mut sum0) = (0.0f64, 0.0f64);
    let mut best: Option<(f64, usize)> = None;
    for (t, &count) in hist.iter().enumerate().take(HIST_BINS - 1) {
        w0 += count as f64;
        sum0 += bin_center(t) * count as f64;
        let w1 = total - w0;
        if w0 <= 0.0 || w1 <= 0.0 {
            continue;
        }
        let m0 = sum0 / w0;
        let m1 = (sum_all - sum0) / w1;
        // 类间方差 —— 与总方差只差一个常数因子，比大小用不上那个因子
        let between = w0 * w1 * (m0 - m1) * (m0 - m1);
        // `map_or` 而不是 `is_none_or`：后者的 MSRV 是 1.82，本 crate 是 1.75
        if best.map_or(true, |(b, _)| between > b) {
            best = Some((between, t));
        }
    }
    best.map(|(_, t)| t)
}

/// 桶区间 `[from, to]` 上的加权平均电平。区间内无样点时返回 `None`。
fn bin_mean(hist: &[u16; HIST_BINS], from: usize, to: usize) -> Option<f64> {
    let (mut w, mut sum) = (0.0f64, 0.0f64);
    let end = to.min(HIST_BINS - 1) + 1;
    for (i, &count) in hist.iter().enumerate().take(end).skip(from) {
        w += count as f64;
        sum += bin_center(i) * count as f64;
    }
    if w <= 0.0 {
        None
    } else {
        Some(sum / w)
    }
}

/// 按 `boundary`（LSB）把桶重新分成两组，返回两组各自的加权平均。
///
/// 分组按**桶中心**判，与样点分组用同一个边界 —— 两边口径不一致会让
/// 「桶上迭代出来的中心」与「样点上算出来的均值」对不上。
fn rebalance(hist: &[u16; HIST_BINS], boundary: f64) -> (Option<f64>, Option<f64>) {
    let (mut w0, mut s0) = (0.0f64, 0.0f64);
    let (mut w1, mut s1) = (0.0f64, 0.0f64);
    for (i, &count) in hist.iter().enumerate() {
        let c = bin_center(i);
        let w = count as f64;
        if c < boundary {
            w0 += w;
            s0 += c * w;
        } else {
            w1 += w;
            s1 += c * w;
        }
    }
    (
        if w0 > 0.0 { Some(s0 / w0) } else { None },
        if w1 > 0.0 { Some(s1 / w1) } else { None },
    )
}

/// 样点均值。空切片返回 0.0 —— 只在已经判过 `n < MIN_SAMPLES` 的分支里调，
/// 那里要么非空、要么整个判定已经结束。
fn mean_of(s: &[u16]) -> f64 {
    if s.is_empty() {
        0.0
    } else {
        s.iter().map(|&v| v as f64).sum::<f64>() / s.len() as f64
    }
}

/// 一个通道的分类结果（**形状 + 证据**）。
///
/// 形状单独看没有意义 —— 它进提示词时是作为「事实」用的，所以必须带上支撑它的
/// 那些数字（[`ShapeEvidence`]），判错了读者要能一眼看出来。见模块头第 2 条。
///
/// # `Unknown` 为什么带载荷
///
/// 独立字段 `uncertain: Option<...>` 允许三种非法状态：「判不出却无原因」、
/// 「判出了却带着原因」。把原因收进 `Unknown` 的载荷之后，非法状态在**类型上**
/// 构造不出来 —— 渲染层永远有原因可印。
#[derive(Debug, Clone, PartialEq)]
pub struct Classification {
    /// 判成了什么。
    pub kind: SignalKind,
    /// 判定所用的全部数字。**渲染层只格式化这些，不得重算**（report.rs 的章程）。
    pub evidence: ShapeEvidence,
}

/// 波形形状。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalKind {
    /// 没有可测的波动 —— 平线。
    Dc,
    /// 噪声（沿很多但没有规律）。
    Noise,
    /// 双电平周期信号，占空比接近一半。
    Square,
    /// 双电平周期信号，占空比远离一半（窄脉冲 / 宽脉冲）。
    Pulse,
    /// 阶跃响应 —— 只朝一个方向跃变过。
    Step,
    /// 正弦。
    Sine,
    /// 调幅 / 幅度起伏。
    Am,
    /// 判不出来，附原因。
    Unknown(UncertainReason),
}

/// 判不出形状的原因。
///
/// ⚠ 显示措辞由渲染层负责，本枚举只承载**分类**。渲染层必须把它们翻成
/// 「本工具边界」句式（如「未检出稳定周期」而不是「找不到周期」），
/// 否则会被用户读成信号缺陷。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UncertainReason {
    /// 样点太少。
    TooFewSamples,
    /// 有波动，但幅度低于判形状的门限。
    TooSmallAmplitude,
    /// 沿数 / 间隔数不足以谈周期性。
    NoPeriodicity,
    /// 边沿间隔没有规律（数据线、突发、毛刺串）。
    IntervalUnstable,
    /// 窗内频率在漂移（扫频 / 变频）。
    FrequencyDrift,
    /// 周期短到接近奈奎斯特 —— 采样值本身就不可信。
    Aliased,
    /// 有周期，但形状不在已知的表里（三角、锯齿、失真的波形）。
    ShapeUnrecognized,
}

/// 判定所用的全部数字 —— 见 [`Classification`] 的说明。
///
/// 每个字段在**判不出**的路径上也都有值（尽力估计），渲染层不得在
/// `kind == Unknown` 时把它们当结论引用。
#[derive(Debug, Clone, PartialEq)]
pub struct ShapeEvidence {
    /// 参与判定的样点数。
    pub n: usize,
    /// 峰峰值（LSB）。
    pub pp_lsb: u16,
    /// 扣除直流后的 RMS（LSB）。
    pub ac_rms_lsb: f64,
    /// 双电平判定的完整结果。
    pub levels: LevelStats,
    /// 边沿判定的中点（LSB）—— **实测中点** `(窗内 min + max) / 2`，
    /// 不是 `ChannelSummary` 那个固定 2048 的计数器。
    pub edge_mid_lsb: f64,
    /// 中点处的上升沿数。
    pub rising: usize,
    /// 中点处的下降沿数。
    pub falling: usize,
    /// 过滤后的间隔统计；沿太少时为 `None`。
    pub gaps: Option<GapStats>,
    /// 平均周期（样点）—— 与 [`crate::measure`] 的周期同一来源。
    pub period_samples: Option<f64>,
    /// 频率（Hz）—— 同 `Measurements::freq_hz`。
    pub freq_hz: Option<f64>,
    /// 占空比（%）—— 同 `Measurements::duty_pct`。
    pub duty_pct: Option<f64>,
    /// 波形因数 `ac_rms / pp`。幅值非零时才有。
    pub form_factor: Option<f64>,
    /// 滑窗峰峰起伏（0..∞）；窗数不足时为 `None` —— 渲染层要把它印成
    /// 「调幅检查未做」，而不是省略。
    pub seg_var: Option<f64>,
    /// 过冲（%）；仅双电平信号有。
    pub overshoot_pct: Option<f64>,
}

/// 过滤后间隔的统计（与 [`crate::measure`] 同一份过滤：中位数 × 1.5）。
#[derive(Debug, Clone, PartialEq)]
pub struct GapStats {
    /// 通过过滤的间隔数。
    pub kept: usize,
    /// 被当空闲丢掉的间隔数。
    pub dropped: usize,
    /// 过滤后间隔的均值（样点）。
    pub mean: f64,
    /// 过滤后间隔的变异系数（总体标准差 / 均值）。
    pub cv: f64,
    /// 前后半段均值差的相对值 `|mean(后) − mean(前)| / mean`。
    /// 间隔不足 4 条时为 0.0 —— 样本太少，不做漂移结论。
    pub drift: f64,
}

// ── 分类用的门限 ──────────────────────────────────────────────────

/// 波形因数差多少以内算「正弦」。
///
/// 正弦的波形因数 `ac_rms / pp = 1/(2√2) ≈ 0.35355` 是**物理值**；
/// 三角 / 锯齿是 `1/(2√3) ≈ 0.2887`，差 0.065。
/// 0.045 是拍脑袋的：放这么宽是为了收下**削顶的正弦**（过驱动的示波器上
/// 最常见的波形 —— 削掉峰值 20% 后因数升到约 0.40），同时仍然把三角
/// （0.289）拒在门外。再宽就会把三角收进来。
const FORM_FACTOR_TOL: f64 = 0.045;

/// 正弦的波形因数（理论值，不是拍的）。
const SINE_FORM_FACTOR: f64 = 1.0 / (2.0 * std::f64::consts::SQRT_2);

/// 占空比在这个区间内叫 [`SignalKind::Square`]，否则 [`SignalKind::Pulse`]。
///
/// **拍脑袋的命名边界**，不是物理判据 —— 25 与 75 之间并没有一条物理分界。
/// 而且测量本身有偏差（标称 25.0% 实测可能 24.97%），贴着边界的信号会在
/// 两个名字间摆动。好在 Square / Pulse 的证据完全相同，摆错了影响很小。
const SQUARE_DUTY_LO: f64 = 25.0;
const SQUARE_DUTY_HI: f64 = 75.0;

/// 间隔变异系数超过这个值就判「间隔不固定」。
///
/// 等周期时钟的 cv 通常 < 2%；数据线（I2C 的 SDA）约 0.27。
/// 但**纯量化也会贡献 cv**：周期只有 4 个样点时，4/5 交替本身就给出
/// 0.10 量级的 cv —— 那不是信号性质。所以实际门限取
/// `max(本值, 0.5 / 中位间隔)`：0.5 样点是整数采样下两类交替间隔的最大
/// 标准差，周期越短量化贡献越大，门限随之放宽。
/// 2026-10-06 实测：200 kHz 正弦（周期 4.286 样点）cv = 0.105 ——
/// 固定 0.10 会把它判成噪声。
const MAX_GAP_CV: f64 = 0.10;

/// 窗内频率漂移超过这个比例就不再按单一频率归类。
///
/// **拍脑袋的。** 扫频 / 变频信号超过它就是预期行为，不是故障。
const MAX_FREQ_DRIFT: f64 = 0.15;

/// 周期短于这么多样点就判 [`UncertainReason::Aliased`]。
///
/// 周期 ≤ 3 样点已接近奈奎斯特，任何形状判断都不成立。
/// ⚠ 这条**必须排在间隔稳定性之前**：周期 2.14 样点的信号（i2c_400k 的 SCL）
/// 原始 cv 高达 0.55，会被 cv 判据先截走，永远到不了这里。
const MIN_PERIOD_SAMPLES: f64 = 3.0;

/// 少于这么多个沿，不谈「有规律 / 没规律」。
///
/// 拍脑袋的。与 [`MIN_NOISE_EDGES`] 同族：一个要求「至少这么多才敢说有周期」，
/// 一个要求「至少这么多才敢说是噪声」，中间的落 [`UncertainReason::NoPeriodicity`]。
const MIN_PERIOD_EDGES: usize = 4;

/// 沿数达到这个数且间隔无规律 → [`SignalKind::Noise`]。
///
/// **拍脑袋的。** 少于它的无规律信号落 `NoPeriodicity` —— 证据太少，
/// 「噪声」与「没采够」分不开。
const MIN_NOISE_EDGES: usize = 8;

/// 滑窗峰峰起伏超过这个比例 → [`SignalKind::Am`]。
///
/// 拍脑袋的。真实调幅（模拟器 `am` 场景）的起伏远大于它（约 1.5），
/// 而纯正弦各窗峰峰几乎相同（约 0.01），中间隔着两个数量级。
const SLIDING_VAR_THRESHOLD: f64 = 0.25;

/// 调幅检查至少要有这么多个完整窗才下结论。
const MIN_AM_WINDOWS: usize = 3;

/// 判「直流」的 AC-RMS 门限（LSB）。值 = `16/√12 ≈ 4.6188`。
///
/// 与硬件迟滞带同源：16 LSB 的均匀噪声标准差是 `16/√12`。采集链把 16 LSB
/// 以内的差异当成同一个电平（`TRIGGER_HYSTERESIS_LSB`），所以低于这个
/// 噪声底的波动就是「平」。数字预先算好是因为 `sqrt` 不是 const fn。
///
/// 2026-10-06 评审实测：旧值 2.0 LSB 会把「1.65 V 轨 + 60 mVpp 纹波」
/// 判成 Unknown，而那是直流测量最常见的场景。
const DC_AC_RMS_LSB: f64 = 4.6188;

/// 判形状的最小幅度（LSB）。
///
/// 与 [`MIN_SEP_LSB`] 同值 —— 双电平检测都嫌分不开的两个电平，形状判定
/// 也不该去猜。低于它报 [`UncertainReason::TooSmallAmplitude`]。
const MIN_PP_LSB: u16 = MIN_SEP_LSB as u16;

/// 对一个通道判形状。通道越界或为空时返回 `None`。
///
/// 纯函数：不碰标定、不碰设备；LSB 域判定，伏特只在渲染层换算。
pub fn classify(cap: &Capture, ch: usize) -> Option<Classification> {
    let s = cap.samples(ch)?;
    let sum = cap.summary(ch)?;
    let n = s.len();
    if n == 0 {
        return None;
    }

    // 测量：频率 / 占空比 / 过冲直接复用（同一套口径，禁止第二份实现）
    let m = crate::measure::measure(cap, ch, &crate::capture::ChannelScale::default())?;
    let levels = detect_levels(s);

    // 实测中点上的边沿（见模块头：不用固定 2048 的计数器）
    let (lo, hi) = (sum.min_lsb, sum.max_lsb);
    let edge_mid = (lo as f64 + hi as f64) / 2.0;
    // 与 measure() 同一套迟滞口径：h = min(3σ̂, 0.25·sep)。
    let sep = if levels.is_two_level() {
        levels.sep_lsb
    } else {
        sum.pp_lsb as f64
    };
    let h = crate::edges::sigma_hat(s)
        .map(|sigma| (3.0 * sigma).min(0.25 * sep))
        .unwrap_or(0.0);
    let (rising, falling) = crate::edges::edges_hysteretic(s, edge_mid, h);
    let gaps = crate::measure::filter_gaps(&rising).map(|f| {
        let mean = f.kept.iter().sum::<f64>() / f.kept.len() as f64;
        let var = f.kept.iter().map(|g| (g - mean) * (g - mean)).sum::<f64>() / f.kept.len() as f64;
        let cv = var.sqrt() / mean;
        let drift = if f.kept.len() >= 4 {
            let half = f.kept.len() / 2;
            let first = f.kept[..half].iter().sum::<f64>() / half as f64;
            let second = f.kept[half..].iter().sum::<f64>() / (f.kept.len() - half) as f64;
            ((second - first).abs() / mean).min(1.0)
        } else {
            0.0
        };
        GapStats {
            kept: f.kept.len(),
            dropped: f.dropped,
            mean,
            cv,
            drift,
        }
    });
    let period_samples = m.freq_hz.map(|f| cap.rate_hz as f64 / f);
    let pp = sum.pp_lsb;
    let form_factor = if pp > 0 {
        Some(sum.ac_rms_lsb / pp as f64)
    } else {
        None
    };

    let mut evidence = ShapeEvidence {
        n,
        pp_lsb: pp,
        ac_rms_lsb: sum.ac_rms_lsb,
        levels: levels.clone(),
        edge_mid_lsb: edge_mid,
        rising: rising.len(),
        falling: falling.len(),
        gaps: gaps.clone(),
        period_samples,
        freq_hz: m.freq_hz,
        duty_pct: m.duty_pct,
        form_factor,
        seg_var: None,
        overshoot_pct: m.overshoot_pct,
    };

    // 滑窗起伏：Am 判定与证据都用它。O(n) 一次扫完，代价可忽略；
    // 双电平路径不用它，但先算着比在判定里传参简单。
    let seg_var = period_samples.and_then(|p| sliding_pp_var(s, p));

    let kind = decide_kind(
        n,
        &sum,
        &m,
        &levels,
        &rising,
        &falling,
        gaps.as_ref(),
        seg_var,
    );
    evidence.seg_var = seg_var;
    Some(Classification { kind, evidence })
}

/// 判定主体。独立出来是为了让它**不碰 Capture** —— 测试可以直接喂中间量。
#[allow(clippy::too_many_arguments)]
fn decide_kind(
    n: usize,
    sum: &ChannelSummary,
    m: &crate::measure::Measurements,
    levels: &LevelStats,
    rising: &[usize],
    falling: &[usize],
    gaps: Option<&GapStats>,
    seg_var: Option<f64>,
) -> SignalKind {
    use SignalKind::*;
    use UncertainReason::*;

    // ── 1. 样本量 / 直流 / 幅度 ──
    if n < MIN_SAMPLES {
        return Unknown(TooFewSamples);
    }
    if sum.ac_rms_lsb <= DC_AC_RMS_LSB {
        return Dc;
    }
    if sum.pp_lsb < MIN_PP_LSB {
        return Unknown(TooSmallAmplitude);
    }

    // ── 2. 双电平分支（只有它知道电平）──
    if levels.is_two_level() {
        // 只朝一个方向跃变 → 阶跃响应
        if rising.is_empty() || falling.is_empty() {
            return Step;
        }
        // 周期样点数从 gaps 的均值拿（与 measure 的周期同一份过滤）
        let period = gaps.map(|g| g.mean);
        if let Some(p) = period {
            if p < MIN_PERIOD_SAMPLES {
                // **必须在稳定性之前**：周期 2 样点的交替信号 cv=0 会通过
                // 稳定性检查，而它什么形状都不是（见 MIN_PERIOD_SAMPLES 注释）
                return Unknown(Aliased);
            }
        }
        // 间隔稳定性（量化感知门限，见 MAX_GAP_CV 的注释）
        if let Some(g) = gaps {
            let cv_limit = MAX_GAP_CV.max(0.5 / g.mean);
            if g.cv > cv_limit {
                return Unknown(IntervalUnstable);
            }
            if g.drift > MAX_FREQ_DRIFT {
                return Unknown(FrequencyDrift);
            }
        }
        // 到这里：干净的双电平周期信号，按占空比命名。
        // 周期测不出（沿太少，比如只有两个脉冲）→ Pulse：
        // 占空比无意义的信号不该叫方波。
        let Some(duty) = m.duty_pct else {
            return Pulse;
        };
        return if (SQUARE_DUTY_LO..=SQUARE_DUTY_HI).contains(&duty) {
            Square
        } else {
            Pulse
        };
    }

    // ── 3. 非双电平：先过周期性这一关 ──
    let periodic = rising.len() >= MIN_PERIOD_EDGES
        && matches!(gaps, Some(g) if g.cv <= MAX_GAP_CV.max(0.5 / g.mean));
    if periodic {
        let g = gaps.expect("periodic 已保证 gaps 是 Some");
        if g.mean < MIN_PERIOD_SAMPLES {
            return Unknown(Aliased);
        }
        if g.drift > MAX_FREQ_DRIFT {
            return Unknown(FrequencyDrift);
        }
        // ⚠ Am 必须排在 Sine 前：一个「前段正常、中段被压掉一截」的正弦
        // 波形因数仍可能落在正弦容差内（实测 0.342），只有幅度起伏抓得住它。
        // 窗数不够时 seg_var 是 None —— 检查没做成，不装成做成了。
        if seg_var.is_some_and(|v| v > SLIDING_VAR_THRESHOLD) {
            return Am;
        }
        // 波形因数判定在最后：削顶正弦（约 0.40）仍要落在 Sine。
        let ff = sum.ac_rms_lsb / sum.pp_lsb.max(1) as f64;
        if (ff - SINE_FORM_FACTOR).abs() <= FORM_FACTOR_TOL {
            return Sine;
        }
        return Unknown(ShapeUnrecognized);
    }

    // ── 4. 噪声 ──
    if rising.len() >= MIN_NOISE_EDGES {
        return Noise;
    }
    Unknown(NoPeriodicity)
}

/// 滑窗峰峰起伏：把信号按「窗长 2 个周期、步长 1 个周期」切成窗，
/// 各窗峰峰值的 `(max − min) / mean`。
///
/// # 为什么用滑窗而不是固定 8 段
///
/// 固定分段是划分依赖的：包络结构恰好落进某一段还是跨两段，会改变结果，
/// 而「段数」这个常量改任何值都不会让任何测试变红（变异测试抓出来的）。
/// 窗长 2 个周期、步长 1 个周期的滑窗保证：任何不短于 1 个周期的幅度
/// 凹陷**至少被一个完整窗覆盖**，与信号相位无关。
///
/// 窗数不足 [`MIN_AM_WINDOWS`] 时返回 `None` —— 调幅检查没做成。
fn sliding_pp_var(s: &[u16], period: f64) -> Option<f64> {
    let w = (2.0 * period).ceil().max(4.0) as usize;
    let step = (w / 2).max(1);
    let mut pps = Vec::new();
    let mut start = 0usize;
    while start + w <= s.len() {
        let mut lo = u16::MAX;
        let mut hi = u16::MIN;
        for &v in &s[start..start + w] {
            lo = lo.min(v);
            hi = hi.max(v);
        }
        pps.push(hi as f64 - lo as f64);
        start += step;
    }
    if pps.len() < MIN_AM_WINDOWS {
        return None;
    }
    let mean = pps.iter().sum::<f64>() / pps.len() as f64;
    if mean <= 0.0 {
        return None;
    }
    let mut mx = f64::MIN;
    let mut mn = f64::MAX;
    for &p in &pps {
        mx = mx.max(p);
        mn = mn.min(p);
    }
    Some((mx - mn) / mean)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 3.3 V 逻辑电平在 ADC 域的典型值（与模拟器 `level_to_lsb` 相同的 0.15 / 0.85）。
    const LO: u16 = 614;
    const HI: u16 = 3481;

    /// 两个电平交替，每个电平连续 `run` 个样点，共 `periods` 个周期。
    fn alternating(lo: u16, hi: u16, run: usize, periods: usize) -> Vec<u16> {
        let mut v = Vec::with_capacity(run * 2 * periods);
        for _ in 0..periods {
            v.resize(v.len() + run, lo);
            v.resize(v.len() + run, hi);
        }
        v
    }

    /// 正弦，`cycles` 个周期铺满 `n` 个样点。
    fn sine(n: usize, cycles: f64, mid: f64, amp: f64) -> Vec<u16> {
        (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let v = mid + amp * (2.0 * std::f64::consts::PI * cycles * t).sin();
                v.round().clamp(0.0, 4095.0) as u16
            })
            .collect()
    }

    /// 梯形波：平台占 `flat`，其余均分为两个线性边沿。
    fn trapezoid(n: usize, cycles: usize, flat: f64, lo: f64, hi: f64) -> Vec<u16> {
        let e = (1.0 - flat) / 2.0;
        (0..n)
            .map(|i| {
                let p = (i * cycles) as f64 / n as f64;
                let p = p - p.floor();
                let v = if p < e {
                    lo + (hi - lo) * (p / e)
                } else if p < e + flat {
                    hi
                } else if p < e + flat + e {
                    hi - (hi - lo) * ((p - e - flat) / e)
                } else {
                    lo
                };
                v.round().clamp(0.0, 4095.0) as u16
            })
            .collect()
    }

    /// 确定性伪随机（线性同余）—— **不能用真随机**，否则回归会随机红。
    fn pseudo_noise(n: usize, span: f64) -> Vec<u16> {
        let mut x: u64 = 0x2545_F491_4F6C_DD1D;
        (0..n)
            .map(|_| {
                x = x
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let u = ((x >> 11) as f64) / ((1u64 << 53) as f64);
                (2048.0 + (u - 0.5) * span).round().clamp(0.0, 4095.0) as u16
            })
            .collect()
    }

    // ── 判定成立的情形 ────────────────────────────────────────────

    /// 干净的两个电平：电平值必须**精确**落在真实轨上。
    ///
    /// 变异（杀掉）：只拿桶中心当电平而不回原始样点精修 —— 桶宽 16 LSB，
    /// 614 落在 38 号桶、中心是 616，差 2 就能被 ±1 的断言拿住。
    #[test]
    fn a_clean_two_level_signal_reports_the_true_rails() {
        let s = alternating(LO, HI, 50, 20);
        let l = detect_levels(&s);
        assert!(l.is_two_level(), "干净方波必须判成双电平：{:?}", l.verdict);
        assert!(
            (l.vlo_lsb - LO as f64).abs() <= 1.0,
            "低平台均值 {:?} 偏离真实轨 {}",
            l.vlo_lsb,
            LO
        );
        assert!(
            (l.vhi_lsb - HI as f64).abs() <= 1.0,
            "高平台均值 {:?} 偏离真实轨 {}",
            l.vhi_lsb,
            HI
        );
        assert!(l.occupancy > 0.99, "占比 {:?} 应接近 1", l.occupancy);
    }

    /// 占用率是个**算得出来**的数，不是一个模糊的门限。
    ///
    /// 每周期 98 个平台样点 + 2 个中点样点 → 占用率恰为 0.98。
    /// 断言写死 0.98 而不是「> 0.9」：后者在占用率公式写错时照样通过。
    #[test]
    fn occupancy_is_exactly_the_plateau_share() {
        let mid = ((LO as u32 + HI as u32) / 2) as u16;
        let mut s = Vec::new();
        for _ in 0..10 {
            s.resize(s.len() + 49, LO);
            s.push(mid);
            s.resize(s.len() + 49, HI);
            s.push(mid);
        }
        let l = detect_levels(&s);
        assert!(l.is_two_level(), "仍应是双电平：{:?}", l.verdict);
        assert!(
            (l.occupancy - 0.98).abs() < 1e-9,
            "占用率应为 0.98（980/1000），实测 {:?}",
            l.occupancy
        );
    }

    /// 窄脉冲：高电平只占 5% 也不该被丢掉。
    ///
    /// 变异（杀掉）：把少数派占比门限放到 5% 以上 —— 这条会红。
    #[test]
    fn a_narrow_pulse_is_still_a_level() {
        // 4096 点里 205 个高电平（5%）
        let mut s = vec![LO; 4096];
        for i in 0..205 {
            s[i * 20] = HI;
        }
        let l = detect_levels(&s);
        assert!(l.is_two_level(), "5% 窄脉冲应仍是双电平：{:?}", l.verdict);
        assert!(
            (l.minority_frac - 205.0 / 4096.0).abs() < 0.002,
            "少数派占比 {:?}",
            l.minority_frac
        );
    }

    /// **干净方波上叠一个毛刺 —— 高平台均值不得被它拽走。**
    ///
    /// 这是本模块最值钱的一条：把 `vhi` 实现成「全窗最大值」是最诱人的
    /// 错误实现，而且它在所有**没有毛刺**的测试上都能通过。
    ///
    /// 变异（杀掉）：`vhi = s.iter().max()` —— 这条红。
    #[test]
    fn a_glitch_does_not_drag_the_high_rail() {
        let mut s = alternating(LO, HI, 50, 20);
        let idx = 137;
        s[idx] = 4095; // 一个样点的满量程尖峰
        let l = detect_levels(&s);
        assert!(l.is_two_level(), "毛刺不该让判定失效：{:?}", l.verdict);
        assert!(
            (l.vhi_lsb - HI as f64).abs() <= 2.0,
            "高平台均值被毛刺拽到了 {:?}（真实轨 {}）",
            l.vhi_lsb,
            HI
        );
    }

    /// 边沿慢的梯形波仍然算双电平 —— 判据不该卡得太紧。
    ///
    /// 变异（杀掉）：把 `MIN_OCCUPANCY` 抬到 0.9 —— 这条红。
    #[test]
    fn a_slow_edge_still_qualifies() {
        // 平台 80%，两个边沿各 10%
        let s = trapezoid(4000, 20, 0.80, LO as f64, HI as f64);
        let l = detect_levels(&s);
        assert!(l.is_two_level(), "慢边沿仍应判双电平：{:?}", l.verdict);
        assert!(l.occupancy > 0.70, "占比 {:?} 应过门限", l.occupancy);
    }

    // ── 判定不成立的情形 —— 每条都要连原因一起断言 ────────────────
    //
    // ⚠ 只断言「不是双电平」是不够的：一个「永远返回 NotTwoLevel」的变异体
    //   能通过所有这样的测试。所以正例与反例都必须有足够多的条数。

    /// **正弦不是方波。** 占用率是区分两者的唯一依据。
    ///
    /// 变异（杀掉）：删掉占用率门限 —— 这条红。
    #[test]
    fn a_sine_is_not_two_level() {
        let s = sine(4096, 20.0, 2048.0, 1700.0);
        let l = detect_levels(&s);
        assert_eq!(
            l.verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::OccupancyTooLow),
            "正弦不该被判成双电平（实测占用率 {:?}）",
            l.occupancy
        );
        assert!(
            l.occupancy < 0.30,
            "正弦占比 {:?} 应远低于门限",
            l.occupancy
        );
    }

    /// 噪声同理。
    #[test]
    fn uniform_noise_is_not_two_level() {
        let s = pseudo_noise(4096, 2000.0);
        let l = detect_levels(&s);
        assert!(
            matches!(l.verdict, LevelVerdict::NotTwoLevel(_)),
            "噪声不该判成双电平（占比 {:?}）",
            l.occupancy
        );
        assert!(l.occupancy < 0.30, "噪声占比 {:?}", l.occupancy);
    }

    /// 一条平线：只有一座峰。
    #[test]
    fn a_flat_line_has_a_single_peak() {
        let s = vec![2048u16; 1000];
        let l = detect_levels(&s);
        assert_eq!(
            l.verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::SinglePeak)
        );
        assert!(
            l.vlo_lsb.is_finite() && l.vhi_lsb.is_finite(),
            "不得出现 NaN"
        );
        assert_eq!(l.sep_lsb, 0.0);
    }

    /// 三电平（0 / 2048 / 4095 各占三分之一）不是双电平。
    ///
    /// 变异（杀掉）：删掉占用率门限 —— 中间那三分之一会落进某一个平台，
    /// 占用率掉到约 0.33，这条红。
    #[test]
    fn three_levels_are_rejected() {
        let mut s = Vec::new();
        for _ in 0..340 {
            s.extend_from_slice(&[0, 2048, 4095]);
        }
        let l = detect_levels(&s);
        assert!(
            !l.is_two_level(),
            "三电平不该判成双电平（占比 {:?}）",
            l.occupancy
        );
        assert!(l.occupancy < 0.70, "占比 {:?} 应低于门限", l.occupancy);
    }

    /// 两个电平挨得太近 —— 与噪声底不可分，报出来是垃圾。
    ///
    /// 变异（杀掉）：`MIN_SEP_LSB` 改成 0 —— 这条红。
    #[test]
    fn a_tiny_separation_is_rejected() {
        let s = alternating(2048, 2200, 50, 20); // 间隔 152 LSB
        let l = detect_levels(&s);
        assert_eq!(
            l.verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::SeparationTooSmall),
            "152 LSB 的间隔不该算两个电平"
        );
    }

    /// 长窗口里的一个尖峰**不是**一个电平 —— 那是毛刺。
    ///
    /// 变异（杀掉）：删掉整个少数派检查 —— 这条红。
    #[test]
    fn a_lone_spike_in_a_long_window_is_not_a_level() {
        let mut s = vec![LO; 4096];
        s[2000] = HI;
        let l = detect_levels(&s);
        assert_eq!(
            l.verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::MinorityTooSmall),
            "一个样点的高电平不该被当成平台"
        );
    }

    /// **占比门限的两侧** —— 这一条才是唯一能证明它还活着的。
    ///
    /// 两条门限必须是**两条**：绝对数（3 个样点）在短窗口上是唯一的拦截者，
    /// 占比（0.5%）在长窗口上是唯一的拦截者。用一个 4096 点 + 1 个尖峰的
    /// 用例去测占比分支是**测不到的** —— 绝对数分支自己就把 1 拦住了，
    /// 把占比门限调到 0.0001 测试照样绿（变异跑出来的，不是看出来的）。
    ///
    /// 所以这里刻意让**绝对数刚刚够**（都是 3 个样点），只剩占比在起作用：
    /// 1000 点里 3 个占 0.3%（不过），500 点里同样 3 个占 0.6%（过）。
    ///
    /// 变异（杀掉）：`MIN_MINORITY_FRAC` 调到 0.0001 —— 前半段红；
    ///                   调到 0.05   —— 后半段红。
    #[test]
    fn the_minority_share_threshold_has_live_inputs_on_both_sides() {
        let with_three_high = |n: usize| {
            let mut s = vec![LO; n];
            for k in 0..3 {
                s[100 + k] = HI;
            }
            detect_levels(&s)
        };

        // 3/1000 = 0.3% —— 低于 0.5%，且**绝对数正好卡在下限上**
        assert_eq!(
            with_three_high(1000).verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::MinorityTooSmall),
            "0.3% 的高电平不该被当成一个平台"
        );

        // 3/500 = 0.6% —— 同样的绝对数，只是窗口短了一半
        let l = with_three_high(500);
        assert!(
            l.is_two_level(),
            "0.6% 应过占比门限（实测 {:?}）：{:?}",
            l.minority_frac,
            l.verdict
        );
    }

    /// **短窗口里 2 个样点也不算电平，3 个才算** —— 这一条钉的是
    /// **绝对数**分支，而不是上一条的占比分支。
    ///
    /// 为什么要两条分开：占比门限在短窗口上几乎不起作用 —— 100 点里 2 个
    /// 样点占 2%，远高于 0.5%。**把绝对数门限删掉，上一条测试照样通过**，
    /// 因为长窗口里占比分支自己就把 1 个尖峰拦住了。这一条才是唯一能
    /// 证明绝对数门限还活着的东西。
    ///
    /// 变异（杀掉）：`MIN_MINORITY_COUNT` 由 3 改成 1 —— 前半段红。
    ///                   改成 4 —— 后半段红。
    #[test]
    fn a_short_window_needs_three_samples_to_call_something_a_level() {
        // 100 点里 2 个高电平：占比 2% 过得了占比门限，只有绝对数拦得住
        let mut two = vec![LO; 100];
        two[10] = HI;
        two[60] = HI;
        assert_eq!(
            detect_levels(&two).verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::MinorityTooSmall),
            "2 个样点不是一个电平"
        );

        // 同样 100 点，3 个高电平 —— 恰好到绝对数下限
        let mut three = vec![LO; 100];
        three[10] = HI;
        three[60] = HI;
        three[90] = HI;
        let l = detect_levels(&three);
        assert!(l.is_two_level(), "3 个样点应被接受：{:?}", l.verdict);
        assert!(
            (l.minority_frac - 0.03).abs() < 1e-9,
            "少数派占比 {:?}",
            l.minority_frac
        );
    }

    /// 2 个样点凑不出任何结论。
    ///
    /// 变异（杀掉）：删掉 `n < MIN_SAMPLES` 早退 —— `[0, 4095]` 会被判成
    /// 一个「完美的双电平」（两个平台各 50%，占用率 1.0）。
    #[test]
    fn two_samples_are_too_short() {
        let l = detect_levels(&[0, 4095]);
        assert_eq!(
            l.verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::TooShort),
            "2 个样点必须被拒绝"
        );
    }

    /// 空切片不得 panic。
    #[test]
    fn an_empty_slice_does_not_panic() {
        let l = detect_levels(&[]);
        assert_eq!(
            l.verdict,
            LevelVerdict::NotTwoLevel(NotLevelReason::TooShort)
        );
        assert_eq!(l.n, 0);
    }

    /// 判定结果必须**确定** —— 同一输入永远同一输出。
    ///
    /// 变异（杀掉）：在平局判定里用 `<=` 的顺序不稳定，或者引入浮点比较
    /// 的随机顺序 —— 这条会红。（`==` 的推导是精确浮点比较，见 `PartialEq`。）
    #[test]
    fn the_same_input_gives_the_same_verdict() {
        let s = trapezoid(3000, 13, 0.62, LO as f64, HI as f64);
        let a = detect_levels(&s);
        let b = detect_levels(&s);
        assert_eq!(a, b, "同一输入两次判定必须一致");
    }

    // ── 判据边界：门限两侧各来一个 ────────────────────────────────

    /// 占用率门限的两侧。
    ///
    /// 用梯形波的平台占比控制占用率：平台 75% 过门限，平台 55% 不过。
    /// 边沿样点均匀铺在中段，远离两个平台带宽（间隔的 5%），所以占用率
    /// 基本上就是平台占比本身。
    ///
    /// 变异（杀掉）：`MIN_OCCUPANCY` 单边挪动到 0.95 或 0.5 —— 两条里必红一条。
    #[test]
    fn occupancy_threshold_has_live_inputs_on_both_sides() {
        let wide = trapezoid(4000, 20, 0.75, LO as f64, HI as f64);
        assert!(
            detect_levels(&wide).is_two_level(),
            "平台 75% 应过门限（实测占比 {:?}）",
            detect_levels(&wide).occupancy
        );

        let narrow = trapezoid(4000, 20, 0.55, LO as f64, HI as f64);
        let l = detect_levels(&narrow);
        assert!(
            !l.is_two_level(),
            "平台 55% 不该过门限（实测占比 {:?}）",
            l.occupancy
        );
    }

    // ══════════════════════════════════════════════════════════════
    // classify
    // ══════════════════════════════════════════════════════════════
    //
    // 纪律（见模块头）：期望 Unknown 的用例**必须断言具体 reason** ——
    // 否则「恒返回 Unknown」的变异体能通过整套测试；期望具体类型的用例
    // 必须至少断言一个**证据数字** —— 否则「恒返回 Square」这类变异体
    // 也能混过去。

    const RATE: u32 = 1_000_000;

    fn cap_of(s: Vec<u16>, rate: u32) -> Capture {
        let mut c = Capture::new(1, rate, 1, s.len() as u32);
        c.channels[0] = s;
        c
    }

    fn c(s: Vec<u16>, rate: u32) -> Classification {
        classify(&cap_of(s, rate), 0).expect("单通道捕获必须判得出")
    }

    /// 周期方波：每周期 `high` 个高电平 + `low` 个低电平。
    fn pwm(high: usize, low: usize, periods: usize) -> Vec<u16> {
        let mut v = Vec::with_capacity((high + low) * periods);
        for _ in 0..periods {
            v.resize(v.len() + high, HI);
            v.resize(v.len() + low, LO);
        }
        v
    }

    /// 带事务空闲段的方波：`cycles` 个周期（高 `h` 低 `l`）后接 `idle` 个高电平。
    fn bursty(h: usize, l: usize, idle: usize, groups: usize) -> Vec<u16> {
        let mut v = Vec::new();
        for _ in 0..groups {
            v.extend(pwm(h, l, 10));
            v.resize(v.len() + idle, HI);
        }
        v
    }

    /// 两电平、每次电平保持的时长由确定性序列决定 —— 「数据线」的样子。
    fn data_line(runs: &[usize]) -> Vec<u16> {
        let mut v = Vec::new();
        let mut hi = true;
        for &r in runs {
            v.resize(v.len() + r, if hi { HI } else { LO });
            hi = !hi;
        }
        v
    }

    // ── 直流 / 幅度 / 样本量 ──────────────────────────────────────

    /// 平线是直流。
    #[test]
    fn a_flat_line_is_dc() {
        let x = c(vec![2048u16; 500], RATE);
        assert_eq!(x.kind, SignalKind::Dc);
    }

    /// **小纹波的轨仍然是直流** —— 判据看 AC-RMS，不看峰峰值。
    ///
    /// 2026-10-06 评审实测：旧门限 2.0 LSB 会把这条（≈ 5 mV 纹波）判成
    /// Unknown，而「带纹波的电源轨」是直流测量最常见的场景。
    ///
    /// 变异（杀掉）：`DC_AC_RMS_LSB` 改回 2.0 —— 这条红。
    #[test]
    fn a_rail_with_small_ripple_is_still_dc() {
        let mut v = Vec::new();
        for i in 0..400 {
            v.push((2048i32 + if i % 2 == 0 { 3 } else { -3 }) as u16);
        }
        let x = c(v, RATE);
        assert_eq!(x.kind, SignalKind::Dc, "±3 LSB 的纹波不该改变直流判定");
    }

    /// 幅度低于形状门限时如实说「不判」，而不是硬猜。
    ///
    /// 变异（杀掉）：`MIN_PP_LSB` 从 256 调到 100 —— 这条红
    /// （pp=200 的信号会掉进双电平/混叠分支）。
    #[test]
    fn too_small_an_amplitude_is_named_not_guessed() {
        let mut v = Vec::new();
        for i in 0..400 {
            v.push((2048i32 + if i % 2 == 0 { 100 } else { -100 }) as u16);
        }
        let x = c(v, RATE);
        assert_eq!(
            x.kind,
            SignalKind::Unknown(UncertainReason::TooSmallAmplitude)
        );
    }

    /// 2 个样点凑不出结论。
    ///
    /// 变异（杀掉）：删掉 `n < MIN_SAMPLES` 早退 —— `[0, 4095]` 会被判成
    /// 一个「完美的双电平」（周期 2 → Aliased 或别的具体类型）。
    #[test]
    fn two_samples_are_too_few_for_a_shape() {
        let x = c(vec![0, 4095], RATE);
        assert_eq!(x.kind, SignalKind::Unknown(UncertainReason::TooFewSamples));
    }

    // ── 双电平周期信号 ───────────────────────────────────────────

    /// 50% 方波是 Square，证据里的占空比要跟着。
    #[test]
    fn a_fifty_percent_square_is_square() {
        let x = c(pwm(10, 10, 30), RATE);
        assert_eq!(x.kind, SignalKind::Square);
        let d = x.evidence.duty_pct.expect("占空比证据");
        assert!((d - 50.0).abs() < 1.0, "占空比实测 {d}");
    }

    /// **直流偏置的方波仍判 Square** —— 边沿必须按实测中点数，不是固定 2048。
    ///
    /// 3000..4000 之间的方波在固定 2048 的计数器下 rising=0 → 会被判成
    /// Step。这是三个独立评审都抓到的同一个洞。
    ///
    /// 变异（杀掉）：`rising` 改读 `ChannelSummary::rising_edges` —— 这条红。
    #[test]
    fn a_dc_biased_square_is_still_square() {
        // rails 300/1300（间隔 1000 ≥ 256），再整体抬 2000 → 2300..3300。
        // ⚠ 不能用 614/3481 加 3000 —— 那会顶破 12-bit 量程。
        let mut v = pwm(10, 10, 30);
        for s in v.iter_mut() {
            *s = (*s - LO) / 3 + 300; // 614→300, 3481→1256
            *s += 2000;
        }
        let x = c(v, RATE);
        assert_eq!(x.kind, SignalKind::Square, "偏置方波不得被判成别的");
        assert!(x.evidence.rising >= 29, "实测中点应数到约 30 个上升沿");
    }

    /// 占空比边界的**两侧**各来一个。
    ///
    /// 变异（杀掉）：`SQUARE_DUTY_LO` 从 25 挪到 10（或 `HI` 从 75 挪到 90）
    /// —— 对应一侧红。断言写死字面量 24/25/75/76，不引用常量本身。
    #[test]
    fn the_square_pulse_boundary_has_live_inputs_on_both_sides() {
        let duty = |high: usize, low: usize| c(pwm(high, low, 30), RATE).kind;
        // 24% → Pulse；25% → Square（闭区间）
        assert_eq!(duty(24, 76), SignalKind::Pulse);
        assert_eq!(duty(25, 75), SignalKind::Square);
        // 75% → Square；76% → Pulse
        assert_eq!(duty(75, 25), SignalKind::Square);
        assert_eq!(duty(76, 24), SignalKind::Pulse);
    }

    /// 只有两个脉冲、周期测不出 → Pulse（占空比无意义的信号不该叫方波）。
    ///
    /// 变异（杀掉）：把 `duty_pct.is_none()` 的分支写死成 Square —— 这条红。
    #[test]
    fn a_pair_of_pulses_without_a_period_is_pulse_not_square() {
        let mut v = vec![LO; 100];
        v.resize(120, HI);
        v.resize(200, LO);
        v.resize(220, HI);
        v.resize(400, LO);
        let x = c(v, RATE);
        assert_eq!(x.kind, SignalKind::Pulse);
    }

    /// **周期 2 样点的交替信号必须判混叠**，而不是拿 cv=0 混进「干净方波」。
    ///
    /// 变异（杀掉）：删掉双电平分支里的 Aliased 判定 —— 这条红
    /// （会被判成 Square，因为间隔全 2、cv=0）。
    #[test]
    fn alternating_samples_are_aliased_not_square() {
        let x = c(alternating(LO, HI, 1, 100), RATE);
        assert_eq!(x.kind, SignalKind::Unknown(UncertainReason::Aliased));
    }

    /// **Aliased 必须排在稳定性之前。**
    ///
    /// 周期 2.5 样点的信号（上升沿间隔 2/3 交替）cv 高达 0.2 —— 若稳定性
    /// 检查在前，会先被判 IntervalUnstable，永远到不了 Aliased。
    ///
    /// 变异（杀掉）：把 Aliased 判定挪到 cv 之后 —— 这条红。
    #[test]
    fn aliased_wins_over_interval_instability() {
        // runs [1,1,1,2] 循环 → 上升沿间隔 2,3,2,3…（周期 2.5 样点）
        let mut runs = Vec::new();
        for _ in 0..100 {
            runs.extend_from_slice(&[1, 1, 1, 2]);
        }
        let x = c(data_line(&runs), RATE);
        assert_eq!(x.kind, SignalKind::Unknown(UncertainReason::Aliased));
    }

    /// 带事务空闲段的时钟**仍判 Square** —— 间隔稳定性必须用过滤后的间隔。
    ///
    /// I2C 的 SCL 就是这样：每帧之间有几十微秒的空闲。原始间隔的 cv 高达
    /// 0.5 量级，过滤后才 0.0 量级。
    ///
    /// 变异（杀掉）：`filter_gaps` 的过滤删掉（kept = 全部间隔）—— 这条红。
    #[test]
    fn a_clock_with_idle_gaps_is_still_square() {
        let x = c(bursty(4, 5, 40, 6), RATE);
        assert_eq!(
            x.kind,
            SignalKind::Square,
            "事务间空闲不该让时钟变成「间隔不固定」"
        );
        let g = x.evidence.gaps.expect("间隔统计");
        assert!(
            g.dropped >= 5,
            "空闲间隔应被过滤掉（dropped={}），而不是混进 cv",
            g.dropped
        );
        assert!(g.cv < 0.05, "过滤后 cv 应接近 0，实测 {}", g.cv);
    }

    /// 间隔按数据内容变化的两电平信号 → IntervalUnstable，不是 Square。
    ///
    /// I2C 的 SDA 就是它：电平是两值的，但间隔没有规律。
    ///
    /// 变异（杀掉）：双电平分支删掉稳定性检查 —— 这条红（会被判成 Square）。
    #[test]
    fn an_irregular_two_level_line_is_interval_unstable() {
        // 确定性「数据」：2..10 样点随机时长，模拟数据线上的位序列
        let mut runs = Vec::new();
        let mut x: u64 = 0x1234_5678_9ABC_DEF1;
        for _ in 0..200 {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
            runs.push(2 + (x % 9) as usize);
        }
        let x = c(data_line(&runs), RATE);
        assert_eq!(
            x.kind,
            SignalKind::Unknown(UncertainReason::IntervalUnstable),
            "数据线不该被叫方波"
        );
        assert!(x.evidence.levels.is_two_level(), "它确实是个双电平信号");
    }

    /// **脉冲串 + 单样点毛刺** 也会落 IntervalUnstable —— 这是已接受的边界。
    ///
    /// 毛刺把间隔序列劈出 100/100 的双倍间隔（cv ≈ 0.27），与数据线
    /// 在「间隔稳定性」上不可分。判据在证据里写得明明白白，读者看得懂。
    /// 模拟器的 `pulse_glitch` 场景就是它（评审实测 cv = 0.271）。
    #[test]
    fn a_pulse_train_with_glitches_is_interval_unstable_by_design() {
        let mut v = pwm(20, 180, 15);
        // 每 5 个周期的中点插一个单样点满量程尖峰
        for k in 0..3 {
            let idx = k * 5 * 200 + 100;
            v[idx] = 4095;
        }
        let x = c(v, RATE);
        assert_eq!(
            x.kind,
            SignalKind::Unknown(UncertainReason::IntervalUnstable),
            "毛刺脉冲串与数据线不可分 —— 落 IntervalUnstable 是刻意的"
        );
    }

    /// 窗内频率漂移 → FrequencyDrift。
    ///
    /// 前半周期 100 样点、后半 118 样点：drift = 0.165 > 0.15。
    ///
    /// 变异（杀掉）：`MAX_FREQ_DRIFT` 从 0.15 挪到 0.2 —— 这条红；
    /// 前半 100 后半 110（drift=0.095）判 Square 的那半钉住下侧。
    #[test]
    fn a_drifting_frequency_is_named_not_passed_off_as_stable() {
        let mut slow = Vec::new();
        for _ in 0..6 {
            slow.resize(slow.len() + 50, LO);
            slow.resize(slow.len() + 50, HI);
        }
        for _ in 0..6 {
            slow.resize(slow.len() + 59, LO);
            slow.resize(slow.len() + 59, HI);
        }
        let x = c(slow, RATE);
        assert_eq!(x.kind, SignalKind::Unknown(UncertainReason::FrequencyDrift));

        // 下侧：漂移 0.095 的仍判 Square
        let mut ok = Vec::new();
        for _ in 0..6 {
            ok.resize(ok.len() + 50, LO);
            ok.resize(ok.len() + 50, HI);
        }
        for _ in 0..6 {
            ok.resize(ok.len() + 55, LO);
            ok.resize(ok.len() + 55, HI);
        }
        assert_eq!(c(ok, RATE).kind, SignalKind::Square);
    }

    // ── 阶跃 ─────────────────────────────────────────────────────

    /// 只朝一个方向跃变 → Step，证据带沿数与过冲。
    #[test]
    fn a_single_transition_is_a_step() {
        let mut v = vec![LO; 300];
        v.resize(700, HI);
        v.resize(1000, HI);
        let x = c(v, RATE);
        assert_eq!(x.kind, SignalKind::Step);
        assert_eq!((x.evidence.rising, x.evidence.falling), (1, 0));
        assert_eq!(
            x.evidence.overshoot_pct,
            Some(0.0),
            "干净阶跃的过冲应如实报 0"
        );
    }

    /// 阶跃响应的证据里要带**过冲数值** —— 那是提示词引用它的唯一途径。
    ///
    /// 变异（杀掉）：`overshoot_pct` 从证据里删掉 —— 这条红。
    #[test]
    fn a_step_carries_its_overshoot_in_the_evidence() {
        let mut v = vec![LO; 300];
        v.resize(890, HI);
        v.push(4095); // 冲顶：比高平台 3481 高 614，占间隔 2867 的 21.4%
        v.resize(1000, HI);
        let x = c(v, RATE);
        assert_eq!(x.kind, SignalKind::Step);
        let over = x.evidence.overshoot_pct.expect("过冲证据");
        assert!(
            (over - 21.4).abs() < 1.5,
            "过冲实测 {over}%，应为 614/2867 ≈ 21.4%"
        );
    }

    // ── 正弦 / 调幅 / 噪声 ───────────────────────────────────────

    /// 1 kHz 正弦 → Sine；波形因数在证据里，且**调幅检查做得成、判得过**。
    ///
    /// 窗长 2 个周期 = 1714 样点，4096 点恰好 3 个完整窗 —— 起伏接近 0。
    /// 纯正弦必须在「做了检查」的前提下仍判 Sine（没做检查的 Sine 与
    /// 做了检查的 Sine，可信度不同）。
    ///
    /// 变异（杀掉）：`sliding_pp_var` 恒返回 None —— 这条的
    /// `seg_var.is_some()` 断言红。
    #[test]
    fn a_sine_is_sine_and_the_am_check_runs() {
        let s = sine(4096, 4.78, 2048.0, 1700.0); // 1 kHz @ 857142
        let x = c(s, 857_142);
        assert_eq!(x.kind, SignalKind::Sine);
        let ff = x.evidence.form_factor.expect("波形因数");
        assert!((ff - SINE_FORM_FACTOR).abs() < 0.01, "波形因数实测 {ff}");
        let sv = x.evidence.seg_var.expect("窗数足够，调幅检查应做过");
        assert!(sv < 0.05, "纯正弦的滑窗起伏应接近 0，实测 {sv}");
    }

    /// **削顶的正弦仍是正弦** —— 过驱动是示波器上最常见的波形。
    ///
    /// 削掉峰值 20%（clip 0.8A）后波形因数约 0.40，仍在 0.045 的容差内。
    ///
    /// 变异（杀掉）：`FORM_FACTOR_TOL` 改回 0.03 —— 这条红。
    #[test]
    fn a_clipped_sine_is_still_sine() {
        let clip = 0.8;
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / 857_142.0;
                let raw = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
                let v = 2048.0 + 1700.0 * raw.clamp(-clip, clip);
                v.round().clamp(0.0, 4095.0) as u16
            })
            .collect();
        let x = c(s, 857_142);
        assert_eq!(x.kind, SignalKind::Sine, "削顶正弦不该掉出正弦");
    }

    /// 三角波 → ShapeUnrecognized（波形因数 0.289，差 0.065）。
    ///
    /// 变异（杀掉）：`FORM_FACTOR_TOL` 放大到 0.07 —— 这条红（三角会变 Sine）。
    #[test]
    fn a_triangle_is_not_passed_off_as_a_sine() {
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let p = (i as f64 * 20.0 / 4096.0) % 1.0;
                let v = if p < 0.5 { p * 2.0 } else { 2.0 - p * 2.0 };
                (2048.0 + 1700.0 * (v * 2.0 - 1.0)).round() as u16
            })
            .collect();
        let x = c(s, 857_142);
        assert_eq!(
            x.kind,
            SignalKind::Unknown(UncertainReason::ShapeUnrecognized)
        );
    }

    /// **200 kHz 正弦必须判正弦** —— 周期 4.29 样点时纯量化就给出 cv=0.105。
    ///
    /// 固定 0.10 的 cv 门限会把这条判成噪声（评审实测）。量化感知门限
    /// `max(0.10, 0.5/中位间隔)` 把它收回来。
    ///
    /// 变异（杀掉）：cv 门限退化成固定 0.10 —— 这条红。
    #[test]
    fn a_200khz_sine_is_not_quantization_noise() {
        let cycles = 4096.0 * 200_000.0 / 857_142.0; // ≈ 955.7
        let s = sine(4096, cycles, 2048.0, 1700.0);
        let x = c(s, 857_142);
        assert_eq!(
            x.kind,
            SignalKind::Sine,
            "周期 4.29 样点的正弦被量化成 cv 0.105，仍必须是正弦"
        );
    }

    /// 20 kHz 正弦：调幅检查**做得成**，且起伏接近 0。
    ///
    /// 变异（杀掉）：`am_checked` 相关逻辑写死「不做」（seg_var 恒 None）
    /// —— 这条的 `seg_var.is_some()` 断言红。
    #[test]
    fn a_20khz_sine_runs_the_am_check_and_passes_it() {
        let cycles = 4096.0 * 20_000.0 / 857_142.0;
        let s = sine(4096, cycles, 2048.0, 1700.0);
        let x = c(s, 857_142);
        assert_eq!(x.kind, SignalKind::Sine);
        let sv = x.evidence.seg_var.expect("窗数足够，调幅检查应做过");
        assert!(sv < 0.05, "纯正弦的滑窗起伏应接近 0，实测 {sv}");
    }

    /// **Am 必须排在 Sine 前** —— 中段幅度被压掉一截的正弦，波形因数仍在
    /// 正弦容差内（0.342），只有滑窗起伏抓得住它。
    ///
    /// 变异（杀掉）：把 Am 检查挪到波形因数之后 —— 这条红（会被判 Sine）。
    #[test]
    fn a_dropout_is_am_not_sine() {
        let cycles = 4096.0 * 20_000.0 / 857_142.0;
        let mut s = sine(4096, cycles, 2048.0, 1700.0);
        // 第 2 个 1/8 段幅度压到 0.7 —— 一个包络凹陷
        for v in &mut s[512..1024] {
            *v = (2048.0 + (*v as f64 - 2048.0) * 0.7).round() as u16;
        }
        let x = c(s, 857_142);
        assert_eq!(x.kind, SignalKind::Am, "中段凹陷必须判 Am 而不是 Sine");
        let sv = x.evidence.seg_var.expect("起伏证据");
        assert!(sv > SLIDING_VAR_THRESHOLD, "滑窗起伏实测 {sv}");
    }

    /// 真调幅（20 kHz 载波 + 200 Hz 包络）→ Am。
    #[test]
    fn a_true_am_signal_is_am() {
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / 857_142.0;
                let carrier = (2.0 * std::f64::consts::PI * 20_000.0 * t).sin();
                let envelope = 0.5 * (1.0 + (2.0 * std::f64::consts::PI * 200.0 * t).sin());
                (2048.0 + 1500.0 * envelope * carrier).round() as u16
            })
            .collect();
        let x = c(s, 857_142);
        assert_eq!(x.kind, SignalKind::Am);
    }

    /// **1 kHz 载波的调幅判得出来** —— 滑窗方案（窗长 2 周期、步长 1 周期）
    /// 在 4096 点里凑得出 3 个完整窗，检查做得成。
    ///
    /// 这是对旧「固定 8 段」方案的回归：那时 1 kHz 载波每段只有 0.6 个
    /// 周期，守卫永远触发，1 kHz 的 AM **永远判不出 Am**（评审实测）。
    ///
    /// 变异（杀掉）：`sliding_pp_var` 的窗长写成 8 个周期 —— 窗数掉到
    /// 0，`seg_var` 变 None，这条的 kind 断言红。
    #[test]
    fn a_1khz_carrier_am_is_found_by_the_sliding_window() {
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / 857_142.0;
                let carrier = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
                let envelope = 0.5 * (1.0 + (2.0 * std::f64::consts::PI * 200.0 * t).sin());
                (2048.0 + 1500.0 * envelope * carrier).round() as u16
            })
            .collect();
        let x = c(s, 857_142);
        assert_eq!(x.kind, SignalKind::Am, "1 kHz 载波 AM 应由滑窗检查判出");
        let sv = x.evidence.seg_var.expect("检查应做过");
        assert!(sv > SLIDING_VAR_THRESHOLD, "起伏实测 {sv}");
    }

    /// **低频载波的 AM 判不出也不硬猜** —— 300 Hz 载波在 4096 点里只有
    /// 1.4 个周期，连周期性都不成立，落 NoPeriodicity 并如实标注
    /// 调幅检查没做。
    ///
    /// 变异（杀掉）：把周期性门（rising ≥ 4）删掉 —— 这条红
    /// （会带着 2 个沿去做 Am / 形状判定）。
    #[test]
    fn a_300hz_carrier_am_is_not_guessed() {
        let s: Vec<u16> = (0..4096)
            .map(|i| {
                let t = i as f64 / 857_142.0;
                let carrier = (2.0 * std::f64::consts::PI * 300.0 * t).sin();
                let envelope = 0.5 * (1.0 + (2.0 * std::f64::consts::PI * 50.0 * t).sin());
                (2048.0 + 1500.0 * envelope * carrier).round() as u16
            })
            .collect();
        let x = c(s, 857_142);
        assert_ne!(x.kind, SignalKind::Am, "周期都不成立，不得判调幅");
        assert!(x.evidence.seg_var.is_none(), "证据要标注调幅检查未做");
    }

    /// 确定性噪声 → Noise。
    #[test]
    fn noise_is_noise() {
        let s = pseudo_noise(4096, 2000.0);
        let x = c(s, RATE);
        assert_eq!(x.kind, SignalKind::Noise);
    }

    /// **噪声判定的沿数门限两侧**：6 个沿 → NoPeriodicity（证据太少），
    /// 9 个沿 → Noise。
    ///
    /// 阶梯游走刻意用 3 个电平 —— 双电平信号会先被两电平分支截走，
    /// 永远到不了噪声判定（那是「测试测到别人」）。中档取 2100 恰好等于
    /// 实测中点，从它出发的切换不算穿越，所以每轮要**数实际的上升沿数**
    /// （与分类器的 Noise 判据同一口径），数够了再停。
    ///
    /// 变异（杀掉）：`MIN_NOISE_EDGES` 从 8 挪到 4 或 12 —— 对应一侧红。
    #[test]
    fn the_noise_threshold_has_live_inputs_on_both_sides() {
        let walk = |target_rising: usize| -> Vec<u16> {
            let levels = [700u16, 2100, 3500];
            let mut v = Vec::new();
            let mut x: u64 = 0xDEAD_BEEF;
            let mut lvl = 2usize;
            loop {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
                let run = 40 + (x % 80) as usize;
                v.resize(v.len() + run, levels[lvl]);
                let next = ((x >> 8) % 3) as usize;
                if next != lvl {
                    lvl = next;
                    let mid = ((*v.iter().min().unwrap() + *v.iter().max().unwrap()) / 2) as f64;
                    if crate::measure::rising_edges(&v, mid).len() >= target_rising {
                        return v;
                    }
                }
            }
        };

        let few = c(walk(6), RATE);
        assert_eq!(
            few.kind,
            SignalKind::Unknown(UncertainReason::NoPeriodicity),
            "6 个沿不足以判噪声"
        );
        let many = c(walk(9), RATE);
        assert_eq!(many.kind, SignalKind::Noise, "9 个沿应判噪声");
    }

    // ── 一致性 oracle 与确定性 ───────────────────────────────────

    /// **kind 与证据必须自洽** —— 对构造信号断言蕴含关系。
    ///
    /// 这些蕴含是「手写期望」抓不到的性质：任何一条实现写歪了，
    /// 数据对不上就会红。
    #[test]
    fn the_kind_and_its_evidence_agree() {
        let signals: Vec<Vec<u16>> = vec![
            vec![2048u16; 500], // Dc
            pwm(10, 10, 30),    // Square
            pwm(5, 95, 10),     // Pulse
            {
                let mut v = vec![LO; 300];
                v.resize(700, HI);
                v
            }, // Step
            sine(4096, 4.78, 2048.0, 1700.0), // Sine
            pseudo_noise(4096, 2000.0), // Noise
            {
                let mut v = pwm(20, 180, 15);
                v[500] = 4095;
                v
            }, // 毛刺脉冲串 → Unknown
        ];
        for s in signals {
            let x = c(s, RATE);
            match x.kind {
                SignalKind::Step => assert!(
                    x.evidence.rising + x.evidence.falling <= 2,
                    "Step 却数出 {} 个沿",
                    x.evidence.rising + x.evidence.falling
                ),
                SignalKind::Square | SignalKind::Pulse => assert!(
                    x.evidence.levels.is_two_level(),
                    "{} 却不在双电平上",
                    match x.kind {
                        SignalKind::Square => "Square",
                        _ => "Pulse",
                    }
                ),
                SignalKind::Sine => {
                    let ff = x.evidence.form_factor.expect("波形因数");
                    assert!(
                        (ff - SINE_FORM_FACTOR).abs() <= FORM_FACTOR_TOL,
                        "Sine 但波形因数 {ff} 不在容差内"
                    );
                }
                SignalKind::Dc => assert!(
                    x.evidence.ac_rms_lsb <= DC_AC_RMS_LSB,
                    "Dc 但 AC-RMS 超出直流门限"
                ),
                _ => {}
            }
            if x.evidence.overshoot_pct.is_some() {
                assert!(x.evidence.levels.is_two_level(), "过冲必须来自双电平");
            }
        }
    }

    /// 分类必须**确定** —— 同一输入两次结果逐字节一致。
    #[test]
    fn classification_is_deterministic() {
        let s = sine(4096, 4.78, 2048.0, 1700.0);
        let a = format!("{:?}", c(s.clone(), 857_142));
        let b = format!("{:?}", c(s, 857_142));
        assert_eq!(a, b, "同一输入两次分类必须一致");
    }

    /// 通道越界 / 空采集 → None。
    ///
    /// 变异（杀掉）：把「恒返回 None」与「恒返回 Some(Dc)」区分开 ——
    /// 下面的正向用例已经保证了 Some 路径，这条只钉 None 语义。
    #[test]
    fn a_missing_channel_is_none() {
        let cap = cap_of(vec![2048u16; 100], RATE);
        assert!(classify(&cap, 1).is_none(), "通道 1 不存在");
        assert!(classify(&cap, 0).is_some());
    }
}
