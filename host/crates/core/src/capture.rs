//! 采集存储 —— MCP 三层 token 防护的执行者。
//!
//! **核心原则：全量波形永远不进 LLM 上下文。**
//!
//! 一次 4096 点采集 = 8192 字节 ≈ 4096 个 JSON 数字 ≈ 上万 token。
//! 把它塞给 LLM 既贵又没用 —— Agent 要的是「这里面发生了什么」，
//! 不是「第 2713 个样点是 2047」。
//!
//! 所以：
//!
//! | 出口 | 给谁 | 形态 |
//! |---|---|---|
//! | [`Capture::summary`] | Agent 默认 | 统计量，几十字节 |
//! | [`Capture::preview`] | Agent 默认 | ≤256 点 minmax 降采样 |
//! | [`Capture::samples`] | 测量算法 | 全速原始样点 |
//! | [`Capture::to_csv`] | 用户 | 落盘，不进上下文 |

use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

/// 采集存储保留的历史条数。
///
/// Agent 可以「引用上一次采集」而不必重抓，但不会无限增长。
pub const DEFAULT_HISTORY: usize = 16;

/// 采集数据的来源。
///
/// 与 [`crate::analysis::Source`] 的分工：后者是**渲染时**的出处描述
/// （带链路名，由调用方给出），本类型是**采集对象自身**携带的来源，
/// 从 CSV 导入的采集无法再从调用方的链路参数反推，必须由数据本身记住。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CaptureSource {
    /// 真机抓的。
    Device,
    /// 模拟器生成的。
    Simulator,
    /// 从 CSV 文件导入的。
    File {
        /// 导入时的文件路径（原样保留，不展开、不规范化）。
        path: String,
    },
}

/// 一次采集。
#[derive(Debug, Clone)]
pub struct Capture {
    /// 采集编号（设备分配，导入的采集另占一段编号）。
    pub id: u16,
    /// 这份数据从哪来。
    pub source: CaptureSource,
    /// 实际采样率。**时间轴以它为准**，不是请求值。
    pub rate_hz: u32,
    /// 每个通道的原始样点（12-bit，0..4095）。
    pub channels: Vec<Vec<u16>>,
    /// 触发点在窗内的相对样点号；`None` 表示软件触发或无触发。
    pub trigger_index: Option<u32>,
    /// 抓取时设备时钟（µs）。
    pub device_tick_us: u32,
    /// 抓取时的墙钟时间（Unix 秒），用于 `list_captures` 排序。
    pub wall_time: u64,
    /// 期间是否发生溢出 —— 有这个标志的数据**不能假装完整**。
    pub overrun: bool,
    /// 期望的样点数（用于识别缺口）。
    pub expected_samples: u32,
}

impl Capture {
    /// 创建一次空采集。
    ///
    /// 来源默认 [`CaptureSource::Device`]；模拟器与导入端用
    /// [`Self::with_source`] 覆写。
    pub fn new(id: u16, rate_hz: u32, ch_count: usize, expected_samples: u32) -> Capture {
        Capture {
            id,
            source: CaptureSource::Device,
            rate_hz,
            channels: vec![Vec::with_capacity(expected_samples as usize); ch_count],
            trigger_index: None,
            device_tick_us: 0,
            wall_time: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0),
            overrun: false,
            expected_samples,
        }
    }

    /// 覆写数据来源（模拟器 / 导入）。
    pub fn with_source(mut self, source: CaptureSource) -> Capture {
        self.source = source;
        self
    }

    /// 样点数（以第一个通道为准）。
    pub fn len(&self) -> usize {
        self.channels.first().map(|c| c.len()).unwrap_or(0)
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 时间跨度（µs）。
    pub fn duration_us(&self) -> u64 {
        if self.rate_hz == 0 {
            return 0;
        }
        (self.len() as u64) * 1_000_000 / self.rate_hz as u64
    }

    /// 每个样点的间隔（µs）。
    pub fn dt_us(&self) -> f64 {
        if self.rate_hz == 0 {
            0.0
        } else {
            1_000_000.0 / self.rate_hz as f64
        }
    }

    /// 某通道的原始样点。
    pub fn samples(&self, ch: usize) -> Option<&[u16]> {
        self.channels.get(ch).map(|v| v.as_slice())
    }

    /// 统计量摘要 —— Agent 的默认出口。
    pub fn summary(&self, ch: usize) -> Option<ChannelSummary> {
        let s = self.samples(ch)?;
        if s.is_empty() {
            return None;
        }

        let mut min = u16::MAX;
        let mut max = u16::MIN;
        let mut sum: u64 = 0;
        let mut sum_sq: u64 = 0; // 必须 u64：4095² × 4096 ≈ 2³⁶
        let mut rising = 0u32;
        let mut falling = 0u32;

        let mid = 2048u16; // 12-bit 中点（未标定时只作过零参考）
        let mut prev = s[0];

        for &v in s {
            if v < min {
                min = v;
            }
            if v > max {
                max = v;
            }
            sum += v as u64;
            sum_sq += (v as u64) * (v as u64);

            if prev < mid && v >= mid {
                rising += 1;
            } else if prev >= mid && v < mid {
                falling += 1;
            }
            prev = v;
        }

        let n = s.len() as u64;
        let mean = sum as f64 / n as f64;
        let mean_sq = sum_sq as f64 / n as f64;

        // 两个量名字相近但物理意义完全不同，**必须分开算、分开命名**：
        //
        //   rms_lsb     = sqrt(E[x²])          相对 ADC 零点 0 的**真 RMS**
        //   ac_rms_lsb  = sqrt(E[x²] − E[x]²)  扣除直流分量后的 **AC RMS**（标准差）
        //
        // 回归：这里曾经用 sqrt(variance) 填进名叫 rms_lsb 的字段 ——
        // 算的是标准差却叫 RMS，全 4095 的直流信号会报出 0。
        // 当时的测试还把错误行为写成了期望值，属于测试把 bug 固化下来。
        let rms = mean_sq.max(0.0).sqrt();
        let ac_rms = (mean_sq - mean * mean).max(0.0).sqrt();

        Some(ChannelSummary {
            channel: ch as u8,
            n_samples: s.len() as u32,
            min_lsb: min,
            max_lsb: max,
            pp_lsb: max - min,
            mean_lsb: mean,
            rms_lsb: rms,
            ac_rms_lsb: ac_rms,
            rising_edges: rising,
            falling_edges: falling,
        })
    }

    /// minmax 降采样预览 —— 显示与 Agent 视觉化用的标准答案。
    ///
    /// 每个桶返回 `(min, max)`。桶大小 = `len / target_points`（至少 1）。
    /// 这样任何尖峰都不会被平均掉 —— 这正是为什么示波器降采样用 minmax
    /// 而不是简单抽取。
    pub fn preview(&self, ch: usize, target_points: usize) -> Option<MinMaxPreview> {
        let s = self.samples(ch)?;
        if s.is_empty() || target_points == 0 {
            return None;
        }

        let bucket = (s.len() / target_points).max(1);
        let mut mins = Vec::with_capacity(s.len() / bucket + 1);
        let mut maxs = Vec::with_capacity(s.len() / bucket + 1);

        for chunk in s.chunks(bucket) {
            let mut lo = u16::MAX;
            let mut hi = u16::MIN;
            for &v in chunk {
                if v < lo {
                    lo = v;
                }
                if v > hi {
                    hi = v;
                }
            }
            mins.push(lo);
            maxs.push(hi);
        }

        Some(MinMaxPreview {
            channel: ch as u8,
            bucket,
            dt_us: self.dt_us() * bucket as f64,
            t0_us: 0.0,
            y_min: mins,
            y_max: maxs,
        })
    }

    /// 导出 CSV —— **全量数据的唯一出口，且只落盘不进上下文**。
    ///
    /// 时间列用绝对秒，与通用两通道 CSV 格式（`Time(s),CH1V,CH2V`）兼容。
    ///
    /// # 为什么第一行是 `# rate_hz=<n>`
    ///
    /// 时间列只有 9 位小数，**速率无法从它反推**：857142 Hz 的
    /// `dt = 1.166666667 µs` 写成 `1.167 µs`，反推得 856898 Hz。
    /// 于是「导出再导入，采样率不变」这条 round-trip 若靠时间列，
    /// 不可能成立。速率是导出端自己知道的，就该写在文件里。
    ///
    /// `#` 开头的行是注释，读取端（`import`）忽略之，第三方工具亦然。
    pub fn to_csv(&self, scale: &[ChannelScale]) -> String {
        let mut out = String::with_capacity(self.len() * 16 * self.channels.len().max(1));
        let dt = self.dt_us() * 1e-6;

        out.push_str(&format!("# rate_hz={}\n", self.rate_hz));
        out.push_str("Time(s)");
        for ch in 0..self.channels.len() {
            out.push_str(&format!(",CH{}V", ch + 1));
        }
        out.push('\n');

        for i in 0..self.len() {
            out.push_str(&format!("{:.9}", i as f64 * dt));
            for (ch, chan) in self.channels.iter().enumerate() {
                let lsb = chan.get(i).copied().unwrap_or(0);
                let volts = scale
                    .get(ch)
                    .map(|s| s.lsb_to_volts(lsb))
                    .unwrap_or(lsb as f64);
                out.push_str(&format!(",{:.6}", volts));
            }
            out.push('\n');
        }
        out
    }
}

/// 单通道统计量。
#[derive(Debug, Clone, serde::Serialize)]
pub struct ChannelSummary {
    /// 通道号（0 起）。
    pub channel: u8,
    /// 参与统计的样点数。
    pub n_samples: u32,
    /// 最小值（ADC LSB）。
    pub min_lsb: u16,
    /// 最大值（ADC LSB）。
    pub max_lsb: u16,
    /// 峰峰值（ADC LSB）。
    pub pp_lsb: u16,
    /// 均值（ADC LSB）。
    pub mean_lsb: f64,
    /// **真 RMS**（ADC LSB）：`sqrt(E[x²])`，相对 ADC 零点 0。
    ///
    /// 直流信号（例如全 4095）的 `rms_lsb` 等于它的电平本身。
    pub rms_lsb: f64,
    /// **AC RMS**（ADC LSB）：`sqrt(E[x²] − E[x]²)`，即标准差，已扣除直流分量。
    ///
    /// 衡量"信号波动多大"，与直流偏置无关。示波器上说的 "AC RMS" 就是它。
    pub ac_rms_lsb: f64,
    /// 上升沿数（以 2048 LSB 为参考）。
    pub rising_edges: u32,
    /// 下降沿数。
    pub falling_edges: u32,
}

/// minmax 降采样预览。
#[derive(Debug, Clone, serde::Serialize)]
pub struct MinMaxPreview {
    /// 通道号。
    pub channel: u8,
    /// 每个桶包含的原始样点数。
    pub bucket: usize,
    /// 每个桶代表的时间跨度（µs）。
    pub dt_us: f64,
    /// 预览起始时间（µs）。
    pub t0_us: f64,
    /// 每桶最小值。
    pub y_min: Vec<u16>,
    /// 每桶最大值。
    pub y_max: Vec<u16>,
}

/// 通道标定 —— 伏特换算**只在上位机做**，MCU 只认 LSB。
///
/// `Serialize` / `Deserialize` 是给 [`crate::calib`] 的标定表用的。
/// **字段名是文件格式的一部分** —— 改了就是格式变更，要配 `CALIB_VERSION` 一起动。
///
/// 反序列化**刻意不设 `#[serde(default)]`**：字段缺失应当让整个文件解析失败
/// 并被记录，而不是静默退回占位值 —— 那等于悄悄用一个错的换算。
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct ChannelScale {
    /// 每 LSB 对应的伏特数。
    pub volts_per_lsb: f64,
    /// 零电平对应的 ADC LSB。
    pub zero_lsb: f64,
}

impl Default for ChannelScale {
    fn default() -> Self {
        // 未标定的粗估：3.3 V / 4096 LSB，中点 2048
        ChannelScale {
            volts_per_lsb: 3.3 / 4096.0,
            zero_lsb: 2048.0,
        }
    }
}

impl ChannelScale {
    /// LSB → 伏特。
    pub fn lsb_to_volts(&self, lsb: u16) -> f64 {
        (lsb as f64 - self.zero_lsb) * self.volts_per_lsb
    }

    /// 伏特 → LSB。
    pub fn volts_to_lsb(&self, v: f64) -> i32 {
        (v / self.volts_per_lsb + self.zero_lsb).round() as i32
    }
}

/// 采集存储：保留最近 N 次采集，供 Agent 引用而不必重抓。
#[derive(Debug)]
pub struct CaptureStore {
    entries: VecDeque<Capture>,
    capacity: usize,
}

impl Default for CaptureStore {
    fn default() -> Self {
        Self::with_capacity(DEFAULT_HISTORY)
    }
}

impl CaptureStore {
    /// 创建指定容量的存储。
    pub fn with_capacity(capacity: usize) -> Self {
        CaptureStore {
            entries: VecDeque::with_capacity(capacity),
            capacity: capacity.max(1),
        }
    }

    /// 存入一次采集；超出容量时淘汰最旧的。
    ///
    /// # 同编号的旧条目会被替换掉（不是并列留存）
    ///
    /// 因为 [`Self::get`] 是按编号找**第一个**匹配 —— 一旦 store 里出现两个
    /// 同编号的采集，`get` 就永远返回旧的那个，而调用方没有任何办法表达
    /// 「我要的是新的那份」。
    ///
    /// 这个不变量是必须的：**设备每次（重）连之后编号会从 1 重新开始**
    /// （模拟器 `SimDevice` 即如此），所以
    ///
    /// ```text
    /// 连接 → 采集(#1) → 断开 → 连接 → 采集(#1) → ...
    /// ```
    ///
    /// 是常规操作，不是边角情形。没有这条，历史面板会出现两行都写 `#1`、
    /// 两行同时高亮（它只比编号），而点第二行显示的是**第一行的数据**。
    ///
    /// 替换而不是清空整个 store：历史是给人回看的，重连一次就全没了太糙；
    /// 而按编号替换之后，`get` 的语义重新变得明确。
    pub fn push(&mut self, c: Capture) {
        // 先摘掉同编号的旧条目（如果有）
        if let Some(pos) = self.entries.iter().position(|old| old.id == c.id) {
            self.entries.remove(pos);
        }
        if self.entries.len() >= self.capacity {
            self.entries.pop_front();
        }
        self.entries.push_back(c);
    }

    /// 按编号查找。
    pub fn get(&self, id: u16) -> Option<&Capture> {
        self.entries.iter().find(|c| c.id == id)
    }

    /// 最近一次采集。
    pub fn latest(&self) -> Option<&Capture> {
        self.entries.back()
    }

    /// 全部采集（旧 → 新）。
    pub fn iter(&self) -> impl Iterator<Item = &Capture> {
        self.entries.iter()
    }

    /// 条数。
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mk_capture() -> Capture {
        let mut c = Capture::new(1, 1_000_000, 1, 8);
        c.channels[0] = vec![0, 1000, 2000, 3000, 4095, 3000, 2000, 1000];
        c
    }

    #[test]
    fn summary_computes_pp_and_edges() {
        let c = mk_capture();
        let s = c.summary(0).unwrap();
        assert_eq!(s.min_lsb, 0);
        assert_eq!(s.max_lsb, 4095);
        assert_eq!(s.pp_lsb, 4095);
        // 2048 为参考：0→1000→2000→3000 上穿一次；3000→2000→1000 下穿一次
        assert_eq!(s.rising_edges, 1);
        assert_eq!(s.falling_edges, 1);
    }

    #[test]
    fn rms_and_ac_rms_are_different_quantities() {
        // 回归：曾经把标准差（sqrt(variance)）填进名叫 rms_lsb 的字段，
        // 于是全 4095 的直流信号报出 rms = 0 —— 名字与物理量不符。
        // 4095² × 4096 ≈ 2³⁶ 也顺带验证 u64 累加不会溢出。
        let mut c = Capture::new(1, 1_000_000, 1, 4096);
        c.channels[0] = vec![4095u16; 4096];
        let s = c.summary(0).unwrap();

        assert!((s.mean_lsb - 4095.0).abs() < 1e-6);
        assert!(
            (s.rms_lsb - 4095.0).abs() < 1e-6,
            "直流 4095 的真 RMS 就是 4095，得到 {}",
            s.rms_lsb
        );
        assert!(
            s.ac_rms_lsb.abs() < 1e-6,
            "直流信号的 AC RMS（波动）应为 0，得到 {}",
            s.ac_rms_lsb
        );
    }

    #[test]
    fn ac_rms_matches_known_square_wave() {
        // 0/4095 各占一半的方波：
        //   均值    = 2047.5
        //   真 RMS  = sqrt((0² + 4095²)/2) = 4095/√2 ≈ 2895.6
        //   AC RMS  = 扣掉均值后是 ±2047.5 的方波 → 2047.5
        //
        // 注意两者**不相等**：全 4095 的直流信号真 RMS 是 4095、AC RMS 是 0；
        // 这个方波则分别是 2895.6 和 2047.5。名字相近，物理量不同。
        let mut c = Capture::new(1, 1_000_000, 1, 4);
        c.channels[0] = vec![0, 4095, 0, 4095];
        let s = c.summary(0).unwrap();

        let true_rms = 4095.0 / 2.0f64.sqrt(); // 2895.60
        let ac_rms = 2047.5;

        assert!(
            (s.mean_lsb - 2047.5).abs() < 1e-6,
            "均值应为 2047.5，得到 {}",
            s.mean_lsb
        );
        assert!(
            (s.rms_lsb - true_rms).abs() < 1.0,
            "真 RMS 应约 {true_rms:.1}，得到 {:.1}",
            s.rms_lsb
        );
        assert!(
            (s.ac_rms_lsb - ac_rms).abs() < 1.0,
            "AC RMS 应约 {ac_rms:.1}，得到 {:.1}",
            s.ac_rms_lsb
        );

        // Parseval：真 RMS² = 均值² + AC RMS²
        // 这是两者关系的硬约束，比上面的具体数值更能防住"把标准差当 RMS"这类错误
        let lhs = s.rms_lsb * s.rms_lsb;
        let rhs = s.mean_lsb * s.mean_lsb + s.ac_rms_lsb * s.ac_rms_lsb;
        assert!(
            (lhs - rhs).abs() < 1.0,
            "Parseval 恒等式不成立：RMS²={lhs:.1} 应等于 mean²+ac²={rhs:.1}"
        );
    }

    #[test]
    fn preview_preserves_spikes() {
        // 桶大小 4，中间有一个尖峰；minmax 必须保住它，简单抽取会丢
        let mut c = Capture::new(1, 1_000_000, 1, 8);
        c.channels[0] = vec![0, 0, 0, 4095, 0, 0, 0, 0];
        let p = c.preview(0, 2).unwrap();
        assert_eq!(p.bucket, 4);
        assert_eq!(p.y_max[0], 4095, "尖峰必须出现在 max 里");
        assert_eq!(p.y_min[0], 0);
    }

    #[test]
    fn store_evicts_oldest() {
        let mut st = CaptureStore::with_capacity(2);
        for id in 1..=3u16 {
            st.push(Capture::new(id, 1000, 1, 1));
        }
        assert_eq!(st.len(), 2);
        assert!(st.get(1).is_none(), "最旧的应被淘汰");
        assert!(st.get(3).is_some());
    }

    /// **同编号必须唯一 —— 否则 `get(id)` 永远拿到旧的那份。**
    ///
    /// 回归：`push` 从前是无条件 `push_back`，于是
    ///
    /// ```text
    /// 连接 → 采集(#1) → 断开 → 连接 → 采集(#1)
    /// ```
    ///
    /// 之后 store 里有两份 `#1`，而 `get(1)` 用 `.iter().find()` 返回**第一个**
    /// —— 界面上表现为：历史面板两行都写 `#1`、两行同时高亮（它只比编号），
    /// **点第二行显示的是第一行的数据**。
    ///
    /// 设备每次重连编号都会从 1 重新开始（模拟器 `SimDevice` 即如此），
    /// 所以这是常规操作能走到的，不是边角情形。
    #[test]
    fn store_replaces_a_capture_with_the_same_id() {
        let mut st = CaptureStore::default();
        let mut old = Capture::new(1, 1000, 1, 8);
        old.overrun = true; // 给旧的那份留个记号
        let new = Capture::new(1, 2000, 1, 8);

        st.push(old);
        st.push(new);

        assert_eq!(
            st.len(),
            1,
            "同编号不该并列留存 —— get() 会永远拿到旧的那份"
        );
        let got = st.get(1).expect("应当还找得到");
        assert_eq!(got.rate_hz, 2000, "拿到的应当是新的那份");
        assert!(!got.overrun, "不该是旧的那份");
    }

    /// 替换**不该把不相关的采集挤掉** —— 它不占新槽位。
    ///
    /// ⚠ 这条测试第一版是假的：我选的场景（替换 1，然后断言 1 和 3 还在）
    /// **在有无去重时结果相同** —— 淘汰掉的都是 1，断言照样过。
    /// 是变异测试把它抓出来的（去掉去重只有另一条红，这条纹丝不动）。
    ///
    /// 换成能区分的场景：**替换的是 2，检查的是 1**。
    ///
    /// ```text
    /// 有去重： [1, 2] → 摘掉旧的 2 → [1] → push 2' → [1, 2']   ← 1 保住
    /// 无去重： [1, 2] → 计数已满 → 淘汰队首 1 → [2] → push 2' → [2, 2'] ← 1 丢了
    /// ```
    #[test]
    fn replacing_a_capture_does_not_evict_an_unrelated_one() {
        let mut st = CaptureStore::with_capacity(2);
        st.push(Capture::new(1, 1000, 1, 1));
        st.push(Capture::new(2, 1000, 1, 1));
        st.push(Capture::new(2, 2000, 1, 1)); // 替换 2

        assert!(
            st.get(1).is_some(),
            "替换 2 不该把 1 挤掉 —— 替换不占新槽位"
        );
        assert_eq!(st.get(1).unwrap().rate_hz, 1000, "1 应当原封不动");
        assert_eq!(st.get(2).unwrap().rate_hz, 2000, "2 应当是新份");
        assert_eq!(st.len(), 2);
    }

    /// 容量上限仍要守 —— 替换是「摘掉再放」，不是「无条件多放一个」。
    #[test]
    fn store_still_respects_capacity_after_a_replacement() {
        let mut st = CaptureStore::with_capacity(2);
        st.push(Capture::new(1, 1000, 1, 1));
        st.push(Capture::new(2, 1000, 1, 1));
        st.push(Capture::new(2, 2000, 1, 1)); // 替换
        st.push(Capture::new(3, 1000, 1, 1)); // 这次才该淘汰

        assert_eq!(st.len(), 2, "不能超过容量");
        assert!(st.get(3).is_some(), "新采的应当在");
        assert_eq!(st.get(2).unwrap().rate_hz, 2000, "2 是替换过的新份");
        assert!(st.get(1).is_none(), "最旧的 1 这时才该被淘汰");
    }

    #[test]
    fn csv_has_header_and_scaling() {
        let c = mk_capture();
        let csv = c.to_csv(&[ChannelScale::default()]);
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines[0], "# rate_hz=1000000");
        assert_eq!(lines[1], "Time(s),CH1V");
        assert_eq!(lines.len(), 10); // 注释 + 表头 + 8 样点
    }

    /// 速率必须**逐位精确**写在文件里 —— 时间列只有 9 位小数，反推会差。
    ///
    /// 857142 Hz 的 `dt = 1.166666667 µs` 会被写成 `1.167 µs`，
    /// 由时间列反推得 856898 Hz，与真值差 244 Hz。
    #[test]
    fn csv_rate_header_is_exact_for_non_representable_rates() {
        let mut c = Capture::new(1, 857_142, 1, 2);
        c.channels[0] = vec![0, 4095];
        let csv = c.to_csv(&[ChannelScale::default()]);
        assert_eq!(csv.lines().next().unwrap(), "# rate_hz=857142");
    }

    #[test]
    fn scale_roundtrip() {
        let s = ChannelScale {
            volts_per_lsb: 1.0,
            zero_lsb: 2048.0,
        };
        assert!((s.lsb_to_volts(2048) - 0.0).abs() < 1e-9);
        assert!((s.lsb_to_volts(2148) - 100.0).abs() < 1e-9);
        assert_eq!(s.volts_to_lsb(100.0), 2148);
    }
}
