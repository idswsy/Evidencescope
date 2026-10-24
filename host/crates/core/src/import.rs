//! 第三方波形导入 —— 把外部 CSV 变成 [`Capture`]。
//!
//! # 为什么导入端不做任何「聪明」的事
//!
//! 示波器导出的列名五花八门，量程与偏置通常不在文件里。凡是靠猜的地方，
//! 猜错的结果都是**静默的错误数据** —— 波形看着正常，时间轴或幅值却整体
//! 偏了一截，而下游（测量、解码、AI 分析）没有能力发现它。
//!
//! 所以这里只支持**本项目自己的契约**加显式参数：
//!
//! ```text
//! # rate_hz=857142
//! Time(s),CH1V,CH2V
//! 0.000000000,0.000000,3.300000
//! ```
//!
//! 缺信息即报错，不做隐式推断。
//!
//! # 速率为什么必须写在文件里
//!
//! 时间列只有 9 位小数。857142 Hz 的 `dt = 1.166666667 µs` 会被写成
//! `1.167 µs`，由它反推得 856898 Hz —— 差 244 Hz，而且**看起来完全合理**。
//! 因此 [`Capture::to_csv`] 在首行写入 `# rate_hz=<n>`，导入端优先读它。
//!
//! 时间列反推只在「该间隔能被 9 位小数精确表示」时才允许，否则
//! [`ImportError::RateMissing`]，绝不四舍五入成一个整速率。

use std::fmt;

use crate::capture::{Capture, CaptureSource, ChannelScale};

/// 导入采集的编号下界（设备分配的编号不会进入这一段）。
pub const IMPORT_ID_BASE: u16 = 40_000;
/// 导入采集的编号上界（不含）。
pub const IMPORT_ID_LIMIT: u16 = 60_000;
/// 一次导入最多几个通道。
pub const MAX_IMPORT_CHANNELS: usize = 4;
/// 12-bit ADC 的满量程电压（未标定时的默认值）。
pub const DEFAULT_FULL_SCALE_V: f64 = 3.3;
/// 12-bit ADC 的码值上限。
const MAX_CODE: f64 = 4095.0;

/// 采样率是从哪来的。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RateSource {
    /// `# rate_hz=` 注释。
    Header,
    /// 调用方显式给出的参数。
    Option,
    /// 由时间列的中位差分反推。
    TimeColumn,
}

/// 导入选项。
#[derive(Debug, Clone)]
pub struct ImportOptions {
    /// 分配给这次采集的编号（应落在 [`IMPORT_ID_BASE`], [`IMPORT_ID_LIMIT`））。
    pub id: u16,
    /// 源文件路径，写进 [`CaptureSource::File`]。
    pub path: String,
    /// 显式给出的采样率（`--rate-hz`）。
    pub rate_hz: Option<u64>,
    /// 满量程电压，默认 3.3 V。
    pub full_scale_v: f64,
    /// 码值 0 对应的输入电压（伏特），默认 0。
    ///
    /// 本项目导出的文件是双极性的（0 V 在码值 2048），用
    /// [`ImportOptions::from_scale`] 时会自动折算成 −1.65 V。
    pub offset_v: f64,
    /// 把所有通道都当原始码值（`CH1V` 列也按 LSB 解析）。
    pub raw: bool,
    /// 时间原点不为零时是否归一化（否则拒绝）。
    pub normalize_t0: bool,
}

impl Default for ImportOptions {
    fn default() -> Self {
        ImportOptions {
            id: IMPORT_ID_BASE,
            path: String::new(),
            rate_hz: None,
            full_scale_v: DEFAULT_FULL_SCALE_V,
            offset_v: 0.0,
            raw: false,
            normalize_t0: false,
        }
    }
}

impl ImportOptions {
    /// 由一组 `ChannelScale` 构造电压换算参数。
    ///
    /// 这是与 [`Capture::to_csv`] 对齐的唯一正确方式：导出时用的是
    /// `volts_per_lsb` 与 `zero_lsb`，导入必须用同一组值才能逐点还原。
    /// 取 0 号通道的换算作为代表（本项目的文件各通道用同一套默认换算；
    /// 若各通道不同，调用方应改用 [`ImportOptions::full_scale_v`] /
    /// [`ImportOptions::offset_v`] 显式给出）。
    pub fn from_scale(scale: &ChannelScale) -> Self {
        let vpl = scale.volts_per_lsb;
        ImportOptions {
            full_scale_v: vpl * 4096.0,
            offset_v: -scale.zero_lsb * vpl,
            ..ImportOptions::default()
        }
    }
}

/// 导入结果。
#[derive(Debug, Clone)]
pub struct ImportedCapture {
    /// 还原出的采集。
    pub capture: Capture,
    /// 采样率的来源。
    pub rate_source: RateSource,
    /// 原始文件里首个时间值（秒）。归一化时只用它记录，不改变时间轴语义。
    pub t0_s: f64,
}

/// 导入失败的原因。
#[derive(Debug, Clone, PartialEq)]
pub enum ImportError {
    /// 没有数据行。
    Empty,
    /// 表头里没有任何可识别的数据列。
    NoDataColumns,
    /// 列数超过 [`MAX_IMPORT_CHANNELS`]。
    TooManyChannels {
        /// 实际列数。
        count: usize,
    },
    /// 通道列名不认识（或编号不连续）。
    BadChannelName {
        /// 原始列名。
        name: String,
    },
    /// 某一行的列数与表头不一致。
    ColumnCountMismatch {
        /// 行号（1 起）。
        line: usize,
        /// 表头列数。
        expected: usize,
        /// 实际列数。
        got: usize,
    },
    /// 某个数解析不了。
    BadNumber {
        /// 行号（1 起）。
        line: usize,
        /// 列名。
        column: String,
        /// 原始文本。
        text: String,
    },
    /// 相邻时间差分不一致（相对偏差 > 1%）。
    NonUniformTime {
        /// 出问题的行号（1 起）。
        line: usize,
        /// 该处的间隔（秒）。
        dt_s: f64,
        /// 中位间隔（秒）。
        median_s: f64,
    },
    /// 时间原点不为零，且没要求归一化。
    NonZeroTimeOrigin {
        /// 首个时间值（秒）。
        t0_s: f64,
        /// 采样间隔（秒）。
        dt_s: f64,
    },
    /// 既没有速率来源，时间列也无法反推。
    RateMissing,
    /// 速率不是正整数，或超出 `u32`。
    RateUnsupported {
        /// 文件/参数里给出的速率。
        rate_hz: f64,
    },
    /// 电压换算出的码值越界。
    OutOfRange {
        /// 行号（1 起）。
        line: usize,
        /// 列名。
        column: String,
        /// 原始电压/码值。
        value: f64,
    },
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImportError::Empty => write!(f, "文件里没有数据行"),
            ImportError::NoDataColumns => write!(
                f,
                "表头里没有可识别的数据列（需要 `Time(s),CH1V` 这样的表头）"
            ),
            ImportError::TooManyChannels { count } => {
                write!(f, "通道数 {count} 超过上限 {MAX_IMPORT_CHANNELS}")
            }
            ImportError::BadChannelName { name } => write!(f, "通道列名不认识：`{name}`"),
            ImportError::ColumnCountMismatch {
                line,
                expected,
                got,
            } => write!(f, "第 {line} 行有 {got} 列，表头有 {expected} 列"),
            ImportError::BadNumber { line, column, text } => {
                write!(f, "第 {line} 行的 `{column}` 不是有效数字：`{text}`")
            }
            ImportError::NonUniformTime {
                line,
                dt_s,
                median_s,
            } => write!(
                f,
                "第 {line} 行的时间间隔 {dt_s:.9} s 与中位间隔 {median_s:.9} s 相差超过 1%"
            ),
            ImportError::NonZeroTimeOrigin { t0_s, dt_s } => write!(
                f,
                "时间原点为 {t0_s:.9} s，偏离 0 超过一个采样间隔 {dt_s:.9} s；\
                 可加 --normalize-t0 归一化"
            ),
            ImportError::RateMissing => write!(
                f,
                "无法确定采样率：文件没有 `# rate_hz=` 注释、未给 --rate-hz，\
                 且时间列间隔不能用 9 位小数精确表示"
            ),
            ImportError::RateUnsupported { rate_hz } => write!(
                f,
                "采样率 {rate_hz} Hz 不是正整数或超出 u32 范围，不能建时间轴"
            ),
            ImportError::OutOfRange {
                line,
                column,
                value,
            } => write!(
                f,
                "第 {line} 行的 `{column}` 值 {value} 超出 12-bit 码值范围 0..=4095"
            ),
        }
    }
}

impl std::error::Error for ImportError {}

/// 从 CSV 文本导入一次采集。
///
/// 纯函数：不做 IO。文件读取与编号分配由调用方负责。
pub fn from_csv(text: &str, opts: &ImportOptions) -> Result<ImportedCapture, ImportError> {
    let mut header_rate: Option<f64> = None;
    let mut header: Option<Vec<String>> = None;
    let mut rows: Vec<(usize, Vec<String>)> = Vec::new();

    for (idx, raw_line) in text.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if header_rate.is_none() {
                header_rate = comment_rate(comment);
            }
            continue;
        }
        if header.is_none() {
            header = Some(split_fields(line));
        } else {
            rows.push((idx + 1, split_fields(line)));
        }
    }

    let header = header.ok_or(ImportError::Empty)?;
    if rows.is_empty() {
        return Err(ImportError::Empty);
    }

    let layout = parse_header(&header)?;
    let columns = layout.chans.len() + usize::from(layout.has_time);

    let mut times: Vec<f64> = Vec::with_capacity(rows.len());
    let mut row_lines: Vec<usize> = Vec::with_capacity(rows.len());
    let mut channels: Vec<Vec<u16>> = vec![Vec::with_capacity(rows.len()); layout.chans.len()];

    for (line, fields) in &rows {
        if fields.len() != columns {
            return Err(ImportError::ColumnCountMismatch {
                line: *line,
                expected: columns,
                got: fields.len(),
            });
        }
        row_lines.push(*line);
        if layout.has_time {
            let text = fields[0].trim();
            let t = parse_f64(text).ok_or_else(|| ImportError::BadNumber {
                line: *line,
                column: header[0].clone(),
                text: text.to_string(),
            })?;
            times.push(t);
        }
        for (k, (ch_idx, volts)) in layout.chans.iter().enumerate() {
            let field = k + usize::from(layout.has_time);
            let text = fields[field].trim();
            let v = parse_f64(text).ok_or_else(|| ImportError::BadNumber {
                line: *line,
                column: header[field].clone(),
                text: text.to_string(),
            })?;
            let code = if opts.raw || !volts {
                to_raw_code(v).ok_or(ImportError::OutOfRange {
                    line: *line,
                    column: header[field].clone(),
                    value: v,
                })?
            } else {
                volts_to_lsb(v, opts).ok_or(ImportError::OutOfRange {
                    line: *line,
                    column: header[field].clone(),
                    value: v,
                })?
            };
            channels[*ch_idx].push(code);
        }
    }

    // ── 等间隔性（有时间列且至少两个点时）──
    let median = if layout.has_time && times.len() >= 2 {
        Some(check_uniform(&times, &row_lines)?)
    } else {
        None
    };

    // ── 速率：注释 > 显式参数 > 时间列 ──
    let (rate, rate_source) = if let Some(r) = header_rate {
        (validate_rate(r)?, RateSource::Header)
    } else if let Some(r) = opts.rate_hz {
        (validate_rate(r as f64)?, RateSource::Option)
    } else if let Some(median) = median {
        (infer_rate_from_median(median)?, RateSource::TimeColumn)
    } else {
        return Err(ImportError::RateMissing);
    };

    // ── 时间原点 ──
    let dt = median.unwrap_or_else(|| 1.0 / rate as f64);
    let t0 = if layout.has_time {
        let t0 = times[0];
        if t0.abs() > dt && !opts.normalize_t0 {
            return Err(ImportError::NonZeroTimeOrigin { t0_s: t0, dt_s: dt });
        }
        t0
    } else {
        0.0
    };

    let n = rows.len() as u32;
    let mut capture =
        Capture::new(opts.id, rate, channels.len(), n).with_source(CaptureSource::File {
            path: opts.path.clone(),
        });
    capture.channels = channels;

    Ok(ImportedCapture {
        capture,
        rate_source,
        t0_s: t0,
    })
}

/// 表头布局。
struct Layout {
    /// 首列是不是时间。
    has_time: bool,
    /// 每个通道列：`(通道下标, 是否伏特列)`。列序任意，按编号落位。
    chans: Vec<(usize, bool)>,
}

fn parse_header(header: &[String]) -> Result<Layout, ImportError> {
    let mut has_time = false;
    let mut start = 0;
    if let Some(first) = header.first() {
        let name = first.trim();
        if name.eq_ignore_ascii_case("Time(s)") || name.eq_ignore_ascii_case("Time") {
            has_time = true;
            start = 1;
        }
    }

    let mut chans: Vec<(usize, bool)> = Vec::with_capacity(header.len() - start);
    for name in &header[start..] {
        let (n, volts) = parse_channel_name(name)?;
        chans.push((n, volts));
    }
    if chans.is_empty() {
        return Err(ImportError::NoDataColumns);
    }
    if chans.len() > MAX_IMPORT_CHANNELS {
        return Err(ImportError::TooManyChannels { count: chans.len() });
    }

    // 编号必须恰好覆盖 1..=N，允许列序任意；这样 `CH2V,CH1V` 也能正确落位。
    let mut nums: Vec<usize> = chans.iter().map(|(idx, _)| idx + 1).collect();
    nums.sort_unstable();
    let want: Vec<usize> = (1..=chans.len()).collect();
    if nums != want {
        return Err(ImportError::BadChannelName {
            name: header[start..].join(","),
        });
    }

    Ok(Layout { has_time, chans })
}

/// 解析 `CH<n>` 或 `CH<n>V`，返回**0 起的通道下标**。
fn parse_channel_name(name: &str) -> Result<(usize, bool), ImportError> {
    let up = name.trim().to_ascii_uppercase();
    let rest = up
        .strip_prefix("CH")
        .ok_or_else(|| ImportError::BadChannelName {
            name: name.trim().to_string(),
        })?;
    let (digits, volts) = match rest.strip_suffix('V') {
        Some(d) => (d, true),
        None => (rest, false),
    };
    let n: usize = digits.parse().map_err(|_| ImportError::BadChannelName {
        name: name.trim().to_string(),
    })?;
    if n == 0 {
        return Err(ImportError::BadChannelName {
            name: name.trim().to_string(),
        });
    }
    Ok((n - 1, volts))
}

/// 注释里的 `rate_hz=`。未知键一律忽略（向前兼容）。
fn comment_rate(comment: &str) -> Option<f64> {
    for token in comment.split_whitespace() {
        if let Some((key, value)) = token.split_once('=') {
            if key.trim() == "rate_hz" {
                return parse_f64(value.trim());
            }
        }
    }
    None
}

fn split_fields(line: &str) -> Vec<String> {
    line.split(',').map(|s| s.trim().to_string()).collect()
}

fn parse_f64(text: &str) -> Option<f64> {
    text.parse::<f64>().ok().filter(|v| v.is_finite())
}

/// 速率必须是 u32 范围内的正整数值。
fn validate_rate(rate: f64) -> Result<u32, ImportError> {
    if !rate.is_finite() || rate <= 0.0 || rate.fract() != 0.0 || rate > u32::MAX as f64 {
        return Err(ImportError::RateUnsupported { rate_hz: rate });
    }
    Ok(rate as u32)
}

/// 由时间列的中位差分反推速率。
///
/// 只接受**整数速率**且其间隔能被 9 位小数精确表示：857142 Hz 的 `dt`
/// 写成 `0.000001167 s`，反推得 856898.03 Hz —— 一个看起来完全合理的
/// 错值，所以必须报 `RateMissing`，不能取整后接受。
fn infer_rate_from_median(median: f64) -> Result<u32, ImportError> {
    let ns = median * 1e9;
    if !ns.is_finite() || ns <= 0.0 {
        return Err(ImportError::RateMissing);
    }
    let rate = 1e9 / ns;
    let rounded = rate.round();
    if (rate - rounded).abs() > 1e-9 * rounded.abs().max(1.0) {
        return Err(ImportError::RateMissing);
    }
    validate_rate(rounded)
}

/// 检查等间隔性，返回中位差分。
fn check_uniform(times: &[f64], row_lines: &[usize]) -> Result<f64, ImportError> {
    let diffs: Vec<f64> = times.windows(2).map(|w| w[1] - w[0]).collect();

    let mut sorted = diffs.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    let median = if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    };
    if median <= 0.0 {
        return Err(ImportError::RateMissing);
    }

    for (k, d) in diffs.iter().enumerate() {
        if *d <= 0.0 || (d - median).abs() / median > 0.01 {
            let line = row_lines.get(k + 1).copied().unwrap_or(k + 2);
            return Err(ImportError::NonUniformTime {
                line,
                dt_s: *d,
                median_s: median,
            });
        }
    }
    Ok(median)
}

/// 电压 → 12-bit 码值。
///
/// `full_scale_v` 是 4096 个码值覆盖的电压跨度，`offset_v` 是**码值 0
/// 对应的输入电压**（默认 0，即输入 0..3.3 V 对应码值 0..4095）。
///
/// 要与 [`Capture::to_csv`] 的 `ChannelScale` 对齐时用
/// [`ImportOptions::from_scale`]：那里会把 `zero_lsb` 折算成 `offset_v`。
fn volts_to_lsb(v: f64, opts: &ImportOptions) -> Option<u16> {
    if !v.is_finite() || !opts.full_scale_v.is_finite() || opts.full_scale_v <= 0.0 {
        return None;
    }
    let code = ((v - opts.offset_v) * 4096.0 / opts.full_scale_v).round();
    if !(0.0..=MAX_CODE).contains(&code) {
        return None;
    }
    Some(code as u16)
}

/// 原始码值：必须是 0..=4095 的整数。
fn to_raw_code(v: f64) -> Option<u16> {
    if !v.is_finite() || v.fract() != 0.0 || !(0.0..=MAX_CODE).contains(&v) {
        return None;
    }
    Some(v as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn csv_of(cap: &Capture) -> String {
        let scale = ChannelScale::default();
        cap.to_csv(&vec![scale; cap.channels.len()])
    }

    fn opts(path: &str) -> ImportOptions {
        ImportOptions {
            path: path.to_string(),
            ..ImportOptions::default()
        }
    }

    /// C1：导出再导入，样点与速率逐点相等。
    ///
    /// 速率靠 `# rate_hz=` 头 —— 857142 Hz 的时间列反推不出来。
    /// 电压用导出的同一套 `ChannelScale` 解释（`from_scale`）。
    #[test]
    fn c1_round_trip_is_exact_including_rate() {
        let mut cap = Capture::new(1, 857_142, 2, 8);
        cap.channels[0] = vec![0, 512, 1024, 2048, 3072, 4095, 1000, 2000];
        cap.channels[1] = vec![4095, 3000, 2048, 1024, 0, 1, 2048, 4095];

        let mut o = ImportOptions::from_scale(&ChannelScale::default());
        o.path = "x.csv".into();
        let got = from_csv(&csv_of(&cap), &o).unwrap();

        assert_eq!(got.rate_source, RateSource::Header);
        assert_eq!(got.capture.rate_hz, 857_142);
        assert_eq!(got.capture.channels, cap.channels);
        assert_eq!(got.capture.len(), 8);
        assert_eq!(
            got.capture.source,
            CaptureSource::File {
                path: "x.csv".into()
            }
        );
    }

    /// C2：两通道文件的 `CH2V` 落在第二通道。
    #[test]
    fn c2_second_voltage_column_lands_in_channel_two() {
        // 0 V 是导出端的中点（码值 2048），±1.649194 V 对应两端码值。
        let text = "# rate_hz=1000000\nTime(s),CH1V,CH2V\n\
                    0.000000000,0.000000,-1.649194\n\
                    0.000001000,1.649194,0.000000\n";
        let mut o = ImportOptions::from_scale(&ChannelScale::default());
        o.path = "two.csv".into();
        let got = from_csv(text, &o).unwrap();

        assert_eq!(got.capture.channels.len(), 2);
        assert_eq!(got.capture.channels[0], vec![2048, 4095]);
        assert_eq!(got.capture.channels[1], vec![1, 2048]);
    }

    /// C3：5 GS/s 超出 `u32`，必须报错；1.5 GS/s 在范围内，应接受。
    #[test]
    fn c3_rate_above_u32_is_rejected_but_1p5_gsps_is_not() {
        let bad = "# rate_hz=5000000000\nTime(s),CH1V\n0.0,0.0\n1.0,0.0\n";
        assert_eq!(
            from_csv(bad, &opts("bad.csv")).unwrap_err(),
            ImportError::RateUnsupported {
                rate_hz: 5_000_000_000.0
            }
        );

        let ok = "# rate_hz=1500000000\nTime(s),CH1V\n0.0,0.0\n1.0,0.0\n";
        assert_eq!(
            from_csv(ok, &opts("ok.csv")).unwrap().capture.rate_hz,
            1_500_000_000
        );
    }

    /// C4：非等间隔必须报错，不得用平均间隔静默补齐。
    #[test]
    fn c4_non_uniform_time_is_rejected() {
        let text = "Time(s),CH1V\n\
                    0.000000000,0.0\n\
                    0.000001000,0.0\n\
                    0.000002200,0.0\n";
        match from_csv(text, &opts("nu.csv")).unwrap_err() {
            ImportError::NonUniformTime { .. } => {}
            other => panic!("应报 NonUniformTime，得到 {other:?}"),
        }
    }

    /// C6：`#` 注释与未知键被忽略。
    #[test]
    fn c6_comment_lines_and_unknown_keys_are_ignored() {
        let text = "# source=wavesurfer model=X\n\
                    # rate_hz=1000000\n\
                    # future_key=whatever\n\
                    Time(s),CH1V\n\
                    # 中间也可以有注释\n\
                    0.000000000,0.000000\n\
                    0.000001000,1.650000\n";
        let got = from_csv(text, &opts("c6.csv")).unwrap();
        assert_eq!(got.capture.rate_hz, 1_000_000);
        assert_eq!(got.capture.len(), 2);
    }

    /// 速率优先级：注释 > 显式参数 > 时间列。
    #[test]
    fn rate_priority_header_then_option_then_time_column() {
        let body = "Time(s),CH1V\n0.000000000,0.0\n0.000002000,0.0\n";

        let with_header = format!("# rate_hz=1000000\n{body}");
        let mut o = opts("p.csv");
        o.rate_hz = Some(2_000_000);
        assert_eq!(
            from_csv(&with_header, &o).unwrap().capture.rate_hz,
            1_000_000
        );

        assert_eq!(from_csv(body, &o).unwrap().capture.rate_hz, 2_000_000);

        o.rate_hz = None;
        let got = from_csv(body, &o).unwrap();
        assert_eq!(got.rate_source, RateSource::TimeColumn);
        assert_eq!(got.capture.rate_hz, 500_000);
    }

    /// 时间列反推只在 9 位小数能精确表达时成立。
    ///
    /// 857142 Hz 的 `dt` 写成 9 位小数是 `0.000001167`，反推得 856898 Hz ——
    /// 一个看起来完全合理的错值，所以必须报 `RateMissing` 而不是接受它。
    #[test]
    fn time_column_inference_refuses_a_quantized_rate() {
        let text = "Time(s),CH1V\n0.000000000,0.0\n0.000001167,0.0\n";
        assert_eq!(
            from_csv(text, &opts("q.csv")).unwrap_err(),
            ImportError::RateMissing
        );
    }

    /// 没有时间列时，必须显式给速率。
    #[test]
    fn no_time_column_requires_an_explicit_rate() {
        let text = "CH1V,CH2V\n0.000000,1.650000\n0.000000,1.650000\n";
        assert_eq!(
            from_csv(text, &opts("n.csv")).unwrap_err(),
            ImportError::RateMissing
        );

        let mut o = opts("n.csv");
        o.rate_hz = Some(1000);
        let got = from_csv(text, &o).unwrap();
        assert_eq!(got.capture.rate_hz, 1000);
        assert_eq!(got.capture.channels.len(), 2);
        assert_eq!(got.rate_source, RateSource::Option);
    }

    /// 时间原点不为零：默认拒绝，`normalize_t0` 时接受并记录。
    #[test]
    fn non_zero_time_origin_is_rejected_unless_normalized() {
        let text = "# rate_hz=1000\nTime(s),CH1V\n0.500000000,0.0\n0.501000000,0.0\n";

        match from_csv(text, &opts("t.csv")).unwrap_err() {
            ImportError::NonZeroTimeOrigin { t0_s, .. } => assert!((t0_s - 0.5).abs() < 1e-12),
            other => panic!("应报 NonZeroTimeOrigin，得到 {other:?}"),
        }

        let mut o = opts("t.csv");
        o.normalize_t0 = true;
        let got = from_csv(text, &o).unwrap();
        assert!((got.t0_s - 0.5).abs() < 1e-12);
        assert_eq!(got.capture.rate_hz, 1000);
    }

    /// `CH<n>` 列按原始码值解析，且 `--raw` 可强制所有列如此。
    #[test]
    fn raw_columns_are_adc_codes() {
        let text = "# rate_hz=1000\nTime(s),CH1\n0.0,0\n0.001,4095\n";
        let got = from_csv(text, &opts("r.csv")).unwrap();
        assert_eq!(got.capture.channels[0], vec![0, 4095]);

        let volts = "# rate_hz=1000\nTime(s),CH1V\n0.0,0\n0.001,4095\n";
        let mut o = opts("r.csv");
        o.raw = true;
        let got = from_csv(volts, &o).unwrap();
        assert_eq!(got.capture.channels[0], vec![0, 4095]);
    }

    /// 电压越界报 `OutOfRange`，不做钳位。
    #[test]
    fn voltage_out_of_range_is_rejected() {
        let text = "# rate_hz=1000\nTime(s),CH1V\n0.0,99.0\n0.001,0.0\n";
        match from_csv(text, &opts("o.csv")).unwrap_err() {
            ImportError::OutOfRange { .. } => {}
            other => panic!("应报 OutOfRange，得到 {other:?}"),
        }
    }

    /// 列数上限与表头校验。
    #[test]
    fn header_limits_are_enforced() {
        let five = "# rate_hz=1000\nTime(s),CH1V,CH2V,CH3V,CH4V,CH5V\n\
                    0,0,0,0,0,0\n";
        assert_eq!(
            from_csv(five, &opts("5.csv")).unwrap_err(),
            ImportError::TooManyChannels { count: 5 }
        );

        let bad = "# rate_hz=1000\nTime(s),FOO\n0,0\n";
        assert!(matches!(
            from_csv(bad, &opts("b.csv")).unwrap_err(),
            ImportError::BadChannelName { .. }
        ));

        let no_cols = "# rate_hz=1000\nTime(s)\n0\n";
        assert_eq!(
            from_csv(no_cols, &opts("e.csv")).unwrap_err(),
            ImportError::NoDataColumns
        );

        let mismatched = "# rate_hz=1000\nTime(s),CH1V\n0,0,0\n";
        assert!(matches!(
            from_csv(mismatched, &opts("m.csv")).unwrap_err(),
            ImportError::ColumnCountMismatch { .. }
        ));
    }

    /// 没有数据行 → `Empty`。
    #[test]
    fn header_only_file_is_empty() {
        let text = "# rate_hz=1000\nTime(s),CH1V\n";
        assert_eq!(
            from_csv(text, &opts("h.csv")).unwrap_err(),
            ImportError::Empty
        );
    }

    /// `from_scale` 与 `to_csv` 的换算同源：任意 `ChannelScale` 都能往返。
    #[test]
    fn from_scale_matches_the_exporter_scale() {
        let scale = ChannelScale {
            volts_per_lsb: 0.001,
            zero_lsb: 1000.0,
        };
        let mut cap = Capture::new(1, 1_000_000, 1, 4);
        cap.channels[0] = vec![0, 1000, 2000, 4095];
        let csv = cap.to_csv(&[scale]);

        let mut o = ImportOptions::from_scale(&scale);
        o.path = "s.csv".into();
        let got = from_csv(&csv, &o).unwrap();
        assert_eq!(got.capture.channels[0], cap.channels[0]);
    }
}
