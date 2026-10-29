//! 视图模型 —— 把采集与解码结果翻译成**可绘制的数据**。
//!
//! # 为什么单独一层
//!
//! 这里的函数**不依赖 egui**，全是纯函数：输入 `Capture` / `I2cDecode`，
//! 输出坐标数组和字符串。好处是它们能在没有显示器、没有 GPU 的环境里单测 ——
//! 这是 CI 唯一能真正验证 GUI 逻辑的途径（CI 是 Ubuntu 无头机，
//! 装不了、也不该装图形栈）。
//!
//! # 两条纪律在这里落地
//!
//! - **时间轴由 `capture.rate_hz` 推导**（设备回显的实际值），绝不用请求值，
//!   也不用分片到达时刻。见 `docs/03-protocol.md`。
//! - **伏特换算只发生在显示层**。控制链路和存储里全是 ADC LSB 整数，
//!   `ChannelScale` 是唯一的换算点（ADR-006）。

use scope_core::i2c_decode::{EventKind, I2cDecode, I2cWarning};

// 测量算法在 core 里 —— GUI 与 MCP 共用一份。
// 只有测试用到它（界面走 `AnalysisReport`），故按测试导入，避免
// 非测试构建里的 unused-import 警告。
#[cfg(test)]
use scope_core::measure::measure;
use scope_core::{Capture, ChannelScale};

/// 一段带标注的时间区间，画在数字泳道上。
#[derive(Debug, Clone, PartialEq)]
pub struct EventSpan {
    /// 起始时间（µs，相对采集起点）。
    pub t0_us: f64,
    /// 结束时间（µs）。
    pub t1_us: f64,
    /// 这一段是什么。
    pub kind: SpanKind,
    /// 只有这一位是 NACK（要从视觉上区分出来）。
    pub nack: bool,
}

/// 解码事件的分类 —— 决定泳道色块的颜色。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpanKind {
    /// 起始位 / 重复起始位。
    Start,
    /// 地址字节。
    Address,
    /// 数据字节。
    Data,
    /// 停止位。
    Stop,
    /// 被采集窗口截断。
    Truncated,
}

/// 采集的时间跨度（µs）。
///
/// 用 `rate_hz`（回显的实际采样率）换算 —— 请求值可能被定时器分频量化过。
pub fn duration_us(cap: &Capture) -> f64 {
    cap.duration_us() as f64
}

/// 把一个通道的画线坐标摊平。
///
/// 返回 `[[时间µs, 电压V], ...]`。**不做降采样** —— F103 的上限为 4096 点，
/// 直接绘制全量，样点信息无损失；minmax 抽点是给 Agent 省 token 用的，不是给人的。
pub fn waveform_points(cap: &Capture, ch: usize, scale: &ChannelScale) -> Vec<[f64; 2]> {
    let Some(samples) = cap.samples(ch) else {
        return Vec::new();
    };
    if cap.rate_hz == 0 {
        return Vec::new();
    }
    let dt_us = 1_000_000.0 / cap.rate_hz as f64;
    samples
        .iter()
        .enumerate()
        .map(|(i, &v)| [i as f64 * dt_us, scale.lsb_to_volts(v)])
        .collect()
}

/// 采样率档位菜单。
///
/// 用 `f103::nearest_achievable_rate` 把常用值**吸附到定时器真的能出的档位** ——
/// 直接给整数会让用户选到设备根本做不到的值，然后被回显打脸。
///
/// 上限是 **857142**，不是 1000000。`docs/04-performance.md:35` 写死了
/// 「任何文档、UI、宣传里都不许写 1 MSPS」。
pub fn rate_choices() -> Vec<u32> {
    const REQUESTED: &[u32] = &[
        857_142, // 默认档：72 MHz / 84
        600_000, 461_538, 292_683, 176_471, 47_619,
    ];
    let mut out: Vec<u32> = REQUESTED
        .iter()
        .map(|&r| scope_core::f103::nearest_achievable_rate(r))
        .collect();
    out.dedup();
    out
}

/// 把一次解码拆成泳道上的色块。
///
/// 每个事件的结束时间是**下一个事件**的起点；最后一个事件延伸到最后
/// （截断帧本来就没有终点）。
pub fn event_spans(decode: &I2cDecode, total_us: f64) -> Vec<EventSpan> {
    let mut flat: Vec<(f64, SpanKind, bool)> = Vec::new();
    for tx in &decode.transactions {
        for ev in &tx.events {
            let t = ev.time_us as f64;
            let (kind, nack) = match &ev.kind {
                EventKind::Start | EventKind::RepeatedStart => (SpanKind::Start, false),
                EventKind::Address(a) => (SpanKind::Address, !a.acked),
                EventKind::Data { acked, .. } => (SpanKind::Data, !*acked),
                EventKind::Stop => (SpanKind::Stop, false),
                EventKind::Truncated => (SpanKind::Truncated, false),
            };
            flat.push((t, kind, nack));
        }
    }
    flat.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    flat.iter()
        .enumerate()
        .map(|(i, &(t0, kind, nack))| {
            let t1 = flat
                .get(i + 1)
                .map(|n| n.0)
                .unwrap_or_else(|| total_us.max(t0));
            EventSpan {
                t0_us: t0,
                // 保证宽度为正，否则 egui_plot 的多边形会退化
                t1_us: t1.max(t0 + 0.001),
                kind,
                nack,
            }
        })
        .collect()
}

/// 一行「帧 N @ x.xxx ms 地址 … 数据 …」的摘要，给交易表用。
pub fn transaction_summary(decode: &I2cDecode) -> Vec<String> {
    decode
        .transactions
        .iter()
        .enumerate()
        .map(|(i, tx)| {
            let hex: Vec<String> = tx.bytes.iter().map(|b| format!("{b:02X}")).collect();
            format!(
                "帧 {:<3} @ {:>7.3} ms   {:<10} {:<7} {}{}",
                i + 1,
                tx.start_time_us as f64 / 1000.0,
                tx.address_str(),
                if tx.bytes.is_empty() {
                    String::new()
                } else {
                    hex.join(" ")
                },
                if tx.has_nack() { "NACK" } else { "" },
                if tx.complete { "" } else { "  (截断)" }
            )
        })
        .collect()
}

/// 面板上要显示的告警文案（复用 core 的 `I2cWarning::text()`，不另写一份）。
///
/// **滤掉 `TruncatedFrame`**：采集窗口切断最后一帧是**每次采集都必然发生**的
/// —— I2C 总线一直在跑，4.778 ms 的窗口切在哪儿是随机的，切在两笔事务之间
/// 才是小概率。把它标成黄色警告，等于一个天天亮着的警报，真正的问题
/// （CRC 错、判决带告警）会被一起忽略。
///
/// 信息本身没丢：交易表里那一帧标着「(截断)」，图上那段也是灰色色块。
///
/// 注意 `I2cDecode::warnings` 里**仍然保留**这一条 —— MCP 侧的 Agent 需要它，
/// 否则会把「共 16 帧」当成 16 笔完整事务来回答。
pub fn warning_lines(decode: &I2cDecode) -> Vec<String> {
    decode
        .warnings
        .iter()
        .filter(|w| !matches!(w, I2cWarning::TruncatedFrame))
        .map(|w| w.text())
        .collect()
}

/// 没有 I2C 解码结果时，估一个合适的显示窗口。
///
/// 用 [`ChannelSummary::rising_edges`] 反推周期 —— 那个数 core 已经算好了，
/// 不需要在这里重扫一遍样点。
///
/// 目标是「大约 4 个周期」：1 kHz 正弦能看全，50 kHz 方波也看得清沿。
/// 没有边沿（直流）就全览；边沿太密（噪声，每个过零点都算一次）则退回 5% 打底，
/// 否则会缩到只剩几十个样点，屏幕上什么也看不出。
///
/// [`ChannelSummary::rising_edges`]: scope_core::ChannelSummary
pub fn suggest_span_us(cap: &Capture) -> f64 {
    let total = duration_us(cap);
    if total <= 0.0 {
        return 1.0;
    }

    let edges = (0..cap.channels.len())
        .filter_map(|ch| cap.summary(ch))
        .map(|s| s.rising_edges)
        .max()
        .unwrap_or(0);

    if edges < 2 {
        return total; // 直流 / 几乎不变：全览
    }
    let period_us = total / edges as f64;
    (period_us * 4.6).max(total / 20.0).min(total)
}

/// 视窗钳制：把 x 视窗拉回数据范围 `[lo, hi]` 内。
///
/// 返回 `None` 表示视窗本就在范围内，无需纠正；`Some((a, b))` 是纠正后
/// 的范围，由调用方在下一帧用 `PlotUi::set_plot_bounds_x` 施加。
///
/// # 为什么只能在 show 之后做
///
/// egui_plot 的内置拖拽/缩放在用户闭包**之后**处理（其 `show()` 先
/// `build_fn` 再 `handle_interactions`），闭包内的钳制会被同一帧的拖拽
/// 覆盖。所以只能「读本帧最终 bounds → 越界就记下来 → 下一帧闭包开头
/// 拉回」。拖拽到边界时约有 ≤ 一帧的回弹（几像素），静止后立即贴合。
///
/// # 规则（示波器惯例）
///
/// - 平移越界 → **保宽度平移**回界内（拖动不改变缩放档位）；视窗整个落在
///   数据外也贴到最近的数据边界，同样保宽度；
/// - 缩得比数据范围还小 → 钳成完整 `[lo, hi]`（最远只能「缩」到全览）。
pub fn clamp_x_view(x0: f64, x1: f64, lo: f64, hi: f64) -> Option<(f64, f64)> {
    if x0 >= x1 || lo >= hi {
        return None; // 退化输入（空视窗/空数据）不钳
    }
    let w = x1 - x0;
    let span = hi - lo;
    if w >= span {
        let full = (x0 - lo).abs() < f64::EPSILON && (x1 - hi).abs() < f64::EPSILON;
        return (!full).then_some((lo, hi));
    }
    match (x0 < lo, x1 > hi) {
        (true, _) => Some((lo, lo + w)), // 左越界：贴左缘，宽度不变
        (_, true) => Some((hi - w, hi)), // 右越界：贴右缘，宽度不变
        _ => None,                       // 在范围内
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scope_core::i2c_decode::{I2cDecodeConfig, Levels};
    use scope_core::Capture;

    /// 造一份能解出确定内容的两通道采集（0x88 / 0x00 / 0x1A）。
    fn sample_capture() -> Capture {
        // 每个半周期 4 个样点 → 一个比特 8 个样点 → 100 kHz @ 800 kSPS
        let rate = 800_000u32;
        const HIGH: u16 = 4095;
        const LOW: u16 = 0;

        // 声明必须在宏**之前** —— macro_rules! 里的局部变量在定义处解析（卫生性），
        // 宏体里的 `scl` 看不见定义之后才声明的同名变量。
        let mut scl: Vec<u16> = Vec::new();
        let mut sda: Vec<u16> = Vec::new();

        // 用宏而不是闭包：闭包会各自可变借走 scl/sda，两个闭包共存时编译器不让过
        macro_rules! hold {
            ($sv:expr, $dv:expr, $n:expr) => {
                for _ in 0..$n {
                    scl.push($sv);
                    sda.push($dv);
                }
            };
        }
        macro_rules! bit {
            ($sv:expr) => {{
                let d = if $sv { HIGH } else { LOW };
                for _ in 0..4 {
                    scl.push(LOW);
                    sda.push(d);
                }
                for _ in 0..4 {
                    scl.push(HIGH);
                    sda.push(d);
                }
            }};
        }

        // START：SCL 高时 SDA 由高变低
        hold!(HIGH, HIGH, 4);
        hold!(HIGH, LOW, 4);
        hold!(LOW, LOW, 4);
        for b in [0x88u8, 0x00, 0x1A] {
            for i in (0..8).rev() {
                bit!((b >> i) & 1 == 1);
            }
            bit!(false); // ACK：SDA 拉低
        }
        // STOP：SCL 高时 SDA 由低变高
        hold!(LOW, LOW, 4);
        hold!(HIGH, LOW, 4);
        hold!(HIGH, HIGH, 4);

        let mut cap = Capture::new(1, rate, 2, scl.len() as u32);
        cap.channels = vec![scl, sda];
        cap
    }

    #[test]
    fn duration_uses_the_echoed_rate_not_the_requested_one() {
        let cap = sample_capture();
        // START 12 + 3 字节 × 9 位 × 8 样点 = 216 + STOP 12 → 240
        assert_eq!(cap.len(), 240, "构造的波形应有 240 个样点");
        // 240 点 @ 800 kHz = 300 µs（若误用别的采样率，这个数会明显不同）
        let d = duration_us(&cap);
        assert!(
            (d - 300.0).abs() < 1.0,
            "时间轴必须按 rate_hz(=800000) 算，期望 300 µs，实测 {d} µs"
        );
    }

    #[test]
    fn rate_choices_never_exceed_the_f103_ceiling() {
        // docs/04-performance.md:35 —— UI 里不许出现 1 MSPS
        for r in rate_choices() {
            assert!(
                r <= scope_core::f103::MAX_SAMPLE_RATE_HZ,
                "档位 {r} 超过了 F103 上限 857142"
            );
        }
        assert_eq!(rate_choices()[0], 857_142, "默认档应是 857142");
    }

    #[test]
    fn waveform_points_span_the_whole_capture() {
        let cap = sample_capture();
        let scale = ChannelScale::default();
        let pts = waveform_points(&cap, 0, &scale);
        assert_eq!(pts.len(), cap.len(), "不做降采样，应逐点摊平");
        assert_eq!(pts[0][0], 0.0, "第一个点的时间应是 0");
        let last = pts.last().unwrap()[0];
        assert!(last > 0.0 && last < 4000.0);
    }

    #[test]
    fn waveform_points_of_a_missing_channel_is_empty_not_panic() {
        let cap = sample_capture();
        assert!(waveform_points(&cap, 9, &ChannelScale::default()).is_empty());
    }

    #[test]
    fn event_spans_cover_the_decode_in_order() {
        let cap = sample_capture();
        let d = scope_core::decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        assert_eq!(d.frame_count(), 1, "应解出 1 帧");

        let spans = event_spans(&d, duration_us(&cap));
        assert_eq!(spans.len(), d.transactions[0].events.len());
        // 时间必须单调不减
        for w in spans.windows(2) {
            assert!(w[1].t0_us >= w[0].t0_us, "事件跨度必须按时间排好序");
        }
        // 宽度必须为正（egui_plot 的多边形不接受零宽/负宽）
        for s in &spans {
            assert!(s.t1_us > s.t0_us);
        }
        // 至少有一个地址段和一个数据段
        assert!(spans.iter().any(|s| s.kind == SpanKind::Address));
        assert!(spans.iter().any(|s| s.kind == SpanKind::Data));
    }

    #[test]
    fn transaction_summary_reports_address_and_bytes() {
        let cap = sample_capture();
        let d = scope_core::decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let lines = transaction_summary(&d);
        assert_eq!(lines.len(), 1);
        assert!(lines[0].contains("0x44 (W)"), "应显示地址: {}", lines[0]);
        assert!(lines[0].contains("00 1A"), "应显示数据字节: {}", lines[0]);
    }

    #[test]
    fn no_signal_produces_warnings_not_silent_success() {
        let mut cap = Capture::new(1, 800_000, 2, 256);
        cap.channels = vec![vec![0u16; 256], vec![0u16; 256]];
        let d = scope_core::decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        assert!(d.is_empty());
        assert!(
            !warning_lines(&d).is_empty(),
            "没信号必须给出告警，不能静默返回空结果"
        );
    }

    #[test]
    fn span_adapts_to_the_signal_frequency() {
        // 低频信号（1 kHz 正弦，4.778 ms 里才 ~5 个周期）必须全览，
        // 否则只看得到一小段单调上升的曲线 —— 这正是这个函数存在的理由。
        let mut low = Capture::new(1, 857_142, 1, 4096);
        low.channels = vec![vec![2048u16; 4096]];
        // 手工塞 5 个上升沿：每 800 点翻越一次中点
        for i in 0..4096 {
            let seg = i / 800;
            low.channels[0][i] = if seg % 2 == 0 { 3000 } else { 1000 };
        }
        let s = suggest_span_us(&low);
        assert!(
            (s - duration_us(&low)).abs() < 1.0,
            "低频应全览，实测 {s} µs / 总长 {} µs",
            duration_us(&low)
        );

        // 直流（无边沿）也应该是全览
        let mut dc = Capture::new(1, 857_142, 1, 4096);
        dc.channels = vec![vec![2048u16; 4096]];
        assert!((suggest_span_us(&dc) - duration_us(&dc)).abs() < 1.0);

        // 高频方波：应缩到几个周期，而不是全览
        let mut fast = Capture::new(1, 857_142, 1, 4096);
        fast.channels = vec![(0..4096)
            .map(|i| if (i / 10) % 2 == 0 { 3000 } else { 1000 })
            .collect()];
        let sf = suggest_span_us(&fast);
        assert!(
            sf < duration_us(&fast) * 0.5,
            "高频应放大到几个周期，实测 {sf} µs（总长 {}）",
            duration_us(&fast)
        );
        assert!(sf > 1.0);
    }

    #[test]
    fn vpp_and_rms_are_magnitudes_not_levels() {
        // Vpp / RMS / AC-RMS 都是**幅值**，不能走带零偏的 lsb_to_volts。
        // 回归：曾经一律用 lsb_to_volts，于是 2.31 V 的 Vpp 被报成 0.66 V
        // （减掉了 2048 的零点），AC-RMS 更是直接变成负数。
        let mut cap = Capture::new(1, 800_000, 1, 100);
        cap.channels = vec![(0..100)
            .map(|i| if i % 10 < 5 { 614u16 } else { 3481 })
            .collect()];
        let m = measure(&cap, 0, &ChannelScale::default()).unwrap();

        // 2867 LSB × 3.3/4096 = 2.31 V
        assert!(
            (m.vpp - 2.31).abs() < 0.02,
            "Vpp 应为 2.31 V（幅值），实测 {:.3}",
            m.vpp
        );
        assert!(m.ac_rms >= 0.0, "AC-RMS 不可能是负数，实测 {}", m.ac_rms);
        assert!(m.rms >= 0.0, "RMS 不可能是负数，实测 {}", m.rms);
        // 电平量仍应带零偏：低电平 614 LSB → (614-2048)×3.3/4096 = -1.15 V
        assert!(
            (m.min + 1.15).abs() < 0.02,
            "min 是电平，应为 -1.15 V，实测 {:.3}",
            m.min
        );
    }

    #[test]
    fn frequency_and_duty_ignore_idle_gaps() {
        // 20 个周期的 50% 方波，后面拖一段长空闲（线拉高不动）。
        // 用「边沿数 ÷ 总时长」会把空闲算进分母 —— 100 kHz 会被算成 ~90 kHz。
        let mut v: Vec<u16> = Vec::new();
        for _ in 0..20 {
            v.extend(std::iter::repeat_n(3481u16, 5));
            v.extend(std::iter::repeat_n(614u16, 5));
        }
        v.extend(std::iter::repeat_n(3481u16, 200)); // 空闲
        let n = v.len();
        let mut cap = Capture::new(1, 1_000_000, 1, n as u32);
        cap.channels = vec![v];

        let m = measure(&cap, 0, &ChannelScale::default()).unwrap();
        let f = m.freq_hz.expect("应能测出频率");
        assert!(
            (f - 100_000.0).abs() < 1.0,
            "周期 10 样点 @1MHz = 100 kHz，实测 {f}"
        );
        let d = m.duty_pct.expect("应能测出占空比");
        assert!((d - 50.0).abs() < 5.0, "50% 方波应报 50%，实测 {d}");
    }

    #[test]
    fn dc_has_levels_but_no_period() {
        // 直流：有电平，但频率/占空比/上升时间都**测不出来** ——
        // 该报 None，不是 0（无可信结论必须显式拒答）
        let mut cap = Capture::new(1, 800_000, 1, 64);
        cap.channels = vec![vec![2048u16; 64]];
        let m = measure(&cap, 0, &ChannelScale::default()).unwrap();
        assert_eq!(m.vpp, 0.0);
        assert!(m.freq_hz.is_none());
        assert!(m.duty_pct.is_none());
        assert!(m.rise_ns.is_none());
    }

    #[test]
    fn truncated_frame_stays_in_core_but_is_hidden_from_the_panel() {
        // 采集窗口切断最后一帧是**必然事件**（I2C 总线一直在跑，4.778 ms 的
        // 窗口切在两笔事务之间才是小概率），所以面板上不该亮黄灯。
        // 但 core 的 warnings 里必须留着 —— MCP 侧的 Agent 要靠它判断
        // 「共 N 帧」里有几帧是不完整的。
        let mut cap = sample_capture();
        for ch in cap.channels.iter_mut() {
            ch.truncate(200); // 砍在第三个字节中间，制造一次截断
        }
        let d = scope_core::decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();

        assert!(
            d.warnings
                .iter()
                .any(|w| matches!(w, I2cWarning::TruncatedFrame)),
            "core 的 warnings 不应滤掉 TruncatedFrame，Agent 需要它"
        );
        assert!(
            !warning_lines(&d).iter().any(|l| l.contains("截断")),
            "面板不应显示截断告警，实测 {:?}",
            warning_lines(&d)
        );
    }

    #[test]
    fn levels_default_is_the_documented_ratio() {
        let l = Levels::default_ratio();
        assert_eq!(l.vih_lsb, (4095.0f32 * 0.70).round() as u16);
        assert_eq!(l.vil_lsb, (4095.0f32 * 0.30).round() as u16);
        assert!(l.is_valid());
    }

    /// A1：`uart_115k` 场景解出的字节序必须是已知内容 ——
    /// `55 0F A5` 循环的首三字节（内容确定，才写得出断言）。
    /// 帧表渲染与证据包共用 core 的 `uart_body_text`，此处一并钉住。
    #[test]
    fn uart_115k_decodes_the_known_byte_sequence() {
        let mut gen = scope_sim::WaveformGen::new(scope_sim::Scenario::Uart115k, 1);
        let channels = gen.generate_multi(1, 4096, 857_142);
        let mut cap = scope_core::Capture::new(1, 857_142, 1, 4096);
        cap.channels = channels;
        let req = scope_core::DecodeRequest {
            protocol: scope_core::Protocol::Uart,
            channels: vec![],
            params: scope_core::DecodeParams::Uart(scope_core::UartConfig::default()),
        };
        let d = scope_core::decoder::decode(&cap, &req).expect("uart_115k 应能解码");
        let bytes = d.uart_bytes().expect("UART 结果应有字节序列");
        assert!(
            bytes.len() >= 3,
            "4096 点 @115200 应解出多帧，实际 {} 帧",
            d.frame_count()
        );
        assert_eq!(
            &bytes[..3],
            &[0x55, 0x0F, 0xA5],
            "首三字节必须是已知模式 55 0F A5"
        );
        // 界面帧表（与证据包同一份文本）里应出现首字节的行
        let text = d.uart_body_text();
        assert!(text.contains("0x55"), "帧表应含首字节 0x55：\n{text}");
    }

    // ── 视窗钳制 ────────────────────────────────────────────────

    /// **在范围内的视窗不钳。**
    #[test]
    fn view_inside_data_range_is_untouched() {
        assert_eq!(clamp_x_view(100.0, 400.0, 0.0, 4778.0), None);
        assert_eq!(clamp_x_view(0.0, 4778.0, 0.0, 4778.0), None);
    }

    /// **平移越界保宽度**：拖到数据边界就停，缩放档位不变。
    /// 变异：越界侧直接截断（`(x0.max(lo), x1.min(hi))`）—— 视窗变窄，
    /// 拖一次等于偷偷缩放，这条红。
    #[test]
    fn panning_past_the_edge_pins_width_intact() {
        // 左越界：视窗 [-100, 400] → 贴左缘 [0, 500]
        assert_eq!(clamp_x_view(-100.0, 400.0, 0.0, 4778.0), Some((0.0, 500.0)));
        // 右越界：数据 4778，视窗 [4600, 5100] → 贴右缘 [4278, 4778]
        assert_eq!(
            clamp_x_view(4600.0, 5100.0, 0.0, 4778.0),
            Some((4278.0, 4778.0))
        );
    }

    /// **缩得比数据还小 → 钳成全览**；已经是全览 → 不钳。
    #[test]
    fn zooming_out_wider_than_data_clamps_to_the_full_view() {
        assert_eq!(
            clamp_x_view(-500.0, 5300.0, 0.0, 4778.0),
            Some((0.0, 4778.0))
        );
        assert_eq!(clamp_x_view(0.0, 5000.0, 0.0, 4778.0), Some((0.0, 4778.0)));
        assert_eq!(clamp_x_view(0.0, 4778.0, 0.0, 4778.0), None);
    }

    /// **视窗完全在数据外 → 贴到最近的数据边界，宽度不变**（拖飞了也停在
    /// 数据边上，缩放档位不被偷偷改掉）。
    #[test]
    fn a_view_entirely_outside_the_data_pins_to_the_nearest_edge() {
        assert_eq!(
            clamp_x_view(6000.0, 6800.0, 0.0, 4778.0),
            Some((3978.0, 4778.0))
        );
        assert_eq!(
            clamp_x_view(-2000.0, -1500.0, 0.0, 4778.0),
            Some((0.0, 500.0))
        );
    }

    /// **退化输入不钳**：空视窗 / 空数据交给上层照常渲染，不制造 NaN。
    #[test]
    fn degenerate_inputs_are_left_alone() {
        assert_eq!(clamp_x_view(500.0, 500.0, 0.0, 4778.0), None);
        assert_eq!(clamp_x_view(600.0, 500.0, 0.0, 4778.0), None);
        assert_eq!(clamp_x_view(0.0, 100.0, 0.0, 0.0), None);
    }
}
