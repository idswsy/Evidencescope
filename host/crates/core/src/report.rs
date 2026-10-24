//! 证据包 —— 将「一次采集中已算出的全部事实」渲染为文本，供语言模型分析。
//!
//! # 输出为文本而非 serde 结构体的原因
//!
//! 结构体加各端各自的渲染器，等于**同一份内容存在两种表述** —— 这正是本项目
//! 反复出现的一类缺陷（协议文档与实现不一致、schema 与参数解析不一致）。
//! 因此本模块只产出**一种形式**：人类可读、语言模型可读、`scope-mcp`
//! 后续亦可直接返回的文本。
//!
//! 文本另有一项实际收益：其 token 开销低于等价 JSON。以通道 0 的峰峰值为例，
//! `"pp_lsb":2867` 为 15 个字符，而 `pp=2867` 为 7 个。
//!
//! # 核心约定：仅转述已算出的结果，不作任何重算
//!
//! 测量取 [`crate::analysis::AnalysisReport`]（其内部调用全项目唯一的
//! [`crate::measure()`]），波形包络取
//! [`Capture::preview()`]（与 MCP 同源的 minmax 降采样）。
//! **本模块不执行任何信号处理。**
//!
//! # 四项必须维持的约束
//!
//! 1. **告警须为未过滤原文**。`I2cDecode::warnings` 中的 `TruncatedFrame`
//!    必须保留 —— 否则模型会把「共 19 帧」当作 19 笔完整事务作答。
//!    （GUI 面板所用的 `warning_lines()` 会**主动滤除**该条，不可用于此处。）
//! 2. **通道来源须如实标注**。GUI 使用 `scl_channel`/`sda_channel` 的
//!    用户选取值，MCP 才执行自动检测。本文本内不得执行 `detect_channels`，
//!    否则将与用户界面所显示的解码结果不一致。
//! 3. **长度须有硬上限**。帧表与包络均可能过长，超出时须**带标记**截断，
//!    不得静默截断 —— 静默截断会被读作「数据到此为止」。
//! 4. **模拟器数据源须显式声明**。缺少该声明时，模型会把模拟器数据
//!    当作真实总线的结论。

use std::fmt::Write as _;

use crate::analysis::EvidenceContext;
use crate::calib::ScaleSet;
use crate::capture::Capture;
use crate::command::DeviceConfig;
use crate::decoder::{DecodeParams, DecodeRequest, DecodeWarning, DecodedFrames, Frame, Protocol};
use crate::i2c_decode::{I2cWarning, Transaction};

/// 波形包络的**目标**桶数。
///
/// 与 MCP 的 `PREVIEW_POINTS`（256）不同：后者服务于 Agent 的工具返回值，
/// 而此处仅为附带上下文，`32` 桶足够看出「这是不是 I2C 该有的样子」，
/// 又不会挤占真正有诊断价值的帧表与信号质量。
///
/// ⚠ 该值为**目标值**，非硬上限。见 [`preview_section`] 的说明：
/// 采集点数较短时，实际桶数可达 `2 * PREVIEW_POINTS - 1`。
///
/// **从 128 降到 32 的原因**：128 桶在短采集上会退化成「每个样点一桶」，
/// 那时 min 恒等于 max，包络变成把整条波形原样喂给模型 —— 正是本模块
/// 要避免的事。32 桶下 168 点的采集每桶 5 点，仍有降采样效果。
pub const PREVIEW_POINTS: usize = 32;

/// 帧表里最多列几帧。超出的在末尾记一条截断说明。
pub const MAX_FRAMES: usize = 64;

/// 整份证据包的字符上限（约合 2300 token）。
///
/// 这是**兜底**，不是主要控制手段 —— 主要控制是上面两个上限。
/// 真触到这条说明有没预料到的膨胀，此时应截断并附说明，不得将完整输出发出。
pub const MAX_CHARS: usize = 8_000;

/// 构造证据包需要、但 [`Capture`] 自己不携带的上下文。
#[derive(Debug, Clone)]
pub struct EvidenceInput<'a> {
    /// 采集本身。
    pub capture: &'a Capture,
    /// 逐通道电压换算，外加「标没标定」的出处。
    ///
    /// 从 `&ChannelScale` 换成 `&ScaleSet`：证据包里那句「是未标定还是
    /// 已标定」必须**如实反映来源**，而不是写死一句「未标定」——
    /// 模型据此决定敢不敢把伏特值当准数用。
    pub scale: &'a ScaleSet,
    /// 链路描述，例如 `sim(i2c_100k)` 或 `COM7 @ 921600`。
    pub link: &'a str,
    /// 数据是不是模拟器产生的。**必须如实填** —— 它决定模型敢不敢下结论。
    pub simulated: bool,
    /// 设备回显的配置。没连上时为 `None`。
    pub config: Option<&'a DeviceConfig>,
    /// 解码结果（任意协议）。没解或通道数不够时为 `None`。
    pub decode: Option<&'a DecodedFrames>,
    /// 解码请求（回显通道/门限/波特率用）。
    pub decode_req: Option<&'a DecodeRequest>,
    /// 用户额外想问的问题。为空则只给证据。
    pub question: Option<&'a str>,
}

/// 把一次采集渲染成证据包文本。
///
/// 纯函数：无 IO、无全局状态、输入相同则输出逐字节相同。
pub fn build_evidence(input: &EvidenceInput<'_>) -> String {
    let report = crate::analysis::AnalysisReport::from_capture(
        input.capture,
        input.scale,
        input.link,
        input.simulated,
    );
    let ctx = crate::analysis::EvidenceContext {
        capture: input.capture,
        scale: input.scale,
        link: input.link,
        simulated: input.simulated,
        config: input.config,
        decode: input.decode,
        decode_req: input.decode_req,
        question: input.question,
    };
    crate::analysis::render_text(&report, &ctx)
}

/// 八段渲染的组装点。**只由 [`crate::analysis::render_text`] 调用**。
pub(crate) fn render_sections(
    report: &crate::analysis::AnalysisReport,
    ctx: &crate::analysis::EvidenceContext<'_>,
) -> String {
    let mut s = String::with_capacity(2048);

    link_section(&mut s, ctx);
    config_section(&mut s, ctx.config);
    capture_section(&mut s, ctx.capture);
    crate::analysis::channels_text(&mut s, report, ctx.capture);
    signal_section(&mut s, ctx);
    preview_section(&mut s, ctx.capture);
    decode_section(&mut s, ctx);
    footer_section(&mut s, ctx);

    truncate_to_cap(s)
}

/// 链路与设备。**第一段就声明模拟器**，不给模型先入为主的机会。
fn link_section(s: &mut String, input: &EvidenceContext<'_>) {
    let _ = writeln!(s, "== 链路与设备 ==");
    if input.simulated {
        let _ = writeln!(
            s,
            "链路: {}   ⚠ 这是【模拟器】产生的数据，不是真实硬件上的总线",
            input.link
        );
    } else {
        let _ = writeln!(s, "链路: {}", input.link);
    }
    let _ = writeln!(
        s,
        "通道数: {}   采样率上限: {} Hz   ADC: 12 bit   单次采集上限: {} 点",
        input.capture.channels.len(),
        crate::f103::MAX_SAMPLE_RATE_HZ,
        crate::f103::MAX_CAPTURE_SAMPLES,
    );
}

/// 设备回显的配置 —— 是**设备返回的实际值**，不是主机请求的值。
fn config_section(s: &mut String, config: Option<&DeviceConfig>) {
    let _ = writeln!(s);
    let _ = writeln!(s, "== 配置（设备回显的实际值）==");
    let Some(c) = config else {
        let _ = writeln!(s, "（未知 —— 本次没读到设备配置）");
        return;
    };
    let _ = writeln!(
        s,
        "实际采样率: {} Hz   采集点数: {}   抽点: {}   采集模式: {}",
        c.rate_hz,
        c.capture_samples,
        c.decimation,
        acq_mode_name(c.acq_mode),
    );
    let _ = writeln!(
        s,
        "触发: 模式={} 边沿={} 源={} 电平={} LSB",
        trigger_mode_name(c.trigger_mode),
        trigger_edge_name(c.trigger_edge),
        channel_label(c.trigger_source as usize),
        c.trigger_level_lsb,
    );
    // 协议里的字段名叫 `ch0_*`（GET_CONFIG 只带 0 号通道的配置），
    // 但**显示名必须跟界面走** —— 界面把索引 0 叫 CH1。
    let _ = writeln!(
        s,
        "{}: 使能={} 耦合={}",
        channel_label(0),
        if c.ch0_enable != 0 { "是" } else { "否" },
        if c.ch0_coupling != 0 { "AC" } else { "DC" },
    );
}

/// 采集本身的元数据。溢出要显式标出来 —— 有它的数据不能假装完整。
fn capture_section(s: &mut String, cap: &Capture) {
    let _ = writeln!(s);
    let _ = writeln!(s, "== 本次采集 ==");
    let n = cap.channels.first().map(|c| c.len()).unwrap_or(0);
    let dur_us = if cap.rate_hz > 0 {
        n as f64 * 1e6 / cap.rate_hz as f64
    } else {
        0.0
    };
    let _ = writeln!(
        s,
        "采集号: {}   实际点数: {}   时长: {:.1} µs @ {} Hz",
        cap.id, n, dur_us, cap.rate_hz
    );
    match cap.trigger_index {
        Some(i) => {
            let _ = writeln!(s, "触发点: 窗口内第 {} 个样点", i);
        }
        None => {
            let _ = writeln!(s, "触发点: 无（软件触发或整窗没等到触发）");
        }
    }
    if cap.overrun {
        let _ = writeln!(
            s,
            "⚠ 期间发生【溢出】：数据不完整，缺口处的波形与解码结论都不可信"
        );
    }
    if n < cap.expected_samples as usize {
        let _ = writeln!(
            s,
            "⚠ 实际点数 {} 少于期望的 {}，说明传输有缺口",
            n, cap.expected_samples
        );
    }
}

/// 通道的显示名。
///
/// **必须与界面一致。** 界面用 `CH{ch + 1}`（`panels.rs` 里到处是 `ch + 1`），
/// 所以索引 0 是 `CH1`。这里曾经直接用索引写成 `ch0` —— 用户在界面上看到的
/// 是 CH1 / CH2，而分析里冒出个 `ch0`，**读起来就是「AI 分析错了」**。
///
/// 这不是措辞问题：同一台设备的同一条通道，两个地方叫两个名字，
/// 用户没有任何办法知道它们指的是同一条线。
pub(crate) fn channel_label(ch: usize) -> String {
    format!("CH{}", ch + 1)
}

/// 波形形状 —— **工具自动判断，不是实测**。
///
/// # 三条纪律
///
/// 1. **表头必须声明这是推断**：分类会作为「事实」进提示词，不标出处的话，
///    模型会把它当实测转述，而用户无从核对。
/// 2. **每个判断必须带依据**（门限、数值、参与计算的 n）。判错了读者要能
///    一眼看出来：数字贴着门限 = 边际薄；数字与标签矛盾 = 判错了。
/// 3. **判不出的要如实说判不出 + 原因** —— 所有措辞是「本工具边界」句式
///    （「未检出」「低于门限」），不是「信号有缺陷」。
///
/// 分类与证据全部来自 [`crate::signal::classify`]，本函数只格式化。
fn signal_section(s: &mut String, input: &EvidenceContext<'_>) {
    let _ = writeln!(s);
    let _ = writeln!(
        s,
        "== 波形形状（工具自动判断，不是实测；依据与判断冲突时以依据为准）=="
    );

    let mut any = false;
    for ch in 0..input.capture.channels.len() {
        let name = channel_label(ch);
        let Some(c) = crate::signal::classify(input.capture, ch) else {
            let _ = writeln!(s, "{name}: 判不出（通道无数据）");
            continue;
        };
        any = true;
        let _ = writeln!(s, "{name}: {}", kind_line(&c));
    }
    if !any {
        let _ = writeln!(s, "（没有可判的通道）");
    }
}

/// 一条通道的分类行：`判成了什么 + 依据`。
///
/// 依据只印**判定路径上真正用过**的两三项 —— 全印出来一行会超预算，
/// 而模型只需要知道「这个判断有多少证据撑着」。
fn kind_line(c: &crate::signal::Classification) -> String {
    use crate::signal::SignalKind;

    let ev = &c.evidence;
    let levels = &ev.levels;

    // 双电平判定的公共依据（只在判定成立时印 —— 判不出路径上的
    // occupancy 是占位值，印出去即成为无依据数值）
    let two_level = |extra: &str| {
        format!(
            "双电平占用率={:.2}(≥0.70)  Vlo={:.0}/Vhi={:.0} LSB{}",
            levels.occupancy, levels.vlo_lsb, levels.vhi_lsb, extra
        )
    };
    let gap_part = |g: &crate::signal::GapStats| format!("间隔cv={:.2}(≤0.10,n={})", g.cv, g.kept);

    match c.kind {
        SignalKind::Dc => format!(
            "直流  依据: AC-RMS={:.1}(≤4.6 LSB)  峰峰={} LSB",
            ev.ac_rms_lsb, ev.pp_lsb
        ),
        SignalKind::Square => {
            let duty = ev.duty_pct.unwrap_or(50.0);
            let g = ev.gaps.as_ref().map(gap_part).unwrap_or_default();
            format!(
                "方波  依据: {}  占空比={duty:.1}%(25–75)  {}",
                two_level(""),
                g
            )
        }
        SignalKind::Pulse => {
            let duty = ev
                .duty_pct
                .map(|d| format!("{d:.1}%"))
                .unwrap_or_else(|| "测不出".into());
            let g = ev.gaps.as_ref().map(gap_part).unwrap_or_default();
            format!(
                "脉冲  依据: {}  占空比={duty}(25–75 之外)  {g}",
                two_level("")
            )
        }
        SignalKind::Step => {
            let over = ev
                .overshoot_pct
                .map(|p| format!("过冲={p:.1}%"))
                .unwrap_or_default();
            format!(
                "阶跃响应  依据: 沿={}升/{}降(实测中点)  {}  {over}",
                ev.rising,
                ev.falling,
                two_level("")
            )
        }
        SignalKind::Sine => {
            let ff = ev.form_factor.unwrap_or(0.0);
            let g = ev.gaps.as_ref().map(gap_part).unwrap_or_default();
            let am = match ev.seg_var {
                Some(v) => format!("调幅检查已做: 滑窗起伏={v:.2}(≤0.25)"),
                None => "调幅检查未做(窗数不足)".to_string(),
            };
            format!("正弦  依据: 波形因数={ff:.3}(0.354±0.045)  {g}  {am}")
        }
        SignalKind::Am => {
            let sv = ev.seg_var.unwrap_or(0.0);
            format!("调幅  依据: 滑窗起伏={sv:.2}(>0.25)")
        }
        SignalKind::Noise => format!("噪声  依据: 上升沿={}(≥8)  未检出稳定周期", ev.rising),
        SignalKind::Unknown(reason) => unknown_line(reason, &c.evidence),
    }
}

/// 判不出时的一行 —— 措辞必须让用户知道「这是工具的边界，不是信号坏了」。
fn unknown_line(
    reason: crate::signal::UncertainReason,
    ev: &crate::signal::ShapeEvidence,
) -> String {
    use crate::signal::UncertainReason::*;
    let head = |name: &str| format!("判不出·{name}");
    match reason {
        TooFewSamples => format!(
            "{}  依据: 实际 {} 点，低于判定下限 16 点 —— 加大采集点数后重判",
            head("样点不足"),
            ev.n
        ),
        TooSmallAmplitude => format!(
            "{}  依据: 峰峰 {} LSB，低于形状判定门限 256 LSB —— 此幅度以下不出形状结论",
            head("幅度太小"),
            ev.pp_lsb
        ),
        NoPeriodicity => format!(
            "{}  依据: 上升沿 {} 个，判定需 ≥4（未检出不等于没有）",
            head("未检出稳定周期"),
            ev.rising
        ),
        IntervalUnstable => {
            let g = ev
                .gaps
                .as_ref()
                .map(|g| format!("间隔cv={:.2}(>0.10,n={})", g.cv, g.kept))
                .unwrap_or_default();
            format!(
                "{}  依据: {} —— 数据/突发信号属正常，不按周期波形归类",
                head("边沿间隔不固定"),
                g
            )
        }
        FrequencyDrift => {
            let d = ev
                .gaps
                .as_ref()
                .map(|g| format!("前后段周期差 {:.0}%(>15%)", g.drift * 100.0))
                .unwrap_or_default();
            format!(
                "{}  依据: {} —— 扫频/变频出现这条是预期行为，不是故障",
                head("窗内频率漂移"),
                d
            )
        }
        Aliased => {
            let p = ev
                .period_samples
                .map(|p| format!("{p:.1} 样点(下限 3)"))
                .unwrap_or_default();
            format!(
                "{}  依据: 周期仅 {} —— 采样密度不足，看到的形状不代表真实波形",
                head("采样密度不足"),
                p
            )
        }
        ShapeUnrecognized => {
            let ff = ev
                .form_factor
                .map(|f| format!("波形因数={f:.3}"))
                .unwrap_or_default();
            format!(
                "{}  依据: {} —— 本工具只判方波/脉冲/正弦/调幅/阶跃/噪声",
                head("形状不在可判范围"),
                ff
            )
        }
    }
}

/// 波形包络。**必须写明「桶内细节不可见」**，否则模型会对着 128 个数编形状。
///
/// ⚠ 桶数是**约** `PREVIEW_POINTS`，不是「最多」：[`Capture::preview`] 里
/// `bucket = (len / target).max(1)`，所以当 `len` 落在 `[target, 2*target)` 时
/// 桶退化到 1，实际桶数能到 `2*target - 1`。这里如实打印**实际**桶数，
/// 不要在表头上写一个会被实际数据推翻的上限。
fn preview_section(s: &mut String, cap: &Capture) {
    let _ = writeln!(s);
    let _ = writeln!(
        s,
        "== 波形包络（每桶的 min/max，目标 {PREVIEW_POINTS} 桶）=="
    );
    let _ = writeln!(
        s,
        "⚠ 每个桶里装了多个样点，只有 min/max 被保留 —— 桶内细节不可见，"
    );
    let _ = writeln!(s, "  不要对桶内的毛刺、振铃、边沿形状下结论。");

    let mut wrote = false;
    for ch in 0..cap.channels.len() {
        let Some(p) = cap.preview(ch, PREVIEW_POINTS) else {
            continue;
        };
        wrote = true;
        let _ = writeln!(
            s,
            "{}  桶宽={:.2} µs  桶数={}",
            channel_label(ch),
            p.dt_us,
            p.y_min.len()
        );
        let _ = writeln!(s, "  min: {}", join_u16(&p.y_min));
        let _ = writeln!(s, "  max: {}", join_u16(&p.y_max));
    }
    if !wrote {
        let _ = writeln!(s, "（没有可用的包络）");
    }
}

/// 总线解码结果 —— 帧表 + **未过滤的**告警原文。
///
/// 段首声明**场景**（有解码 / 无解码），供模型选择分析分支。
/// 场景是**分类**，不是重算 —— 数据里本来就知道的事。
///
/// # 段名为什么不是「I2C 解码」
///
/// 2026-10-06 真机回归：屏幕上给一个 2 kHz 方波，模型的三段回答里两段在讲
/// 「缺 SDA，I2C 解码做不了」—— 而段名写着「I2C 解码」、无解码时还要求
/// 模型解释为什么解不出来，注意力被整个钉在 I2C 上。段名改成「总线解码」，
/// 无解码时**一句话带过**，波形本身的分析交给上一段（波形形状 / 测量）。
fn decode_section(s: &mut String, input: &EvidenceContext<'_>) {
    let _ = writeln!(s);
    let _ = writeln!(s, "== 总线解码 ==");

    let Some(d) = input.decode else {
        // 「场景」行不能没有 —— 它是模型选分析分支的依据（提示词里同名）。
        // 但**不再要求解释为什么解不出来**：非 I2C 波形没有解码结果就是
        // 正常状态，不是故障；原因栏只会把模型拉回 I2C 那一套。
        let _ = writeln!(s, "场景: 模拟信号（无总线解码）");
        return;
    };

    match d.protocol {
        Protocol::I2c => decode_i2c_section(s, d, input.decode_req),
        Protocol::Uart => decode_uart_section(s, d, input.decode_req),
    }
}

/// I²C 解码段 —— 保持既有措辞逐字节不变（B3 回归的一部分）。
fn decode_i2c_section(s: &mut String, d: &DecodedFrames, req: Option<&DecodeRequest>) {
    let _ = writeln!(s, "场景: I2C 总线解码");

    if let Some(DecodeRequest {
        params: DecodeParams::I2c(cfg),
        ..
    }) = req
    {
        // 说明通道是**用户选的**，不是自动检测的 —— 这一点影响模型对结论的信任度
        let _ = writeln!(
            s,
            "SCL={}  SDA={}   （通道是用户在界面上选的，不是自动检测的结果）",
            channel_label(cfg.scl_channel),
            channel_label(cfg.sda_channel)
        );
        let _ = writeln!(
            s,
            "判决门限: VIH={} LSB  VIL={} LSB   去抖: {} ns",
            cfg.levels.vih_lsb, cfg.levels.vil_lsb, cfg.debounce_ns
        );
    }

    let transactions: Vec<&Transaction> = d
        .frames
        .iter()
        .filter_map(|f| match f {
            Frame::I2c(t) => Some(t),
            Frame::Uart(_) => None,
        })
        .collect();

    let _ = writeln!(
        s,
        "帧数: {}   解码可信: {}",
        transactions.len(),
        if i2c_untrustworthy(&d.warnings) {
            "否"
        } else {
            "是"
        }
    );

    let truncated = transactions.iter().filter(|t| !t.complete).count();
    if transactions.is_empty() {
        let _ = writeln!(s, "（一帧都没解出来）");
    } else {
        let _ = writeln!(s, "帧表（每个地址 / 数据字节后的标记）:");
        let _ = writeln!(
            s,
            "  +A = 从机应答    +N = 从机未应答（可能是故障）    \
             +E = 读事务末字节的主机收尾 NACK（正常）"
        );
        for (i, t) in transactions.iter().take(MAX_FRAMES).enumerate() {
            let payload = frame_payload(t);
            let _ = writeln!(
                s,
                "  #{:<3} t={:>9.3} ms  {}{}  {}",
                i + 1,
                t.start_time_us as f64 / 1000.0,
                if t.repeated { "Sr " } else { "" },
                payload,
                if t.complete { "" } else { "⚠被窗口截断" },
            );
        }
        let extra = transactions.len().saturating_sub(MAX_FRAMES);
        if extra > 0 {
            // 不静默截断：说清楚还剩多少没列
            let _ = writeln!(s, "  …… 另有 {extra} 帧未列出（超出 {MAX_FRAMES} 帧上限）");
        }
    }

    if truncated > 0 {
        let _ = writeln!(
            s,
            "⚠ 这 {truncated} 帧没有正常 STOP 收尾 —— I2C 总线一直在跑，采集窗口切在哪是随机的，"
        );
        let _ = writeln!(
            s,
            "  所以最后一帧被切断是正常的。**不要把「帧数」当成「完整事务笔数」。**"
        );
    }

    if !d.warnings.is_empty() {
        let _ = writeln!(s, "告警（原文，未过滤）:");
        for w in &d.warnings {
            let _ = writeln!(s, "  - {}", w.text());
        }
    }
}

/// UART 解码段 —— 与 I²C 共用「总线解码」段名，协议名写进场景行。
fn decode_uart_section(s: &mut String, d: &DecodedFrames, req: Option<&DecodeRequest>) {
    let _ = writeln!(s, "场景: UART 8N1 串行解码");

    if let Some(DecodeRequest {
        params: DecodeParams::Uart(cfg),
        channels,
        ..
    }) = req
    {
        let ch = channels.first().copied().unwrap_or(0);
        let margin = d.quality.sample_margin.unwrap_or(0.0);
        let _ = writeln!(
            s,
            "通道: {}   波特率: {}   每比特样点: {:.2}（低于 4 不出帧）",
            channel_label(ch),
            cfg.baud,
            margin
        );
    }

    let _ = writeln!(
        s,
        "帧数: {}   解码可信: {}",
        d.frame_count(),
        if d.quality.frame_errors == 0 {
            "是"
        } else {
            "否"
        }
    );

    let bytes = d.uart_bytes().unwrap_or_default();
    if !bytes.is_empty() {
        let take = bytes.len().min(16);
        let hex: Vec<String> = bytes[..take].iter().map(|b| format!("{b:02X}")).collect();
        let more = if bytes.len() > take {
            format!(" ……（共 {} 字节）", bytes.len())
        } else {
            String::new()
        };
        let _ = writeln!(s, "字节序列: {}{more}", hex.join(" "));
    }

    s.push_str(&d.uart_body_text());
}

/// I²C 告警里哪些属于「结果不可信」级别。
fn i2c_untrustworthy(warnings: &[DecodeWarning]) -> bool {
    warnings.iter().any(|w| {
        matches!(
            w,
            DecodeWarning::I2c(
                I2cWarning::NoSignal
                    | I2cWarning::LevelsInvalid { .. }
                    | I2cWarning::SclStuck { .. }
            )
        )
    })
}

/// 结尾：把「你该怎么用这些数字」说死，防止模型自由发挥。
fn footer_section(s: &mut String, input: &EvidenceContext<'_>) {
    let _ = writeln!(s);
    let _ = writeln!(s, "== 说明 ==");
    if input.simulated {
        let _ = writeln!(
            s,
            "- 以上是【模拟器】数据。可以分析信号与协议本身，但**不能**据此判断真实硬件好坏。"
        );
    }
    // 满量程 = 每 LSB 的伏特数 × 4096。别写死 3.3 —— 标定一旦变了这里就错了。
    // 三种状态**必须分开说**：说成「已标定」会让模型把占位值当准数，
    // 说成「未标定」则会把标定过的数据白白降级。
    let uid = input.scale.uid_hex().unwrap_or_else(|| "?".into());
    if input.scale.all_calibrated() {
        let full_scale = input.scale.get(0).volts_per_lsb * 4096.0;
        let _ = writeln!(
            s,
            "- 电压按设备标定（uid {uid}）换算，满量程 {full_scale:.2} V。"
        );
    } else if input.scale.record_found() {
        let full_scale = input.scale.get(0).volts_per_lsb * 4096.0;
        let _ = writeln!(
            s,
            "- 电压**部分未标定**：uid {uid} 的记录里缺部分通道，缺的那些按 {full_scale:.2} V 满量程的占位值换算。标定过的通道可以当准数，其余只看相对关系。"
        );
    } else {
        let full_scale = input.scale.get(0).volts_per_lsb * 4096.0;
        let _ = writeln!(
            s,
            "- 电压是按 {full_scale:.2} V 满量程做的**未标定**换算，只看相对关系，不要当成校准值。"
        );
    }
    let _ = writeln!(
        s,
        "- 上面所有数字都来自精确计算，**不要自己估算或重新推导**；\
         它们是判断依据，不是要复述的内容。"
    );
    let _ = writeln!(
        s,
        "- 无法测得的量应如实标注「测不出」。需要更多信息时，明确指出需要什么（例如「需要换个场景重采」）。"
    );

    if let Some(q) = input.question {
        let q = q.trim();
        if !q.is_empty() {
            let _ = writeln!(s);
            let _ = writeln!(s, "== 用户的问题 ==");
            let _ = writeln!(s, "{q}");
        }
    }
}

/// 把一帧渲染成「地址 / 数据字节 + 应答位」的紧凑串。
///
/// # 为什么应答位必须出现
///
/// **对 I2C 诊断来说，从机有没有应答是最要紧的一位。**
/// 本项目 P3 的验收场景就是「告诉我为什么 NACK」—— 帧表里没有这一位，
/// 证据包就根本答不了那个问题。
///
/// 回归：这张表曾经只印地址和字节值，把 `acked` 整个丢了，
/// 而 `to_text()`（GUI 导出用的那份）是一直带着它的 —— 两份描述又漂了。
/// **是一次真跑的模型回复指出来的**：它说「帧表未列出 ACK/NAK 位，
/// 从机是否应答无法从证据确认」。这种「缺失」只有拿真实数据跑才看得见。
///
/// # 三种标记，不是一个「未应答」
///
/// 第二次真跑又暴露了一层：模型看到读帧末尾的 `+N` 说
/// 「读事务末尾的 N 被标为从机未应答……按 I2C 惯例那由主机发出，属正常收尾」。
/// **它说得对** —— 而我的图例把 `+N` 一概解释成「从机未应答」，**在读帧末尾是错的**。
///
/// 这正是 `Transaction::has_nack` 的文档里预警过的事：
/// 「一份完全健康的读数看起来像出了错，一个 Agent 几乎必然会据此报出一个
/// 不存在的问题」。我加应答位的时候把那个修复又绕过去了。
///
/// 所以标记分三种，判断规则**复用** [`Transaction::is_master_terminating_nack`]：
///
/// | 标记 | 含义 |
/// |---|---|
/// | `+A` | 从机应答 |
/// | `+N` | **从机**未应答 —— 这才可能是故障 |
/// | `+E` | 读事务末字节由**主机**回的 NACK，正常收尾（E = End） |
fn frame_payload(t: &Transaction) -> String {
    use crate::i2c_decode::EventKind;
    let mut s = String::new();
    for (i, ev) in t.events.iter().enumerate() {
        match &ev.kind {
            EventKind::Address(a) => {
                let _ = write!(
                    s,
                    "0x{:02X}({}){}  ",
                    a.value,
                    if a.read { "R" } else { "W" },
                    if a.acked { "+A" } else { "+N" }
                );
            }
            EventKind::Data { value, acked } => {
                let mark = if *acked {
                    "+A"
                } else if t.is_master_terminating_nack(i) {
                    "+E"
                } else {
                    "+N"
                };
                let _ = write!(s, "{value:02X}{mark}  ");
            }
            // START / STOP / 截断标记在帧级别已经表达过了，不重复
            _ => {}
        }
    }
    s.trim_end().to_string()
}

fn join_u16(v: &[u16]) -> String {
    let mut s = String::with_capacity(v.len() * 5);
    for (i, x) in v.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        let _ = write!(s, "{x}");
    }
    s
}

fn acq_mode_name(m: u8) -> &'static str {
    match m {
        0 => "single",
        1 => "stream",
        _ => "?",
    }
}

fn trigger_mode_name(m: u8) -> &'static str {
    match m {
        0 => "auto",
        1 => "normal",
        2 => "single",
        _ => "?",
    }
}

fn trigger_edge_name(e: u8) -> &'static str {
    match e {
        0 => "rising",
        1 => "falling",
        _ => "?",
    }
}

/// 兜底截断。**带标记**，不静默。
fn truncate_to_cap(s: String) -> String {
    if s.chars().count() <= MAX_CHARS {
        return s;
    }
    let mut out: String = s.chars().take(MAX_CHARS).collect();
    out.push_str(&format!(
        "\n\n⚠ 证据包超过 {MAX_CHARS} 字符上限，**上面是截断后的内容** —— 缺失的部分请当作未知。"
    ));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::i2c_decode::{decode_capture, I2cDecodeConfig};

    const HIGH: u16 = 4095;
    const LOW: u16 = 0;

    /// 造一份两通道采集：SCL=ch0、SDA=ch1，解得出 `0x88 (W)` + 一个数据字节。
    ///
    /// `with_stop = false` 时**砍掉收尾的 STOP** —— 这样最后一帧没有正常收尾，
    /// 用来验证「截断帧没有被吞掉」。
    fn i2c_capture(with_stop: bool) -> Capture {
        let rate = 800_000u32;
        let mut scl: Vec<u16> = Vec::new();
        let mut sda: Vec<u16> = Vec::new();

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
        for b in [0x88u8, 0x00] {
            for i in (0..8).rev() {
                bit!((b >> i) & 1 == 1);
            }
            bit!(false); // ACK
        }
        if with_stop {
            hold!(LOW, LOW, 4);
            hold!(HIGH, LOW, 4);
            hold!(HIGH, HIGH, 4);
        }

        let mut cap = Capture::new(1, rate, 2, scl.len() as u32);
        cap.channels = vec![scl, sda];
        cap
    }

    /// 造一份**地址被应答、但随后的数据字节被 NACK** 的采集。
    ///
    /// ACK 与 NACK 的区别只在第 9 个时钟的 SDA 电平：从机拉低 = 应答，
    /// 放手（保持高）= 未应答。所以这里地址位之后送 `bit!(false)`、
    /// 数据位之后送 `bit!(true)`。
    fn nacked_i2c_capture() -> Capture {
        let rate = 800_000u32;
        let mut scl: Vec<u16> = Vec::new();
        let mut sda: Vec<u16> = Vec::new();

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
        macro_rules! byte_with_ack {
            ($b:expr, $acked:expr) => {{
                for i in (0..8).rev() {
                    bit!(($b >> i) & 1 == 1);
                }
                bit!(!$acked); // 应答 = SDA 拉低；未应答 = 放手保持高
            }};
        }

        hold!(HIGH, HIGH, 4);
        hold!(HIGH, LOW, 4); // START
        hold!(LOW, LOW, 4);
        byte_with_ack!(0x88u8, true); // 地址：被应答
        byte_with_ack!(0x00u8, false); // 数据：被 NACK
        hold!(LOW, LOW, 4);
        hold!(HIGH, LOW, 4);
        hold!(HIGH, HIGH, 4); // STOP

        let mut cap = Capture::new(9, rate, 2, scl.len() as u32);
        cap.channels = vec![scl, sda];
        cap
    }

    /// 造一份最常见的 I2C 时序：**写寄存器 → 重复起始 → 读两字节**。
    ///
    /// 读事务的最后一字节由**主机**回 NACK（「我读够了」）—— 那是正常收尾。
    /// `nacked_i2c_capture` 造的是另一回事：从机拒绝了写数据。
    fn read_transaction_capture() -> Capture {
        let rate = 800_000u32;
        let mut scl: Vec<u16> = Vec::new();
        let mut sda: Vec<u16> = Vec::new();

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
        macro_rules! byte_with_ack {
            ($b:expr, $acked:expr) => {{
                for i in (0..8).rev() {
                    bit!(($b >> i) & 1 == 1);
                }
                bit!(!$acked);
            }};
        }

        // START：SCL 高时 SDA 由高变低
        hold!(HIGH, HIGH, 4);
        hold!(HIGH, LOW, 4);
        hold!(LOW, LOW, 4);
        // 写：地址 0x44(W) + 寄存器号 0x00
        byte_with_ack!(0x88u8, true);
        byte_with_ack!(0x00u8, true);
        // 重复起始。
        //
        // ⚠ 别和 STOP 搞混：两者都在 SCL 高时动 SDA，
        // **Sr 是「高→低」、STOP 是「低→高」**。
        // 上一版这里写成了低→高，于是解出来是个 STOP，整个读帧都没了 ——
        // 测试直接报「没有读帧」。
        hold!(LOW, HIGH, 4); // SCL 低时先把 SDA 放回高
        hold!(HIGH, HIGH, 4); // SCL 拉高（此时总线处于空闲态的样子）
        hold!(HIGH, LOW, 4); // ← Sr：SCL 高时 SDA 由高变低
        hold!(LOW, LOW, 4);
        // 读：地址 0x44(R)，两个字节，最后一字节由主机回 NACK
        byte_with_ack!(0x89u8, true);
        byte_with_ack!(0x01u8, true);
        byte_with_ack!(0x2Cu8, false); // ← 主机收尾 NACK
                                       // STOP：SCL 高时 SDA 由低变高
        hold!(LOW, LOW, 4);
        hold!(HIGH, LOW, 4);
        hold!(HIGH, HIGH, 4);

        let mut cap = Capture::new(11, rate, 2, scl.len() as u32);
        cap.channels = vec![scl, sda];
        cap
    }

    /// 造一份直流采集 —— 没有边沿，所以频率/占空比/上升时间都测不出。
    fn flat_capture() -> Capture {
        let mut cap = Capture::new(2, 800_000, 2, 512);
        cap.channels = vec![vec![2048u16; 512], vec![2048u16; 512]];
        cap
    }

    /// 证据包输入。换算固定为**未标定** —— 这些测试验的是证据包的结构与
    /// 各项上限，不是换算本身；换算的测试在 `calib::tests` 里。
    fn input<'a>(cap: &'a Capture, scale: &'a ScaleSet) -> EvidenceInput<'a> {
        EvidenceInput {
            capture: cap,
            scale,
            link: "sim(i2c_100k)",
            simulated: true,
            config: None,
            decode: None,
            decode_req: None,
            question: None,
        }
    }

    /// 测试用：把既有 `I2cDecode` + 配置组装成新的统一出口类型。
    fn req_i2c(cfg: &I2cDecodeConfig) -> crate::decoder::DecodeRequest {
        crate::decoder::DecodeRequest {
            protocol: crate::decoder::Protocol::I2c,
            channels: vec![],
            params: crate::decoder::DecodeParams::I2c(cfg.clone()),
        }
    }

    // ── 坑 1 的回归：告警必须用未过滤的原文 ──────────────────────────

    /// **这是本文件最重要的一条测试。**
    ///
    /// `I2cDecode::warnings` 里那条 `TruncatedFrame` 必须活到文本里。
    /// GUI 面板上那层 `vmodel::warning_lines()` 是**故意滤掉它**的
    /// （滤掉的理由写在 vmodel.rs:163-166：截断每采必有，标黄等于天天报警），
    /// 所以这里用错层的话，模型会把「共 N 帧」当成 N 笔完整事务来回答。
    ///
    /// 断的是**行为**不是措辞：只要「截断帧」这件事被说出来了就算过。
    #[test]
    fn truncated_frame_survives_into_the_text() {
        let cap = i2c_capture(false); // 砍掉 STOP → 最后一帧没收尾
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).expect("应能解码");

        // 前提：这份数据里确实有截断帧和对应告警，否则这条测试是假通过的
        assert!(
            d.transactions.iter().any(|t| !t.complete),
            "测试数据本身就没有截断帧 —— 这条测试失去意义"
        );
        assert!(
            d.warnings
                .iter()
                .any(|w| matches!(w, crate::i2c_decode::I2cWarning::TruncatedFrame)),
            "解码器没有给出 TruncatedFrame 告警 —— 测试数据不对"
        );

        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let cfg = I2cDecodeConfig::default();
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(
            text.contains("截断") || text.contains("没有正常 STOP"),
            "截断帧在证据包里消失了 —— 模型会把帧数当成完整事务数\n{text}"
        );
        assert!(
            text.contains("不要把「帧数」当成「完整事务笔数」"),
            "缺失了那条防止误读的说明"
        );

        // ⚠ 上面两条**不足以**证明告警没被过滤：帧表的「⚠被窗口截断」和
        // `count_truncated()` 走的是 `t.complete`，是**独立于 warnings 的另一条路径**。
        // 真把 `d.warnings` 换成 `vmodel::warning_lines()`（那个会滤掉
        // TruncatedFrame 的版本）上面两条照样过。
        //
        // 所以再钉一句**只有未过滤的告警才会产生**的原文 —— 这条才真正守住坑 1。
        assert!(
            text.contains("采集窗口在帧中间结束，最后一帧不完整"),
            "告警原文被过滤掉了。这就是把 `I2cDecode::warnings` 错换成 \
             `vmodel::warning_lines()` 的症状\n{text}"
        );
    }

    #[test]
    fn complete_frames_do_not_get_a_truncation_note() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).expect("应能解码");
        assert!(
            d.transactions.iter().all(|t| t.complete),
            "测试数据里还有没收尾的帧 —— 这条测试失去意义"
        );

        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let cfg = I2cDecodeConfig::default();
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(
            !text.contains("不要把「帧数」当成「完整事务笔数」"),
            "全收尾的数据不该出现截断说明（否则是天天响的假警报）\n{text}"
        );
    }

    // ── 应答位：I2C 诊断里最要紧的一位 ───────────────────────────────

    /// **帧表必须带从机应答位。**
    ///
    /// 回归：这张表曾经只印地址与字节值，把 `acked` 整个丢了。后果不是
    /// 「少了个字段」，而是**证据包根本答不了本项目 P3 的验收问题
    /// 「告诉我为什么 NACK」** —— 模型只能回一句「从机是否应答无法从证据确认」。
    ///
    /// 发现方式值得记：它不是被测试抓到的，是**一次真跑的模型回复**指出来的。
    /// 「信息缺失」这类问题，代码读一百遍也看不出来 —— 因为缺的东西不在那里。
    /// ⚠ **必须断言在数据行上，不能断言整段文本。**
    ///
    /// 第一版这两条测试是这么写的：`assert!(text.contains("+A"))`。
    /// 看起来没问题 —— 但帧表上面那行**图例**里就写着「+A = 从机应答」，
    /// 于是把 `ack_mark()` 改成恒返回空串，断言照样通过。
    /// **测试证明不了它要证明的事**，是变异测试把它抓出来的。
    ///
    /// 所以这里直接对 [`frame_payload`] 断言 —— 那个函数里没有图例，
    /// 只有真正的数据。
    fn payload_of(cap: &Capture) -> Vec<String> {
        let d = decode_capture(cap, &I2cDecodeConfig::default()).unwrap();
        d.transactions.iter().map(frame_payload).collect()
    }

    #[test]
    fn frame_table_carries_the_acknowledge_bit() {
        let cap = i2c_capture(true);
        let payloads = payload_of(&cap);
        assert!(!payloads.is_empty(), "测试数据没解出帧");

        // 数据行里必须真的出现应答标记
        assert!(
            payloads.iter().any(|p| p.contains("+A")),
            "数据行里没有应答位 —— 模型无法判断从机是否应答：{payloads:?}"
        );
        // **每一个**地址 / 数据字节后面都要有，不能只标一个。
        //
        // （这里第一版写错过：拿「以 0x 开头的 token 数」去比「带标记的 token 数」，
        //   而一帧里地址和数据字节各一个 —— 2 ≠ 1 就被判红了。
        //   要断言的是「所有 token 都带标记」，不是两个计数相等。）
        let tokens: Vec<&str> = payloads.iter().flat_map(|p| p.split_whitespace()).collect();
        assert!(!tokens.is_empty(), "数据行是空的：{payloads:?}");
        for t in &tokens {
            assert!(
                t.ends_with("+A") || t.ends_with("+N"),
                "「{t}」没有带应答位 —— 有字节被漏掉了：{payloads:?}"
            );
        }
    }

    /// NACK 必须能被看出来，且**不能和 ACK 混淆**。
    ///
    /// 用带 NACK 的数据（地址被应答、随后的写被拒）验一遍：
    /// 同一个帧里应当同时出现 `+A` 与 `+N`。
    #[test]
    fn a_nack_is_visible_and_distinguishable() {
        let cap = nacked_i2c_capture();
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();

        // 前提：这份数据里确实有 NACK，否则测试是假通过的
        assert!(
            d.transactions.iter().any(|t| t.has_nack()),
            "测试数据里没有 NACK —— 这条测试失去意义"
        );

        let joined = d
            .transactions
            .iter()
            .map(frame_payload)
            .collect::<Vec<_>>()
            .join(" | ");

        assert!(joined.contains("+N"), "NACK 没有体现出来：{joined}");
        assert!(joined.contains("+A"), "ACK 也应当照样标出来：{joined}");
        // 地址那一笔是 ACK、数据那一笔是 NACK —— 两者必须能分辨。
        //
        // 注意地址是 `0x44`：测试数据里送的是 `0x88`，那是**带读写位的地址字节**，
        // 解码器剥掉最低位之后得到 7 位地址 0x44。第一版这里写成 0x88，判红了 ——
        // 又是**期望写错**而不是代码错。
        assert!(joined.contains("0x44(W)+A"), "地址应当标为被应答：{joined}");
        assert!(joined.contains("00+N"), "数据字节应当标为未应答：{joined}");
    }

    /// 图例本身也要在（数据对但没人看得懂记号，等于没给）。
    #[test]
    fn the_frame_table_explains_its_ack_notation() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let cfg = I2cDecodeConfig::default();
        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(text.contains("从机应答"), "缺图例：{text}");
        assert!(text.contains("从机未应答"), "缺图例：{text}");
        assert!(
            text.contains("主机收尾"),
            "缺「+E 是主机收尾 NACK」这一条 —— 少了它，读帧末尾的 NACK \
             会被当成从机故障：{text}"
        );
    }

    /// **读事务末尾的 NACK 不能标成「从机未应答」。**
    ///
    /// 这是第二次真跑暴露出来的：模型看到读帧末尾的 `+N` 说
    /// 「读操作最后一个字节的 ACK/NACK 由主机发出，NACK 表示结束读取，
    /// 属正常收尾，不宜据此判从机故障」—— **它说得对**。
    ///
    /// 一份完全健康的「写寄存器 → 读回」读数，末尾那个 NACK 是主机发的，
    /// 标成从机故障等于**报一个不存在的问题**。`has_nack()` 的文档里
    /// 早就预警过这件事，是我加应答位时绕过去了。
    #[test]
    fn a_read_frames_trailing_nack_is_not_blamed_on_the_slave() {
        // 「写 0x44 → 重复起始 → 读」—— 最常见的 I2C 时序
        let cap = read_transaction_capture();
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();

        let read_frame = d
            .transactions
            .iter()
            .find(|t| t.address.as_ref().is_some_and(|a| a.read))
            .expect("测试数据里没有读帧 —— 这条测试失去意义");

        let payload = frame_payload(read_frame);
        assert!(
            payload.contains("+E"),
            "读帧末尾的主机收尾 NACK 应当标成 +E：{payload}"
        );
        assert!(
            !read_frame.events.iter().enumerate().any(|(i, e)| matches!(
                &e.kind,
                crate::i2c_decode::EventKind::Data { acked: false, .. }
            ) && !read_frame
                .is_master_terminating_nack(i)),
            "读帧里不该有「从机未应答」"
        );
        // 而且这一帧整体不算故障
        assert!(
            !read_frame.has_nack(),
            "一份健康的读事务不该被判定为有 NACK"
        );
    }

    // ── 坑 2 的回归：通道是用户选的就要说 ────────────────────────────

    #[test]
    fn user_selected_channels_are_disclosed() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let cfg = I2cDecodeConfig::new(1, 0); // 故意反接，验证打印的是配置值
        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(text.contains("SCL=CH2"), "应打印配置里的通道：{text}");
        assert!(text.contains("SDA=CH1"), "应打印配置里的通道：{text}");
        assert!(
            text.contains("用户在界面上选"),
            "必须说明通道是用户选的而不是自动检测的"
        );
    }

    /// **通道编号必须与界面一致。**
    ///
    /// 回归：证据包里曾经直接用 0 起的索引写成 `ch0` / `ch1`，而界面
    /// （`panels.rs` 里的 `CH{ch + 1}`）显示的是 `CH1` / `CH2`。
    /// 用户测的是 CH1 和 CH2，分析里冒出个 `ch0` —— **读起来就是「分析错了」**，
    /// 而模型其实什么都没算错，是这份文本把通道叫错了名字。
    ///
    /// 这条测试钉死：文本里**不得出现 0 起的 `chN` 写法**。
    #[test]
    fn channel_names_match_the_ui_numbering() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let cfg = I2cDecodeConfig::default();
        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(text.contains("CH1"), "两通道采集应出现 CH1：{text}");
        assert!(text.contains("CH2"), "两通道采集应出现 CH2：{text}");
        for bad in ["ch0", "ch1", "ch2", "CH0"] {
            assert!(
                !text.contains(bad),
                "证据包里出现了界面不存在的写法 `{bad}` —— \
                 界面用的是 CH1/CH2（1 起），必须一致\n{text}"
            );
        }
    }

    /// 设备配置段的通道名走 `channel_label`，不是协议字段名。
    ///
    /// 回归：这里曾直接写死 `"CH0: 使能=…"` —— 协议字段确实叫 `ch0_enable`，
    /// 但**界面把索引 0 叫 CH1**，于是又一次「分析里出现界面不存在的通道」。
    #[test]
    fn the_config_section_uses_the_display_name_not_the_protocol_field_name() {
        let cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let cfg = DeviceConfig {
            rate_hz: 857_142,
            ch0_enable: 1,
            ..DeviceConfig::default()
        };
        let mut i = input(&cap, &scale);
        i.config = Some(&cfg);
        let text = build_evidence(&i);

        assert!(
            text.contains("CH1: 使能=是"),
            "配置段应使用界面的 CH1 名称：{text}"
        );
        assert!(!text.contains("CH0"), "不得出现 CH0：{text}");
    }

    /// **短采集不得退化成「每个样点一桶」。**
    ///
    /// 回归：`PREVIEW_POINTS` 曾是 128，而 168 点的采集算出来
    /// `bucket = 168/128 = 1` —— 每个样点自成一桶，`min` 恒等于 `max`，
    /// 所谓「降采样包络」变成了**把整条波形原样发给模型**。
    ///
    /// 判据：桶数必须**显著少于**样点数，否则就不是降采样。
    #[test]
    fn a_short_capture_is_still_downsampled() {
        let cap = i2c_capture(true);
        let n = cap.channels[0].len();
        let buckets = cap.preview(0, PREVIEW_POINTS).unwrap().y_min.len();

        assert!(
            buckets < n,
            "采集 {n} 点却排出 {buckets} 桶 —— 没有降采样，等于把波形原样发出去"
        );
        // 多数桶应当真的装了多个样点
        assert!(
            buckets * 2 <= n || buckets <= 64,
            "采集 {n} 点排出 {buckets} 桶，压缩比不足：{n}/{buckets}"
        );
    }

    // ── 坑 3：长度上限 ───────────────────────────────────────────────

    #[test]
    fn oversize_output_is_truncated_with_a_marker() {
        let huge = "x".repeat(MAX_CHARS + 500);
        let out = truncate_to_cap(huge);
        assert!(out.contains("截断后的内容"), "超限必须带标记，不能静默截断");
        assert!(
            out.chars().count() <= MAX_CHARS + 200,
            "截断后还是太长：{}",
            out.chars().count()
        );
    }

    #[test]
    fn under_cap_output_is_left_alone() {
        let small = "x".repeat(MAX_CHARS - 1);
        assert_eq!(
            truncate_to_cap(small).chars().count(),
            MAX_CHARS - 1,
            "没超限就不该动它"
        );
    }

    /// **一份 4096 点的真实采集，证据包有多大。**
    ///
    /// 这条是有用的刻度尺：成本讨论里反复要引用这个数，
    /// 而拍脑袋估的和实际量的差得很远（我估过 2000+ 字符，实测见下）。
    #[test]
    fn a_4096_point_capture_measures_its_own_size() {
        let mut cap = Capture::new(1, 857_142, 2, 4096);
        // 造一条像 I2C 的方波，好让解码与包络都不是空的
        cap.channels = vec![
            (0..4096)
                .map(|i| if (i / 16) % 2 == 0 { 4095 } else { 0 })
                .collect(),
            (0..4096)
                .map(|i| if (i / 64) % 2 == 0 { 4095 } else { 0 })
                .collect(),
        ];
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).ok();
        let cfg = I2cDecodeConfig::default();
        let mut i = input(&cap, &scale);
        let frames = d.as_ref().map(|x| crate::to_decoded(x.clone()));
        i.decode = frames.as_ref();
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        let chars = text.chars().count();
        println!("4096 点双通道证据包：{chars} 字符，约 {} token", chars / 3);

        // 上界：一份正常采集不该逼近 MAX_CHARS，否则截断会天天发生
        assert!(
            chars < MAX_CHARS / 2,
            "4096 点的证据包 {chars} 字符，已经逼近 {MAX_CHARS} 上限"
        );
        // 下界：太小说明有段落整段没写出来
        assert!(chars > 500, "证据包只有 {chars} 字符 —— 像是漏了段");
    }

    /// 真实采集渲染出来必须远低于上限 —— 否则「省略号」会天天出现。
    #[test]
    fn a_real_capture_fits_comfortably() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let cfg = I2cDecodeConfig::default();
        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(
            text.chars().count() < MAX_CHARS,
            "一份普通采集就超了上限（{} 字符）—— 上限定错了或哪里膨胀了",
            text.chars().count()
        );
    }

    // ── 波形形状段（2026-10-06 新增）─────────────────────────────────

    /// 每个通道都有一行分类，且**必须带依据**。
    ///
    /// 分类会作为「事实」进提示词 —— 不带依据的话，判错了谁也看不出来。
    ///
    /// 变异（杀掉）：`kind_line` 只印标签不印依据 —— 这条的「依据」断言红。
    #[test]
    fn every_channel_gets_a_classification_with_evidence() {
        let cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let text = build_evidence(&input(&cap, &scale));
        assert!(text.contains("== 波形形状"), "证据包必须有波形形状段");
        assert!(text.contains("CH1: "), "每个通道都要有一行分类");
        assert!(
            text.contains("依据:"),
            "分类必须带依据 —— 判错了读者要能一眼看出来"
        );
        assert!(
            text.contains("工具自动判断，不是实测"),
            "表头必须声明这是推断，不是实测"
        );
    }

    /// 判不出时，原因必须带着**数字**（实际值 vs 门限）。
    ///
    /// 变异（杀掉）：把 `unknown_line` 里的数值格式删成纯文字 —— 这条红。
    #[test]
    fn an_unknown_verdict_names_the_number_and_the_limit() {
        // 平线 → Dc 是可判的；这里用 2 个样点逼出「样点不足」
        let mut cap = Capture::new(1, 857_142, 1, 2);
        cap.channels = vec![vec![2048, 2049]];
        let scale = ScaleSet::uncalibrated(1);
        let text = build_evidence(&input(&cap, &scale));
        assert!(
            text.contains("判不出·样点不足"),
            "必须用「判不出」句式而不是「异常」"
        );
        assert!(
            text.contains("实际 2 点") && text.contains("16 点"),
            "原因要带实际值与门限：{text}"
        );
    }

    /// 形状段要有**尺寸刻度尺** —— 它每轮都跟着证据包一起发。
    ///
    /// 变异：某一行把全部证据都印上（没有「只印两三样」的约束）——
    /// 这条的尺寸断言红。
    #[test]
    fn the_signal_section_stays_within_its_budget() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let cfg = I2cDecodeConfig::default();
        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        let marker = "== 波形形状";
        let next = "== 波形包络";
        let start = text.find(marker).expect("形状段必须存在");
        let end = text.find(next).expect("包络段必须存在");
        let section: String = text[start..end].to_string();
        let per_channel = section.chars().count() / cap.channels.len().max(1);
        println!(
            "形状段共 {} 字符（{} 通道，每通道约 {}）",
            section.chars().count(),
            cap.channels.len(),
            per_channel
        );
        assert!(
            per_channel < 140,
            "每通道分类行 {per_channel} 字符 —— 依据印得太多了"
        );
    }

    // ── 桶数的**真实**上界 ───────────────────────────────────────────

    /// 桶数是「约」PREVIEW_POINTS，不是「最多」。
    ///
    /// `Capture::preview` 里 `bucket = (len / target).max(1)`，所以
    /// `len` 落在 `[target, 2*target)` 时 bucket 退化成 1，桶数能到 `2*target - 1`。
    /// 这条测试把这个**实际行为**钉住 —— 免得以后有人在文档里
    /// 写一个会被真实数据推翻的上限（我一开始就写错了）。
    #[test]
    fn preview_bucket_count_is_at_most_twice_the_target() {
        for len in [64usize, 127, 128, 129, 255, 256, 300, 512, 1024, 4096] {
            let mut cap = Capture::new(1, 800_000, 1, len as u32);
            cap.channels = vec![(0..len).map(|i| (i % 4096) as u16).collect()];
            let scale = ScaleSet::uncalibrated(cap.channels.len());
            let i = input(&cap, &scale);
            let text = build_evidence(&i);

            // 从文本里把实际桶数读回来
            let buckets = cap.preview(0, PREVIEW_POINTS).unwrap().y_min.len();
            assert!(
                buckets <= 2 * PREVIEW_POINTS,
                "len={len} 时桶数 {buckets} 超过 2×目标（{})",
                2 * PREVIEW_POINTS
            );
            assert!(text.contains("波形包络"), "len={len} 时没有包络段");
        }
    }

    /// 4096 点（硬件上限）时桶数应当**正好**是目标值 —— 这是最常见的情形。
    #[test]
    fn full_capture_hits_the_target_bucket_count_exactly() {
        let mut cap = Capture::new(1, 857_142, 1, 4096);
        cap.channels = vec![(0..4096).map(|i| (i % 4096) as u16).collect()];
        assert_eq!(
            cap.preview(0, PREVIEW_POINTS).unwrap().y_min.len(),
            PREVIEW_POINTS,
            "4096 点应当整除出正好 {PREVIEW_POINTS} 桶"
        );
    }

    // ── 模拟器声明 ───────────────────────────────────────────────────

    #[test]
    fn simulator_data_is_declared_loudly() {
        let cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());

        let mut sim = input(&cap, &scale);
        sim.simulated = true;
        let sim_text = build_evidence(&sim);
        assert!(sim_text.contains("【模拟器】"), "模拟器标志必须出现");
        assert!(
            sim_text.contains("不能") && sim_text.contains("真实硬件"),
            "必须说清模拟器数据的结论不能推及真机"
        );

        let mut real = input(&cap, &scale);
        real.simulated = false;
        real.link = "COM7 @ 921600";
        let real_text = build_evidence(&real);
        assert!(
            !real_text.contains("【模拟器】"),
            "真机数据不该被标成模拟器"
        );
    }

    // ── 测不出就写测不出，不写 0 ─────────────────────────────────────

    /// 项目纪律：无可信结论的量必须显式拒答，不得输出看似精确的无依据数值。
    /// 直流信号没有周期 —— 频率必须写「测不出」，不能写 `0 Hz`。
    #[test]
    fn unmeasurable_metrics_say_so_rather_than_zero() {
        let cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let text = build_evidence(&input(&cap, &scale));

        assert!(text.contains("测不出"), "直流信号应报「测不出」：{text}");
        assert!(
            !text.contains("频率=0.0 Hz"),
            "测不出不能写成 0 Hz —— 那读起来像「测到了，是 0」"
        );
    }

    // ── 溢出必须显式标出 ─────────────────────────────────────────────

    #[test]
    fn overrun_is_flagged() {
        let mut cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        assert!(
            !build_evidence(&input(&cap, &scale)).contains("溢出"),
            "没溢出就不该提溢出"
        );

        cap.overrun = true;
        let text = build_evidence(&input(&cap, &scale));
        assert!(text.contains("溢出"), "溢出了必须标出来");
        assert!(
            text.contains("不完整"),
            "还要说清后果：数据不完整，结论不可信"
        );
    }

    // ── 确定性与纯函数性 ─────────────────────────────────────────────

    /// 同输入必须逐字节同输出。这条是**反例不存在**型的断言 ——
    /// 它抓的是「输出里混进了时间戳/随机数/地址」这类不稳定。
    #[test]
    fn same_input_gives_byte_identical_output() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).unwrap();
        let cfg = I2cDecodeConfig::default();

        let render = || {
            let mut i = input(&cap, &scale);
            let frames = crate::to_decoded(d.clone());
            i.decode = Some(&frames);
            let req = req_i2c(&cfg);
            i.decode_req = Some(&req);
            build_evidence(&i)
        };
        assert_eq!(render(), render(), "同样的输入渲染出了不同的文本");
    }

    // ── 用户的问题进得去 ─────────────────────────────────────────────

    #[test]
    fn the_question_is_included_but_blank_is_ignored() {
        let cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());

        let mut with_q = input(&cap, &scale);
        with_q.question = Some("这段总线有什么问题？");
        assert!(build_evidence(&with_q).contains("这段总线有什么问题？"));

        let mut blank = input(&cap, &scale);
        blank.question = Some("   \n  ");
        assert!(
            !build_evidence(&blank).contains("用户的问题"),
            "空白问题不该产生一个空的提问段"
        );
    }

    // ── 没解码时也要说得清 ───────────────────────────────────────────

    /// **段不能凭空消失，但不再要求解释「为什么解不出来」。**
    ///
    /// 2026-10-06 真机回归：无解码时要求解释原因，把模型的整段回答拉回了
    /// I2C（「缺 SDA，解码做不了」）—— 而非 I2C 波形没有解码结果就是
    /// 正常状态。新契约：场景行必须声明「无总线解码」，但**不再印原因**。
    ///
    /// 变异（杀掉）：把「没有解码结果 —— 通道数不足 2」那句加回来 ——
    /// 这条的第二个断言红。
    #[test]
    fn missing_decode_declares_the_scene_without_explaining_itself() {
        let cap = flat_capture(); // 只有 1 个真通道有数据，且没解
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let text = build_evidence(&input(&cap, &scale));
        assert!(
            text.contains("场景: 模拟信号（无总线解码）"),
            "场景行必须声明无解码 —— 它是模型选分支的依据"
        );
        assert!(
            !text.contains("没有解码结果 —— 通道数不足 2"),
            "不要再向模型解释为什么解不出来 —— 那是它跑偏的入口"
        );
    }

    // ── 场景声明：证据包告诉模型该走哪个分析分支 ─────────────────────

    /// 无解码时声明「模拟信号」场景 —— 提示词的模拟分支据此生效。
    #[test]
    fn missing_decode_declares_the_analog_scenario() {
        let cap = flat_capture();
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let text = build_evidence(&input(&cap, &scale));
        assert!(
            text.contains("场景: 模拟信号（无总线解码）"),
            "无解码时要声明模拟场景，供模型选分支\n{text}"
        );
        assert!(
            !text.contains("场景: I2C 总线解码"),
            "没解码就不能声明 I2C 场景\n{text}"
        );
    }

    /// 有解码时声明 I2C 场景。
    #[test]
    fn decode_declares_the_i2c_scenario() {
        let cap = i2c_capture(true);
        let scale = ScaleSet::uncalibrated(cap.channels.len());
        let d = decode_capture(&cap, &I2cDecodeConfig::default()).expect("应能解码");
        let cfg = I2cDecodeConfig::default();
        let mut i = input(&cap, &scale);
        let frames = crate::to_decoded(d.clone());
        i.decode = Some(&frames);
        let req = req_i2c(&cfg);
        i.decode_req = Some(&req);

        let text = build_evidence(&i);
        assert!(
            text.contains("场景: I2C 总线解码"),
            "有解码时要声明 I2C 场景\n{text}"
        );
    }

    /// B6：证据包的解码段对 UART 也成立，且协议名出现在段标题/场景行。
    #[test]
    fn decode_declares_the_uart_scenario_and_bytes() {
        // 与模拟器 uart_115k 同构：10 位帧 + 2 位空闲、LSB 先发。
        let rate = 857_142u32;
        let baud = 115_200u64;
        let bytes = [0x55u8, 0x0F, 0xA5];
        let n = 4096usize;
        let frame_bits = 12usize;
        let s: Vec<u16> = (0..n)
            .map(|i| {
                let bit = i as u64 * baud / rate as u64;
                let pos = (bit % (bytes.len() * frame_bits) as u64) as usize;
                let frame = pos / frame_bits;
                let within = pos % frame_bits;
                let level = if within >= 10 {
                    true
                } else if within == 0 {
                    false
                } else if within == 9 {
                    true
                } else {
                    (bytes[frame] >> (within - 1)) & 1 == 1
                };
                if level {
                    4000
                } else {
                    0
                }
            })
            .collect();
        let mut cap = Capture::new(1, rate, 1, n as u32);
        cap.channels[0] = s;

        let scale = ScaleSet::uncalibrated(1);
        let req = crate::decoder::DecodeRequest {
            protocol: crate::decoder::Protocol::Uart,
            channels: vec![0],
            params: crate::decoder::DecodeParams::Uart(crate::decoder::UartConfig::default()),
        };
        let frames = crate::decoder::decode(&cap, &req).expect("UART 应能解码");

        let mut i = input(&cap, &scale);
        i.link = "sim(uart_115k)";
        i.decode = Some(&frames);
        i.decode_req = Some(&req);
        let text = build_evidence(&i);

        assert!(
            text.contains("场景: UART 8N1 串行解码"),
            "证据包必须声明 UART 场景\n{text}"
        );
        assert!(
            text.contains("波特率: 115200"),
            "证据包必须回显波特率\n{text}"
        );
        assert!(
            text.contains("字节序列: 55 0F A5"),
            "证据包必须给出字节序列\n{text}"
        );
        assert!(text.contains("帧数: 46"), "4096 点应解出 46 帧\n{text}");
        assert!(
            !text.contains("场景: I2C 总线解码"),
            "UART 结果不能声明成 I2C 场景\n{text}"
        );
    }
}
