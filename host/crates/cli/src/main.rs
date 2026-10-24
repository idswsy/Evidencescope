//! # scope-cli —— 命令行上位机
//!
//! P1 阶段的验收工具：能接硬件抓波形落 CSV，也能对着模拟器开发。
//!
//! ```text
//! # 没有硬件也能跑
//! scope-cli sim info
//! scope-cli sim capture --scenario sine_1k_3v3 -o wave.csv
//!
//! # 有硬件
//! scope-cli serial ports
//! scope-cli serial --port COM3 capture -n 2048 -o wave.csv
//! ```

#![deny(clippy::all)]

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use scope_core::{decode_uid, encode_uid, is_placeholder_uid, CalibrationStore};
use scope_core::{
    from_csv, AcquireParams, Capture, CaptureStore, ChannelScale, CommandBus, DevicePort,
    I2cDecodeConfig, ImportOptions, Levels, RateSource, ScaleSet, State, IMPORT_ID_BASE,
};
use scope_sim::{Scenario, SimDevice};
use scope_transport_serial::{baud, SerialDevice};
use std::io::Write;

#[derive(Parser, Debug)]
#[command(
    name = "scope-cli",
    version,
    about = "F103 Agent Scope 命令行上位机",
    long_about = "可被 AI Agent 控制的数字示波器 / I2C 总线分析仪的调试工具。\n\
                  不带硬件时用 `sim` 子命令，全部功能都可用。"
)]
struct Cli {
    #[command(subcommand)]
    target: Target,
}

#[derive(Subcommand, Debug)]
enum Target {
    /// 对着内置模拟器运行（**不需要硬件**）
    Sim {
        #[command(subcommand)]
        action: Action,
    },

    /// 通过串口连接真实设备
    Serial {
        /// 端口名（如 COM3）
        #[arg(long, default_value = "COM3")]
        port: String,

        /// 波特率
        #[arg(long, default_value_t = baud::B_921600)]
        baud: u32,

        #[command(subcommand)]
        action: Action,
    },

    /// 列出系统中可用的串口
    Ports,

    /// 通道标定表：查看 / 写入 / 删除。**不连接设备**，直接操作主机端文件
    Cal {
        #[command(subcommand)]
        action: CalAction,
    },

    /// 从 CSV 导入一次采集并做测量。**不连接设备**
    ///
    /// 表头形如 `Time(s),CH1V`；速率优先读首行的 `# rate_hz=`。
    /// 两个电压参数都不给时按导出端的双极性换算解释（0 V 在码值 2048），
    /// 所以「本工具导出 → 本命令导入」可逐点还原。
    Import {
        /// CSV 文件路径
        path: String,

        /// 显式采样率（Hz）。文件首行的 `# rate_hz=` 优先
        #[arg(long)]
        rate_hz: Option<u64>,

        /// 满量程电压（V）。与 --offset 都不给时用导出端的双极性换算
        #[arg(long)]
        full_scale: Option<f64>,

        /// 码值 0 对应的输入电压（V）。单极性 0..3.3 V 文件给 0
        #[arg(long)]
        offset: Option<f64>,

        /// 所有列都按原始码值解析（`CH1V` 不换算）
        #[arg(long)]
        raw: bool,

        /// 时间原点不为零时归一化（否则拒绝）
        #[arg(long)]
        normalize_t0: bool,

        /// 只测这一个通道（0 起）。省略则测全部通道
        #[arg(long)]
        channel: Option<usize>,
    },
}

/// `scope-cli cal` 的子命令。
///
/// 标定**只从这里写** —— MCP 侧是只读的。理由：标定会改变之后**所有**的
/// 测量值，它是设备状态而不是单次操作，不该由一个看不见探头的模型来改。
#[derive(Subcommand, Debug)]
enum CalAction {
    /// 列出所有标定记录
    Show,

    /// 写入一条通道标定
    Set {
        /// 设备 uid（十六进制）。接受大小写与 `:` `-` 空格分隔
        #[arg(long)]
        uid: String,

        /// 通道号（0 起）
        #[arg(long, default_value_t = 0)]
        ch: usize,

        /// 每 LSB 对应多少伏
        #[arg(long)]
        volts_per_lsb: f64,

        /// 零电平对应的 ADC LSB
        #[arg(long, default_value_t = 2048.0)]
        zero_lsb: f64,

        /// 备注。**同一占位 uid 下多块板靠它区分**，强烈建议写
        #[arg(long)]
        note: Option<String>,
    },

    /// 删除某个 uid 的记录
    Clear {
        /// 设备 uid（十六进制）
        #[arg(long)]
        uid: String,
    },

    /// 打印标定文件的位置
    Path,
}

#[derive(Subcommand, Debug)]
enum Action {
    /// 读取设备信息与能力
    Info,

    /// 读取当前状态
    Status,

    /// 采集一次并落盘 / 打印摘要
    Capture {
        /// 波形场景（**仅模拟器有效**；接真机时忽略）
        #[arg(long, value_enum, default_value_t = ScenarioArg::Sine1k3v3)]
        scenario: ScenarioArg,

        /// 采样点数（F103 上限 4096）
        #[arg(short = 'n', long, default_value_t = 1024)]
        samples: u16,

        /// 采样率（Hz）。设备会量化到最近档位并回显实际值。
        #[arg(long, default_value_t = 857_142)]
        rate: u32,

        /// 触发电平（ADC LSB，12-bit 范围 0..4095）
        #[arg(long, default_value_t = 2048)]
        level: u16,

        /// 超时（毫秒）
        #[arg(long, default_value_t = 2000)]
        timeout_ms: u64,

        /// 连续采集 n 次并报告帧率（用于验证链路吞吐，不需要 GUI）。
        ///
        /// 任一帧失败即停；只要有过失败，摘要照打但退出码非 0。
        #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..))]
        continuous: u32,

        /// 输出 CSV 路径（省略则只打印摘要，不落盘）
        #[arg(short = 'o', long)]
        out: Option<String>,
    },

    /// 抓一次总线并解码出 I2C 帧序列
    I2c {
        /// 波形场景（**仅模拟器有效**）
        #[arg(long, value_enum, default_value_t = ScenarioArg::I2c100k)]
        scenario: ScenarioArg,

        /// 采样点数（F103 上限 4096）
        #[arg(short = 'n', long, default_value_t = 4096)]
        samples: u16,

        /// 采样率（Hz）
        #[arg(long, default_value_t = 857_142)]
        rate: u32,

        /// SCL 通道（0 起）。与 --sda 一起省略时自动判定
        #[arg(long)]
        scl: Option<usize>,

        /// SDA 通道（0 起）
        #[arg(long)]
        sda: Option<usize>,

        /// 高电平门限（ADC LSB）。省略用 0.7·VDD
        #[arg(long)]
        vih: Option<u16>,

        /// 低电平门限（ADC LSB）。省略用 0.3·VDD
        #[arg(long)]
        vil: Option<u16>,

        /// 去抖时间（ns）
        #[arg(long, default_value_t = 50)]
        debounce_ns: u32,

        /// 超时（毫秒）
        #[arg(long, default_value_t = 2000)]
        timeout_ms: u64,

        /// 输出解码文本路径（`i2c_decode.txt`）
        #[arg(short = 'o', long)]
        out: Option<String>,
    },

    /// 链路自检：PING 往返 + 吞吐
    Ping,

    /// 测量统计：采一次，按通道给出 Vpp / 频率 / 占空比 / 上升与下降时间 / 过冲 / 脉宽 / 跃变数
    Measure {
        /// 波形场景（**仅模拟器有效**；接真机时忽略）
        #[arg(long, value_enum, default_value_t = ScenarioArg::Sine1k3v3)]
        scenario: ScenarioArg,

        /// 采样点数（F103 上限 4096）
        #[arg(short = 'n', long, default_value_t = 1024)]
        samples: u16,

        /// 采样率（Hz）。设备会量化到最近档位并回显实际值。
        #[arg(long, default_value_t = 857_142)]
        rate: u32,

        /// 触发电平（ADC LSB，12-bit 范围 0..4095）
        #[arg(long, default_value_t = 2048)]
        level: u16,

        /// 超时（毫秒）
        #[arg(long, default_value_t = 2000)]
        timeout_ms: u64,

        /// 只测这一个通道（0 起）。省略则测全部通道
        #[arg(long)]
        channel: Option<usize>,
    },

    /// 采一次并解码：I²C 帧序列（需 2 通道）或 UART 8N1 字节流（需 1 通道）
    Decode {
        /// 波形场景（**仅模拟器有效**；接真机时忽略）
        #[arg(long, value_enum, default_value_t = ScenarioArg::Uart115k)]
        scenario: ScenarioArg,

        /// 解码协议
        #[arg(long, value_enum, default_value_t = DecodeProtocol::Uart)]
        protocol: DecodeProtocol,

        /// 采样点数（F103 上限 4096）
        #[arg(short = 'n', long, default_value_t = 4096)]
        samples: u16,

        /// 采样率（Hz）。设备会量化到最近档位并回显实际值
        #[arg(long, default_value_t = 857_142)]
        rate: u32,

        /// 触发电平（ADC LSB，12-bit 范围 0..4095）
        #[arg(long, default_value_t = 2048)]
        level: u16,

        /// 超时（毫秒）
        #[arg(long, default_value_t = 2000)]
        timeout_ms: u64,

        /// UART 波特率（8N1）
        #[arg(long, default_value_t = 115_200)]
        baud: u32,

        /// UART 通道号（0 起）。省略用通道 0
        #[arg(long)]
        channel: Option<usize>,

        /// I²C SCL 通道（0 起）。与 --sda 一起省略时自动判定
        #[arg(long)]
        scl: Option<usize>,

        /// I²C SDA 通道（0 起）
        #[arg(long)]
        sda: Option<usize>,

        /// I²C 高电平门限（ADC LSB）。省略用 0.7·VDD
        #[arg(long)]
        vih: Option<u16>,

        /// I²C 低电平门限（ADC LSB）。省略用 0.3·VDD
        #[arg(long)]
        vil: Option<u16>,

        /// I²C 去抖时间（ns）
        #[arg(long, default_value_t = 50)]
        debounce_ns: u32,

        /// 输出解码文本路径
        #[arg(short = 'o', long)]
        out: Option<String>,
    },
}

/// `decode` 子命令的协议选择。
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum DecodeProtocol {
    /// I²C 总线（需 2 通道）
    #[value(name = "i2c")]
    I2c,
    /// UART 8N1（需 1 通道，真机可用）
    #[value(name = "uart")]
    Uart,
}

impl Action {
    /// 取出本动作隐含的场景；其他动作返回默认场景。
    ///
    /// 模拟器需要在跑动作之前就把设备建出来，所以得先问动作要场景。
    fn scenario_or_default(&self) -> ScenarioArg {
        match self {
            Action::Capture { scenario, .. }
            | Action::I2c { scenario, .. }
            | Action::Measure { scenario, .. }
            | Action::Decode { scenario, .. } => *scenario,
            _ => ScenarioArg::Sine1k3v3,
        }
    }
}

/// 场景名与 `Scenario::name()` 逐字一致 —— 文档、CLI、MCP 三处用同一套字符串，
/// 免得用户照着文档敲却在 CLI 上报「invalid value」。
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum ScenarioArg {
    #[value(name = "sine_1k_3v3")]
    Sine1k3v3,
    #[value(name = "square_50k")]
    Square50k,
    #[value(name = "pulse_glitch")]
    PulseGlitch,
    #[value(name = "noise")]
    Noise,
    #[value(name = "dc")]
    Dc,
    #[value(name = "am")]
    Am,
    #[value(name = "i2c_100k")]
    I2c100k,
    #[value(name = "i2c_400k")]
    I2c400k,
    /// 地址被应答、但随后的一笔写被 NACK —— 用来演示「为什么 NACK」
    #[value(name = "i2c_nack")]
    I2cNack,
    /// 1 kHz、25% 占空比方波
    #[value(name = "pwm_1k_25")]
    Pwm1k25,
    /// 1 kHz、75% 占空比方波
    #[value(name = "pwm_1k_75")]
    Pwm1k75,
    /// 1 kHz、5% 占空比窄脉冲 —— 双电平检测最危险的退化情形
    #[value(name = "pwm_1k_5")]
    Pwm1k5,
    /// 欠阻尼闭环的阶跃响应（过冲约 31%、12 kHz 衰减振铃）
    #[value(name = "step_ring")]
    StepRing,
    /// 一阶 RC 充放电（τ = 150 µs）
    #[value(name = "rc_charge")]
    RcCharge,
    /// 115200 baud 串行数据，0x55 连发
    #[value(name = "uart_115k")]
    Uart115k,
    /// 1 kHz 慢边沿（10-90% 200 µs）+ σ=40 LSB 白噪声
    #[value(name = "slow_edge_noisy")]
    SlowEdgeNoisy,
    /// 400 kHz 混叠振铃 —— 用于验证 Refused(Aliased)
    #[value(name = "ring_alias")]
    RingAlias,
}

impl From<ScenarioArg> for Scenario {
    fn from(a: ScenarioArg) -> Scenario {
        match a {
            ScenarioArg::Sine1k3v3 => Scenario::Sine1k3v3,
            ScenarioArg::Square50k => Scenario::Square50k,
            ScenarioArg::PulseGlitch => Scenario::PulseGlitch,
            ScenarioArg::Noise => Scenario::Noise,
            ScenarioArg::Dc => Scenario::Dc,
            ScenarioArg::Am => Scenario::Am,
            ScenarioArg::I2c100k => Scenario::I2c100k,
            ScenarioArg::I2c400k => Scenario::I2c400k,
            ScenarioArg::I2cNack => Scenario::I2cNack,
            ScenarioArg::Pwm1k25 => Scenario::Pwm1k25,
            ScenarioArg::Pwm1k75 => Scenario::Pwm1k75,
            ScenarioArg::Pwm1k5 => Scenario::Pwm1k5,
            ScenarioArg::StepRing => Scenario::StepRing,
            ScenarioArg::RcCharge => Scenario::RcCharge,
            ScenarioArg::Uart115k => Scenario::Uart115k,
            ScenarioArg::SlowEdgeNoisy => Scenario::SlowEdgeNoisy,
            ScenarioArg::RingAlias => Scenario::RingAlias,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.target {
        Target::Ports => {
            let ports = scope_transport_serial::list_ports();
            if ports.is_empty() {
                println!("没有找到任何串口。");
                println!("提示：插上板子后重试；或对着模拟器开发（不需要硬件）：");
                println!("      scope-cli sim capture --scenario sine_1k_3v3 -n 1024 -o wave.csv");
                return Ok(());
            }
            println!("{:<10} {:<40} 疑似目标", "端口", "描述");
            println!("{}", "-".repeat(70));
            for (name, desc, likely) in ports {
                let mark = if likely { "是" } else { "" };
                println!("{name:<10} {desc:<40} {mark}");
            }
            Ok(())
        }

        Target::Cal { action } => run_cal(action),

        Target::Import {
            path,
            rate_hz,
            full_scale,
            offset,
            raw,
            normalize_t0,
            channel,
        } => run_import(
            path,
            rate_hz,
            full_scale,
            offset,
            raw,
            normalize_t0,
            channel,
        ),

        Target::Sim { action } => {
            let mut dev = SimDevice::new(action.scenario_or_default().into());
            println!("# 模拟器: {}", dev.describe());
            run(&mut dev, action)
        }

        Target::Serial { port, baud, action } => {
            let mut dev = SerialDevice::open(&port, baud, 5)?;
            println!("# 链路: {}", dev.describe());
            run(&mut dev, action)
        }
    }
}

/// `scope-cli import` —— 从 CSV 导入一次采集并做测量。**不连接设备。**
///
/// 电压口径（与 MCP `scope_import_capture` 一致）：
/// - `--full-scale` / `--offset` 都不给 → 按导出端的双极性换算
///   （3.3 V 满量程、0 V 在码值 2048），「本工具导出 → 本命令导入」逐点还原；
/// - 给了任一参数 → 显式模型，另一个取默认（满量程 3.3 V、offset 0，即单极性）。
fn run_import(
    path: String,
    rate_hz: Option<u64>,
    full_scale: Option<f64>,
    offset: Option<f64>,
    raw: bool,
    normalize_t0: bool,
    channel: Option<usize>,
) -> Result<()> {
    let text = std::fs::read_to_string(&path).with_context(|| format!("读取 CSV 失败：{path}"))?;

    let explicit_voltage = full_scale.is_some() || offset.is_some();
    let mut opts = if explicit_voltage {
        ImportOptions::default()
    } else {
        ImportOptions::from_scale(&ChannelScale::default())
    };
    opts.id = IMPORT_ID_BASE;
    opts.path = path.clone();
    opts.rate_hz = rate_hz;
    if let Some(v) = full_scale {
        opts.full_scale_v = v;
    }
    if let Some(v) = offset {
        opts.offset_v = v;
    }
    opts.raw = raw;
    opts.normalize_t0 = normalize_t0;

    let imported = from_csv(&text, &opts).map_err(|e| anyhow::anyhow!(e.to_string()))?;
    let capture = imported.capture;
    let ch_count = capture.channels.len();

    let targets: Vec<usize> = match channel {
        Some(c) if c >= ch_count => bail!(
            "--channel {c} 超出范围：文件里只有 {ch_count} 个通道（0..={}）",
            ch_count.saturating_sub(1)
        ),
        Some(c) => vec![c],
        None => (0..ch_count).collect(),
    };

    let rate_src = match imported.rate_source {
        RateSource::Header => "文件首行 # rate_hz=",
        RateSource::Option => "命令行 --rate-hz",
        RateSource::TimeColumn => "时间列中位差分反推",
    };
    println!("# 导入: {path}");
    println!(
        "# 采样率: {} Hz（来源：{rate_src}）  {} 点 × {} 通道  {:.3} ms",
        capture.rate_hz,
        capture.len(),
        ch_count,
        capture.duration_us() as f64 / 1000.0
    );
    println!(
        "# 电压换算: code = round((v - ({:+.6} V)) × 4096 / {:.6} V){}",
        opts.offset_v,
        opts.full_scale_v,
        if raw {
            "（--raw：所有列按原始码值）"
        } else {
            ""
        }
    );
    if imported.t0_s != 0.0 {
        println!("# 时间原点 t0 = {:.9} s（已归一化到 0）", imported.t0_s);
    }

    // 测量必须用导入时的同一套换算，否则「文件里是 3.3 V」会被打印成别的值。
    // 这里**故意不查设备标定表**：导入的是第三方数据，与当前连没连设备无关。
    let vpl = opts.full_scale_v / 4096.0;
    let scale = ChannelScale {
        volts_per_lsb: vpl,
        // 写成 `0 − offset` 而不是 `−offset`：后者在 offset = 0 时得到 −0.0，
        // 打印出来是「零点 -0」，看起来像个 bug。
        zero_lsb: (0.0 - opts.offset_v) / vpl,
    };
    let scales = ScaleSet::from_scales(vec![scale; ch_count]);
    // 导入的数据不是设备采的 —— 明确标成非模拟器，也不能冒充设备链路。
    print_measurements(
        &capture,
        &scales,
        &targets,
        &format!("import({path})"),
        false,
    );

    println!();
    println!(
        "换算来源：导入参数（不是设备标定表）—— {vpl:.6} V/LSB、零点 {}",
        scale.zero_lsb
    );
    Ok(())
}

/// 逐通道测量文本。`Action::Measure` 与 `import` 共用，免得两处措辞漂移。
///
/// 瞬态 / TIE 行直接取自 [`scope_core::analysis::transient_tie_lines`]，
/// 与证据包、MCP 是同一份文本。
fn measurement_text(
    capture: &Capture,
    scales: &ScaleSet,
    targets: &[usize],
    link: &str,
    simulated: bool,
) -> String {
    use std::fmt::Write as _;

    let mut out = String::new();
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "测量  capture_id={}  {} 点  {:.3} ms",
        capture.id,
        capture.len(),
        capture.duration_us() as f64 / 1000.0
    );

    let report = scope_core::AnalysisReport::from_capture(capture, scales, link, simulated);
    for &ch in targets {
        // 「测不出」要写成「测不出」，不能写 0 —— 项目纪律：宁可说不知道
        let Some(ca) = report.channel(ch) else {
            let _ = writeln!(out, "  CH{}: 测不出（通道无数据）", ch + 1);
            continue;
        };
        let m = &ca.measurements;
        let _ = writeln!(out, "  CH{}:", ch + 1);
        let _ = writeln!(
            out,
            "    Vpp={:.4} V  min={:.4} V  max={:.4} V  mean={:.4} V  AC-RMS={:.4} V  RMS={:.4} V",
            m.vpp, m.min, m.max, m.mean, m.ac_rms, m.rms
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
        let _ = writeln!(out, "    频率={freq}  占空比={duty}  上升时间={rise}");
        let _ = writeln!(
            out,
            "    下降时间={fall}  过冲/下冲={over}/{under}  高电平脉宽={high}  跃变数={}",
            m.edges
        );

        for line in scope_core::analysis::transient_tie_lines(ca) {
            let _ = writeln!(out, "    {line}");
        }
    }
    out
}

fn print_measurements(
    capture: &Capture,
    scales: &ScaleSet,
    targets: &[usize],
    link: &str,
    simulated: bool,
) {
    print!(
        "{}",
        measurement_text(capture, scales, targets, link, simulated)
    );
}

/// 把 uid 字符串解析成 12 字节，失败时给一条能照着改的错误。
fn parse_uid(s: &str) -> Result<[u8; 12]> {
    decode_uid(s).ok_or_else(|| {
        anyhow::anyhow!(
            "uid 解析失败：{s:?}。应为 12 字节十六进制（24 个字符，如 \
             `5eed00010203040506070809`）；`:`、`-`、空格分隔与大小写都接受"
        )
    })
}

/// Unix 秒。core 不引时间库，时间戳由写方给。
fn now_unix() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// `scope-cli cal` —— 标定表的读写。**不连接设备。**
fn run_cal(action: CalAction) -> Result<()> {
    let mut store = CalibrationStore::load_default();
    if let Some(note) = store.note() {
        println!("# {note}");
    }
    let where_ = store
        .path()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|| "(无路径)".into());

    match action {
        CalAction::Path => {
            if store.path().is_none() {
                bail!(
                    "没有可用的配置目录（APPDATA / XDG_CONFIG_HOME / HOME 均未设置），\
                     标定无处存放"
                );
            }
            println!("{where_}");
            Ok(())
        }

        CalAction::Show => {
            println!("标定表  {} 条记录  {where_}", store.len());
            if store.is_read_only() {
                println!("⚠ 文件版本高于本程序，本次只读");
            }
            if store.is_empty() {
                println!();
                println!(
                    "（空。写入一条：scope-cli cal set --uid <24位十六进制> --volts-per-lsb <V>）"
                );
                return Ok(());
            }
            for (uid, rec) in store.iter() {
                println!();
                match rec.updated_unix {
                    Some(t) => println!("{uid}  更新于 {t}"),
                    None => println!("{uid}"),
                }
                if let Some(n) = &rec.note {
                    println!("  备注: {n}");
                }
                if let Ok(b) = parse_uid(uid) {
                    if let Some(why) = is_placeholder_uid(&b) {
                        println!("  ⚠ {why}");
                    }
                }
                for (ch, c) in rec.channels.iter().enumerate() {
                    println!(
                        "  CH{}: {:.9} V/LSB  零点 {:.1} LSB  满量程 {:.3} V",
                        ch + 1,
                        c.volts_per_lsb,
                        c.zero_lsb,
                        c.volts_per_lsb * 4096.0
                    );
                }
            }
            Ok(())
        }

        CalAction::Set {
            uid,
            ch,
            volts_per_lsb,
            zero_lsb,
            note,
        } => {
            let uid_b = parse_uid(&uid)?;
            // 占位 uid 不拒绝，但必须让用户知道：拿它标定等于给所有
            // 共用这串占位值的板子一起标了。
            if let Some(why) = is_placeholder_uid(&uid_b) {
                println!("⚠ {why}");
            }
            let scale = ChannelScale {
                volts_per_lsb,
                zero_lsb,
            };
            // core 的持久化 API 返回 `Result<_, String>`（错误文本是给人看的，
            // 本来就带「怎么办」）。`String` 不是 `std::error::Error`，
            // 所以进 anyhow 要显式转一次。
            store
                .set_channel(uid_b, ch, scale, note, now_unix())
                .map_err(anyhow::Error::msg)?;
            store.save().map_err(anyhow::Error::msg)?;
            println!(
                "已写入 {}  CH{}  {:.9} V/LSB  零点 {:.1}",
                encode_uid(&uid_b),
                ch + 1,
                volts_per_lsb,
                zero_lsb
            );
            println!("文件: {where_}");
            Ok(())
        }

        CalAction::Clear { uid } => {
            let uid_b = parse_uid(&uid)?;
            if store.remove(&uid_b) {
                store.save().map_err(anyhow::Error::msg)?;
                println!("已删除 {}", encode_uid(&uid_b));
            } else {
                println!("没有 {} 的记录，未改动", encode_uid(&uid_b));
            }
            Ok(())
        }
    }
}

fn run<P: DevicePort>(port: &mut P, action: Action) -> Result<()> {
    let mut bus = CommandBus::new(RefPort(port));
    // 标定表在**连接之前**读一次：一是 `bus` 之后会借走可变引用，
    // 二是标定属于会话开始时的设备状态，中途换掉会让同一次采集里
    // 前后两次换算用的不是同一套参数。
    let calib = CalibrationStore::load_default();

    match action {
        Action::Info => {
            let info = bus.connect().context("连接失败（GET_INFO 无响应）")?;
            println!("协议版本   : 0x{:02X}", info.proto_ver);
            println!(
                "固件版本   : {}.{}.{}",
                (info.fw_ver >> 16) & 0xFF,
                (info.fw_ver >> 8) & 0xFF,
                info.fw_ver & 0xFF
            );
            println!("型号       : 0x{:04X}", info.model);
            println!("UID        : {}", hex(&info.uid));
            println!(
                "标定       : {}",
                calib
                    .scales_for(Some(&info.uid), info.ch_count as usize)
                    .summary_note()
            );
            println!("ADC        : {} bit", info.adc_bits);
            println!("通道数     : {}", info.ch_count);
            println!(
                "采样率范围 : {} .. {} Hz",
                info.rate_min_hz, info.rate_max_hz
            );
            println!("单次上限   : {} 点", info.capture_max_samples);
            println!(
                "payload    : 上行 {} B / 下行 {} B",
                info.max_rx_payload, info.max_tx_payload
            );
            println!("建议分片   : {} 点", info.preferred_chunk_samples);
            println!("能力位     : 0x{:08X}", info.caps);

            if info.rate_max_hz > scope_core::f103::MAX_INTERLEAVED_HZ {
                println!();
                println!("⚠ 设备上报的采样率上限超过 F103 的物理极限，可能是固件 bug。");
            }
            Ok(())
        }

        Action::Status => {
            bus.connect()?;
            let st = bus.get_status()?;
            let s = st.state;
            println!("状态: {} ({})", state_name(s), s as u8);
            if let Some(cfg) = &bus.config {
                println!("采样率: {} Hz", cfg.rate_hz);
                println!(
                    "采集模式: {} | 样点: {} | 格式: {} | 抽点: {}",
                    cfg.acq_mode, cfg.capture_samples, cfg.format, cfg.decimation
                );
                println!(
                    "触发: mode={} src={} edge={} level={} LSB",
                    cfg.trigger_mode, cfg.trigger_source, cfg.trigger_edge, cfg.trigger_level_lsb
                );
            }
            // 连通性闸门看的是这几个累计计数器（docs/09 §4.2：全 0 才算链路
            // 干净）。从前这里只取 state，其余 32 字节一路都到不了命令行。
            println!(
                "链路: 溢出 {} · 收 CRC 错 {} · 收丢 {} · 发丢 {}",
                st.overrun_samples, st.rx_crc_err, st.rx_dropped, st.tx_dropped
            );
            println!(
                "链路判定: {}",
                if st.link_is_clean() {
                    "干净（溢出 / CRC 错 / 收丢均为 0）"
                } else {
                    "不干净（有溢出或链路错误，采集数据不可全信）"
                }
            );
            Ok(())
        }

        Action::Ping => {
            bus.connect()?;
            println!("PING 往返测试（10 次）");
            let mut total = std::time::Duration::ZERO;
            let mut ok = 0;
            for i in 0..10u8 {
                let payload = [i, i.wrapping_mul(7), i.wrapping_add(3)];
                let (echoed, rtt) = bus.ping(&payload)?;
                if echoed == payload {
                    ok += 1;
                    total += rtt;
                    println!(
                        "  #{} {:>8.2} ms  echo ✓",
                        i + 1,
                        rtt.as_secs_f64() * 1000.0
                    );
                } else {
                    println!("  #{} 回显不符: {:?}", i + 1, echoed);
                }
            }
            if ok > 0 {
                println!(
                    "平均往返: {:.2} ms",
                    total.as_secs_f64() * 1000.0 / ok as f64
                );
            }
            Ok(())
        }

        Action::Capture {
            // 场景在 main() 里已经用来构造模拟器了，这里用不上
            scenario: _,
            samples,
            rate,
            level,
            timeout_ms,
            continuous,
            out,
        } => {
            // 连续采集：跑 `continuous` 次，只有最后一帧打摘要。
            // 失败即停 —— 与 GUI 的「连续刷新」同一语义（docs/09 §8）。
            let start = std::time::Instant::now();
            let mut ok: u32 = 0;
            let mut overruns: u32 = 0;
            let mut last_err: Option<scope_core::ScopeError> = None;
            let mut capture: Option<Capture> = None;
            for i in 0..continuous {
                let quiet = i + 1 < continuous;
                match capture_once(&mut bus, samples, rate, level, timeout_ms, quiet) {
                    Ok(c) => {
                        ok += 1;
                        if c.overrun {
                            overruns += 1;
                        }
                        capture = Some(c);
                    }
                    Err(e) => {
                        last_err = Some(e);
                        break;
                    }
                }
            }
            let elapsed = start.elapsed();
            // 单次采集（默认）不打这行 —— 输出与从前逐字一致，fps 对 n=1 也没意义
            if continuous > 1 && ok > 0 {
                let fps = ok as f64 / elapsed.as_secs_f64();
                // 溢出是链路健康的信号之一。quiet 把中间帧的溢出警告吞掉了，
                // 所以这里按帧数补一个计数 —— 「验证吞吐」的入口不能漏掉它。
                if overruns > 0 {
                    println!(
                        "# 连续采集 {ok}/{continuous} 次成功，平均 {fps:.1} fps，其中 {overruns} 帧溢出"
                    );
                } else {
                    println!("# 连续采集 {ok}/{continuous} 次成功，平均 {fps:.1} fps");
                }
            }
            if let Some(e) = &last_err {
                // 用 `summary()` 而不是 Display —— 与 GUI / MCP 的显示纪律
                // 一致（见 `scope_core::error` 里对这两者的说明）。
                println!("✗ 第 {} 次失败：{}", ok + 1, e.summary());
            }
            // 一帧都没成：没有摘要可打，把错误原样交出去（退出码非 0）。
            // `--continuous` 有范围校验（≥1），到这里必然是首帧即败。
            let Some(capture) = capture else {
                return Err(last_err
                    .map(anyhow::Error::new)
                    .unwrap_or_else(|| anyhow::anyhow!("--continuous 至少为 1")));
            };
            let ch_count = capture.channels.len();
            let capture_id = capture.id;

            // 摘要（这是 Agent 默认能看到的东西）
            println!();
            println!(
                "采集摘要  capture_id={}  {} 点  {:.3} ms",
                capture_id,
                capture.len(),
                capture.duration_us() as f64 / 1000.0
            );
            for ch in 0..ch_count {
                if let Some(s) = capture.summary(ch) {
                    // rms = 相对 ADC 零点的真 RMS；ac_rms = 扣除直流后的波动（标准差）
                    println!(
                        "  CH{}: min={} max={} pp={} mean={:.1} rms={:.1} ac_rms={:.1}  上升沿={} 下降沿={}",
                        ch + 1,
                        s.min_lsb,
                        s.max_lsb,
                        s.pp_lsb,
                        s.mean_lsb,
                        s.rms_lsb,
                        s.ac_rms_lsb,
                        s.rising_edges,
                        s.falling_edges
                    );
                }
            }

            // 换算表按设备 uid 查；没有记录就退回占位值，并如实说明。
            let scales = calib.scales_for(bus.info.as_ref().map(|i| &i.uid), ch_count);
            println!("  {}", scales.summary_note());

            if let Some(path) = out {
                let csv = capture.to_csv(scales.as_slice());
                let mut f =
                    std::fs::File::create(&path).with_context(|| format!("无法创建 {path}"))?;
                f.write_all(csv.as_bytes())?;
                println!();
                println!(
                    "已写出 {} ({} 字节, {} 行)",
                    path,
                    csv.len(),
                    capture.len() + 1
                );
                println!("提示：CSV 全量落盘，不进 LLM 上下文 —— 这是三层 token 防护的第三层。");
            } else {
                println!();
                println!("提示：加 -o wave.csv 可导出全量数据供绘图或喂给解码器。");
            }

            let _ = bus.stop();
            let mut store = CaptureStore::default();
            store.push(capture);
            if let Some(e) = last_err {
                // 部分成功：摘要已打，但退出码必须是失败 ——
                // 这个入口的存在意义就是给脚本/Agent 验证链路
                return Err(e.into());
            }
            Ok(())
        }

        Action::I2c {
            scenario: _,
            samples,
            rate,
            scl,
            sda,
            vih,
            vil,
            debounce_ns,
            timeout_ms,
            out,
        } => {
            let capture = capture_once(&mut bus, samples, rate, 2048, timeout_ms, false)?;
            decode_i2c_capture(&capture, scl, sda, vih, vil, debounce_ns, out)
        }

        Action::Decode {
            scenario: _,
            protocol,
            samples,
            rate,
            level,
            timeout_ms,
            baud,
            channel,
            scl,
            sda,
            vih,
            vil,
            debounce_ns,
            out,
        } => match protocol {
            DecodeProtocol::I2c => {
                let capture = capture_once(&mut bus, samples, rate, level, timeout_ms, false)?;
                decode_i2c_capture(&capture, scl, sda, vih, vil, debounce_ns, out)
            }
            DecodeProtocol::Uart => {
                // 触发：0 = auto，1 = falling。UART 空闲为高、起始位为低，
                // 起始位本身就是一个下降沿；默认的 normal + rising 等不到它。
                let capture = capture_once_with_trigger(
                    &mut bus,
                    samples,
                    rate,
                    level,
                    timeout_ms,
                    true,
                    Some((0, 1)),
                )?;
                decode_uart_capture(&capture, channel, baud, out)
            }
        },

        Action::Measure {
            // 场景在 main() 里已经用来构造模拟器了，这里用不上
            scenario: _,
            samples,
            rate,
            level,
            timeout_ms,
            channel,
        } => {
            // 没有跨调用的采集存储（`Action::Capture` 的 store 是局部变量），
            // 所以测量必须自己先采一次 —— 与 `i2c` 同样复用 `capture_once`。
            let capture = capture_once(&mut bus, samples, rate, level, timeout_ms, false)?;
            let ch_count = capture.channels.len();

            let targets: Vec<usize> = match channel {
                Some(c) if c >= ch_count => {
                    anyhow::bail!(
                        "--channel {c} 超出范围：本次采集只有 {ch_count} 个通道（0..={}）",
                        ch_count.saturating_sub(1)
                    )
                }
                Some(c) => vec![c],
                None => (0..ch_count).collect(),
            };

            // 换算表按设备 uid 查；长度恒等于通道数，`to_csv` 的短表兜底路径不可达。
            let scales: ScaleSet = calib.scales_for(bus.info.as_ref().map(|i| &i.uid), ch_count);

            let link = bus.describe();
            let simulated = bus.is_simulated();
            print_measurements(&capture, &scales, &targets, &link, simulated);

            println!();
            println!("{}", scales.summary_note());

            let _ = bus.stop();
            Ok(())
        }
    }
}

/// I²C 解码的共用实现（`i2c` 子命令与 `decode --protocol i2c` 同一份）。
fn decode_i2c_capture(
    capture: &Capture,
    scl: Option<usize>,
    sda: Option<usize>,
    vih: Option<u16>,
    vil: Option<u16>,
    debounce_ns: u32,
    out: Option<String>,
) -> Result<()> {
    // I2C 解码至少要两条线。单通道场景（如 dc / sine_1k_3v3）在这里
    // 就要说清楚，而不是等 decode_capture 报一个「通道越界」让人猜。
    if capture.channels.len() < 2 {
        bail!(
            "I2C 解码需要至少 2 个通道，当前场景只有 {} 个。\n\
             可以试试：\n  \
             • 换用双通道场景：--scenario i2c_100k 或 i2c_400k\n  \
             · 接真机时确认 GET_INFO 上报的 ch_count ≥ 2",
            capture.channels.len()
        );
    }

    let levels = match (vih, vil) {
        (Some(h), Some(l)) => Levels {
            vih_lsb: h,
            vil_lsb: l,
        },
        (Some(h), None) => Levels {
            vih_lsb: h,
            vil_lsb: Levels::default_ratio().vil_lsb,
        },
        (None, Some(l)) => Levels {
            vih_lsb: Levels::default_ratio().vih_lsb,
            vil_lsb: l,
        },
        (None, None) => Levels::default_ratio(),
    };
    println!(
        "# 门限: VIH={} VIL={} LSB（判决带内不硬判 0/1）",
        levels.vih_lsb, levels.vil_lsb
    );

    // 通道：显式指定优先；都省略时自动判定，并把判定结果告诉用户
    let (scl_ch, sda_ch) = match (scl, sda) {
        (Some(a), Some(b)) => (a, b),
        (Some(a), None) => (a, if a == 0 { 1 } else { 0 }),
        (None, Some(b)) => (if b == 0 { 1 } else { 0 }, b),
        (None, None) => match scope_core::detect_channels(capture, levels, debounce_ns) {
            Some((a, b)) => {
                println!("# 自动判定: SCL=CH{}  SDA=CH{}", a + 1, b + 1);
                (a, b)
            }
            None => {
                println!("# 自动判定失败（两条线都没有边沿）；按默认 CH1=SCL CH2=SDA 继续");
                (0, 1)
            }
        },
    };

    let cfg = I2cDecodeConfig {
        scl_channel: scl_ch,
        sda_channel: sda_ch,
        levels,
        debounce_ns,
    };
    let result = scope_core::decode_capture(capture, &cfg)?;

    println!();
    print!("{}", result.to_text());

    let bytes = result.all_bytes();
    if !bytes.is_empty() {
        let hex: Vec<String> = bytes.iter().map(|b| format!("0x{b:02X}")).collect();
        println!("\n数据字节: [{}]", hex.join(", "));
    }

    if let Some(path) = out {
        let text = result.to_text();
        std::fs::write(&path, text.as_bytes()).with_context(|| format!("无法写入 {path}"))?;
        println!("已写出 {path}");
    }

    Ok(())
}

/// UART 8N1 解码（`decode --protocol uart`）。
fn decode_uart_capture(
    capture: &Capture,
    channel: Option<usize>,
    baud: u32,
    out: Option<String>,
) -> Result<()> {
    let ch = channel.unwrap_or(0);
    let n = capture.channels.len();
    if ch >= n {
        bail!(
            "--channel {ch} 超出范围：本次采集只有 {n} 个通道（0..={}）",
            n.saturating_sub(1)
        );
    }

    let req = scope_core::decoder::DecodeRequest {
        protocol: scope_core::decoder::Protocol::Uart,
        channels: vec![ch],
        params: scope_core::decoder::DecodeParams::Uart(scope_core::decoder::UartConfig {
            baud,
            ..scope_core::decoder::UartConfig::default()
        }),
    };
    let result = scope_core::decoder::decode(capture, &req)?;

    println!();
    println!(
        "# UART 8N1: capture_id={}  {} 点  {:.3} ms  采样率 {} Hz  CH{}",
        capture.id,
        capture.len(),
        capture.duration_us() as f64 / 1000.0,
        capture.rate_hz,
        ch + 1
    );
    print!("{}", result.uart_text());

    if let Some(path) = out {
        let text = result.uart_text();
        std::fs::write(&path, text.as_bytes()).with_context(|| format!("无法写入 {path}"))?;
        println!("\n已写出 {path}");
    }

    Ok(())
}

/// 跑一次完整采集：连接 → 配置 → 武装 → 等触发 → 分片拉取。
///
/// `capture` / `i2c` / `measure` / `decode` 四个动作共用 —— 采集逻辑只写一遍，
/// 免得解码那条路上的分片处理与落盘那条路走出两个版本的 bug。
///
/// `quiet = true` 时跳过函数内部的打印 —— 连续采集的中间帧用，
/// 只留最后一帧的摘要。
///
/// 错误保持 `ScopeError` 原样（core 的错误模型，本函数也只可能产生它）：
/// 连续采集的失败行要用 [`scope_core::ScopeError::summary`]，裹成 `anyhow`
/// 之后就只剩 Display 可打了。调用方照常 `?` 进 `anyhow::Result`。
fn capture_once<D: DevicePort>(
    bus: &mut CommandBus<D>,
    samples: u16,
    rate: u32,
    level: u16,
    timeout_ms: u64,
    quiet: bool,
) -> scope_core::Result<Capture> {
    capture_once_with_trigger(bus, samples, rate, level, timeout_ms, quiet, None)
}

/// [`capture_once`] 的可选触发覆盖。
///
/// `trigger = Some((mode, edge))` 时在 `acquire` 之前显式下发一次
/// `SET_TRIGGER` —— `acquire` 沿用的是**设备当前配置**，所以这次下发会被它
/// 原样沿用。UART 需要 auto 模式 + 下降沿（起始位是下降沿），而默认配置
/// 是 normal + 上升沿，会一直等不到触发。
fn capture_once_with_trigger<D: DevicePort>(
    bus: &mut CommandBus<D>,
    samples: u16,
    rate: u32,
    level: u16,
    timeout_ms: u64,
    quiet: bool,
    trigger: Option<(u8, u8)>,
) -> scope_core::Result<Capture> {
    bus.connect()?;

    if let Some((mode, edge)) = trigger {
        let pre = (samples / 2).min(2048);
        bus.set_trigger(mode, 0, edge, level, pre, 1000)?;
    }

    // 采集编排在 core 里（`scope_core::acquire`）—— 它替我们处理了
    // EVENT_TRIGGER 的 18 字节解析、分片上限、以及「没有触发点」的哨兵映射。
    // CLI / GUI / MCP 三端共用这一份，不各写一遍。
    let params = AcquireParams {
        samples,
        rate_hz: rate,
        trigger_level_lsb: level,
        timeout: std::time::Duration::from_millis(timeout_ms),
    };
    let capture = scope_core::acquire(bus, &params)?;

    if quiet {
        return Ok(capture);
    }

    if capture.rate_hz != rate {
        println!(
            "# 采样率被量化: {} Hz → {} Hz（时间轴以 {} Hz 为准）",
            rate, capture.rate_hz, capture.rate_hz
        );
    }
    match capture.trigger_index {
        Some(i) => println!(
            "# 触发: capture_id={} index={} tick={} µs",
            capture.id, i, capture.device_tick_us
        ),
        // 软触发 / 未找到触发点时设备发哨兵 0xFFFF_FFFF —— core 已映射成 None
        None => println!(
            "# 触发: capture_id={} **无触发点**（软触发或未找到）tick={} µs",
            capture.id, capture.device_tick_us
        ),
    }

    if capture.overrun {
        println!("⚠ 采集期间发生溢出 —— 这份数据不完整，不能当作完整波形用");
    }
    Ok(capture)
}

fn state_name(s: State) -> &'static str {
    match s {
        State::Idle => "空闲",
        State::Armed => "已武装",
        State::Streaming => "流推送中",
        State::Done => "采集完成",
        State::Fault => "故障",
    }
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(":")
}

/// 让 `run` 能接受 `&mut P` 而不是夺取所有权 ——
/// 调用者（`main`）需要在 `run` 返回后继续持有设备。
struct RefPort<'a, P: DevicePort>(&'a mut P);

impl<P: DevicePort> DevicePort for RefPort<'_, P> {
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), scope_core::LinkError> {
        self.0.write_all(bytes)
    }
    fn read_some(&mut self) -> Result<Vec<u8>, scope_core::LinkError> {
        self.0.read_some()
    }
    fn describe(&self) -> String {
        self.0.describe()
    }
    fn is_simulated(&self) -> bool {
        self.0.is_simulated()
    }
    fn byte_rate(&self) -> u32 {
        self.0.byte_rate()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::ValueEnum;

    /// CLI 的场景清单必须与 `scope_sim::Scenario` **逐字一致**。
    ///
    /// CLI 这里有一份自己的枚举（为了 clap 的命令行补全与帮助文本），
    /// 而 `Scenario` 那边有 `ALL` 与 `name()` —— 同一份东西三处各写一遍。
    ///
    /// 回归：从前没有任何东西保证它们对得上。给模拟器加了 `i2c_nack` 之后，
    /// **CLI 与 GUI 都静默地少了一个场景**，没有一条测试会红。
    /// GUI 那边已经改成直接遍历 `Scenario::ALL`（把重复消灭掉）；
    /// CLI 因为要用 clap 的 `ValueEnum`，保留枚举，改用这条测试卡住。
    #[test]
    fn scenario_list_matches_the_simulator() {
        let cli: Vec<String> = ScenarioArg::value_variants()
            .iter()
            .filter_map(|v| v.to_possible_value().map(|p| p.get_name().to_string()))
            .collect();
        let sim: Vec<String> = Scenario::all_names().map(String::from).collect();
        assert_eq!(cli, sim, "CLI 的场景清单与 scope_sim::Scenario 分家了");

        // 而且每一个都要真的能转过去 —— 防止只加了枚举、忘了写 From 分支
        for v in ScenarioArg::value_variants() {
            let s: Scenario = (*v).into();
            assert_eq!(
                s.name(),
                v.to_possible_value().unwrap().get_name(),
                "枚举名与场景名对不上"
            );
        }
    }

    /// uid 解析宽容、但错的要给出能照着改的提示。
    ///
    /// 宽容是有代价的：**两种拼法会静默产生两条记录**。所以写入一律走
    /// `encode_uid` 规范化，读取才宽容 —— 这条测试钉的是「读」这一侧。
    #[test]
    fn uid_parsing_accepts_separators_and_explains_junk() {
        let want = decode_uid("5eed00010203040506070809").unwrap();
        for s in [
            "5eed00010203040506070809",
            "5E:ED:00:01:02:03:04:05:06:07:08:09",
            "5e-ed-00-01-02-03-04-05-06-07-08-09",
        ] {
            assert_eq!(parse_uid(s).unwrap(), want, "应能解析 {s}");
        }
        let e = parse_uid("xyz").unwrap_err().to_string();
        assert!(e.contains("24 个字符"), "错误要写清格式：{e}");
    }

    /// F2：`sim measure` 与 `import` 必须与证据包用同一套瞬态 / TIE 措辞。
    ///
    /// 回归：这两个出口从前只打印基础测量，瞬态与 TIE 只存在于证据包和
    /// MCP 里 —— 同一份数据在 CLI 上看不见。
    #[test]
    fn measurement_text_carries_transient_and_tie_lines() {
        let cap = sim_capture(Scenario::StepRing, 1024);
        let text = measurement_text(
            &cap,
            &ScaleSet::uncalibrated(1),
            &[0],
            "sim(step_ring)",
            true,
        );
        for want in [
            "瞬态: 振铃=",
            "阻尼比=",
            "调节(±2%)=",
            "RC τ=",
            "压摆(max/avg)=",
            "runt=",
        ] {
            assert!(text.contains(want), "缺「{want}」：\n{text}");
        }
        assert!(
            text.contains("TIE: 测不出（边沿太少）"),
            "step_ring 的 TIE 必须写明原因，而不是静默省略：\n{text}"
        );
    }

    fn sim_capture(scenario: Scenario, n: usize) -> Capture {
        let rate = 857_142;
        let ch_count = scenario.channel_count();
        let mut gen = scope_sim::WaveformGen::new(scenario, 1);
        let channels = gen.generate_multi(ch_count, n, rate);
        let mut cap = Capture::new(1, rate, ch_count, n as u32);
        cap.channels = channels;
        cap
    }

    /// 钉住 `cal set` 的命令行表面 —— 默认值改错了是**用户可见**的。
    #[test]
    fn the_cal_set_command_line_parses_with_its_defaults() {
        let cli = Cli::try_parse_from([
            "scope-cli",
            "cal",
            "set",
            "--uid",
            "5eed00010203040506070809",
            "--volts-per-lsb",
            "0.001",
        ])
        .expect("`cal set` 应当能解析");

        match cli.target {
            Target::Cal {
                action:
                    CalAction::Set {
                        uid,
                        ch,
                        volts_per_lsb,
                        zero_lsb,
                        note,
                    },
            } => {
                assert_eq!(uid, "5eed00010203040506070809");
                assert_eq!(ch, 0, "通道默认 0 起");
                assert_eq!(zero_lsb, 2048.0, "零点默认是 12-bit 中点");
                assert_eq!(volts_per_lsb, 0.001);
                assert!(note.is_none(), "备注是可选的");
            }
            other => panic!("解析成了别的子命令：{other:?}"),
        }
    }
}
