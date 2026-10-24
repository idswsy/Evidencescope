//! # scope-gui —— 桌面上位机
//!
//! 把 `scope-core` 的命令层做成给人用的界面：连接 → 配置 → 采集 → 看波形 → 解码 I2C。
//!
//! 与 CLI / MCP 同级 —— 三者都只依赖 [`scope_core::CommandBus`] 与
//! [`scope_core::DevicePort`]，不关心背后是真硬件还是模拟器（见 `docs/01-architecture.md`）。
//!
//! ```text
//! scope-gui  ┃  scope-cli  ┃  scope-mcp      ← 三个消费方，同一套命令层
//!            ↓
//!        CommandBus
//!            ↓  DevicePort（唯一的解耦点）
//!     UART │ 模拟器
//! ```
//!
//! ## 三条纪律
//!
//! 1. **构建必须走 `./host/run.sh`** —— 中文路径会让 MinGW `ld.exe` 链接失败
//!    （见 `docs/07-dev-env.md` 坑 #1）。
//! 2. **阻塞 IO 一律在 worker 线程**（[`worker`]），UI 线程只做绘制。
//!    逻辑与绘制分家正好对应 `eframe::App` 在 0.36 起的 `logic()` / `ui()` 两段。
//! 3. **没有硬件也能运行**：默认走模拟器，`i2c_100k` 场景开箱即见波形与解码。
//!
//! ## 用法
//!
//! ```text
//! ./host/run.sh run -p scope-gui
//! ./host/run.sh run -p scope-gui -- --font "D:/fonts/思源黑体.ttf"
//! ```

#![deny(clippy::all)]
#![warn(missing_docs)]

use scope_sim::Scenario;

mod ai;
mod app;
mod config;
mod drive;
mod font;
mod msg;
mod panels;
mod vmodel;
mod worker;

fn main() -> eframe::Result<()> {
    // 只认一个可选参数：中文字体路径。
    // 系统字体列表覆盖不到时（比如精简版 Windows、CI 容器）用它兜底。
    let args: Vec<String> = std::env::args().collect();
    let font_override: Option<String> = args
        .windows(2)
        .find(|w| w[0] == "--font")
        .map(|w| w[1].clone());

    // --demo：启动后自动连模拟器并采一次。
    // 用途一是给零硬件的人一个"打开就有东西看"的入口（也方便截图核对布局），
    // 用途二是空状态和满状态的布局差很多，调界面时需要在两者之间切换。
    let demo = args.iter().any(|a| a == "--demo");

    // --demo-cont：--demo 的加强版，采完第一帧后打开「连续刷新」。
    // 用途是脚本验证连续采集下的交互（缩放/平移/双击是否仍可用），
    // 不用人手去点勾选框。
    let demo_cont = args.iter().any(|a| a == "--demo-cont");

    // --scenario <name>：指定初始模拟器场景。
    // 逐场景核对波形显示时用得上（界面上那个下拉框没法用脚本点）。
    let scenario = args
        .windows(2)
        .find(|w| w[0] == "--scenario")
        .and_then(|w| Scenario::parse(&w[1]));

    // ── --drive <需求>：**不起窗口**，直接在命令行执行一次「AI 自己配置并采集」 ──
    //
    // 存在的理由是**把会话本身与界面接线分开验**：出问题时能分清
    // 是这条路没打通，还是只是界面没接上。
    //
    //     ./host/run.sh run -p scope-gui -- --drive "看看这条总线上在发生什么"
    //     ./host/run.sh run -p scope-gui -- --drive "…" --port COM7
    if let Some(pos) = args.iter().position(|a| a == "--drive") {
        let task = args
            .get(pos + 1)
            .filter(|s| !s.starts_with("--"))
            .cloned()
            .unwrap_or_else(|| "看看这条总线上在发生什么，有没有值得注意的地方".to_string());
        let port = args
            .windows(2)
            .find(|w| w[0] == "--port")
            .map(|w| w[1].clone());
        return drive_cli(&task, port, scenario);
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 760.0])
            .with_min_inner_size([880.0, 560.0])
            .with_title("F103 Agent Scope"),
        ..Default::default()
    };

    eframe::run_native(
        "F103 Agent Scope",
        options,
        Box::new(move |cc| {
            // 字体必须在建 App 之前装好 —— 界面全是中文，缺字体就是满屏方框
            let outcome = font::install(&cc.egui_ctx, font_override.as_deref());
            Ok(Box::new(app::App::new(
                &cc.egui_ctx,
                outcome,
                demo || demo_cont,
                demo_cont,
                scenario,
            )))
        }),
    )
}

/// `--drive` 的实现：命令行执行一次完整会话，事件打到 stdout。
///
/// 它是**会话这一侧的验收台** —— 界面还没接上时用它验；
/// 界面接上之后它仍然有用（出问题时能分清是会话的毛病还是界面的）。
fn drive_cli(task: &str, port: Option<String>, scenario: Option<Scenario>) -> eframe::Result<()> {
    use drive::{DriveEvent, DriveJob, McpProcess};
    use std::io::Write as _;

    let fail = |msg: &str| -> eframe::Result<()> {
        eprintln!("\n✗ {msg}");
        std::process::exit(1);
    };

    // 配置与单发分析共用一份（同一个 key / 端点 / 模型）
    let (cfg, note) = config::load(config::default_config_path().as_deref());
    if let Some(n) = note {
        println!("配置：{n}");
    }
    if !cfg.has_key() {
        return fail("未配置 API 密钥 —— 先在 GUI 面板的「设置」中填写并保存");
    }

    let exe = match drive::find_scope_mcp() {
        Ok(p) => p,
        Err(e) => return fail(&format!("{}｜{}", e.message, e.hint)),
    };
    println!("scope-mcp: {}", exe.display());

    // **连接参数由这里钉死**，不让 AI 自己挑目标
    let connect_args = match &port {
        Some(p) => serde_json::json!({ "transport": "serial", "port": p, "baud": 921_600 }),
        None => {
            let s = scenario.unwrap_or(Scenario::I2c100k);
            serde_json::json!({ "transport": "sim", "sim_scenario": s.name() })
        }
    };

    let job = DriveJob {
        task: task.to_string(),
        cfg,
        connect_args,
    };

    println!("\n需求：{task}");
    println!("目标：{}\n", job.connect_args);

    let mut mcp = match McpProcess::spawn(&exe) {
        Ok(m) => m,
        Err(e) => return fail(&format!("{}｜{}", e.message, e.hint)),
    };

    let outcome = drive::run_session(&job, &mut mcp, &mut |ev| match ev {
        DriveEvent::Phase(p) => println!("… {p}"),
        DriveEvent::ToolCall { name, args } => println!("→ {name} {args}"),
        DriveEvent::ToolResult { name, ok, summary } => {
            println!("  {} {name}：{summary}", if ok { "✓" } else { "✗" });
        }
        DriveEvent::Warnings(ws) => {
            for w in ws {
                println!("  ⚠ {w}");
            }
        }
        DriveEvent::Capture(c) => println!(
            "  ▤ 采集 #{}  {} 点 × {} 通道 @ {} Hz",
            c.id,
            c.channels.first().map(|v| v.len()).unwrap_or(0),
            c.channels.len(),
            c.rate_hz
        ),
        DriveEvent::Finished {
            text,
            turns,
            hit_limit,
        } => {
            println!(
                "\n── 结论（{turns} 轮{}）──\n{text}",
                if hit_limit { "，撞上限被停" } else { "" }
            );
        }
        DriveEvent::Failed(e) => println!("\n✗ {}｜{}", e.message, e.hint),
    });

    mcp.shutdown(); // 优雅收场：让它自己断开，再等退出

    match outcome {
        Ok(()) => {
            let _ = std::io::stdout().flush();
            std::process::exit(0);
        }
        Err(e) => fail(&format!("{}｜{}", e.message, e.hint)),
    }
}
