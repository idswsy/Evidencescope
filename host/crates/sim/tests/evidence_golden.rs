//! A2：6 个代表场景的证据包黄金文本逐字节比对。
//!
//! # 为什么需要它
//!
//! `build_evidence` 的现有测试几乎都是 `contains` 断言 —— 它们只保证
//! 「某个词还在」，抓不住措辞、顺序、单位的漂移。而这份文本是**直接喂给
//! 模型**的：一句「未标定」变成「已标定」、一个 `Refused` 变成 `Ok`，
//! 下游结论就会跟着变，而所有旧测试仍然全绿。
//!
//! # 为什么放在 `scope-sim` 而不是 `scope-core`
//!
//! 黄金文本要来自**真实生成的波形**，而不是测试里手捏的几个样点。
//! `scope-sim` 依赖 `scope-core`，能同时拿到模拟器与 `build_evidence`；
//! 反过来放在 core 里会形成循环依赖。
//!
//! # 为什么这 6 个场景
//!
//! 覆盖证据包里所有会「分叉」的渲染路径：
//!
//! | 场景 | 它守住的路径 |
//! |---|---|
//! | `sine_1k_3v3` | 单通道周期信号：正常测量 + 频率/占空比判决 |
//! | `step_ring` | 瞬态：过冲/下陷、上升沿、分类依据 |
//! | `noise` | 判不出与 `Refused`：**不得**给 `Ok` 判决的负样本 |
//! | `i2c_100k` | 双通道 + I²C 帧表、告警原文、通道来源 |
//! | `uart_115k` | 单通道 + UART 帧/字节、真机形态 |
//! | `clock_62k` | 自然场景的数值 TIE：边沿数、σ、CI95、周期抖动 |
//!
//! # 确定性
//!
//! 模拟器同场景同 seed 逐样点确定（`waveform.rs` 的 `generation_is_
//! deterministic_for_same_seed` 已钉住）；`build_evidence` 是纯函数，
//! 不读时钟、不读环境。因此文本可以逐字节比对。
//!
//! # 更新黄金文本
//!
//! 措辞**有意**改动时，跑：
//!
//! ```text
//! SCOPE_UPDATE_EVIDENCE_GOLDEN=1 cargo test -p scope-sim --test evidence_golden
//! ```
//!
//! 然后**逐行审阅** `git diff` —— 这条命令会重写全部 6 个文件，
//! 它不判断新文本是否更好，只负责把「当前输出」记下来。

use scope_core::decoder::{decode, DecodeParams, DecodeRequest, Protocol, UartConfig};
use scope_core::{build_evidence, AcquireParams, CommandBus, EvidenceInput, ScaleSet};
use scope_sim::{Scenario, SimDevice};
use std::path::PathBuf;
use std::time::Duration;

/// 采集参数与 CLI/MCP 的默认一致：857142 Hz、中点触发。
fn acquire_params(samples: u16) -> AcquireParams {
    AcquireParams {
        samples,
        rate_hz: 857_142,
        trigger_level_lsb: 2048,
        timeout: Duration::from_secs(3),
    }
}

/// 生成一个场景的证据包。
///
/// `decode` 为 `Some` 时把解码结果与请求一并交给证据包 —— 这正是
/// MCP/GUI 的真实调用形状（`decode_req` 用来回显通道/门限）。
fn evidence_for(scenario: Scenario, req: Option<&DecodeRequest>, samples: u16) -> String {
    let mut bus = CommandBus::new(SimDevice::new(scenario));
    bus.connect().expect("模拟器连接不该失败");

    let capture =
        scope_core::acquire(&mut bus, &acquire_params(samples)).expect("模拟器采集不该失败");
    let scales = ScaleSet::uncalibrated(capture.channels.len());
    let decoded = req.map(|r| decode(&capture, r).expect("模拟器场景应当可解码"));
    let config = bus.config.clone();
    let link = bus.describe();

    build_evidence(&EvidenceInput {
        capture: &capture,
        scale: &scales,
        link: &link,
        simulated: true,
        config: config.as_ref(),
        decode: decoded.as_ref(),
        decode_req: req,
        question: None,
    })
}

/// 一个代表场景。
struct Case {
    /// 黄金文件名（不含目录）。
    file: &'static str,
    scenario: Scenario,
    /// 采集点数。TIE 场景用 4096 点（约 298 条同向沿），其余沿用默认 1024。
    samples: u16,
    /// `None` 表示这次不看解码段。
    decode: Option<fn() -> DecodeRequest>,
}

fn i2c_request() -> DecodeRequest {
    DecodeRequest {
        protocol: Protocol::I2c,
        channels: Vec::new(),
        params: DecodeParams::I2c(scope_core::I2cDecodeConfig::default()),
    }
}

fn uart_request() -> DecodeRequest {
    DecodeRequest {
        protocol: Protocol::Uart,
        channels: vec![0],
        params: DecodeParams::Uart(UartConfig {
            baud: 115_200,
            ..UartConfig::default()
        }),
    }
}

fn cases() -> Vec<Case> {
    vec![
        Case {
            file: "sine_1k_3v3.txt",
            scenario: Scenario::Sine1k3v3,
            samples: 1024,
            decode: None,
        },
        Case {
            file: "step_ring.txt",
            scenario: Scenario::StepRing,
            samples: 1024,
            decode: None,
        },
        Case {
            file: "noise.txt",
            scenario: Scenario::Noise,
            samples: 1024,
            decode: None,
        },
        Case {
            file: "i2c_100k.txt",
            scenario: Scenario::I2c100k,
            samples: 1024,
            decode: Some(i2c_request),
        },
        Case {
            file: "uart_115k.txt",
            scenario: Scenario::Uart115k,
            samples: 1024,
            decode: Some(uart_request),
        },
        Case {
            file: "clock_62k.txt",
            scenario: Scenario::Clock62k,
            samples: 4096,
            decode: None,
        },
    ]
}

/// 黄金文本目录。
fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/evidence")
}

#[test]
fn evidence_text_matches_golden_byte_for_byte() {
    let dir = fixture_dir();
    let update = std::env::var("SCOPE_UPDATE_EVIDENCE_GOLDEN").as_deref() == Ok("1");
    let mut failures = Vec::new();

    for case in cases() {
        let req = case.decode.map(|f| f());
        let got = evidence_for(case.scenario, req.as_ref(), case.samples);
        let path = dir.join(case.file);

        if update {
            std::fs::create_dir_all(&dir).expect("创建黄金文本目录");
            std::fs::write(&path, &got).expect("写黄金文本");
            continue;
        }

        match std::fs::read_to_string(&path) {
            Ok(want) if want == got => {}
            Ok(want) => failures.push(format!(
                "{} 与黄金文本不一致：\n  {}\n\n--- 期望（黄金）---\n{}\n--- 实际 ---\n{}",
                case.file,
                path.display(),
                want,
                got
            )),
            Err(e) => failures.push(format!(
                "读不到黄金文本 {}：{e}\n  运行 SCOPE_UPDATE_EVIDENCE_GOLDEN=1 cargo test -p scope-sim --test evidence_golden 生成",
                path.display()
            )),
        }
    }

    if update {
        // 更新模式下不比对：文件刚被重写，本用例的职责只是生成。
        return;
    }
    assert!(
        failures.is_empty(),
        "证据包文本发生漂移（{} 个场景）：\n\n{}",
        failures.len(),
        failures.join("\n\n")
    );
}
