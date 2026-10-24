//! 各个面板的绘制。
//!
//! 每个 `draw_*` 都是自由函数 `(&mut App, &mut Ui)`，不持有状态 ——
//! 状态全在 [`crate::app::App`] 里，绘制只读它、并按需发 `Request`。

use crate::app::App;
use crate::msg::{Request, TransportKind};
use crate::vmodel::{self, SpanKind};
use egui_plot::{HLine, Line, Plot, Polygon, Text, VLine};
use scope_core::{state_name, ScaleSet, State};
use scope_sim::Scenario;

/// 状态栏的标定角标：`(是否已标定, 短文本, 悬浮说明)`。
///
/// 抽成**纯函数**是为了能在无头 job 里测 —— GUI 的渲染没法自动验，
/// 但这三样东西的取值可以，而它们恰好是「用户会不会把占位值当准数」
/// 的唯一提示。三态**必须分开**：把部分标定说成未标定，会把已经标好的
/// 通道白白降级；说成已标定则相反，是更糟的那个方向。
pub(crate) fn calib_badge(view: &ScaleSet) -> (bool, String, String) {
    let uid = view.uid_hex().unwrap_or_else(|| "?".into());
    if view.all_calibrated() {
        let c = view.get(0);
        (
            true,
            "✓ 电压已标定".into(),
            format!(
                "uid {uid}\nCH1：{:.9} V/LSB，零点 {:.1} LSB\n修改：scope-cli cal set",
                c.volts_per_lsb, c.zero_lsb
            ),
        )
    } else if view.record_found() {
        (
            false,
            "⚠ 电压部分未标定".into(),
            format!("{}\n修改：scope-cli cal set", view.summary_note()),
        )
    } else {
        (
            false,
            "⚠ 电压未标定".into(),
            format!(
                "{}\n\n真机上要按设备 uid 存标定表才对得上。\
                 \n模拟器的 I2C 电平是 0.15/0.85·VDD，按占位换算约 ±1.15 V。",
                view.summary_note()
            ),
        )
    }
}

/// 顶部工具条：采集 / 停止 / 复位。
pub fn toolbar(app: &mut App, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.strong("EvidenceScope");
        ui.weak("F103 双通路示波器");
        ui.separator();

        let busy = app.is_busy();
        let connected = app.connected;
        // 设备让给 AI 期间，这几个按钮点下去也够不到设备 —— 禁用，
        // 并用 hover 说明**为什么**，否则用户会以为界面异常
        let driving = app.device_is_elsewhere();
        let why = "设备已交给 AI —— 左侧控件停用是交接的必然，不是界面故障";

        let mut acq = ui.add_enabled(
            connected && !busy && !driving && !app.continuous,
            egui::Button::new(acq_label(busy)),
        );
        if driving {
            acq = acq.on_disabled_hover_text(why);
        } else if app.continuous {
            acq = acq.on_disabled_hover_text("连续刷新已开启 —— 取消勾选即停，恢复单次采集");
        }
        if acq.clicked() {
            app.acquire_in_flight = true;
            app.worker.send(Request::Acquire {
                samples: app.want_samples,
                rate_hz: app.want_rate,
                trigger_level_lsb: app.want_trigger_level,
                timeout_ms: 2000,
            });
        }

        if ui
            .add_enabled(busy, egui::Button::new("取消"))
            .on_hover_text("中断正在等待的触发（最长要等 2 秒）")
            .clicked()
        {
            app.worker.cancel();
        }

        let mut rst = ui.add_enabled(connected && !busy && !driving, egui::Button::new("复位"));
        if driving {
            rst = rst.on_disabled_hover_text(why);
        }
        if rst
            .on_hover_text("RESET。设备进入 Fault 态后这是唯一的出路")
            .clicked()
        {
            app.worker.send(Request::Reset);
        }

        // 连续刷新开关（docs/09 §8：默认关、出错即停、点数/采样率沿用当前设置）。
        // 交接期间与旁边的采集/复位按钮一样禁用 —— 那时设备不在 GUI 手上。
        let cont_why = if driving { why } else { "连接设备后可用" };
        let cont_resp = ui
            .add_enabled_ui(connected && !driving, |ui| {
                ui.checkbox(&mut app.continuous, "连续刷新")
            })
            .inner
            .on_hover_text("采集一帧后立即采下一帧；出错自动停止。测直流或无信号时受 AUTO 模式 200 ms 超时限制（约 5 fps）")
            .on_disabled_hover_text(cont_why);
        // 打开开关即发第一帧 —— 之后每帧回来由 Update::Acquired 自动续采。
        // 守卫用 `acquire_in_flight` 而不是 `busy`：应用配置 / 复位期间
        // `busy` 也是 Some，但那些操作只回 ConfigApplied / StateChanged，
        // 不会有 Acquired 来接着续采，用它们守卫会把开关卡成「开着但不采」；
        // 已经有一帧在途时不重发，那一帧的回执会自己接上循环。
        if cont_resp.clicked() && app.continuous && !app.acquire_in_flight {
            app.acquire_in_flight = true;
            // 连续模式在 Update::Acquired 里不再每帧重置视口（否则一开连续
            // 就没法缩放/平移），所以会话第一帧的适配由这里置一次。
            app.fit_pending = true;
            app.worker.send(Request::Acquire {
                samples: app.want_samples,
                rate_hz: app.want_rate,
                trigger_level_lsb: app.want_trigger_level,
                timeout_ms: 2000,
            });
        }

        ui.separator();
        ui.checkbox(&mut app.show_volts, "显示电压")
            .on_hover_text("关闭则显示 ADC LSB 整数（控制链路的原生单位）");

        ui.separator();
        if ui
            .selectable_label(app.show_help, "帮助")
            .on_hover_text("技术规格 / 使用限制 / 注意事项")
            .clicked()
        {
            app.show_help = !app.show_help;
        }

        ui.separator();
        // AI 面板开关。运行中时按钮上带个点，这样面板被收起也看得出还在等结果。
        let ai_label = if app.ai.is_running() {
            "AI 分析 ●"
        } else {
            "AI 分析"
        };
        if ui
            .selectable_label(app.ai.panel_open, ai_label)
            .on_hover_text("把这次采集的测量、信号分类、解码帧与信号质量发给大模型，让它给一段解释")
            .clicked()
        {
            app.ai.panel_open = !app.ai.panel_open;
        }

        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if let Some(st) = app.state {
                let color = match st {
                    State::Fault => egui::Color32::RED,
                    State::Armed | State::Streaming => egui::Color32::YELLOW,
                    _ => egui::Color32::GRAY,
                };
                ui.colored_label(color, state_name(st));
            } else {
                ui.colored_label(egui::Color32::GRAY, "未连接");
            }
        });
    });
}

fn acq_label(busy: bool) -> &'static str {
    if busy {
        "采集中…"
    } else {
        "采集一次"
    }
}

/// 左侧：设备与连接。
pub fn device(app: &mut App, ui: &mut egui::Ui) {
    ui.heading("设备");

    ui.horizontal(|ui| {
        ui.selectable_value(&mut app.transport_kind, TransportKind::Sim, "模拟器");
        ui.selectable_value(&mut app.transport_kind, TransportKind::Serial, "串口");
    });

    match app.transport_kind {
        TransportKind::Sim => {
            ui.horizontal(|ui| {
                ui.label("场景");
                egui::ComboBox::from_id_salt("scenario")
                    .selected_text(app.scenario.name())
                    .show_ui(ui, |ui| {
                        // 直接用 `Scenario::ALL`，**不要再手抄一份清单** ——
                        // 从前这里是一个写死的数组，于是往模拟器加场景时
                        // GUI 会静默地少一个，而没有任何东西会提醒你。
                        for s in Scenario::ALL {
                            ui.selectable_value(&mut app.scenario, s, s.name());
                        }
                    });
                // 场景切换给解码协议一个合理默认（uart→UART、i2c*→I2C；用户可改）
                app.sync_decode_protocol();
            });
            if app.connected
                && ui
                    .add_enabled(!app.device_is_elsewhere(), egui::Button::new("切换场景"))
                    .on_disabled_hover_text("设备已交给 AI")
                    .clicked()
            {
                app.worker.send(Request::SetScenario(app.scenario));
            }
        }
        TransportKind::Serial => {
            ui.horizontal(|ui| {
                ui.label("端口");
                let text = if app.port.is_empty() {
                    "（先点刷新）".to_string()
                } else {
                    app.port.clone()
                };
                egui::ComboBox::from_id_salt("port")
                    .selected_text(text)
                    .show_ui(ui, |ui| {
                        let ports = app.ports.clone();
                        for (name, desc, likely) in ports {
                            let label = if likely {
                                format!("{name}  ← 疑似目标")
                            } else {
                                format!("{name}  {desc}")
                            };
                            ui.selectable_value(&mut app.port, name, label);
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("波特率");
                ui.add(egui::DragValue::new(&mut app.baud).speed(100));
            });
        }
    }

    ui.horizontal(|ui| {
        let busy = app.is_busy();
        // 「刷新端口」只对串口有意义，模拟器模式下不该占位置
        if app.transport_kind == TransportKind::Serial && ui.button("刷新端口").clicked() {
            app.worker.send(Request::ListPorts);
        }
        if app.connected {
            if ui
                .add_enabled(
                    !busy && !app.device_is_elsewhere(),
                    egui::Button::new("断开"),
                )
                .on_disabled_hover_text("设备已交给 AI")
                .clicked()
            {
                // 先关连续刷新再断开 —— 顺序反了的话，在途的续采会排在
                // Disconnect 之后，对着一个已断开的链路报错（docs/09 §8 断开即停）
                app.continuous = false;
                app.worker.send(Request::Disconnect);
            }
        } else if ui.add_enabled(!busy, egui::Button::new("连接")).clicked() {
            app.worker.send(Request::Connect {
                transport: app.transport_kind,
                port: app.port.clone(),
                baud: app.baud,
                scenario: app.scenario,
            });
        }
    });

    if let Some(info) = &app.info {
        ui.separator();
        egui::Grid::new("info").num_columns(2).show(ui, |ui| {
            ui.label("固件");
            ui.label(format!(
                "{}.{}.{}",
                (info.fw_ver >> 16) & 0xFF,
                (info.fw_ver >> 8) & 0xFF,
                info.fw_ver & 0xFF
            ));
            ui.end_row();
            ui.label("通道数");
            ui.label(info.ch_count.to_string());
            ui.end_row();
            ui.label("采样率上限");
            // 857142，不是 1000000 —— docs/04-performance.md:35 写死了这条
            ui.label(format!("{} Hz", info.rate_max_hz));
            ui.end_row();
            ui.label("单次上限");
            ui.label(format!("{} 点", info.capture_max_samples));
            ui.end_row();
        });
    }
}

/// 左侧：采集与触发配置。
///
/// 用 `Grid` 而不是一排 `ui.horizontal`：两列对齐后左标签、右控件各成一条线。
/// 控件宽度统一成 [`CTRL_W`] —— combo / 数字框 / 滑块三种控件的默认宽度各不相同，
/// 并排时右边缘参差不齐。
pub fn config(app: &mut App, ui: &mut egui::Ui) {
    ui.heading("配置");
    let editable = app.connected && !app.is_busy();

    ui.add_enabled_ui(editable, |ui| {
        egui::Grid::new("cfg_grid")
            .num_columns(2)
            .spacing([10.0, 7.0])
            .show(ui, |ui| {
                ui.label("采样率");
                egui::ComboBox::from_id_salt("rate")
                    .width(CTRL_W)
                    .selected_text(format!("{} Hz", app.want_rate))
                    .show_ui(ui, |ui| {
                        for r in vmodel::rate_choices() {
                            ui.selectable_value(&mut app.want_rate, r, format!("{r} Hz"));
                        }
                    });
                ui.end_row();

                ui.label("点数");
                ui.add_sized(
                    [CTRL_W, 20.0],
                    egui::DragValue::new(&mut app.want_samples).range(16..=4096),
                );
                ui.end_row();

                ui.label("模式");
                egui::ComboBox::from_id_salt("trig_mode")
                    .width(CTRL_W)
                    .selected_text(match app.want_trigger_mode {
                        0 => "auto",
                        2 => "single",
                        _ => "normal",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut app.want_trigger_mode, 0, "auto");
                        ui.selectable_value(&mut app.want_trigger_mode, 1, "normal");
                        ui.selectable_value(&mut app.want_trigger_mode, 2, "single");
                    });
                ui.end_row();

                ui.label("边沿");
                egui::ComboBox::from_id_salt("trig_edge")
                    .width(CTRL_W)
                    .selected_text(if app.want_trigger_edge == 0 {
                        "上升沿"
                    } else {
                        "下降沿"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut app.want_trigger_edge, 0, "上升沿");
                        ui.selectable_value(&mut app.want_trigger_edge, 1, "下降沿");
                    });
                ui.end_row();

                ui.label("触发电平");
                ui.spacing_mut().slider_width = CTRL_W - 58.0;
                ui.add(egui::Slider::new(&mut app.want_trigger_level, 0..=4095).suffix(" LSB"));
                ui.end_row();
            });

        ui.add_space(2.0);
        ui.small(format!(
            "触发比较带 ±{} LSB 迟滞",
            scope_core::f103::TRIGGER_HYSTERESIS_LSB
        ));

        // 请求值 vs 设备回显值 —— 采样率会被定时器分频量化，差异必须摆在明面上
        if let Some(cfg) = &app.config {
            if cfg.rate_hz != app.want_rate {
                ui.colored_label(
                    egui::Color32::from_rgb(230, 190, 90),
                    format!("↳ 设备实际生效 {} Hz", cfg.rate_hz),
                );
            }
        }

        ui.add_space(4.0);
        if ui
            .add_enabled(!app.device_is_elsewhere(), egui::Button::new("应用到设备"))
            .on_disabled_hover_text("设备已交给 AI —— 它的改动由它自己下发")
            .clicked()
        {
            app.worker.send(Request::ApplyConfig {
                rate_hz: app.want_rate,
                samples: app.want_samples,
                trigger_mode: app.want_trigger_mode,
                trigger_source: 0,
                trigger_edge: app.want_trigger_edge,
                trigger_level_lsb: app.want_trigger_level,
                pre_samples: (app.want_samples / 2).min(2048),
                holdoff_us: 1000,
            });
        }
    });

    if !app.connected {
        ui.small("未连接");
    } else if app.is_busy() {
        ui.small("操作进行中，配置暂不可改");
    }
}

/// 配置面板里控件的统一宽度。
const CTRL_W: f32 = 150.0;

/// 左侧：模拟器故障注入。**只在真的连着模拟器时出现。**
///
/// 这些开关的价值在于：错误路径（丢帧、CRC 错、不触发、溢出）在真实硬件上
/// 很难复现，而它们恰恰是最容易写错的地方。
pub fn faults(app: &mut App, ui: &mut egui::Ui) {
    ui.heading("故障注入");
    ui.weak("模拟器专用。用来验证错误路径不会把界面卡死。");

    let f = &mut app.faults;
    egui::Grid::new("fault_grid")
        .num_columns(2)
        .spacing([10.0, 5.0])
        .show(ui, |ui| {
            ui.label("丢帧");
            ui.add(
                egui::DragValue::new(&mut f.drop_every_n_frames)
                    .range(0..=100u32)
                    .suffix(" 帧"),
            );
            ui.end_row();

            ui.label("CRC 错");
            ui.add(
                egui::DragValue::new(&mut f.crc_err_every_n_frames)
                    .range(0..=100u32)
                    .suffix(" 帧"),
            );
            ui.end_row();

            ui.label("延迟尖峰概率");
            ui.add(egui::DragValue::new(&mut f.latency_spike_probability).range(0.0..=1.0));
            ui.end_row();

            ui.label("尖峰时长");
            ui.add(
                egui::DragValue::new(&mut f.latency_spike_ms)
                    .range(0..=5000u64)
                    .suffix(" ms"),
            );
            ui.end_row();
        });

    ui.checkbox(&mut f.no_trigger, "永不触发")
        .on_hover_text("验证「等待触发超时」时界面不会阻塞");
    ui.checkbox(&mut f.force_overrun, "强制溢出")
        .on_hover_text("验证界面会如实标记数据不完整");

    ui.horizontal(|ui| {
        let can_inject = !app.device_is_elsewhere();
        if ui
            .add_enabled(can_inject, egui::Button::new("注入"))
            .on_disabled_hover_text("设备已交给 AI")
            .clicked()
        {
            app.worker
                .send(Request::SetFaults(Box::new(app.faults.clone())));
        }
        if ui
            .add_enabled(can_inject, egui::Button::new("清除全部"))
            .on_disabled_hover_text("设备已交给 AI")
            .clicked()
        {
            app.faults = scope_sim::FaultInjection::default();
            app.worker
                .send(Request::SetFaults(Box::new(app.faults.clone())));
        }
    });
}

/// 中央区：帮助页。
///
/// 写成**说明书**的样子，不是"讲解"：名词化标题、条目以参数名打头、陈述句。
/// 参考同类工具（Keil µVision / Saleae Logic / PulseView）的说明页结构 ——
/// Specifications / Limitations / Notes。不含第二人称、感叹、建议。
pub fn help_page(app: &mut App, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.heading("帮助");
        if ui.button("← 返回波形").clicked() {
            app.show_help = false;
        }
    });
    ui.separator();

    egui::ScrollArea::vertical()
        .id_salt("help_scroll")
        .auto_shrink([false, false])
        .show(ui, |ui| {
            section(ui, "技术规格", &[
                ("I2C 协议解码", "100 kHz / 400 kHz / 1 MHz（数字通路）"),
                ("模拟带宽", "≤ 100 kHz"),
                ("采样率", "双通道同步 857 kSPS / 通道"),
                ("单次采集深度", "4096 点"),
                ("触发电平迟滞", "±16 LSB"),
                ("边沿时间戳分辨率", "13.9 ns"),
            ]);

            ui.add_space(12.0);
            section(ui, "使用限制", &[
                ("采样率", "上限 857 142 Hz。定时器整数分频，请求值将被量化"),
                ("传输", "不支持 1 MSPS 与 USB 并存"),
                ("存储", "不支持外扩；单次采集上限 4096 点"),
                ("带宽", "上限 0.5 MHz"),
                ("I2C 时序合规性", "不提供（tSU;DAT / tr 的 ns 级判定）"),
                ("测量精度", "不提供计量级结果（THD / SFDR、12-bit 绝对精度）"),
                ("量程", "机械开关切换，不支持程控"),
            ]);

            ui.add_space(12.0);
            ui.strong("注意事项");
            ui.add_space(3.0);
            // ⚠ 这些字符串不要用 `\` 续行 —— 续行后源码里的缩进会原样进到
            // 字符串里，渲染出来每句中间凭空多一大段空白。
            let notes = [
                "电压换算按设备 UID 查标定表；没有记录时用占位参数（3.3 V / 4096、零点 2048），状态栏会标出「未标定」。写入用 scope-cli cal set。",
                "上升时间的分辨率下限为 1 个采样周期。",
                "频率与占空比已排除事务之间的空闲区间；非周期信号报出的是边沿速率。",
                "协议解码走数字通路（LM393 比较器 + 定时器输入捕获），与 ADC 采样率无关；ADC 通路负责信号质量评估。",
                "AI 分析结果的正确性不作保证。其输入为本次采集的测量值、信号分类、解码帧与信号质量，\
                 输出文本应视为注释而非测量数据。",
                "API 密钥以明文存储于用户配置目录（Windows: %APPDATA%\\scope-gui\\config.json）。\
                 本应用不提供加密存储；该文件不得置于项目目录之内（应用会拒绝写入此类路径）。",
                "AI 分析结果仅保存在内存中，应用退出后不保留。如需留存须执行导出。",
            ];
            for (i, t) in notes.iter().enumerate() {
                ui.label(format!("    {}. {t}", i + 1));
            }

            ui.add_space(16.0);
            ui.separator();
            ui.weak(format!(
                "scope-gui {}  ·  协议 v1  ·  设计与性能数据见 docs/",
                scope_core::VERSION
            ));
        });
}

/// 说明页里的一段「参数名 | 参数值」表格。
fn section(ui: &mut egui::Ui, title: &str, rows: &[(&str, &str)]) {
    ui.strong(title);
    ui.add_space(3.0);
    egui::Grid::new(title)
        .num_columns(2)
        .spacing([20.0, 4.0])
        .show(ui, |ui| {
            for (k, v) in rows {
                ui.label(format!("    {k}"));
                ui.label(*v);
                ui.end_row();
            }
        });
}

/// 左侧：历史采集（最近 16 次，容量由 core 的 CaptureStore 决定）。
pub fn history(app: &mut App, ui: &mut egui::Ui) {
    ui.heading("历史采集");
    if app.store.is_empty() {
        ui.weak("无采集记录。执行采集后，记录将保留于此处，可选中回看。");
        return;
    }

    let current = app.capture.as_ref().map(|c| c.id);
    let entries: Vec<(u16, u32, u64, bool)> = app
        .store
        .iter()
        .map(|c| (c.id, c.rate_hz, c.wall_time, c.overrun))
        .collect();

    egui::ScrollArea::vertical()
        .id_salt("hist")
        .max_height(120.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            for (id, rate, _t, overrun) in entries {
                let label = format!("#{id}  {} Hz{}", rate, if overrun { "  ⚠溢出" } else { "" });
                if ui.selectable_label(current == Some(id), label).clicked() {
                    // 回看历史：换掉当前采集并重算解码，**不重新采集**
                    if let Some(c) = app.store.get(id) {
                        app.capture = Some(c.clone());
                        app.mark_decode_dirty();
                        app.fit_pending = true;
                    }
                }
            }
        });
}

/// 左侧：滚动日志。
pub fn log(app: &mut App, ui: &mut egui::Ui) {
    ui.heading("日志");
    egui::ScrollArea::vertical()
        .id_salt("log_scroll")
        .max_height(150.0)
        .auto_shrink([false, true])
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for line in &app.log {
                ui.small(line);
            }
        });
    if ui.small_button("清空").clicked() {
        app.log.clear();
    }
}

/// 底部状态栏。
pub fn status_bar(app: &mut App, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        if let Some((text, hint)) = &app.last_error {
            ui.colored_label(egui::Color32::RED, format!("✗ {text}"));
            if let Some(h) = hint {
                ui.separator();
                ui.colored_label(egui::Color32::LIGHT_RED, h);
            }
        } else if let Some(cap) = &app.capture {
            ui.label(format!(
                "{} 点 · {} Hz · {:.3} ms",
                cap.len(),
                cap.rate_hz,
                cap.duration_us() as f64 / 1000.0
            ));
            // 标定状态必须放在**一定看得见**的地方 —— y 轴标签会被
            // show_axes([true,false]) 一起藏掉，放那儿等于没写。
            if app.show_volts {
                ui.separator();
                let (ok, text, help) = calib_badge(&app.calib_view);
                let color = if ok {
                    egui::Color32::from_rgb(120, 190, 120)
                } else {
                    egui::Color32::from_rgb(180, 150, 90)
                };
                ui.colored_label(color, text).on_hover_text(help);
            }
            if cap.overrun {
                ui.colored_label(
                    egui::Color32::RED,
                    "⚠ 溢出：这份数据不完整，不能当完整波形用",
                );
            }
            if cap.trigger_index.is_none() {
                ui.colored_label(egui::Color32::YELLOW, "· 无触发点（软触发）");
            }
        } else if let Some(n) = &app.font_notice {
            ui.colored_label(egui::Color32::YELLOW, format!("⚠ {n}"));
        } else {
            ui.weak("就绪 —— 选模拟器 + i2c_100k，点「连接」再「采集一次」");
        }
    });
}

/// 中央：波形 + 解码泳道。
///
/// # 为什么每个通道占独立泳道
///
/// I2C 的 SCL 与 SDA 都是 0→3.3V 的数字信号，画在同一个坐标系里**完全重合**，
/// 根本分不出哪条是哪条。所以按通道纵向错开成"泳道"（示波器的 stacked 模式），
/// 每条泳道内部仍然是真实的电压刻度，VIH/VIL 参考线画在各自泳道里。
///
/// # 为什么默认不显示全部 4096 点
///
/// 4.778 ms 的窗口里塞了约 478 个 SCL 周期，摊在 770 px 上每周期只有 1.6 像素 ——
/// 全览时就是一团竖条纹，看不出任何形状。所以默认只显示十几个周期，
/// 鼠标滚轮可以自由缩放。
pub fn plot(app: &mut App, ui: &mut egui::Ui) {
    let Some(cap) = app.capture.clone() else {
        ui.centered_and_justified(|ui| {
            ui.weak("还没有数据。左侧选「模拟器」+ 场景 i2c_100k，点「连接」，再点「采集一次」。");
        });
        return;
    };

    let ch_count = cap.channels.len().max(1);
    // 逐通道换算，按设备 uid 查。**克隆一份**：后面 `app` 还要可变借用。
    let scales = app.calib_view.clone();

    // 操作提示与重置按钮。egui_plot 的缩放/平移是内置的，但**用户不知道** ——
    // 不给提示的话没人会去滚轮。另外缩进去之后没有出口，得给个按钮。
    ui.horizontal(|ui| {
        if ui.small_button("重置缩放").clicked() {
            app.fit_pending = true;
        }
        ui.checkbox(&mut app.lanes_overlap, "泳道重叠")
            .on_hover_text("打开后两条泳道同基线，看时序错开更直观");
        ui.weak("· 滚轮缩放 · 拖拽平移 · 双击重置");
    });

    // ── 布局常量（虚拟 y 坐标，不是电压）──
    const LANE_H: f64 = 3.2; // 一条泳道的高度（信号本身 0~2.8V，留点余量）
    const LANE_GAP: f64 = 0.7; // 泳道间隔
    const DECODE_H: f64 = 1.7; // 底部解码带的高度

    // 重叠模式：所有泳道同基线 —— 看两路信号**时序上的错开**更直观
    // （比如 SDA 在 SCL 高电平期间变化 = START/STOP）。
    // 分开模式：纵向错开，看各自的波形形状更清楚。
    let lane_step = if app.lanes_overlap {
        0.0
    } else {
        LANE_H + LANE_GAP
    };
    let lane_bottom = |i: usize| -> f64 { -(i as f64) * lane_step };
    // 解码带要贴在**最后一条泳道的下方**。
    // 回归：这里曾经写成 `lane_bottom(ch_count)`，那是"再往下一条泳道"的位置，
    // 于是 2 通道时中间凭空多出 4.9 个单位的空白。
    // 只有多于一条泳道时才需要解码带；单通道时它纯属占地方
    let show_band = ch_count >= 2;
    let decode_top = lane_bottom(ch_count - 1) - LANE_GAP;
    let decode_bottom = decode_top - DECODE_H;

    let show_volts = app.show_volts;
    // 解码门限是**解码层的量**（按 LSB 定义），不是某个通道的电压。
    // 它要在每条泳道上画成参考线，所以下面按各泳道自己的换算去画。
    let levels = app.decode_cfg.levels;

    // ── 各通道折线 ──
    let palette = [
        egui::Color32::from_rgb(120, 200, 255),
        egui::Color32::from_rgb(255, 190, 120),
        egui::Color32::from_rgb(160, 255, 160),
        egui::Color32::from_rgb(230, 160, 255),
    ];
    let mut series: Vec<(String, Vec<[f64; 2]>, egui::Color32)> = Vec::new();
    let mut lane_labels: Vec<(String, f64)> = Vec::new();
    for ch in 0..ch_count {
        let base = lane_bottom(ch);
        let sc = scales.get(ch);
        // 几何一律用电压 —— 泳道里保持真实电压刻度。
        // 「显示电压」开关只换**标签单位**，不换形状：
        // 之前关掉时把 y 算成 volts/3.3*LANE_H，那既不是伏特也不是 LSB，
        // 是个没有物理意义的缩放。
        let pts: Vec<[f64; 2]> = vmodel::waveform_points(&cap, ch, &sc)
            .into_iter()
            .map(|p| [p[0], base + p[1]])
            .collect();

        // 泳道左侧的标签：给出这条泳道的**真实**电压（或 LSB）范围。
        // 纵向错开之后 y 刻度不再等于电压，所以这个标签是唯一的读数来源。
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        for p in &pts {
            lo = lo.min(p[1]);
            hi = hi.max(p[1]);
        }
        let span_txt = if lo.is_finite() && hi > lo {
            let (lo_v, hi_v) = (lo - base, hi - base);
            if show_volts {
                format!("CH{}  {lo_v:.2}–{hi_v:.2} V", ch + 1)
            } else {
                format!(
                    "CH{}  {}–{} LSB",
                    ch + 1,
                    sc.volts_to_lsb(lo_v),
                    sc.volts_to_lsb(hi_v)
                )
            }
        } else if lo.is_finite() {
            // 直流：范围零宽，直接报那一个电平
            let v = lo - base;
            if show_volts {
                format!("CH{}  {v:.2} V", ch + 1)
            } else {
                format!("CH{}  {} LSB", ch + 1, sc.volts_to_lsb(v))
            }
        } else {
            format!("CH{}", ch + 1)
        };
        // 重叠模式下所有泳道同基线，标签会叠在同一个位置互相盖住 ——
        // 按通道号纵向错开。分开模式下各泳道本来就不在一处，居中即可。
        // 单通道自适应纵轴时（见 y_top/y_bot 的计算），标签贴在波形顶部
        // 上方 —— 固定位置会落在自适应范围之外，标签直接消失。
        let label_y = if ch_count == 1 && !show_band {
            let hi = pts.iter().map(|p| p[1]).fold(f64::MIN, f64::max);
            (hi + 0.25).max(0.4)
        } else if app.lanes_overlap {
            LANE_H * (0.42 - ch as f64 * 0.20)
        } else {
            base + LANE_H * 0.5
        };
        lane_labels.push((span_txt, label_y));

        series.push((format!("CH{}", ch + 1), pts, palette[ch % palette.len()]));
    }

    // ── 每条泳道内的 VIH / VIL 参考线 ──
    // 逐泳道用各自的换算：线的位置必须和那条泳道的纵轴刻度同源，
    // 否则标定之后参考线会与波形错位。
    let mut thresholds: Vec<([f64; 2], egui::Color32)> = Vec::new();
    for ch in 0..ch_count {
        let base = lane_bottom(ch);
        let sc = scales.get(ch);
        let c = egui::Color32::from_gray(80);
        thresholds.push((
            [
                base + sc.lsb_to_volts(levels.vih_lsb),
                base + sc.lsb_to_volts(levels.vil_lsb),
            ],
            c,
        ));
    }

    // ── 解码色块，只画在底部专用带里，不覆盖波形 ──
    let total_us = vmodel::duration_us(&cap);
    let spans = app
        .decode
        .as_ref()
        .map(|d| vmodel::event_spans(d, total_us))
        .unwrap_or_default();
    let span_polys: Vec<(Vec<[f64; 2]>, egui::Color32)> = spans
        .iter()
        .map(|s| {
            let color = span_color(s.kind, s.nack);
            (
                vec![
                    [s.t0_us, decode_bottom],
                    [s.t1_us, decode_bottom],
                    [s.t1_us, decode_top],
                    [s.t0_us, decode_top],
                ],
                color,
            )
        })
        .collect();

    // ── 默认时间窗口：正好装下**第一笔完整事务** ──
    //
    // 一笔 I2C 事务（START + 3 字节 + STOP）要 29 个 SCL 周期 ≈ 290 µs @100 kHz。
    // 按"周期数"取窗口会只能看到 1.5 个事件，色块颜色都来不及变；
    // 按"一笔事务"取，才是 I2C 调试真正要看的最小单位。
    let span_us = app
        .decode
        .as_ref()
        .and_then(|d| d.transactions.first())
        .map(|tx| {
            let last = tx.events.last().map(|e| e.time_us).unwrap_or(0) as f64;
            (last - tx.start_time_us as f64) * 1.15 + 20.0
        })
        .or_else(|| {
            // 还没解出事务时，退回"约 15 个 SCL 周期"
            app.decode
                .as_ref()
                .and_then(|d| d.quality.scl_freq_hz)
                .map(|f| 15.0 * 1_000_000.0 / f as f64)
        })
        // 没有解码结果（非 I2C 信号）：按上升沿反推周期，显示约 4 个周期。
        // 回归：这里曾经硬取 total_us * 0.03，把 1 kHz 正弦显示成了一条直线。
        .unwrap_or_else(|| vmodel::suggest_span_us(&cap))
        .min(total_us.max(1.0))
        .max(1.0);

    let trig_x = cap
        .trigger_index
        .map(|i| i as f64 * 1_000_000.0 / cap.rate_hz.max(1) as f64);

    // 单通道时 y 轴按信号实际范围自适应 —— 示波器的「自动量程」。
    //
    // 泳道布局为 3.3 V 满量程信号预留了固定高度：一个 ±1.15 V 的方波
    // 只占纵向空间的一半多，上下留白把波形衬得很小（2026-10-07 用户做
    // 单通道连续检测时的反馈：「波形在采集框里看不全」）。
    //
    // 双通道保持泳道布局：两路信号要对齐同一纵轴做时序对照，各自
    // 适配会破坏对比。`default_y_bounds` 每帧都设，所以**双击重置**会
    // 按当帧信号重新适配；连续模式下信号幅度稳定时画面不跳。
    let (y_bot, y_top) = if ch_count == 1 && !show_band {
        let mut lo = f64::MAX;
        let mut hi = f64::MIN;
        if let Some((_, pts, _)) = series.first() {
            for p in pts {
                lo = lo.min(p[1]);
                hi = hi.max(p[1]);
            }
        }
        if lo.is_finite() && hi > lo {
            // 上下各留 10% 边距，最小 0.15 个单位 —— 全平之外的退化路径由下方分支处理
            let m = ((hi - lo) * 0.1).max(0.15);
            (lo - m, hi + m)
        } else {
            // 全平信号（直流）：以信号电平为中心的固定小窗。
            // ⚠ 不能用固定 (-0.3, 0.3)：信号不在 0 V 附近时（比如 0.24 V
            // 的直流），平线会被压在绘图区边缘贴边（2026-10-07 截图验证）。
            let center = if lo.is_finite() { lo } else { 0.0 };
            (center - 0.3, center + 0.3)
        }
    } else {
        let y_top = lane_bottom(0) + LANE_H + 0.4;
        let y_bot = if show_band {
            decode_bottom - 0.3
        } else {
            lane_bottom(ch_count - 1) - 0.3
        };
        (y_bot, y_top)
    };

    let fit = app.fit_pending;
    app.fit_pending = false;

    let mut p = Plot::new("scope")
        .x_axis_label("时间 (µs)")
        // 泳道纵向错开之后，y 刻度对第 2 条以后的泳道**不再是电压**
        // （CH1 恰好是，CH2 整体下移）。留着刻度会被误读，所以关掉 y 刻度，
        // 改由每条泳道左侧的标签给出真实电压范围。
        .show_axes([true, false])
        // 十字准线的 y 读数在泳道模式下不是电压（CH2 起整体下移了），
        // 留着会被当成电压读。时间信息由交易表和横轴给。
        .show_crosshair(false)
        // 电压换算的出处必须标在轴上：不标的话等于把一个猜测当成测量值
        // 展示 —— 违反项目纪律「无可信结论必须显式拒答，不得输出看似精确的无依据数值」。
        // 注意 `show_axes([true, false])` 在泳道模式下会**连轴标签一起藏掉**，
        // 所以这句话在别处（状态栏角标）也有一份 —— 那才是兜底。
        .y_axis_label(if scales.all_calibrated() {
            "泳道（电压已标定）"
        } else {
            "泳道（电压未标定）"
        })
        .legend(egui_plot::Legend::default())
        .allow_scroll(false)
        .default_y_bounds(y_bot, y_top);

    // 新采集时贴合到默认窗口；鼠标滚轮可自由缩放
    p = if fit {
        p.reset().default_x_bounds(0.0, span_us)
    } else {
        p
    };

    p.show(ui, move |pui| {
        // **普通滚轮 = 时间轴缩放**（示波器惯例，与上方提示「滚轮缩放」对齐）。
        //
        // egui 的 `zoom_delta` 只由 **Ctrl+滚轮** / 触控板捏合驱动（见 egui
        // input_state 的说明），普通滚轮走 scroll 路径 —— 而本图显式关了
        // `allow_scroll`，于是「滚轮缩放」这句提示在普通滚轮上是**假的**。
        // 2026-10-06 注入测试实测：普通滚轮 0 像素变化，Ctrl+滚轮才缩放。
        //
        // 这里把普通滚轮手动映射成 x 轴缩放（只动时间轴，泳道布局动 y 没有
        // 意义）；Ctrl+滚轮仍走 egui_plot 内置路径，两者不冲突。
        let (scroll_y, ctrl) = pui
            .ctx()
            .input(|i| (i.smooth_scroll_delta.y, i.modifiers.ctrl));
        if scroll_y.abs() > 0.0 && !ctrl && pui.response().hovered() {
            // exp 映射：向上滚放大、向下滚缩小，格数越多缩放越平滑。
            // 0.003 / 格（约 50 pt）≈ 每格 1.16 倍。
            let factor = ((scroll_y as f64) * 0.003).exp();
            pui.zoom_bounds_around_hovered(egui::Vec2::new(factor as f32, 1.0));
        }

        for (pts, color) in span_polys {
            pui.polygon(Polygon::new("", pts).fill_color(color));
        }
        for ([hi, lo], color) in thresholds {
            pui.hline(HLine::new("VIH", hi).color(color));
            pui.hline(HLine::new("VIL", lo).color(color));
        }
        if show_band {
            for y in [decode_top, decode_bottom] {
                pui.hline(HLine::new("", y).color(egui::Color32::from_gray(55)));
            }
        }
        for (name, pts, color) in series {
            pui.line(Line::new(name, pts).color(color));
        }
        for (text, y) in lane_labels {
            // 锚点必须显式写成左对齐：egui_plot 的 `Text` 默认是
            // `Align2::CENTER_CENTER`，文本以给定点为中心，左半截会伸到
            // 绘图区外被裁掉 —— 「CH1」那三个字就是这么凭空消失的。
            pui.text(
                Text::new("", egui_plot::PlotPoint::new(span_us * 0.02, y), text)
                    .anchor(egui::Align2::LEFT_CENTER),
            );
        }
        if let Some(x) = trig_x {
            pui.vline(VLine::new("触发", x).color(egui::Color32::from_rgb(255, 80, 80)));
        }
    });
    // 色标**不在这里**画 —— `Plot::show` 会占用全部可用高度，
    // 跟在它后面的行会被压成零高，只漏出一排残影。
    // 唯一的那份在下面 `decode_panel` 里。
}

/// 色块不透明版 —— 图例里用。
///
/// 数据带上的色块是半透明的（压在黑色绘图区上），直接拿来做图例会发灰，
/// 跟用户在图上看到的对不上。所以图例取同样的色相、拉满不透明度。
fn span_color_solid(kind: SpanKind, nack: bool) -> egui::Color32 {
    let c = span_color(kind, nack);
    egui::Color32::from_rgb(c.r(), c.g(), c.b())
}

/// 画一个「色块 + 文字」的图例项。
///
/// 不用 `allocate_exact_size` 配 `ui.small` 的组合 —— egui 的横向布局会把
/// 两者各自按当前行高对齐，结果是**色块浮在文字上沿**（实测偏 3~4 px）。
/// 这里把两者的位置**都从同一个 `rect` 的中心线算出来**，从根上杜绝错位。
///
/// `swatch = None` 时只画文字（用于行首的「色标」两个字）。
fn legend_item(ui: &mut egui::Ui, swatch: Option<egui::Color32>, text: &str) {
    const SW: f32 = 14.0; // 色块宽
    const SH: f32 = 11.0; // 色块高
    const GAP: f32 = 5.0; // 色块与文字的间距
    const ROW: f32 = 17.0; // 单项占的行高，固定值才可预期

    let color = ui.visuals().text_color();
    let galley =
        ui.painter()
            .layout_no_wrap(text.to_owned(), egui::FontId::proportional(11.5), color);

    let lead = if swatch.is_some() { SW + GAP } else { 0.0 };
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(lead + galley.size().x, ROW),
        egui::Sense::hover(),
    );
    let cy = rect.center().y;

    if let Some(c) = swatch {
        ui.painter().rect_filled(
            egui::Rect::from_center_size(
                egui::pos2(rect.left() + SW / 2.0, cy),
                egui::vec2(SW, SH),
            ),
            2.0,
            c,
        );
    }
    ui.painter().galley(
        egui::pos2(rect.left() + lead, cy - galley.size().y / 2.0),
        galley,
        color,
    );
}

/// 解码色标 —— 告诉人「黄蓝绿紫分别是什么」。
///
/// 不放进 egui_plot 的图例里：那样会和 CH1/CH2/VIH/VIL/触发 挤成一长条，
/// 而且数据带上的色块是半透明的，图例里的色块看起来对不上色。
fn span_legend(ui: &mut egui::Ui) {
    let items: [(SpanKind, bool, &str); 6] = [
        (SpanKind::Start, false, "起始位"),
        (SpanKind::Address, false, "地址"),
        (SpanKind::Data, false, "数据"),
        (SpanKind::Stop, false, "停止位"),
        (SpanKind::Data, true, "NACK"),
        (SpanKind::Truncated, false, "截断"),
    ];

    ui.horizontal_wrapped(|ui| {
        legend_item(ui, None, "色标");
        ui.add_space(8.0);
        for (kind, nack, label) in items {
            legend_item(ui, Some(span_color_solid(kind, nack)), label);
            ui.add_space(10.0);
        }
    });
}

fn span_color(kind: SpanKind, nack: bool) -> egui::Color32 {
    if nack {
        return egui::Color32::from_rgba_unmultiplied(255, 70, 70, 150);
    }
    // 事件跨度是首尾相接的，色块连成一片。不透明度压得太低会糊成一色，
    // 所以这里给到 130~150 —— 既压得住深色背景，又能分出相邻事件。
    match kind {
        SpanKind::Start => egui::Color32::from_rgba_unmultiplied(255, 200, 60, 150),
        SpanKind::Address => egui::Color32::from_rgba_unmultiplied(70, 150, 255, 140),
        SpanKind::Data => egui::Color32::from_rgba_unmultiplied(70, 210, 110, 120),
        SpanKind::Stop => egui::Color32::from_rgba_unmultiplied(200, 120, 255, 150),
        SpanKind::Truncated => egui::Color32::from_rgba_unmultiplied(200, 200, 200, 110),
    }
}

/// 底部面板：解码结果（参数、质量、告警、交易表）。
pub fn detail_panel(app: &mut App, ui: &mut egui::Ui) {
    measure_section(app, ui);
    ui.separator();
    decode_panel(app, ui);
}

/// 测量结果 —— 每个通道一行。
fn measure_section(app: &mut App, ui: &mut egui::Ui) {
    let Some(cap) = app.capture.clone() else {
        return;
    };
    // 逐通道换算，按设备 uid 查。**克隆一份**：下面 `app` 还要可变借用。
    let scales = app.calib_view.clone();
    // 与证据包/MCP 同一个分析结果对象 —— 界面不再自己解释 Measurements。
    let link = app.link_description();
    let simulated = app.evidence_simulated();
    let report = scope_core::AnalysisReport::from_capture(&cap, &scales, &link, simulated);

    ui.horizontal(|ui| {
        ui.strong("测量");
        // 导出路径可编辑（无文件对话框依赖）；空 = 默认 captures/capture_<id>.csv。
        // 写入前过 core::persist 闸门 —— 导出不得落入项目目录。
        let default_path = crate::app::default_csv_path(&cap);
        let shown = if app.csv_path.is_empty() {
            default_path.clone()
        } else {
            app.csv_path.clone()
        };
        ui.add(
            egui::TextEdit::singleline(&mut app.csv_path)
                .hint_text(default_path)
                .desired_width(220.0),
        );
        if ui
            .button("导出 CSV")
            .on_hover_text("全量样点写盘（不进界面、不占内存）；与 CLI/MCP 同一序列化")
            .clicked()
        {
            match crate::app::export_csv(&shown, &cap, &scales) {
                Ok(p) => app.note(format!("已写出 {p}")),
                Err(e) => app.note(format!("✗ {e}")),
            }
        }
        // 导出解码结果：按当前协议给文本（I2C=交易表，UART=帧表），与证据包同源。
        let decode_export = match app.decode_protocol {
            scope_core::Protocol::I2c => app.decode.as_ref().map(|d| (d.to_text(), "i2c.txt")),
            scope_core::Protocol::Uart => app
                .uart_decode
                .as_ref()
                .map(|d| (d.uart_text(), "uart.txt")),
        };
        if let Some((text, ext)) = decode_export {
            if ui.button("导出解码结果").clicked() {
                let path = capture_path(&cap, ext);
                match std::fs::write(&path, text) {
                    Ok(()) => app.note(format!("已写出 {path}")),
                    Err(e) => app.note(format!("✗ 写解码结果失败：{e}")),
                }
            }
        }
    });

    for ch in 0..cap.channels.len() {
        let Some(ca) = report.channel(ch) else {
            continue;
        };
        let m = &ca.measurements;
        let freq = m
            .freq_hz
            .map(|f| format!("{:.1} kHz", f / 1000.0))
            .unwrap_or_else(|| "—".into());
        let duty = m
            .duty_pct
            .map(|d| format!("{d:.1}%"))
            .unwrap_or_else(|| "—".into());
        let rise = m
            .rise_ns
            .map(|r| format!("{r:.0} ns"))
            .unwrap_or_else(|| "—".into());
        let fall = m
            .fall_ns
            .map(|f| format!("{f:.0} ns"))
            .unwrap_or_else(|| "—".into());
        let over = m
            .overshoot_pct
            .map(|p| format!("{p:.1}%"))
            .unwrap_or_else(|| "—".into());
        let under = m
            .undershoot_pct
            .map(|p| format!("{p:.1}%"))
            .unwrap_or_else(|| "—".into());
        let high = m
            .high_ns
            .map(|h| {
                if h >= 1000.0 {
                    format!("{:.1} µs", h / 1000.0)
                } else {
                    format!("{h:.0} ns")
                }
            })
            .unwrap_or_else(|| "—".into());
        ui.monospace(format!(
            "CH{:<2}  Vpp {:>6.2} V   均值 {:>6.2} V   有效值 {:>6.2} V   频率 {:>10}   占空比 {:>6}",
            ch + 1, m.vpp, m.mean, m.ac_rms, freq, duty
        ));
        // 第二行：新增的量。过冲/下冲的参考电平是平台均值（不是全窗最大值），
        // 这个口径必须和证据包一致 —— 两个地方两个口径，用户对不上数。
        ui.monospace(format!(
            "      上升 {:>8}   下降 {:>8}   过冲/下冲 {:>11}   脉宽 {:>8}   跃变 {:>3}",
            rise,
            fall,
            format!("{over}/{under}"),
            high,
            m.edges
        ));
        // 第三行：质量参数 + 判决。判决文本与证据包/CLI 是同一份
        // （core::analysis::verdict_line），界面不另写第二套措辞 ——
        // 与 F2 在 CLI 的做法一致。
        let noise = ca
            .quality
            .noise_lsb
            .map(|v| format!("{v:.1}"))
            .unwrap_or_else(|| "—".into());
        let h = ca
            .quality
            .hysteresis_lsb
            .map(|v| format!("{v:.1}"))
            .unwrap_or_else(|| "—".into());
        ui.weak(format!("      σ̂={noise} LSB  迟滞={h} LSB"));
        if let Some(v) = scope_core::analysis::verdict_line(&ca.quality) {
            ui.weak(format!("      {v}"));
        }
    }
    // 这几个边界必须写出来，否则数字会被过度解读：
    // 「有效值」是扣除直流后的 RMS（交流信号看这个）；频率/占空比排除了
    // 事务间的空闲（否则 100 kHz 会被算成 95 kHz）；上升时间受采样率限制；
    // 数据线的「频率」只是跳变速率，不是时钟。
    let dt_ns = if cap.rate_hz > 0 {
        1e9 / cap.rate_hz as f64
    } else {
        0.0
    };
    ui.weak(format!(
        "有效值 = 扣除直流后的 RMS · 占空比按 min/max 中值判定（与解码门限无关）\
         · 频率与占空比已排除事务间空闲 · 上升/下降时间分辨率下限 1 个采样周期（{dt_ns:.0} ns）\
         · 过冲参考电平为平台均值 · 非周期信号（数据线）的「频率」只是边沿速率，参考意义有限"
    ));

    // 瞬态与 TIE —— 与证据包同一渲染（analysis::transient_tie_lines），单一来源；
    // 「测不出」与判决纪律（无模型即拒答）由此原样进入界面。
    ui.add_space(2.0);
    ui.strong("瞬态与 TIE");
    for ch in 0..cap.channels.len() {
        if let Some(ca) = report.channel(ch) {
            for line in scope_core::analysis::transient_tie_lines(ca) {
                ui.monospace(format!("CH{:<2} {}", ch + 1, line));
            }
        }
    }
}

/// 生成导出路径：`<当前目录>/captures/<时间>_<id>.<后缀>`。
///
/// 不用文件对话框（`rfd` 在 Linux 上要拖 GTK 进 CI）；固定目录 + 把完整路径
/// 写进日志，用户知道文件去哪了。
fn capture_path(cap: &scope_core::Capture, ext: &str) -> String {
    let dir = std::path::PathBuf::from("captures");
    let _ = std::fs::create_dir_all(&dir);
    // 用采集编号 + 采样率命名，不用墙钟（避免依赖时间函数，也便于复现）
    let name = format!("cap{}_{}Hz.{}", cap.id, cap.rate_hz, ext);
    dir.join(name).to_string_lossy().into_owned()
}

/// 右侧的 AI 分析面板。
///
/// # 这一栏里最要紧的东西是那行锚点
///
/// 三个独立设计都把「拿模拟器的结论当成自己板子的结论」列为最严重的风险。
/// 所以「这是第几次采集 / 是不是模拟器」必须**常显**，不能折叠、不能滚走。
pub fn ai_panel(app: &mut App, ui: &mut egui::Ui) {
    ui.heading("AI 分析");
    ui.add_space(4.0);

    // ── 分析对象：常显，不折叠 ──
    let anchor = app.capture_anchor();
    if app.simulated {
        ui.colored_label(egui::Color32::from_rgb(230, 160, 30), format!("⚠ {anchor}"));
        ui.small("数据源为模拟器。结论不适用于真实硬件。");
    } else {
        ui.label(&anchor);
    }
    ui.separator();

    // ── 设置 ──
    egui::CollapsingHeader::new("设置（密钥 / 端点 / 模型）")
        .default_open(!app.ai.cfg.has_key())
        .show(ui, |ui| config_section(app, ui));

    ui.add_space(6.0);

    // ── 交出去 / 自己分析 ──
    let has_capture = app.capture.is_some();
    let can_start = app.ai.can_start() && has_capture;
    let running = app.ai.is_running();
    let driving = app.device_is_elsewhere();

    ui.label("需求 / 分析要求");
    ui.add_enabled(
        !running && !driving,
        egui::TextEdit::multiline(&mut app.ai.question)
            .desired_rows(2)
            .hint_text("示例：看看这条总线上在发生什么"),
    );

    ui.horizontal(|ui| {
        // 「交给 AI 采集」—— AI 接管设备，自行配置并按需重采
        let can_drive = app.can_start_drive() && app.ai.cfg.has_key();
        let drive_btn = ui.add_enabled(can_drive, egui::Button::new("交给 AI 采集"));
        let drive_btn = if !app.connected {
            drive_btn.on_disabled_hover_text("尚未连接设备。连接后方可交接。")
        } else if !app.ai.cfg.has_key() {
            drive_btn.on_disabled_hover_text("未配置 API 密钥。请在「设置」中填写。")
        } else if driving {
            drive_btn.on_disabled_hover_text("设备当前由 AI 持握。")
        } else {
            // 可用时也要说明它**做什么**。按钮上只放得下两个字，而它实际
            // 会配置、按需重采并分析，期间设备控件全部停用。
            drive_btn.on_hover_text(
                "AI 接管设备：自行配置、采集并分析，必要时会重采。\
                 期间界面上的设备控件停用；结束后设备自动交还。",
            )
        };
        if drive_btn.clicked() {
            app.start_drive();
        }

        // 「分析当前采集」—— 只解释屏幕上这一窗，不碰设备。
        //
        // 「当前」不是「这次」：代码里管这一窗就叫「当前采集」
        // （`app.capture`、以及「结论对不上当前采集」那句提示），
        // 界面上换个说法就是又一处同一件事两种叫法。
        let btn = egui::Button::new(if running {
            "分析中…"
        } else {
            "分析当前采集"
        });
        let resp = ui.add_enabled(can_start && !driving, btn);
        let resp = if !has_capture {
            resp.on_disabled_hover_text("无采集数据。请先连接设备并执行一次采集。")
        } else if !app.ai.cfg.has_key() {
            resp.on_disabled_hover_text("未配置 API 密钥。请在「设置」中填写。")
        } else {
            resp
        };
        if resp.clicked() {
            start_analysis(app);
        }

        if running && ui.button("取消").clicked() {
            app.ai.cancel();
            app.note("AI 分析已取消。在途请求将于超时后返回，届时应标记为迟到结果。");
        }
    });

    // ── 设备在 AI 手上 ──
    if driving {
        ui.add_space(4.0);
        ui.colored_label(
            egui::Color32::from_rgb(230, 160, 30),
            "⚠ 设备已交给 AI —— 左侧所有设备控件已停用",
        );

        // ⚠ 只在 `Running` 阶段给终止。
        //
        // `Disconnecting` 时会话还没起、句柄还是空的，点了什么也终止不了 ——
        // 摆一个按下去没反应的按钮比不给更糟。而那个窗口只有约一秒
        // （断开本身很快），之后就能终止了。
        let can_stop = app.drive.phase == crate::drive::DrivePhase::Running;
        if ui
            .add_enabled(can_stop, egui::Button::new("终止"))
            .on_disabled_hover_text("正在让出设备，会话尚未开始")
            .clicked()
        {
            app.stop_drive();
        }
    }

    if running {
        ui.small("分析在后台线程执行，界面保持响应。分析期间可继续缩放、浏览历史或重新采集。");
    }

    ui.separator();

    drive_section(app, ui);

    // ── 结果 ──
    if let Some(err) = app.ai.error.clone() {
        ui.colored_label(egui::Color32::RED, format!("✗ {}", err.message));
        ui.small(format!("处理：{}", err.hint));
        ui.separator();
    }

    let Some(answer) = app.ai.answer.clone() else {
        ui.weak("无分析结果。");
        return;
    };

    // 结果对应的采集已经不是当前显示的那个 —— 这条必须显眼
    if app
        .ai
        .answer_is_about_another_capture(app.capture.as_ref().map(|c| c.id))
    {
        ui.colored_label(
            egui::Color32::from_rgb(230, 160, 30),
            format!(
                "⚠ 下列结论对应 {}，非当前显示的采集。",
                app.ai.answer_anchor
            ),
        );
    }
    if app.ai.answer_late {
        ui.weak("（该结果迟于请求返回：此前已取消，或已发起新的分析。）");
    }

    // 只报模型与耗时。
    //
    // **不显示 token 用量**：响应里的 `usage` 字段与请求体大小对不上
    // （实测 3512 字符的请求报 239 输入 token，中文按那个比值不可能），
    // 含义说不清就不该摆出来 —— 显示一个来路不明的数字比不显示更坏，
    // 它会让人当成真的并据此做成本判断。刻度用字符数，那个是确定的。
    ui.horizontal_wrapped(|ui| {
        ui.small(format!("模型 {}", answer.model));
        ui.small(format!("· 用时 {:.1} s", answer.elapsed.as_secs_f32()));
    });

    // 被截断的正文与完整的正文在屏幕上长得一样 —— 而「话说了一半」正是
    // 最容易被当成「说完了」的那种错。所以这条提示必须紧贴正文上方。
    if answer.truncated {
        ui.colored_label(
            egui::Color32::from_rgb(230, 160, 30),
            "⚠ 响应被服务端截断（max_tokens）—— 以下内容不完整。",
        );
    }
    ui.add_space(4.0);

    // 正文用等宽 + 可选中：结论里全是数字，等宽好读也好复制
    ui.add(
        egui::TextEdit::multiline(&mut answer.text.as_str())
            .desired_width(f32::INFINITY)
            .desired_rows(16)
            .font(egui::TextStyle::Monospace),
    );

    ui.horizontal(|ui| {
        if ui.button("复制").clicked() {
            ui.ctx().copy_text(answer.text.clone());
            app.note("AI 结论已复制到剪贴板。");
        }
        if ui.button("导出 .md").clicked() {
            if let Some(cap) = app.capture.clone() {
                export_answer(app, &cap, &answer);
            }
        }
    });
    ui.small("分析结果仅保存在内存中，应用退出后不保留。如需留存请执行导出。");
}

/// AI 驱动会话的那一段：阶段 + 动作轨迹 + 结论 + 恢复设置。
///
/// # 为什么是「动作清单」而不是聊天记录
///
/// 示波器上选错时基或触发电平，会得到一份**看起来完全合理**的新波形，
/// 而模型会带着同样的自信下结论 —— 错误是沉默的。用户恰恰是为了
/// 「不再关心这些参数」才用这个功能，所以**「改了什么」必须看得见**。
fn drive_section(app: &mut App, ui: &mut egui::Ui) {
    use crate::drive::{DrivePhase, TrajectoryItem};

    if app.drive.phase == DrivePhase::Idle && app.drive.trajectory.is_empty() {
        return; // 没跑过 —— 不占地方
    }

    ui.heading("AI 动作");

    // 阶段横幅
    let (label, color) = match &app.drive.phase {
        DrivePhase::Idle => ("已结束".to_string(), egui::Color32::GRAY),
        DrivePhase::Disconnecting => ("正在让出设备…".to_string(), egui::Color32::YELLOW),
        DrivePhase::Running => (
            "AI 正在操作设备".to_string(),
            egui::Color32::from_rgb(230, 160, 30),
        ),
        DrivePhase::Reconnecting => ("正在收回设备…".to_string(), egui::Color32::YELLOW),
    };
    ui.colored_label(color, label);

    // 轨迹
    egui::ScrollArea::vertical()
        .id_salt("drive_trace")
        .max_height(180.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            for item in &app.drive.trajectory {
                match item {
                    TrajectoryItem::Phase(p) => {
                        ui.small(egui::RichText::new(format!("· {p}")).weak());
                    }
                    TrajectoryItem::Call { name, args } => {
                        ui.small(format!("→ {name}  {args}"));
                    }
                    TrajectoryItem::Result { name, ok, summary } => {
                        let mark = if *ok { "✓" } else { "✗" };
                        let text = format!("   {mark} {name}：{summary}");
                        if *ok {
                            ui.small(text);
                        } else {
                            ui.small(egui::RichText::new(text).color(egui::Color32::RED));
                        }
                    }
                    // 警告要显眼 —— 它讲的往往是「用户以为已生效，实际未生效」
                    TrajectoryItem::Warning(w) => {
                        ui.small(
                            egui::RichText::new(format!("   ⚠ {w}"))
                                .color(egui::Color32::from_rgb(230, 160, 30)),
                        );
                    }
                    TrajectoryItem::Capture {
                        id,
                        points,
                        channels,
                    } => {
                        ui.small(format!(
                            "   ▤ 采集 #{id}  {points} 点 × {channels} 通道 —— 已上屏"
                        ));
                    }
                }
            }
        });

    // AI 的最终结论
    if let Some(text) = app.drive.final_text.clone() {
        ui.add_space(4.0);
        ui.label("结论");
        // ⚠ 高度**随内容**，不写死。
        //
        // 写死 `desired_rows(10)` 时，一条一行的结论也占十行 ——
        // 而它下面的「恢复交接前的设置」是「AI 改错了怎么改回去」的唯一入口，
        // 被挤到滚动线以下就**不滚动看不到**，等于没有。
        // （这个是盯着截图才发现的：读代码时那个按钮明明在那儿。）
        let rows = text.lines().count().clamp(3, 16);
        ui.add(
            egui::TextEdit::multiline(&mut text.as_str())
                .desired_width(f32::INFINITY)
                .desired_rows(rows)
                .font(egui::TextStyle::Monospace),
        );
    }

    // 设备收回来之后才谈得上恢复设置
    if app.drive.phase == DrivePhase::Idle && app.drive.handoff.is_some() {
        ui.add_space(4.0);
        if ui
            .button(crate::drive::RESTORE_LABEL)
            .on_hover_text("将「期望配置」填回交接时的值，由「应用到设备」下发")
            .clicked()
        {
            app.restore_handoff_config();
        }
        ui.small("AI 的改动保留在设备上。左侧「配置」面板为设备回显值。");
    }

    ui.separator();
}

/// 设置区：key / 端点 / 模型。
fn config_section(app: &mut App, ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.label("密钥");
        ui.add(
            egui::TextEdit::singleline(&mut app.ai.cfg.api_key)
                .password(!app.ai.show_key)
                .desired_width(180.0)
                .hint_text("sk-…"),
        );
        ui.toggle_value(&mut app.ai.show_key, "👁")
            // ⚠ 不要在这里写 markdown —— egui 的 Label 与 tooltip 都**不渲染**它，
            // 星号会原样显示。这个坑 HANDOFF 里记着，我还是踩了一次。
            .on_hover_text("明文显示 / 掩码显示。明文显示状态下，请确认当前屏幕无未授权旁观者。");
    });

    ui.horizontal(|ui| {
        ui.label("端点");
        // 占满剩余宽度：这个 URL 很长，写死宽度会把 "/messages" 的尾巴裁掉,
        // 而**恰好是那一截**决定了它是不是 messages 接口。
        ui.add(
            egui::TextEdit::singleline(&mut app.ai.cfg.base_url)
                .desired_width(ui.available_width())
                .hint_text(scope_gui_default_url()),
        );
    });

    ui.horizontal(|ui| {
        ui.label("模型");
        ui.add(egui::TextEdit::singleline(&mut app.ai.cfg.model).desired_width(180.0));
    });

    ui.horizontal(|ui| {
        if ui.button("保存到配置文件").clicked() {
            let msg = app.ai.save_config();
            app.note(format!("AI 配置：{msg}"));
        }
        if ui.button("恢复默认端点").clicked() {
            app.ai.cfg.base_url = crate::config::DEFAULT_BASE_URL.to_string();
            app.ai.cfg.model = crate::config::DEFAULT_MODEL.to_string();
        }
    });

    // ⚠ `config_note` **不在此处呈现**。
    //
    // 它讲的是「文件在哪、存没存过」这类部署细节，属于文档内容而非操作界面内容 ——
    // 常驻显示只会挤占面板，而且每次重绘都在重复同一句话。
    // 保存动作本身会在日志里留下结果（见下面 `save_config` 的调用），
    // 存储方式的完整说明见帮助页「注意事项」。
    //
    // 只有**异常**才当场提示：端点用了明文 HTTP 时 key 会以明文过网，这不是
    // 部署细节而是当前会话的安全状态，必须让操作者看得见。
    if app.ai.cfg.is_plaintext_endpoint() {
        ui.colored_label(
            egui::Color32::RED,
            "⚠ 端点协议为 http://，API key 将以明文传输",
        );
    }
}

/// 默认端点（拿来做输入框的占位提示）。
fn scope_gui_default_url() -> &'static str {
    crate::config::DEFAULT_BASE_URL
}

/// 发起一次分析：先把证据包冻结下来，再交给 AI 线程。
fn start_analysis(app: &mut App) {
    let Some(evidence) = app.build_evidence() else {
        app.note("无可分析的采集数据。");
        return;
    };
    let Some(cap) = app.capture.as_ref() else {
        return;
    };
    let capture_id = cap.id;
    let anchor = app.capture_anchor();

    // 证据包多大。**只报字符数，不折算 token** ——
    // 服务端那个用量字段与请求体对不上（见 `AiAnswer` 的说明），
    // 而自己拿 chars/3 估一个出来同样是在给一个说不清的数字。
    // 字符数是确定的，够用了。
    let chars = evidence.chars().count();
    if app.ai.start(evidence, capture_id, anchor) {
        app.note(format!("AI 分析已发起（证据包 {chars} 字符）。"));
    }
}

/// 把结论导出成 Markdown。
fn export_answer(app: &mut App, cap: &scope_core::Capture, answer: &crate::ai::AiAnswer) {
    let path = capture_path(cap, "ai.md");
    let body = format!(
        "# AI 分析 · 采集 #{}\n\n\
         - 链路：{}\n\
         - 采样率：{} Hz\n\
         - 触发点：{}\n\
         - 模型：{}（用时 {:.1} s）\n\n\
         ---\n\n{}\n",
        cap.id,
        app.link_description(),
        cap.rate_hz,
        cap.trigger_index
            .map(|i| i.to_string())
            .unwrap_or_else(|| "无".into()),
        answer.model,
        answer.elapsed.as_secs_f32(),
        answer.text,
    );
    match std::fs::write(&path, body) {
        Ok(()) => app.note(format!("AI 结论已导出至 {path}。")),
        Err(e) => app.note(format!("导出失败：{e}。请检查 captures/ 目录的写权限。")),
    }
}

fn decode_panel(app: &mut App, ui: &mut egui::Ui) {
    // ★ 色标的**唯一**一份在这里。
    // 不要同时在 `plot()` 末尾再画一次：`Plot::show` 会占用全部可用高度，
    // 那一份会被压成零高、只漏出一排残影，看起来像界面错位了。
    span_legend(ui);

    ui.horizontal(|ui| {
        // 面板叫「总线解码」而不是「I2C 解码」—— 与证据包里的段名
        // `== 总线解码 ==` 同一个名字。这里曾经两个地方各叫各的，
        // 用户按界面名去对 AI 的分析文本对不上（同物两名缺陷）。
        ui.heading("总线解码");

        let has_cap = app.capture.is_some();
        let ch_count = app.capture.as_ref().map(|c| c.channels.len()).unwrap_or(0);

        ui.separator();
        // 协议选择（A1）：I2C / UART。切换后立即按新协议重算。
        ui.label("协议");
        egui::ComboBox::from_id_salt("decode_protocol")
            .width(64.0)
            .selected_text(match app.decode_protocol {
                scope_core::Protocol::I2c => "I2C",
                scope_core::Protocol::Uart => "UART",
            })
            .show_ui(ui, |ui| {
                let before = app.decode_protocol;
                ui.selectable_value(&mut app.decode_protocol, scope_core::Protocol::I2c, "I2C");
                ui.selectable_value(&mut app.decode_protocol, scope_core::Protocol::Uart, "UART");
                if app.decode_protocol != before {
                    app.mark_decode_dirty();
                }
            });

        if app.decode_protocol == scope_core::Protocol::Uart {
            // UART：通道自动取 0（DecodeRequest.channels 留空）；
            // 波特率可调，8N1 其余固定 —— 语义在 core::UartConfig，不在此重复。
            ui.label("波特率");
            ui.add_enabled_ui(has_cap, |ui| {
                ui.add(
                    egui::DragValue::new(&mut app.uart_baud)
                        .range(1..=2_000_000u32)
                        .speed(1000),
                );
            });
            ui.weak("8N1 固定；每比特样点数 < 4 时不出帧");
        } else {
            // 用下拉框而不是 `DragValue`。后者有个坑：它的 range 在**渲染时就会夹值**，
            // 而 `add_enabled(false)` 挡不住 —— 没采集时 ch_count=0，范围算成 0..=0，
            // 会把 sda_channel 从 1 悄悄夹成 0，两条线都指向同一条通道，
            // 界面看着正常却永远解不出东西。下拉框只可能设成合法值，从根上没有这个问题。
            let names: Vec<String> = (0..ch_count.max(1))
                .map(|i| format!("CH{}", i + 1))
                .collect();
            let label_of =
                |idx: usize| -> String { names.get(idx).cloned().unwrap_or_else(|| "—".into()) };

            ui.label("SCL");
            ui.add_enabled_ui(has_cap, |ui| {
                egui::ComboBox::from_id_salt("scl_ch")
                    .width(64.0)
                    .selected_text(label_of(app.scl_channel))
                    .show_ui(ui, |ui| {
                        for (i, n) in names.iter().enumerate() {
                            ui.selectable_value(&mut app.scl_channel, i, n);
                        }
                    });
            });
            ui.label("SDA");
            ui.add_enabled_ui(has_cap, |ui| {
                egui::ComboBox::from_id_salt("sda_ch")
                    .width(64.0)
                    .selected_text(label_of(app.sda_channel))
                    .show_ui(ui, |ui| {
                        for (i, n) in names.iter().enumerate() {
                            ui.selectable_value(&mut app.sda_channel, i, n);
                        }
                    });
            });

            if ui
                .add_enabled(ch_count >= 2, egui::Button::new("自动检测"))
                .on_hover_text("按边沿密度判断哪条是 SCL —— 通道接反时用")
                .clicked()
            {
                let picked = app.capture.as_ref().and_then(|cap| {
                    scope_core::detect_channels(
                        cap,
                        app.decode_cfg.levels,
                        app.decode_cfg.debounce_ns,
                    )
                });
                match picked {
                    Some((a, b)) => {
                        app.scl_channel = a;
                        app.sda_channel = b;
                        app.mark_decode_dirty();
                        app.note(format!("自动检测：SCL=CH{} SDA=CH{}", a + 1, b + 1));
                    }
                    None => app.note("自动检测失败：两条线都没有边沿"),
                }
            }
            ui.label("去抖");
            ui.add(
                egui::DragValue::new(&mut app.decode_cfg.debounce_ns)
                    .range(0..=10_000u32)
                    .speed(10)
                    .suffix(" ns"),
            );
        }

        if ui.button("重新解码").clicked() {
            app.mark_decode_dirty();
        }

        match app.decode_protocol {
            scope_core::Protocol::I2c => {
                if let Some(d) = &app.decode {
                    ui.separator();
                    if d.is_untrustworthy() {
                        ui.colored_label(egui::Color32::RED, "结果不可信");
                    }
                    ui.label(format!("共 {} 帧", d.frame_count()));
                }
            }
            scope_core::Protocol::Uart => {
                if let Some(d) = &app.uart_decode {
                    ui.separator();
                    if d.quality.frame_errors > 0 {
                        ui.colored_label(
                            egui::Color32::YELLOW,
                            format!("{} 帧停止位校验失败（字节不可信）", d.quality.frame_errors),
                        );
                    }
                    ui.label(format!("共 {} 帧", d.frame_count()));
                }
            }
        }
    });

    match app.decode_protocol {
        scope_core::Protocol::I2c => {
            let Some(d) = app.decode.clone() else {
                ui.weak("尚无解码结果。采集一次后会自动解码。");
                return;
            };

            // 信号质量
            ui.horizontal_wrapped(|ui| {
                let q = &d.quality;
                ui.label(format!("SCL 边沿 {}", q.scl_edges));
                if let Some(f) = q.scl_freq_hz {
                    ui.separator();
                    ui.label(format!("频率 {:.1} kHz", f as f64 / 1000.0));
                }
                if let Some(duty) = q.scl_duty {
                    ui.separator();
                    ui.label(format!("占空比 {:.1}%", duty * 100.0));
                }
                ui.separator();
                ui.label(format!("判决带样点 {}", q.unknown_scl + q.unknown_sda));
            });

            for w in vmodel::warning_lines(&d) {
                ui.colored_label(egui::Color32::YELLOW, format!("⚠ {w}"));
            }

            ui.separator();
            egui::ScrollArea::vertical()
                .id_salt("tx_table")
                // 横向不收缩 —— 否则 ScrollArea 只占内容那么宽，
                // 滚动条会站在面板正中间（看起来像一根乱入的竖线）
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    // num_columns 必须与「每行实际放几个单元格」一致。
                    // 写成 2 但每行只放一个，斑马纹会按两列铺，行首行尾各冒出一块空底色。
                    // 不用 Grid —— 列对齐已经由 `transaction_summary` 的 format! 做好了，
                    // 而 Grid 的斑马纹按单元格宽度铺，行长不一时会只盖住右半截。
                    for line in vmodel::transaction_summary(&d) {
                        ui.monospace(line);
                    }
                });
        }
        scope_core::Protocol::Uart => {
            let Some(d) = app.uart_decode.clone() else {
                ui.weak("尚无解码结果。采集一次后会自动解码。");
                return;
            };
            ui.horizontal_wrapped(|ui| {
                let q = &d.quality;
                ui.label(format!("通过校验 {} 帧", q.frame_ok));
                ui.separator();
                ui.label(format!("错误 {} 帧", q.frame_errors));
                if let Some(m) = q.sample_margin {
                    ui.separator();
                    ui.label(format!("每比特样点 {m:.2}"));
                }
            });
            ui.separator();
            egui::ScrollArea::vertical()
                .id_salt("uart_table")
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    // 帧表与告警复用 core 的 `uart_body_text` —— 与证据包同一份
                    // 文本（帧序号/字节/错误标志），界面不另写第二套措辞。
                    for line in d.uart_body_text().lines() {
                        ui.monospace(line);
                    }
                });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scope_core::ChannelScale;

    fn one_mv() -> ChannelScale {
        ChannelScale {
            volts_per_lsb: 1e-3,
            zero_lsb: 2048.0,
        }
    }

    /// 标定角标的**三态必须分开**。
    ///
    /// 把「部分标定」说成「未标定」会把已经标好的通道白白降级；
    /// 反过来（未标定说成已标定）更糟 —— 那是把占位值当成准数展示。
    /// 这条测试是这三态唯一的自动守卫：GUI 的渲染验不了，取值可以。
    #[test]
    fn the_calibration_badge_distinguishes_three_states() {
        let uid = [1u8; 12];

        // (1) 没有记录 —— 未标定，且不该说成「部分」
        let none = ScaleSet::uncalibrated(2);
        let (ok, text, help) = calib_badge(&none);
        assert!(!ok, "没有记录不能显示成已标定");
        assert!(text.contains("未标定"), "实测：{text}");
        assert!(!text.contains("部分"), "没记录不是「部分」：{text}");
        assert!(help.contains("cal set"), "悬浮说明要给出路：{help}");

        // (2) 有记录但缺通道
        let mut store = scope_core::CalibrationStore::empty();
        store.set_channel(uid, 0, one_mv(), None, 0).unwrap();
        let partial = store.scales_for(Some(&uid), 2);
        let (ok, text, help) = calib_badge(&partial);
        assert!(!ok, "缺通道不能显示成已标定");
        assert!(text.contains("部分未标定"), "实测：{text}");
        assert!(help.contains("CH2"), "要指出缺哪个通道：{help}");

        // (3) 全部标定
        store.set_channel(uid, 1, one_mv(), None, 0).unwrap();
        let full = store.scales_for(Some(&uid), 2);
        let (ok, text, help) = calib_badge(&full);
        assert!(ok, "全标定应显示为已标定");
        assert!(text.contains("已标定"), "实测：{text}");
        assert!(
            help.contains("0.001000000"),
            "悬浮说明要给出实际换算值，否则用户没法核对：{help}"
        );
    }

    /// 空集（还没连过设备）也要走「未标定」这一支，不能因为
    /// `all_calibrated()` 在空集上返回 false 而漏判成别的状态。
    #[test]
    fn an_empty_scale_set_reads_as_uncalibrated() {
        let (ok, text, _) = calib_badge(&ScaleSet::uncalibrated(0));
        assert!(!ok);
        assert!(text.contains("未标定"), "实测：{text}");
    }
}
