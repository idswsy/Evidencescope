//! 会话状态与 14 个工具的实现。
//!
//! # 三层 token 防护（硬性，见 `docs/05-roadmap.md`）
//!
//! 一次 4096 点采集 ≈ 8192 字节 ≈ 4096 个 JSON 数字 ≈ 上万 token。
//! 塞给 LLM 既贵又没用 —— Agent 要的是「这里面发生了什么」，不是第 2713 个样点。
//!
//! 1. [`capture`](Session::capture) / [`watch`](Session::watch) 只回**统计量 + 预览**，
//!    预览点数硬顶 [`PREVIEW_MAX`]
//! 2. [`read_waveform`](Session::read_waveform) 分页、默认 [`READ_DEFAULT`]、
//!    硬顶 [`READ_HARD_CAP`]，超出**拒绝**并提示改用 `scope_save_capture`
//! 3. 全量数据只进 capture store 与磁盘，**永不进上下文**
//!
//! # 为什么它不是线程化的
//!
//! MCP 是一问一答：客户端发一个请求，等一个响应。没有 UI 要刷新，
//! 所以直接同步处理即可 —— GUI 那边需要 worker 线程是因为绘制不能阻塞，
//! 这里没有这个问题。

use crate::params::{
    defaults, AcqMode, CaptureArgs, ConfigureArgs, ConnectArgs, Coupling, DebugRawArgs, DecodeArgs,
    I2cDecodeArgs, ImportCaptureArgs, MeasureArgs, MetricKind, ProtocolArg, ReadWaveformArgs,
    SampleFormat, SaveCaptureArgs, SaveFormat, SimSetScenarioArgs, TransportArg, TriggerEdge,
    TriggerMode, WatchArgs,
};
use scope_core::acquire::MAX_TIMEOUT_MS;
use scope_core::i2c_decode::{decode_capture, detect_channels, I2cDecodeConfig, Levels};
use scope_core::{
    acquire_cancellable, from_csv, state_name, AcquireParams, CalibrationStore, Capture,
    CaptureStore, Cmd, CommandBus, DeviceInfo, ImportOptions, RateSource, ScaleSet, ScopeError,
    IMPORT_ID_BASE, IMPORT_ID_LIMIT,
};
use scope_device::{Transport, TransportKind};
use scope_sim::Scenario;
use serde_json::{json, Value};
use std::sync::atomic::AtomicBool;

/// 预览点数上限 —— 三层防护第一层。
const PREVIEW_MAX: u64 = defaults::PREVIEW_POINTS;
/// `read_waveform` 的硬顶 —— 第二层。
const READ_HARD_CAP: u64 = 4096;

/// 判为「高/低」的门限上界（12-bit ADC）。
const LEVEL_MAX_LSB: u64 = 4095;

/// `SET_ACQ.mode` 的单次采集取值（见 `proto/protocol.h`）。
const ACQ_MODE_SINGLE: u8 = 0;

/// 一次 MCP 会话。
pub struct Session {
    bus: Option<CommandBus<Transport>>,
    store: CaptureStore,
    /// 标定表（按设备 uid 索引）。**只读** —— 写入只走 `scope-cli cal set`。
    calib: CalibrationStore,
    /// 当前连接的这一窗该用哪套换算。
    calib_view: ScaleSet,
}

/// 工具失败：说明 + 怎么办。
///
/// 用 `ScopeError::summary()` / `hint()` 那一对，而不是 `Display` ——
/// 有四个变体的 `Display` 已经把 hint 拼进正文了，两者一起显示会出现两遍。
#[derive(Debug)]
pub struct ToolError {
    /// 一句话说明。
    pub message: String,
    /// 「怎么办」。
    pub hint: Option<String>,
}

impl From<ScopeError> for ToolError {
    fn from(e: ScopeError) -> Self {
        ToolError {
            message: e.summary(),
            hint: e.hint(),
        }
    }
}

impl From<scope_core::LinkError> for ToolError {
    fn from(e: scope_core::LinkError) -> Self {
        let scope_err = ScopeError::Link(e);
        ToolError {
            message: scope_err.summary(),
            hint: scope_err.hint(),
        }
    }
}

impl From<anyhow::Error> for ToolError {
    fn from(e: anyhow::Error) -> Self {
        ToolError {
            message: e.to_string(),
            hint: None,
        }
    }
}

type R = std::result::Result<Value, ToolError>;

/// 把 schema 里的整数收窄到实现要用的宽度，越界时给一句带「怎么办」的中文。
///
/// 参数 schema 里所有整数一律是 `u64`（见 `params.rs` 的模块注释），
/// 真实边界靠这里把关。**不能靠 serde 的类型来把关** —— 它只会说
/// `invalid value: integer 65537, expected u16`，既没有边界也没有「怎么办」。
fn narrow<T>(
    v: u64,
    min: u64,
    max: u64,
    field: &str,
    hint: &str,
) -> std::result::Result<T, ToolError>
where
    T: TryFrom<u64>,
{
    // ⚠ 这一步是**真的在检查**，不是只把边界写进错误消息。
    // `T::try_from` 本身只挡住「超出目标类型」的值 —— 它放行 0，
    // 也放行 4096（当目标类型是 u16 时）。只靠它会漏掉所有
    // 「类型装得下、但语义上越界」的输入。
    if v < min || v > max {
        return Err(ToolError {
            message: format!("{field}={v} 超出范围（{min}..={max}）"),
            hint: Some(hint.into()),
        });
    }
    // 走到这里 v 一定在 T 的范围内（各调用点给的 max 都不超过目标类型的上界），
    // 这个分支只是不 panic 的兜底。
    T::try_from(v).map_err(|_| ToolError {
        message: format!("{field}={v} 超出内部上限 {max}"),
        hint: Some(hint.into()),
    })
}

/// 把 schema 里的 `capture_id`（u64）收成 store 用的 `u16`。
fn capture_id(raw: u64) -> std::result::Result<u16, ToolError> {
    narrow(
        raw,
        0,
        65_535,
        "capture_id",
        "用 scope_list_captures 看现存采集的 id",
    )
}

/// 时间长度（ms）必须落在 `1..=MAX_TIMEOUT_MS`。
///
/// 这是**一条真正的防线**，不是参数洁癖：`Duration::from_millis(u64::MAX)`
/// 交给 `Instant::now() + total` 之后，调用会**永久挂住且不报错** ——
/// 实测一次 `scope_watch` 就能让整个 MCP server 从此不再响应任何请求，
/// 只能杀进程重启，在途请求连同后续的全部丢失。
fn check_timeout(ms: u64, field: &str) -> std::result::Result<(), ToolError> {
    if ms == 0 || ms > MAX_TIMEOUT_MS {
        return Err(ToolError {
            message: format!("{field}={ms} 超出范围（1..={MAX_TIMEOUT_MS} 毫秒）"),
            hint: Some(format!(
                "{} 毫秒是上限（{} 分钟）。LLM 常拿大整数当「无限」，但那会让调用永久挂住而不是超时 —— \
                 要长时间观察请分多次调用",
                MAX_TIMEOUT_MS,
                MAX_TIMEOUT_MS / 60_000
            )),
        });
    }
    Ok(())
}

/// 把 12-bit 门限收成 `u16`。
///
/// schema 已经写了 `range(0..=4095)`，这里再拦一道是因为 **`as u16` 会把
/// 65536 静默回绕成 0** —— 一个「门限 = 0」的判决带能让整条总线看起来
/// 全是高电平，而解码器照样返回一份像是成功的结果。
fn level_lsb(
    raw: Option<u64>,
    default: u16,
    field: &'static str,
) -> std::result::Result<u16, ToolError> {
    match raw {
        None => Ok(default),
        Some(v) => narrow(
            v,
            0,
            LEVEL_MAX_LSB,
            field,
            "电平一律用 ADC LSB 整数（0..=4095）；伏特换算只在显示层做",
        ),
    }
}

/// 收窄成 `u8` —— 通道号、量程档位这类。
fn u8_field(v: u64, field: &str) -> std::result::Result<u8, ToolError> {
    narrow(v, 0, 255, field, "取值范围 0..=255")
}

/// 校验导入路径：必须是 `.csv`，且（规范化后）落在当前工作目录内。
///
/// 这不是参数洁癖：`scope_import_capture` 把文件内容读进 capture store，
/// 一个拼错的绝对路径就会让 Agent 拿到主机上不相干的文件。扩展名 + 工作
/// 目录两道闸把「能读什么」收窄到用户主动放进去的那几个采集文件；
/// 不接受 `..`、符号链接逃逸或隐式展开。
fn csv_in_cwd(raw: &str) -> std::result::Result<std::path::PathBuf, ToolError> {
    let p = std::path::Path::new(raw);
    let is_csv = p
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("csv"));
    if !is_csv {
        return Err(ToolError {
            message: format!("只接受 .csv 文件：{raw}"),
            hint: Some("先用 scope_save_capture 导出 CSV，再把文件放到工作目录下".into()),
        });
    }

    let canon = p.canonicalize().map_err(|e| ToolError {
        message: format!("读 {raw} 失败：{e}"),
        hint: Some("确认路径存在、有读权限，且在 scope-mcp 的工作目录内".into()),
    })?;
    // cwd 本身也可能是通过 `..` 或短路径拿到的；规范化后再比前缀。
    let cwd = std::env::current_dir().map_err(|e| ToolError {
        message: format!("取工作目录失败：{e}"),
        hint: None,
    })?;
    let cwd = cwd.canonicalize().unwrap_or(cwd);
    if !canon.starts_with(&cwd) {
        return Err(ToolError {
            message: format!("路径不在工作目录内：{}", canon.display()),
            hint: Some(format!("把文件复制到 {} 之后再导入", cwd.display())),
        });
    }
    Ok(canon)
}

impl Session {
    /// 建一个空会话。
    ///
    /// **刻意不碰磁盘。** 每个 MCP 测试都调它 —— 在这里读 `$HOME` 会让
    /// **每一个测试都依赖开发机的环境**（`%APPDATA%` 里有没有标定文件、
    /// 内容是什么）。真实标定表由 `serve()` 经
    /// [`Session::load_calibration_default`] 装进来。
    pub fn new() -> Session {
        Session {
            bus: None,
            store: CaptureStore::default(),
            calib: CalibrationStore::empty(),
            calib_view: ScaleSet::uncalibrated(0),
        }
    }

    /// 从默认位置装载标定表。**只在 `serve()` 里调。**
    pub fn load_calibration_default(&mut self) {
        self.calib = CalibrationStore::load_default();
    }

    /// 注入一份标定表，**不碰磁盘**。
    ///
    /// 目前只在测试里用，所以标了 `#[cfg(test)]`（否则非测试构建里是死代码，
    /// CI 的 `-D warnings` 会红）。将来若加 `--calib <path>`，去掉这个属性即可。
    #[cfg(test)]
    pub fn with_calibration(mut self, calib: CalibrationStore) -> Session {
        self.calib = calib;
        self
    }

    /// 从已连接的会话里取 bus，没连就报错并给出自救提示。
    fn bus(
        &mut self,
        action: &'static str,
    ) -> std::result::Result<&mut CommandBus<Transport>, ToolError> {
        self.bus.as_mut().ok_or_else(|| ToolError {
            message: format!("尚未连接设备，无法{action}"),
            hint: Some("先调用 scope_connect（不指定 port 时用模拟器）".into()),
        })
    }

    // ══════════════════════════════════════════════════════════
    // 设备
    // ══════════════════════════════════════════════════════════

    /// `scope_list_devices` —— 枚举串口。
    pub fn list_devices(&self) -> R {
        let ports: Vec<Value> = scope_transport_serial::list_ports()
            .into_iter()
            .map(|(name, desc, likely)| {
                json!({ "port": name, "description": desc, "likely_target": likely })
            })
            .collect();
        Ok(json!({
            "ports": ports,
            "note": "不指定 port 调用 scope_connect 时使用内置模拟器，无需硬件",
        }))
    }

    /// `scope_connect` —— 连接设备。
    pub fn connect(&mut self, p: &ConnectArgs) -> R {
        let kind = match p.transport {
            // 只认 schema 里那两个名字 —— `TransportKind::parse` 还接受
            // `simulator` / `uart` / `port` 这些别名（给人用的 CLI 走那条路），
            // 但 MCP 这层要对齐 schema，否则 schema 又成了一句空话。
            Some(TransportArg::Serial) => TransportKind::Serial,
            Some(TransportArg::Sim) => TransportKind::Sim,
            // 没给 transport：给了 port 就当串口，否则模拟器
            None => {
                if p.port.is_some() {
                    TransportKind::Serial
                } else {
                    TransportKind::Sim
                }
            }
        };

        let port = p.port.as_deref().unwrap_or("");
        let baud = p.baud.unwrap_or(921_600);
        // 拼错了要报错，不能静默换成默认场景 —— 否则 Agent 以为连的是自己
        // 指定的波形，实际是另一回事。`ScenarioArg` 的反序列化已经挡了
        // 一层（错误里直接列出可选清单），这里只是把它变成默认值。
        let scenario = p
            .sim_scenario
            .map(Scenario::from)
            .unwrap_or(Scenario::I2c100k);

        if kind == TransportKind::Serial && port.trim().is_empty() {
            return Err(ToolError {
                message: "transport=\"serial\" 但没给 port".into(),
                hint: Some("先用 scope_list_devices 看可用串口；不指定 port 即使用模拟器".into()),
            });
        }

        let dev = Transport::open(kind, port, baud, scenario).map_err(|e| {
            // 连失败时把「现在到底连着谁」讲清楚 —— 之前旧连接静默保留，
            // Agent 会以为已经切到新设备了。
            let te = ToolError::from(e);
            ToolError {
                hint: Some(match (&self.bus, &te.hint) {
                    (Some(b), Some(h)) => format!("{h}；当前仍保持着原来的连接：{}", b.describe()),
                    (None, Some(h)) => h.clone(),
                    (Some(b), None) => format!("当前仍保持着原来的连接：{}", b.describe()),
                    (None, None) => "检查接线与供电".into(),
                }),
                ..te
            }
        })?;
        let mut bus = CommandBus::new(dev);

        let info = bus.connect().map_err(ToolError::from)?;
        // `connect()` 只发 GET_INFO + GET_CONFIG，**不发 GET_STATUS**。
        // 不补这一次的话 `bus.state` 一直是 None，`guard_config_allowed`
        // 就失去判据 —— 已武装时照样能改配置。
        let state = bus
            .get_status()
            .map(|s| s.state)
            .unwrap_or(scope_core::State::Idle);
        let cfg = bus.config.clone();

        // 换设备后 capture_id 从 1 重新分配，而 store 按 id 取数 —— 不清空的话
        // 旧采集会**遮蔽**同 id 的新采集，measure / decode / read / save
        // 全部返回过期数据（而且是静默的）。历史本就属于单次设备会话，
        // 换连接即失效。
        self.store = CaptureStore::default();

        // 标定按 uid 查，且**重新读一遍文件** —— 用户可能在两次会话之间
        // 在另一个终端跑了 `cal set`。
        self.calib.reload();
        self.calib_view = self
            .calib
            .scales_for(Some(&info.uid), info.ch_count as usize);
        let calib_json = json!({
            "uid": scope_core::encode_uid(&info.uid),
            // 有没有记录，与「记录里齐不齐」是两回事，都要给
            "record_found": self.calib_view.record_found(),
            "calibrated": self.calib_view.all_calibrated(),
            "note": self.calib_view.summary_note(),
        });

        let out = json!({
            "link": bus.describe(),
            "simulated": bus.is_simulated(),
            "state": state_name(state),
            "device": info_json(&info),
            "config": cfg.map(|c| config_json(&c)),
            "calibration": calib_json,
        });
        self.bus = Some(bus);
        Ok(out)
    }

    /// `scope_disconnect`。
    pub fn disconnect(&mut self) -> R {
        if let Some(bus) = self.bus.as_mut() {
            let _ = bus.stop();
        }
        self.bus = None;
        Ok(json!({ "disconnected": true }))
    }

    /// `scope_status`。
    pub fn status(&mut self) -> R {
        let bus = self.bus("查询状态")?;
        let st = bus.get_status().map_err(ToolError::from)?;
        Ok(json!({
            "link": bus.describe(),
            "simulated": bus.is_simulated(),
            "state": state_name(st.state),
            // 链路健康计数 —— 「这份采集可不可信」的判据。
            // 从前 core 只读第一个字节就把 payload 丢了，这几个字段
            // 一路都到不了 Agent。
            "link_health": {
                "overrun_samples": st.overrun_samples,
                "rx_crc_err": st.rx_crc_err,
                "rx_dropped": st.rx_dropped,
                "tx_dropped": st.tx_dropped,
                "last_error_code": st.last_error_code,
                "err_flags": st.err_flags,
                "clean": st.link_is_clean(),
            },
            "uptime_ms": st.uptime_ms,
            "tick_us": st.tick_us,
            "config": bus.config.as_ref().map(config_json),
        }))
    }

    /// `scope_configure` —— 一个工具替代 N 个 `set_*`。
    pub fn configure(&mut self, p: &ConfigureArgs) -> R {
        // ⚠ **必须在 `self.bus(...)` 之前取。** 那次调用借走整个 `*self`
        // 且活到函数结尾（下面每一处 `bus.xxx()` 都在用它），之后再读
        // `self.calib_view` 编译器不会放行。`ScaleSet` 是 `Clone`，克隆一份
        // 借用即结束。
        let calib_view = self.calib_view.clone();

        let bus = self.bus("配置设备")?;
        let mut applied = serde_json::Map::new();
        let mut warnings: Vec<String> = Vec::new();

        if let Some(hz) = p.sample_rate_hz {
            // 先收窄再下发 —— `as u32` 会把 4294967297 变成 1，
            // 于是「把采样率调到最大」变成「调到 1 Hz」还返回成功
            // 上界必须与 schema 里 `range(max = MAX_SAMPLE_RATE_HZ)` 一致。
            // 回归：这里曾用 `MAX_INTERLEAVED_HZ`(1714285) —— 那是「双 ADC
            // 交织」的理论值、文档明说**只用于显示**，于是 1000000 这种
            // 值能通过收窄、撞到核心层才失败，而 schema 和错误提示互相矛盾。
            let hz32 = narrow::<u32>(
                hz,
                1,
                u64::from(scope_core::f103::MAX_SAMPLE_RATE_HZ),
                "sample_rate_hz",
                "F103 单 ADC 的采样率上限是 857142 Hz（1714285 Hz 是双 ADC 交织的\
                 理论上限，只用于显示、不可作测量依据）",
            )?;
            let actual = bus.set_sample_rate(hz32).map_err(ToolError::from)?;
            if u64::from(actual) != hz {
                // 回显值是唯一真值 —— 定时器只能出整数分频的档位
                warnings.push(format!(
                    "请求采样率 {hz} Hz 被量化为 {actual} Hz，时间轴以 {actual} Hz 为准"
                ));
            }
            applied.insert("sample_rate_hz".into(), json!(actual));
        }

        // 采集方式。`stream` 尚未实现（固件侧 STREAMING 属 P1+），只警告、
        // **不**把它算作「要下发 SET_ACQ」的理由 —— 否则就是回显一个
        // 设备上根本没生效的值。
        let want_acq_mode: Option<u8> = match p.mode {
            None => None,
            Some(AcqMode::Single) => Some(ACQ_MODE_SINGLE),
            Some(AcqMode::Stream) => {
                warnings.push(
                    "流模式尚未实现（固件侧 STREAMING 状态属 P1+），本次仍按单次采集处理".into(),
                );
                None
            }
        };

        // 采集四件套（模式 / 点数 / 格式 / 抽点）一起下发 —— `SET_ACQ` 是一条
        // 命令，分三次调只会互相覆盖。缺省项沿用设备当前配置，不是硬编码的 0/1。
        //
        // ⚠ `mode` 曾经**不在**这个条件里：单独调 `configure {mode:"single"}`
        // 会回显 `applied.mode = "single"`，而设备一条命令都没收到 ——
        // 一个「说生效了、其实没下发」的字段。工具描述还写着「返回实际生效值」。
        let cur_samples = bus.config.as_ref().map(|c| u64::from(c.capture_samples));
        let cur_decim = bus.config.as_ref().map(|c| u64::from(c.decimation));
        let cur_format = bus.config.as_ref().map(|c| c.format);
        let cur_acq_mode = bus
            .config
            .as_ref()
            .map(|c| c.acq_mode)
            .unwrap_or(ACQ_MODE_SINGLE);
        if want_acq_mode.is_some()
            || p.capture_samples.is_some()
            || p.decimation.is_some()
            || p.format.is_some()
        {
            // 先校验再转换 —— `as u16` 会把 65537 变成 1、131072 变成 0，
            // 后者还会让之后每次采集都「成功但空」。
            let max = u64::from(scope_core::f103::MAX_CAPTURE_SAMPLES);
            let samples = p.capture_samples.or(cur_samples).unwrap_or(max);
            let samples = narrow::<u16>(
                samples,
                1,
                max,
                "capture_samples",
                "8 KB 环按 u16 折半就是 4096 点，这是 F103 的物理上限",
            )?;
            let max_decim = u64::from(scope_core::f103::MAX_DECIMATION);
            let decimation = narrow::<u16>(
                p.decimation.or(cur_decim).unwrap_or(1),
                1,
                max_decim,
                "decimation",
                "抽点倍数上界来自协议：SET_ACQ.decimation 是 1..=256",
            )?;
            let (fmt_name, fmt_id) = match p.format {
                None => ("(未改)", cur_format.unwrap_or(0)),
                Some(SampleFormat::Raw) => ("raw", 0),
                Some(SampleFormat::Packed12) => ("packed12", 1),
                Some(SampleFormat::Minmax) => ("minmax", 2),
            };
            // 采集模式：显式给了就用它，否则沿用现值 —— 从前这里写死 0，
            // 会把设备上的采集模式悄悄改回「单次」
            let acq_mode = want_acq_mode.unwrap_or(cur_acq_mode);
            bus.set_acq(acq_mode, samples, fmt_id, decimation)
                .map_err(ToolError::from)?;
            applied.insert("capture_samples".into(), json!(samples));
            applied.insert("decimation".into(), json!(decimation));
            if fmt_name != "(未改)" {
                applied.insert("format".into(), json!(fmt_name));
            }
            // 只有真下发了才回显 —— 这条插入从前在 `set_acq` 之外无条件执行
            if want_acq_mode.is_some() {
                applied.insert("mode".into(), json!("single"));
            }
        }

        if let Some(t) = &p.trigger {
            // 省略的子项**沿用设备当前值** —— 与上面采集三件套同一套语义。
            //
            // 回归：从前这里一律 `unwrap_or(默认)`，于是 Agent 只想改个电平，
            // 却把刚配好的 auto 模式 / 下降沿 / CH1 全部重置回
            // normal / rising / CH0 —— 而 schema 里那句「省略的项沿用设备
            // 当前值」还挂着。两个分支在同一函数里各说各话。
            let cur = bus.config.clone();
            let mode = t
                .mode
                .unwrap_or_else(|| match cur.as_ref().map(|c| c.trigger_mode) {
                    Some(0) => TriggerMode::Auto,
                    Some(2) => TriggerMode::Single,
                    _ => TriggerMode::Normal,
                });
            let mode_id = match mode {
                TriggerMode::Auto => 0u8,
                TriggerMode::Normal => 1,
                TriggerMode::Single => 2,
            };
            let edge = t
                .edge
                .unwrap_or_else(|| match cur.as_ref().map(|c| c.trigger_edge) {
                    Some(1) => TriggerEdge::Falling,
                    _ => TriggerEdge::Rising,
                });
            let edge_id = match edge {
                TriggerEdge::Rising => 0u8,
                TriggerEdge::Falling => 1,
            };
            // 电平既可以给 LSB，也可以给伏特（上位机换算 —— MCU 不碰浮点）
            let level = match (t.level_lsb, t.level_v) {
                (Some(l), _) => narrow::<u16>(
                    l,
                    0,
                    LEVEL_MAX_LSB,
                    "trigger.level_lsb",
                    "电平是 12-bit ADC 的 LSB 整数；也可以改传 level_v（伏特）",
                )?,
                (None, Some(v)) => {
                    // `volts_to_lsb` 回 i32，所以负电平在这里被挡住。
                    //
                    // ⚠ 取**通道 0** 的换算：此刻 `trigger.source`（下面才解析）
                    // 还不知道，而换算表可以逐通道不同。当前底板只有一路模拟
                    // 输入（ADR-010），取 0 与取实际源通道等价；真出现逐通道
                    // 差异时，这里要改成按 `source` 取。
                    let sc = calib_view.get(0);
                    let lsb = sc.volts_to_lsb(v);
                    if !(0..=LEVEL_MAX_LSB as i32).contains(&lsb) {
                        return Err(ToolError {
                            message: format!("trigger.level_v={v} 超出 12-bit ADC 量程"),
                            hint: Some(if calib_view.any_calibrated() {
                                format!(
                                    "换算用的是 uid {} 的标定值（{:.9} V/LSB、零点 {:.1}）；\
                                     也可以直接给 level_lsb",
                                    calib_view.uid_hex().unwrap_or_else(|| "?".into()),
                                    sc.volts_per_lsb,
                                    sc.zero_lsb
                                )
                            } else {
                                "电压换算用的是未标定的占位参数（3.3 V / 4096、零点 2048），\
                                 范围约 -1.65..=+1.65 V；也可以直接给 level_lsb"
                                    .into()
                            }),
                        });
                    }
                    lsb as u16
                }
                (None, None) => cur.as_ref().map(|c| c.trigger_level_lsb).unwrap_or(2048),
            };
            // `trig_src_t`：0=CH1 / 1=CH2 / 2=软件。**不是 0..=255** ——
            // 从前用 u8，`3` 与 `255` 会被静默收下、原样下发给设备。
            let source = match t.source {
                Some(v) => narrow::<u8>(
                    v,
                    0,
                    2,
                    "trigger.source",
                    "触发源见 proto/protocol.h 的 trig_src_t：0=CH1 / 1=CH2 / 2=软件触发",
                )?,
                None => cur.as_ref().map(|c| c.trigger_source).unwrap_or(0),
            };
            bus.set_trigger(mode_id, source, edge_id, level, 1024, 1000)
                .map_err(ToolError::from)?;
            applied.insert(
                "trigger".into(),
                json!({
                    "mode": mode.name(),
                    "edge": edge.name(),
                    "source": source,
                    "level_lsb": level,
                }),
            );
        }

        if let Some(c) = &p.channel {
            // 省略的子项：**能从 GET_CONFIG 读到的就沿用，读不到的就按 0 并明说**。
            //
            // `GET_CONFIG` 只回报 `ch0_enable` 与 `ch0_coupling`，
            // `range_idx` / `offset_lsb` 不在里面 —— 那两个没有「现值」可沿用，
            // 只能按下发值走。**明说，不静默。**
            //
            // 回归：这个分支从前只推一条警告（「暂未接线上层」），一条命令都不发。
            // 工具描述却写着「返回实际生效值」。
            let cur = bus.config.clone();
            let ch = match c.ch {
                Some(v) => u8_field(v, "channel.ch")?,
                None => 0,
            };
            let enable = match c.enable {
                Some(v) => v,
                None => cur.as_ref().map(|x| x.ch0_enable != 0).unwrap_or(true),
            };
            let coupling = match c.coupling {
                Some(v) => v,
                // 沿用现值：GET_CONFIG 报的是裸 u8，映射回枚举好让回显统一
                None => match cur.as_ref().map(|x| x.ch0_coupling).unwrap_or(0) {
                    0 => Coupling::Dc,
                    _ => Coupling::Ac,
                },
            };
            let range_idx = match c.range_idx {
                Some(v) => u8_field(v, "channel.range_idx")?,
                None => {
                    warnings.push(
                        "channel.range_idx 省略 → 按 0 下发（GET_CONFIG 不回报它，无法沿用现值）"
                            .into(),
                    );
                    0
                }
            };
            let offset_lsb = match c.offset_lsb {
                Some(v) => v,
                None => {
                    warnings.push(
                        "channel.offset_lsb 省略 → 按 0 下发（GET_CONFIG 不回报它，无法沿用现值）"
                            .into(),
                    );
                    0
                }
            };

            // **本板改不了的字段要明说。**
            //
            // SW2（AC/DC 耦合）与 SW3（X1/X50 量程）都是**手拨机械开关**
            // （见 `docs/02-hardware.md` §5）—— 设备会记下并回显下发值，
            // 但实际开关位置以硬件为准。不说的话，Agent 会以为耦合真的切到 AC 了，
            // 然后拿一份和它以为的不一样的波形下结论。
            if c.coupling.is_some() {
                warnings.push(
                    "本板的耦合开关 SW2 是手拨机械开关，AI 无法程控 —— 下发值会被记录并回显，\
                     但实际耦合以硬件开关位置为准"
                        .into(),
                );
            }
            if c.range_idx.is_some() {
                warnings.push(
                    "本板的量程开关 SW3 是手拨机械开关，AI 无法程控 —— 下发值会被记录并回显，\
                     但实际量程以硬件开关位置为准"
                        .into(),
                );
            }

            bus.set_channel(ch, enable, range_idx, coupling.as_u8(), offset_lsb)
                .map_err(ToolError::from)?;
            applied.insert(
                "channel".into(),
                json!({
                    "ch": ch,
                    "enable": enable,
                    "range_idx": range_idx,
                    "coupling": coupling.name(),
                    "offset_lsb": offset_lsb,
                }),
            );
        }

        Ok(json!({ "applied": applied, "warnings": warnings }))
    }

    /// `scope_capture` —— 采一次，**只回统计量 + 预览**。
    pub fn capture(&mut self, p: &CaptureArgs) -> R {
        // 从前 `mode` 在 schema 里声明了却**没人读** —— 传 `"stream"` 会静默
        // 按单次跑，而 Agent 以为拿到的是流数据。现在明确告诉它去哪。
        if p.mode == Some(AcqMode::Stream) {
            return Err(ToolError {
                message: "scope_capture 不支持流模式".into(),
                hint: Some(
                    "流模式固件侧尚未实现（P1+）。要在一段时间里连续观察请用 scope_watch".into(),
                ),
            });
        }

        let timeout_ms = p.timeout_ms.unwrap_or(defaults::CAPTURE_TIMEOUT_MS);
        check_timeout(timeout_ms, "timeout_ms")?;
        // 0 会让 `Capture::preview` 返回 `None` —— 一次「成功但没有预览」的
        // 采集，Agent 很容易读成「没采到数据」。硬顶是设计（省 token），
        // 下限也必须挡（否则是另一种静默的怪结果）。
        let max_preview = p.max_preview_points.unwrap_or(PREVIEW_MAX);
        if max_preview == 0 {
            return Err(ToolError {
                message: "max_preview_points 至少为 1".into(),
                hint: Some(format!("上限是 {PREVIEW_MAX}（三层 token 防护第一层）")),
            });
        }
        let max_preview = max_preview.min(PREVIEW_MAX) as usize;

        let cap = self.acquire_raw(timeout_ms)?;
        let out = capture_summary_json(&cap, max_preview, &self.calib_view);
        self.store.push(cap);
        Ok(out)
    }

    /// 采一次但**不入库**。
    ///
    /// `scope_watch` 一次能采上百次；全塞进容量 16 的 store 会把用户已有的
    /// 采集全部挤出去 —— 连它自己返回的那个 capture_id 都会立刻失效。
    fn acquire_raw(&mut self, timeout_ms: u64) -> std::result::Result<Capture, ToolError> {
        let bus = self.bus("采集")?;
        let samples = bus
            .config
            .as_ref()
            .map(|c| c.capture_samples)
            .unwrap_or(scope_core::f103::MAX_CAPTURE_SAMPLES as u16);
        let params = AcquireParams {
            samples,
            rate_hz: bus.config.as_ref().map(|c| c.rate_hz).unwrap_or(857_142),
            trigger_level_lsb: bus
                .config
                .as_ref()
                .map(|c| c.trigger_level_lsb)
                .unwrap_or(2048),
            timeout: std::time::Duration::from_millis(timeout_ms),
        };
        // MCP 是同步一问一答，没有「用户按停止」这回事，所以取消标志永远是假
        static NEVER: AtomicBool = AtomicBool::new(false);
        acquire_cancellable(bus, &params, &NEVER).map_err(ToolError::from)
    }

    /// `scope_read_waveform` —— 分页拉样点。**超出硬顶直接拒绝**。
    pub fn read_waveform(&mut self, p: &ReadWaveformArgs) -> R {
        let id = capture_id(p.capture_id)?;
        let ch = match p.channel {
            Some(v) => u8_field(v, "channel")? as usize,
            None => defaults::ZERO as usize,
        };
        let start = p.start_sample.unwrap_or(defaults::ZERO);
        // `format` 只有一个合法值（schema 里就写着 enum: ["raw"]），解析成功
        // 即等于「要 raw」。取出来是为了**回显在响应里** —— Agent 需要知道
        // 拿到的是原始 12-bit LSB 整数，不是伏特。
        let format = p.format.unwrap_or(crate::params::ReadFormat::Raw);
        // `count` 是主参数；`max_points` 作为旧名保留兼容。两个都给时以 `count`
        // 为准 —— 之前是「谁先出现用谁」，同一个请求换个字段顺序结果就不一样。
        let want_raw = p.count.or(p.max_points).unwrap_or(defaults::READ_COUNT);
        if want_raw == 0 {
            return Err(ToolError {
                message: "count 至少为 1".into(),
                hint: Some("要看空页就不必调用它".into()),
            });
        }

        // 硬顶：超过就**拒绝**，不做静默截断 —— 静默截断会让 Agent
        // 以为拿到了全部数据。要全量请走 scope_save_capture。
        if want_raw > READ_HARD_CAP {
            return Err(ToolError {
                message: format!("一次最多读 {READ_HARD_CAP} 点，请求了 {want_raw} 点"),
                hint: Some(
                    "分页读取（改 start_sample 多次调用），或改用 scope_save_capture 落盘 \
                     后用别的工具分析 —— 全量数据不进 LLM 上下文。"
                        .into(),
                ),
            });
        }
        let want = want_raw as usize;

        let cap = self.store.get(id).ok_or_else(|| ToolError {
            message: format!("找不到采集 {id}"),
            hint: Some("历史只保留最近 16 次；用 scope_list_captures 看现存采集".into()),
        })?;
        let s = cap.samples(ch).ok_or_else(|| ToolError {
            message: format!("采集 {id} 没有通道 {ch}"),
            hint: Some("用 scope_status 看设备的通道数".into()),
        })?;

        // ⚠ 这段的顺序不能动。`start` 是 u64（schema 不设上界，因为「越界的
        // start 返回空页」是**约定行为**），所以**必须先比长度再转 usize**：
        // 从前是 `start + want` 直接算，溢出后 debug 构建 panic，整个 MCP
        // server 进程死掉（rc=101），后续请求连同在途的一起丢。
        // 回归：`start_sample: 18446744073709551615` 一次就能远程打挂它。
        let len = s.len();
        if start >= len as u64 {
            // 空页要**和正常页同形** —— `rate_hz`/`format` 一个都不能少。
            // 回归：`format` 是后加的，只加在下面那条路径上，于是同一个工具
            // 对合法请求返回两种形状；靠 `format` 判断样点单位的调用方
            // 在分页末尾会拿到 undefined。
            return Ok(json!({
                "capture_id": id,
                "channel": ch,
                "start_sample": start,
                "count": 0,
                "rate_hz": cap.rate_hz,
                "format": format.name(),
                "has_more": false,
                "samples": [],
                "note": format!("start_sample 超出采集长度（本次共 {len} 点）"),
            }));
        }
        let from = start as usize; // 上面已保证 start < len，这里转换不会截断
        let end = from.saturating_add(want).min(len);
        let slice: Vec<u16> = s[from..end].to_vec();
        Ok(json!({
            "capture_id": id,
            "channel": ch,
            "start_sample": start,
            "count": slice.len(),
            "rate_hz": cap.rate_hz,
            "format": format.name(),
            "has_more": end < len,
            "samples": slice,
        }))
    }

    /// `scope_measure` —— 主机侧定点测量（精度最高）。
    pub fn measure(&mut self, p: &MeasureArgs) -> R {
        let id = capture_id(p.capture_id)?;
        let cap = self.store.get(id).ok_or_else(|| ToolError {
            message: format!("找不到采集 {id}"),
            hint: Some("用 scope_list_captures 看现存采集".into()),
        })?;

        // 只测指定通道（省略则全测）。
        //
        // 越界的通道号要**报错**，不能回一份空测量 ——
        // 回归：`channel: 7`（本次只有 2 个通道）从前返回
        // `{"measurements":{}, "note":"电压为未标定换算…"}` 且 `isError=false`，
        // 读起来像「这个采集没有数据」，而不是「你给的通道不存在」。
        // 同一个参数名在 `scope_read_waveform` 里是会报错的。
        let only_ch = match p.channel {
            Some(v) => {
                let ch = u8_field(v, "channel")? as usize;
                let n = cap.channels.len();
                if ch >= n {
                    return Err(ToolError {
                        message: format!("通道越界：本次采集只有 {n} 个通道（编号 0..={}）", n - 1),
                        hint: Some("用 scope_status 看设备的通道数".into()),
                    });
                }
                Some(ch)
            }
            None => None,
        };

        // 只回指定指标（省略则全回）。
        //
        // 从前这里有一份手写的 `const KINDS: [&str; 8]`，而 schema 里另有一份
        // 一模一样的枚举 —— 两份靠人眼保持一致。**现在清单就是 `MetricKind`
        // 这个类型**：schema 由它生成，解析由它把关，拼错的名字在参数那一关
        // 就被挡下（错误信息由 serde 给出，还附带合法取值）。
        let want = p.metrics.as_deref();
        let has = |k: MetricKind| match want {
            None => true,
            Some(w) => w.contains(&k),
        };

        // 按 uid 查的逐通道换算。`cap` 借的是 `self.store`、这里借的是
        // `self.calib_view` —— 不同字段，互不冲突；克隆一份是为了不让
        // 「借用能不能同时成立」这种事绊住后面的改动。
        let scales = self.calib_view.clone();
        let calibrated = scales.all_calibrated();
        // link / simulated 用于 provenance；在借 `self.store` 之前取完。
        let (link, simulated) = match self.bus.as_ref() {
            Some(bus) => (bus.describe(), bus.is_simulated()),
            None => ("未连接".to_string(), false),
        };
        let report = scope_core::AnalysisReport::from_capture(cap, &scales, &link, simulated);
        let mut out = serde_json::Map::new();
        for ch in 0..cap.channels.len() {
            if only_ch.is_some_and(|only| ch != only) {
                continue;
            }
            if let Some(ca) = report.channel(ch) {
                let m = &ca.measurements;
                let mut row = serde_json::Map::new();
                if has(MetricKind::Vpp) {
                    row.insert("vpp_v".into(), json!(m.vpp));
                }
                if has(MetricKind::Min) {
                    row.insert("min_v".into(), json!(m.min));
                }
                if has(MetricKind::Max) {
                    row.insert("max_v".into(), json!(m.max));
                }
                if has(MetricKind::Mean) {
                    row.insert("mean_v".into(), json!(m.mean));
                }
                if has(MetricKind::Rms) {
                    // `ac_rms_v` 是扣直流后的 RMS，`rms_v` 是真 RMS。
                    // 既有键保持不变，新键是加性的。
                    row.insert("ac_rms_v".into(), json!(m.ac_rms));
                    row.insert("rms_v".into(), json!(m.rms));
                }
                if has(MetricKind::Freq) {
                    row.insert("freq_hz".into(), json!(m.freq_hz));
                }
                if has(MetricKind::Duty) {
                    row.insert("duty_pct".into(), json!(m.duty_pct));
                }
                if has(MetricKind::Rise) {
                    row.insert("rise_ns".into(), json!(m.rise_ns));
                }
                if has(MetricKind::Fall) {
                    row.insert("fall_ns".into(), json!(m.fall_ns));
                }
                if has(MetricKind::Overshoot) {
                    row.insert("overshoot_pct".into(), json!(m.overshoot_pct));
                }
                if has(MetricKind::Undershoot) {
                    row.insert("undershoot_pct".into(), json!(m.undershoot_pct));
                }
                if has(MetricKind::High) {
                    row.insert("high_ns".into(), json!(m.high_ns));
                }
                if has(MetricKind::Edges) {
                    row.insert("edges".into(), json!(m.edges));
                }
                row.insert("quality".into(), json!(ca.quality));
                out.insert(format!("ch{ch}"), Value::Object(row));
            }
        }
        Ok(json!({
            "schema": report.schema,
            "capture_id": id,
            "provenance": report.provenance,
            "measurements": out,
            "notes": report.notes,
            // 结构化地告诉模型「伏特值可不可信」。写在一句中文 note 里，
            // 模型得读懂那句话才知道要打折；一个布尔字段它不会漏掉。
            "voltage_calibrated": calibrated,
            "note": if calibrated {
                "电压按设备标定换算（该 uid 的各通道都有记录）"
            } else {
                "电压未标定（占位参数 3.3 V / 4096、零点 2048），只看相对关系，\
                 不要当成校准值。用 scope-cli cal set 写入标定。"
            },
            "note_unrelated": "频率与占空比已排除事务间空闲；非周期信号（数据线）报出的是边沿速率",
        }))
    }

    /// `scope_i2c_decode` —— 本项目的招牌能力。
    pub fn i2c_decode(&mut self, p: &I2cDecodeArgs) -> R {
        let id = capture_id(p.capture_id)?;
        let cap = self.store.get(id).ok_or_else(|| ToolError {
            message: format!("找不到采集 {id}"),
            hint: Some("用 scope_list_captures 看现存采集".into()),
        })?;

        if cap.channels.len() < 2 {
            return Err(ToolError {
                message: format!(
                    "I2C 解码至少需要两条线，这次采集只有 {} 个通道",
                    cap.channels.len()
                ),
                hint: Some("换双通道场景（如 --scenario i2c_100k）或换用双通道硬件配置".into()),
            });
        }

        // 单侧给也要生效 —— 之前只有两个都给才用传入值，其余整体退默认，
        // 于是「只调 VIH」静默失效。
        //
        // 顺带堵掉一个回绕：从前 `as u16` 会把 65536 变成 **0**，一个
        // 「门限 = 0」的判决带能让整条总线看起来全是高电平，而解码器
        // 照样返回一份像是成功的结果。
        let dflt = Levels::default_ratio();
        let levels = Levels {
            vih_lsb: level_lsb(p.vih_lsb, dflt.vih_lsb, "vih_lsb")?,
            vil_lsb: level_lsb(p.vil_lsb, dflt.vil_lsb, "vil_lsb")?,
        };
        if !levels.is_valid() {
            return Err(ToolError {
                message: format!(
                    "门限非法：VIH={} 必须大于 VIL={}",
                    levels.vih_lsb, levels.vil_lsb
                ),
                hint: Some("电平用 ADC LSB 整数（12-bit，0..=4095）".into()),
            });
        }

        // 通道没指定就自动判定（SCL 的边沿比 SDA 密）。
        //
        // ⚠ 只给一个的话要报错，不能默默退回自动判定 —— 从前的 `_ =>` 分支
        // 把「只指定了 SCL」变成「你指定的那个也没生效」，而返回的帧列表
        // 看上去完全正常（自动判定大多能猜对），于是这个 bug 几乎不可能被发现。
        let auto = detect_channels(cap, levels, 50).unwrap_or((0, 1));
        let (scl, sda) = match (p.scl_channel, p.sda_channel) {
            (Some(a), Some(b)) => (
                u8_field(a, "scl_channel")? as usize,
                u8_field(b, "sda_channel")? as usize,
            ),
            (None, None) => auto,
            (Some(_), None) | (None, Some(_)) => {
                return Err(ToolError {
                    message: "scl_channel 与 sda_channel 要么都给，要么都不给".into(),
                    hint: Some(format!(
                        "都不给时按边沿密度自动判定（SCL 边沿更密），本次会判成 SCL={} SDA={}",
                        auto.0, auto.1
                    )),
                })
            }
        };

        // 先按 MCP 自己的参数名报越界 —— core 的报错里字段叫 `i2c_channel`，
        // 那个名字在 schema 里根本不存在，Agent 会去找一个没提过的参数。
        let n = cap.channels.len();
        if scl >= n || sda >= n {
            return Err(ToolError {
                message: format!("通道越界：本次采集只有 {n} 个通道（编号 0..={}）", n - 1),
                hint: Some("scl_channel / sda_channel 从 0 起算".into()),
            });
        }
        if scl == sda {
            return Err(ToolError {
                message: format!("scl_channel 与 sda_channel 不能是同一个通道（都是 {scl}）"),
                hint: Some("两条线是两个不同的通道".into()),
            });
        }

        let cfg = I2cDecodeConfig {
            scl_channel: scl,
            sda_channel: sda,
            levels,
            // 0 是合法的（不去抖）—— schema 里曾经写 `range(min = 1)`，
            // 那是把实现做得到的事说小了。上限则**必须**有：窗口开到与时钟
            // 周期一个量级，真实边沿会被当毛刺滤掉，而解码器仍报 trustworthy。
            debounce_ns: narrow::<u32>(
                p.debounce_ns.unwrap_or(defaults::DEBOUNCE_NS),
                0,
                defaults::MAX_DEBOUNCE_NS,
                "debounce_ns",
                "纳秒；0 表示不去抖。窗口接近时钟周期（400 kHz 时半周期 1250 ns）\
                 会把真实边沿当毛刺滤掉",
            )?,
        };
        let d = decode_capture(cap, &cfg).map_err(ToolError::from)?;

        // 帧列表按条给出 —— 这是 Agent 真正要看的东西
        let frames: Vec<Value> = d
            .transactions
            .iter()
            .enumerate()
            .map(|(i, tx)| {
                json!({
                    "index": i + 1,
                    "time_ms": tx.start_time_us as f64 / 1000.0,
                    "repeated_start": tx.repeated,
                    "address": tx.address.as_ref().map(|a| json!({
                        "value": a.value,
                        "read": a.read,
                        "ten_bit": a.ten_bit,
                        "acked": a.acked,
                    })),
                    "bytes": tx.bytes,
                    "nack": tx.has_nack(),
                    "complete": tx.complete,
                })
            })
            .collect();

        Ok(json!({
            "capture_id": id,
            "scl_channel": scl,
            "sda_channel": sda,
            "frame_count": d.frame_count(),
            "frames": frames,
            "all_bytes": d.all_bytes(),
            // 回显生效的门限与去抖 —— 否则「我到底用的是什么阈值」无从得知
            "levels": {
                "vih_lsb": levels.vih_lsb,
                "vil_lsb": levels.vil_lsb,
                "debounce_ns": cfg.debounce_ns,
            },
            "signal_quality": {
                "scl_edges": d.quality.scl_edges,
                "scl_freq_hz": d.quality.scl_freq_hz,
                "scl_duty": d.quality.scl_duty,
                "unknown_level_samples": d.quality.unknown_scl + d.quality.unknown_sda,
            },
            "warnings": d.warnings.iter().map(|w| w.text()).collect::<Vec<_>>(),
            "trustworthy": !d.is_untrustworthy(),
        }))
    }

    /// `scope_decode` —— 统一解码入口（I²C 与 UART）。
    ///
    /// `protocol = "i2c"` 时组装成 [`I2cDecodeArgs`] 走既有的
    /// [`Session::i2c_decode`]（别名语义，返回格式不变）；
    /// `protocol = "uart"` 走 core 的 `decoder::decode`（8N1）。
    ///
    /// 互斥对象显式校验：`protocol=uart` 却给了 `i2c` 参数（或反过来）
    /// 报业务错误而不是静默忽略 —— 混用的调用几乎一定是 Agent 搞错了，
    /// 静默走默认值会让它拿到「自己没要求的配置下」的结果。
    pub fn decode(&mut self, p: &DecodeArgs) -> R {
        let id = capture_id(p.capture_id)?;
        // 先校验互斥对象（未连设备也要报参数错误，先于「找不到采集」）。
        match p.protocol {
            ProtocolArg::I2c => {
                if p.uart.is_some() {
                    return Err(ToolError {
                        message: "协议不匹配：protocol=i2c 不接受 uart 参数".into(),
                        hint: Some("UART 参数只在 protocol=\"uart\" 时给".into()),
                    });
                }
            }
            ProtocolArg::Uart => {
                if p.i2c.is_some() {
                    return Err(ToolError {
                        message: "协议不匹配：protocol=uart 不接受 i2c 参数".into(),
                        hint: Some("I²C 参数只在 protocol=\"i2c\" 时给".into()),
                    });
                }
            }
        }
        let cap = self.store.get(id).ok_or_else(|| ToolError {
            message: format!("找不到采集 {id}"),
            hint: Some("用 scope_list_captures 看现存采集".into()),
        })?;

        match p.protocol {
            ProtocolArg::I2c => {
                // 别名路径：组装既有参数类型，复用 i2c_decode 的全部校验
                // 与输出格式（B3 已证明适配层与直连逐项一致）。
                let i2c = p.i2c.clone().unwrap_or_default();
                let args = I2cDecodeArgs {
                    capture_id: p.capture_id,
                    scl_channel: i2c.scl_channel,
                    sda_channel: i2c.sda_channel,
                    vih_lsb: i2c.vih_lsb,
                    vil_lsb: i2c.vil_lsb,
                    debounce_ns: i2c.debounce_ns,
                };
                self.i2c_decode(&args)
            }
            ProtocolArg::Uart => {
                // 通道：省略用 0（真机只有一路）；显式给则必须恰好 1 个且不越界。
                let ch = match p.channels.as_deref() {
                    None | Some([]) => 0usize,
                    Some([c]) => u8_field(*c, "channels")? as usize,
                    Some(_) => {
                        return Err(ToolError {
                            message: "UART 解码只用 1 个通道，channels 最多给 1 个".into(),
                            hint: Some("I²C 才需要两条线（用 protocol=\"i2c\"）".into()),
                        })
                    }
                };
                let baud = narrow::<u32>(
                    p.uart
                        .as_ref()
                        .and_then(|u| u.baud)
                        .unwrap_or(defaults::DECODE_BAUD),
                    1,
                    4_000_000,
                    "baud",
                    "波特率（1..=4000000）；欠采样时解码器会拒绝出帧",
                )?;

                let req = scope_core::decoder::DecodeRequest {
                    protocol: scope_core::decoder::Protocol::Uart,
                    channels: vec![ch],
                    params: scope_core::decoder::DecodeParams::Uart(
                        scope_core::decoder::UartConfig {
                            baud,
                            ..scope_core::decoder::UartConfig::default()
                        },
                    ),
                };
                let out = scope_core::decoder::decode(cap, &req)
                    .map_err(ScopeError::from)
                    .map_err(ToolError::from)?;

                // 帧列表逐条给 —— ok=false 的帧照列（字节不可信，但错误率
                // 必须可见），不静默丢弃。
                let frames: Vec<Value> = out
                    .frames
                    .iter()
                    .map(|f| match f {
                        scope_core::decoder::Frame::Uart(u) => json!({
                            "index": u.index,
                            "start_sample": u.start_sample,
                            "byte": u.byte,
                            "ok": u.ok,
                        }),
                        scope_core::decoder::Frame::I2c(_) => unreachable!("UART 结果"),
                    })
                    .collect();
                Ok(json!({
                    "schema": "decode/uart-8n1",
                    "capture_id": id,
                    "protocol": "uart",
                    "baud": baud,
                    "frame_count": out.frame_count(),
                    "frames": frames,
                    "bytes": out.uart_bytes(),
                    "quality": {
                        "sample_margin": out.quality.sample_margin,
                        "frame_ok": out.quality.frame_ok,
                        "frame_errors": out.quality.frame_errors,
                    },
                    "warnings": out.warnings.iter().map(|w| w.text()).collect::<Vec<_>>(),
                }))
            }
        }
    }

    /// `scope_list_captures`。
    ///
    /// # 为什么带 `device_tick_us` / `wall_time` / `expected_samples`
    ///
    /// 这三个字段此前**没有任何出口**。对 Agent 来说无所谓（它只看统计量），
    /// 但对「**在另一个进程里重建同一个 `Capture`**」就是硬缺口：
    ///
    /// | 字段 | 拿不到的后果 |
    /// |---|---|
    /// | `device_tick_us` | 重建时只能填 0 |
    /// | `wall_time` | `Capture::new` 会填成本地 now —— **静默不同** |
    /// | `expected_samples` | ⚠ **`report.rs` 的「点数少于期望」告警永远不触发** |
    ///
    /// 最后一条是真问题：那个告警存在的意义就是「传输有缺口时别假装数据完整」，
    /// 而期望值拿不到，它就等于被永久静音了。
    ///
    /// 加了之后，这个方法同时成为镜像侧的**唯一元数据源** —— 重建一个
    /// `Capture` 所需的非样点字段全在这里，不必再去别处凑。
    pub fn list_captures(&self) -> R {
        let list: Vec<Value> = self
            .store
            .iter()
            .map(|c| {
                json!({
                    "capture_id": c.id,
                    "rate_hz": c.rate_hz,
                    "sample_count": c.len(),
                    "channels": c.channels.len(),
                    "duration_ms": c.duration_us() as f64 / 1000.0,
                    "trigger_index": c.trigger_index,
                    "overrun": c.overrun,
                    // 重建 Capture 所需的另外三个字段
                    "device_tick_us": c.device_tick_us,
                    "wall_time": c.wall_time,
                    "expected_samples": c.expected_samples,
                })
            })
            .collect();
        Ok(json!({ "captures": list, "capacity": scope_core::DEFAULT_HISTORY }))
    }

    /// `scope_save_capture` —— 全量落盘。**数据不进 LLM 上下文。**
    pub fn save_capture(&self, p: &SaveCaptureArgs) -> R {
        let id = capture_id(p.capture_id)?;
        // schema 里曾经宣称支持 npy / bin，实现却一律写 CSV —— 静默落错格式
        // 比报错危险得多：Agent 会拿一个 CSV 当二进制去解析。
        // 现在 `SaveFormat` 只有一个变体，非 csv 在参数那一关就被挡下，
        // 这里的 `unwrap_or` 只是取出「用户确认要的那个格式」。
        let format = p.format.unwrap_or(SaveFormat::Csv);
        let path = p.path.as_str();

        let cap = self.store.get(id).ok_or_else(|| ToolError {
            message: format!("找不到采集 {id}"),
            hint: Some("用 scope_list_captures 看现存采集".into()),
        })?;

        // `cap` 借 `self.store`，这里借 `self.calib_view` —— 不同字段，可以同时成立。
        let scales = self.calib_view.clone();
        std::fs::write(path, cap.to_csv(scales.as_slice())).map_err(|e| ToolError {
            message: format!("写 {path} 失败：{e}"),
            hint: Some("确认目录存在、有写权限".into()),
        })?;

        Ok(json!({
            "saved": path,
            "format": format.name(),
            "bytes": std::fs::metadata(path).map(|m| m.len()).unwrap_or(0),
            // CSV 里的伏特列用的就是这套换算，所以要如实说
            "voltage_calibrated": scales.all_calibrated(),
            "note": "全量样点已落盘。不要在对话里读它 —— 4096 点 ≈ 上万 token",
        }))
    }

    /// `scope_import_capture` —— 从 CSV 导入一次采集到本会话。
    ///
    /// **不连设备**：读文件、解析、入 capture store。编号从
    /// `[IMPORT_ID_BASE, IMPORT_ID_LIMIT)` 里挑一个没被占用的，与设备分配的
    /// `1..n` 不会撞号。
    ///
    /// 两个电压换算参数都不给时，按**当前标定窗口**解释电压列 —— 那正是
    /// `scope_save_capture` 写出的那套换算，所以「导出 → 另起会话导入」
    /// 可以逐点还原。
    ///
    /// 路径经 [`csv_in_cwd`] 把关：只认工作目录内的 `.csv`。
    pub fn import_capture(&mut self, p: &ImportCaptureArgs) -> R {
        let path = csv_in_cwd(&p.path)?;
        let text = std::fs::read_to_string(&path).map_err(|e| ToolError {
            message: format!("读 {} 失败：{e}", path.display()),
            hint: Some("确认路径存在且有读权限".into()),
        })?;

        let mut o = if p.full_scale_v.is_none() && p.offset_v.is_none() {
            ImportOptions::from_scale(&self.calib_view.get(0))
        } else {
            ImportOptions::default()
        };
        o.id = self.next_import_id();
        o.path = p.path.clone();
        o.rate_hz = p.rate_hz;
        if let Some(v) = p.full_scale_v {
            o.full_scale_v = v;
        }
        if let Some(v) = p.offset_v {
            o.offset_v = v;
        }
        o.raw = p.raw.unwrap_or(false);
        o.normalize_t0 = p.normalize_t0.unwrap_or(false);

        let imported = from_csv(&text, &o).map_err(|e| ToolError {
            message: e.to_string(),
            hint: Some(
                "确认表头形如 `Time(s),CH1V`；速率写在首行 `# rate_hz=`，或显式给 rate_hz".into(),
            ),
        })?;

        let id = imported.capture.id;
        let rate = imported.capture.rate_hz;
        let samples = imported.capture.len();
        let channels = imported.capture.channels.len();
        let rate_source = match imported.rate_source {
            RateSource::Header => "header",
            RateSource::Option => "option",
            RateSource::TimeColumn => "time_column",
        };
        let t0_s = imported.t0_s;
        self.store.push(imported.capture);

        Ok(json!({
            "capture_id": id,
            "path": p.path,
            "rate_hz": rate,
            "rate_source": rate_source,
            "samples": samples,
            "channels": channels,
            "t0_s": t0_s,
            "voltage_calibrated": self.calib_view.all_calibrated(),
            "note": "已进入本会话的 capture store；可直接用 scope_measure / scope_decode 引用这个 capture_id",
        }))
    }

    /// 找一个没被占用的导入编号。
    fn next_import_id(&self) -> u16 {
        (IMPORT_ID_BASE..IMPORT_ID_LIMIT)
            .find(|id| self.store.get(*id).is_none())
            .unwrap_or(IMPORT_ID_BASE)
    }

    /// `scope_watch` —— 在一段时间里连续单次采集，返回滚动计数与缺口统计。
    ///
    /// **不是设备侧流模式**：固件与模拟器都还没实现 `STREAMING` 状态
    /// （那是 P1 之后的事）。用「重复单次采集」达成同样的目的 ——
    /// Agent 要的是「这一秒里总线上发生过什么」，不是真流。
    ///
    /// 两处与初版不同：
    /// - **未连接当场报错**。初版把一切错误都计进 `gaps`，于是「还没连设备」
    ///   会伪装成「1 秒里 100 万次没触发」，还返回 `isError=false`。
    /// - **中间采集不入库**。初版全部都 push，500 ms 就能把用户已有的采集
    ///   全挤出容量 16 的历史。改为只保留第一次，供 Agent 后续 measure/decode。
    pub fn watch(&mut self, p: &WatchArgs) -> R {
        let duration_ms = p.duration_ms.unwrap_or(defaults::WATCH_MS);
        // ⚠ 这条检查是**必需的**，不是参数洁癖：下面那个 deadline 一旦被算到
        // 天文数字，这个 while 循环会永远转下去，而 `serve()` 是单线程 ——
        // 整个 server 从此不再响应任何请求，只能杀进程重启。
        // 回归：`{"duration_ms": 18446744073709551615}` 一次就能做到（实测
        // 15 秒无任何响应，被外部强杀）。
        check_timeout(duration_ms, "duration_ms")?;

        let max_preview = p.max_points.unwrap_or(defaults::WATCH_POINTS);
        if max_preview == 0 {
            return Err(ToolError {
                message: "max_points 至少为 1".into(),
                hint: Some(format!("上限是 {PREVIEW_MAX}（三层 token 防护第一层）")),
            });
        }
        let max_preview = max_preview.min(PREVIEW_MAX) as usize;

        if self.bus.is_none() {
            return Err(ToolError {
                message: "尚未连接设备，无法观察".into(),
                hint: Some("先调用 scope_connect".into()),
            });
        }

        // 循环外克隆一次：一次 watch 期间标定不该变，而且循环里
        // `self.acquire_raw` 一直占着 `&mut self`。
        let scales = self.calib_view.clone();

        let deadline = std::time::Instant::now() + std::time::Duration::from_millis(duration_ms);
        let mut captured = 0u32;
        let mut failed = 0u32;
        let mut overruns = 0u32;
        let mut first: Option<Capture> = None;

        while std::time::Instant::now() < deadline {
            match self.acquire_raw(300) {
                Ok(cap) => {
                    captured += 1;
                    if cap.overrun {
                        overruns += 1;
                    }
                    if first.is_none() {
                        first = Some(cap);
                    }
                }
                // 这次没等到触发 —— 总线可能正闲，计入缺口而不是报错
                Err(_) => failed += 1,
            }
        }

        // 只把第一次采集入库，且是**最后**才 push —— 保证它一定还在 store 里
        let sample = match first {
            Some(cap) => {
                let s = capture_summary_json(&cap, max_preview, &scales);
                self.store.push(cap);
                Some(s)
            }
            None => None,
        };

        Ok(json!({
            "requested_ms": duration_ms,
            "captures": captured,
            "gaps": failed,
            "overruns": overruns,
            "sample": sample,
            "note": "用重复单次采集实现（设备侧流模式尚未实现），两次采集之间必有间隙； gaps 是其中没等到触发的次数。只有第一次采集进了历史，\"sample\" 的 capture_id 可直接用于 scope_measure / scope_read_waveform。",
        }))
    }

    /// `scope_sim_set_scenario` —— 仅模拟器。
    ///
    /// `scenario` 的合法性在参数那关就验过了（[`ScenarioArg`] 的反序列化
    /// 会列出全部可选值），所以这里不再重复一遍清单。
    pub fn sim_set_scenario(&mut self, p: &SimSetScenarioArgs) -> R {
        let sc = Scenario::from(p.scenario);

        // 先**校验并算好**故障配置，再动设备。
        //
        // 回归：从前这一句跟在 `set_scenario` / `set_seed` **之后**，于是
        // 参数非法时是「半应用」—— 场景与种子已经改了，故障没改，而工具
        // 返回的是错误。Agent 会以为整次调用都没生效。
        let faults = match &p.inject {
            Some(inj) => Some(inj.clone().into_faults().map_err(|m| {
                ToolError {
                    message: m,
                    hint: Some(
                        "概率是 0.0..=1.0 的比值。要关掉这一项就把它设成 0，\
                     不是设成负数 —— 负值会被当成「越界」拒绝而不是「关闭」"
                            .into(),
                    ),
                }
            })?),
            None => None,
        };

        let bus = self.bus("切换场景")?;
        let _ = bus.stop();
        match bus.port_mut().sim_mut() {
            Some(sim) => {
                sim.set_scenario(sc);
                // `seed` 与 `inject` 从前是**声明了但没人读**：传了就返回成功，
                // 而工具说明写着「注入故障」。Agent 于是会拿着干净数据下结论，
                // 以为自己刚注入过故障。现在它们真的生效。
                if let Some(seed) = p.seed {
                    sim.set_seed(seed);
                }
                if let Some(f) = faults {
                    // 整体替换，不做合并 —— 「我还留着哪些故障」不该靠翻历史才知道
                    sim.faults = f;
                }
                let faults = sim.faults.clone();
                let clean = faults.is_clean();

                // 场景换了通道数也会换（i2c 是 2 通道，正弦是 1 通道）。
                // 不重新 GET_INFO 的话，之后 capture 会拿旧的 ch_count 去拉
                // 不存在的通道，设备回 BadParam，而提示会把排查方向指错。
                bus.connect().map_err(ToolError::from)?;
                Ok(json!({
                    "scenario": sc.name(),
                    "channels": bus.info.as_ref().map(|i| i.ch_count),
                    "seed_set": p.seed,
                    // 回显故障状态：Agent 在解读后续采集前，必须先知道
                    // 这些「异常」是不是自己刚注入的
                    "faults_clean": clean,
                    "note": if clean {
                        "模拟器当前不注入任何故障"
                    } else {
                        "模拟器正在注入故障 —— 采集里的异常可能是注入出来的，不是真的"
                    },
                }))
            }
            None => Err(ToolError {
                message: "当前连接的不是模拟器".into(),
                hint: Some("这个工具只在 transport=\"sim\" 时注册".into()),
            }),
        }
    }

    /// `scope_debug_raw` —— 开发者逃生门。
    pub fn debug_raw(&mut self, p: &DebugRawArgs) -> R {
        // 先收窄再查表 —— 从前是 `as u16`，于是 `cmd: 65537` **回绕成 1**，
        // 一个越界的输入变成了一条真实存在的命令。
        let code = narrow::<u16>(
            p.cmd,
            0,
            u64::from(u16::MAX),
            "cmd",
            "命令码是 u16，见 proto/protocol.h 的命令表",
        )?;
        let cmd = Cmd::try_from_u16(code).ok_or_else(|| ToolError {
            message: format!("未知命令码 0x{code:04X}"),
            hint: Some("见 proto/protocol.h 的命令表".into()),
        })?;

        // 逐个 token 严格解析 —— `filter_map(..ok())` 会把解析失败的字节**静默
        // 丢掉**，于是「发的 payload」与「写的 payload」不是一回事，
        // 而调试工具最怕的就是这个。
        let payload: Vec<u8> = match p.payload_hex.as_deref() {
            None => Vec::new(),
            Some(h) => {
                let mut out = Vec::with_capacity(h.len() / 2);
                for tok in h.split_whitespace() {
                    let t = tok.trim_start_matches("0x");
                    let b = u8::from_str_radix(t, 16).map_err(|_| ToolError {
                        message: format!("payload_hex 里有非法字节 {tok:?}"),
                        hint: Some("用空格分隔的两位十六进制，例如 \"11 22 33\"".into()),
                    })?;
                    out.push(b);
                }
                out
            }
        };

        let bus = self.bus("发原始命令")?;
        let resp = bus
            .transaction(cmd, payload, std::time::Duration::from_millis(500))
            .map_err(ToolError::from)?;
        Ok(json!({
            "cmd": format!("{cmd:?}"),
            "seq": resp.seq,
            "flags": resp.flags,
            "payload_hex": resp
                .payload
                .iter()
                .map(|b| format!("{b:02X}"))
                .collect::<Vec<_>>()
                .join(" "),
        }))
    }
}

// ══════════════════════════════════════════════════════════════
// JSON 构造辅助
// ══════════════════════════════════════════════════════════════

fn info_json(i: &DeviceInfo) -> Value {
    json!({
        // 标定表的键。给出来，Agent 才能把「这台设备」和「有没有它的标定」
        // 对上；也给 `scope-cli cal set --uid` 提供现成的值。
        "uid": scope_core::encode_uid(&i.uid),
        "proto_ver": i.proto_ver,
        "fw_ver": format!("{}.{}.{}", (i.fw_ver >> 16) & 0xFF, (i.fw_ver >> 8) & 0xFF, i.fw_ver & 0xFF),
        "model": format!("0x{:04X}", i.model),
        "channels": i.ch_count,
        "adc_bits": i.adc_bits,
        "rate_max_hz": i.rate_max_hz,
        "capture_max_samples": i.capture_max_samples,
        "preferred_chunk_samples": i.preferred_chunk_samples,
        "max_rx_payload": i.max_rx_payload,
        "max_tx_payload": i.max_tx_payload,
    })
}

fn config_json(c: &scope_core::DeviceConfig) -> Value {
    json!({
        "rate_hz": c.rate_hz,
        "acq_mode": c.acq_mode,
        "capture_samples": c.capture_samples,
        "format": c.format,
        "decimation": c.decimation,
        "trigger_mode": c.trigger_mode,
        "trigger_source": c.trigger_source,
        "trigger_edge": c.trigger_edge,
        "trigger_level_lsb": c.trigger_level_lsb,
        // 通道 0 的使能与耦合。`GET_CONFIG` 里就这两个通道字段 ——
        // 量程与偏移不在 payload 里，只能靠 `SET_CHANNEL` 的回显拿到。
        "ch0_enable": c.ch0_enable,
        "ch0_coupling": c.ch0_coupling,
    })
}

/// 采集摘要 —— **三层 token 防护的第一层**。
///
/// 只给统计量 + ≤`max_preview` 点的 minmax 预览，绝不回全量样点。
fn capture_summary_json(cap: &Capture, max_preview: usize, scales: &ScaleSet) -> Value {
    let mut channels = Vec::new();
    for ch in 0..cap.channels.len() {
        let Some(s) = cap.summary(ch) else { continue };
        let m = scope_core::measure(cap, ch, &scales.get(ch));
        channels.push(json!({
            "channel": ch,
            // 叫 sample_count 而不是 samples：后者读起来像「样点数组」，
            // 而这里装的是数量。名字含糊在 LLM 那边会变成误判。
            "sample_count": s.n_samples,
            "min_lsb": s.min_lsb,
            "max_lsb": s.max_lsb,
            "pp_lsb": s.pp_lsb,
            "mean_lsb": s.mean_lsb,
            "rms_lsb": s.rms_lsb,
            "ac_rms_lsb": s.ac_rms_lsb,
            "rising_edges": s.rising_edges,
            "vpp_v": m.as_ref().map(|m| m.vpp),
            "freq_hz": m.as_ref().and_then(|m| m.freq_hz),
            "duty_pct": m.as_ref().and_then(|m| m.duty_pct),
            "rise_ns": m.as_ref().and_then(|m| m.rise_ns),
            "fall_ns": m.as_ref().and_then(|m| m.fall_ns),
            "overshoot_pct": m.as_ref().and_then(|m| m.overshoot_pct),
            "undershoot_pct": m.as_ref().and_then(|m| m.undershoot_pct),
            "high_ns": m.as_ref().and_then(|m| m.high_ns),
            // ⚠ 与上面 `rising_edges` **不同源**：那个固定在 2048 LSB 判定，
            // 这个用实测中点（见 MetricKind::Edges 的说明）
            "edges_mid": m.as_ref().map(|m| m.edges),
        }));
    }

    // 预览只给一条通道 —— minmax 的意义是保住尖峰不被平均掉
    let preview = cap.preview(0, max_preview).map(|p| {
        json!({
            "channel": p.channel,
            "bucket": p.bucket,
            "points": p.y_min.len(),
            "t0_us": p.t0_us,
            "dt_us": p.dt_us,
            "y_min": p.y_min,
            "y_max": p.y_max,
        })
    });

    json!({
        "capture_id": cap.id,
        "rate_hz": cap.rate_hz,
        "sample_count": cap.len(),
        "duration_ms": cap.duration_us() as f64 / 1000.0,
        "trigger_index": cap.trigger_index,
        "overrun": cap.overrun,
        "channels": channels,
        "preview": preview,
        "note": "只给统计量与 minmax 预览。全量样点走 scope_read_waveform（分页）\
                 或 scope_save_capture（落盘）—— 4096 点 ≈ 上万 token，不进上下文。",
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::de::DeserializeOwned;

    /// 把 JSON 解成参数类型 —— 走的是**真的反序列化路径**（含
    /// `deny_unknown_fields` 与各字段的类型检查），不是直接构造结构体。
    /// 直接构造的话，这些测试就绕开了这次重构最想守住的那道关。
    fn args<T: DeserializeOwned>(v: Value) -> T {
        serde_json::from_value(v).expect("测试给的参数应当能解析")
    }

    /// 同上，但预期解析会失败。
    fn try_args<T: DeserializeOwned>(v: Value) -> std::result::Result<T, serde_json::Error> {
        serde_json::from_value(v)
    }

    /// 连模拟器 + 采一次，返回会话。
    fn connected() -> Session {
        let mut s = Session::new();
        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "i2c_100k" }),
        ))
        .expect("连模拟器不该失败");
        s
    }

    #[test]
    fn refuses_to_work_before_connecting() {
        let mut s = Session::new();
        let e = s.capture(&args(json!({}))).unwrap_err();
        assert!(e.message.contains("尚未连接"), "实测 {}", e.message);
        assert!(e.hint.is_some(), "错误必须给出「怎么办」");
    }

    #[test]
    fn capture_returns_stats_and_preview_but_not_the_raw_waveform() {
        // 三层 token 防护的第一层：统计量 + ≤256 点预览，绝不回全量样点。
        let mut s = connected();
        let v = s.capture(&args(json!({}))).unwrap();
        assert!(v["sample_count"].as_u64().unwrap() > 1000);
        assert!(!v["channels"].as_array().unwrap().is_empty());
        let pts = v["preview"]["points"].as_u64().unwrap();
        assert!(pts <= PREVIEW_MAX, "预览桶数 {pts} 超过上限");
        // 关键：**响应体积有上界**。
        //
        // 直接按「有没有样点数组」查会误伤吗？不会 —— 但要区分清楚：
        // `preview` 里那两串 256 个数字是**设计要求**（minmax 预览），
        // 真正不能出现的是 4096 点的全量波形。全量波形一进来，响应
        // 至少 2 万字符，所以卡体积比数数组更贴近要保证的性质。
        let size = serde_json::to_string(&v).unwrap().len();
        assert!(
            size < 8000,
            "摘要响应 {size} 字符，太大了 —— 全量波形漏进来了？"
        );
    }

    #[test]
    fn capture_decodes_by_the_echoed_chunk_format_not_by_the_request() {
        // 回归：主机**无条件请求 PACK12**，但必须按分片头回显的 `format` 解码。
        // 这里把模拟器切成「无论请求什么格式都按 RAW16 回」（分片头 format=0），
        // 复刻真机完全有权这么做的情况。
        //
        // 一旦 `acquire` 的调用点改成按请求常量（PACK12）解码，两次采集的
        // 样点就不再逐点相同 —— 这是那条属性唯一的红灯（这条测试别删）。
        let mut plain = connected();
        let mut forced = connected();
        forced
            .bus
            .as_mut()
            .expect("connected() 必然建立了总线")
            .port_mut()
            .sim_mut()
            .expect("transport=sim 应当拿到 SimDevice")
            .set_read_buffer_force_raw16(true);

        let expect = plain.acquire_raw(2000).expect("正常采集应当成功");
        let got = forced
            .acquire_raw(2000)
            .expect("设备回 RAW16 时采集也必须成功 —— 按回显格式解码就不会报错");

        assert!(
            !expect.channels.is_empty() && !expect.channels[0].is_empty(),
            "采集不应为空，否则这条测试什么都没验证"
        );
        assert_eq!(
            got.channels, expect.channels,
            "同一份波形，RAW16 回包与 PACK12 回包解出的样点必须逐点相同"
        );
    }

    #[test]
    fn capture_rejects_stream_mode_instead_of_silently_doing_one_shot() {
        // 回归：`mode` 从前在 schema 里声明了却**没人读** ——
        // 传 "stream" 会静默按单次跑，而 Agent 以为拿到的是流数据。
        let mut s = connected();
        let e = s.capture(&args(json!({ "mode": "stream" }))).unwrap_err();
        assert!(e.message.contains("流模式"), "实测 {}", e.message);
        assert!(
            e.hint.as_deref().unwrap_or("").contains("scope_watch"),
            "要说清替代方案，实测 {:?}",
            e.hint
        );
        // "single" 照常
        assert!(s.capture(&args(json!({ "mode": "single" }))).is_ok());
    }

    #[test]
    fn read_waveform_rejects_requests_over_the_hard_cap() {
        // 超硬顶必须**拒绝**，不能静默截断 —— 静默截断会让 Agent
        // 以为拿到了全部数据。要全量请走 save_capture。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let e = s
            .read_waveform(&args(json!({ "capture_id": 1, "count": 99999 })))
            .unwrap_err();
        assert!(e.message.contains("4096"), "实测 {}", e.message);
        assert!(
            e.hint.as_deref().unwrap_or("").contains("save_capture"),
            "提示应该指向落盘那条路"
        );
    }

    #[test]
    fn read_waveform_pages() {
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let a = s
            .read_waveform(&args(json!({ "capture_id": 1, "count": 16 })))
            .unwrap();
        assert_eq!(a["count"], 16);
        assert_eq!(a["has_more"], true);
        assert_eq!(
            a["format"], "raw",
            "回显格式 —— 样点是原始 LSB 整数，不是伏特"
        );
        let b = s
            .read_waveform(&args(
                json!({ "capture_id": 1, "start_sample": 16, "count": 16 }),
            ))
            .unwrap();
        assert_ne!(a["samples"], b["samples"], "分页两段不该相同");
    }

    #[test]
    fn read_waveform_count_wins_over_the_legacy_max_points() {
        // 两个都给时以 `count` 为准 —— 回归：从前是「谁先出现用谁」，
        // 同一个请求换个字段顺序结果就不一样。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s
            .read_waveform(&args(json!({
                "capture_id": 1, "max_points": 32, "count": 8
            })))
            .unwrap();
        assert_eq!(v["count"], 8);
        let v = s
            .read_waveform(&args(json!({
                "capture_id": 1, "count": 8, "max_points": 32
            })))
            .unwrap();
        assert_eq!(v["count"], 8, "换个字段顺序不该改变结果");
    }

    #[test]
    fn i2c_decode_returns_frames() {
        // 默认场景现在是一笔**完整的**「写寄存器指针 → 重复起始 → 读数据」。
        // 这条测试锁的是这个形状：两帧、一写一读、第二帧是重复起始。
        //
        // 回归：从前默认事务是 `[0x88, 0x00, 0x1A]` —— **只有写、没有读**，
        // 于是任何问「为什么读出来全是 0」的 Agent 都会得到「总线上根本没有
        // 读事务」这个诊断。那在模拟器上是对的，对真实总线却是推出来的。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s.i2c_decode(&args(json!({ "capture_id": 1 }))).unwrap();
        assert!(v["frame_count"].as_u64().unwrap() >= 2, "应解出多帧");

        let w = &v["frames"][0];
        assert_eq!(w["address"]["value"], 0x44, "地址应是 0x44");
        assert_eq!(w["address"]["read"], false, "第一帧是写");
        assert_eq!(w["bytes"], json!([0x00]), "写的是寄存器指针 0x00");

        let r = &v["frames"][1];
        assert_eq!(r["address"]["value"], 0x44, "同一个从机地址");
        assert_eq!(r["address"]["read"], true, "第二帧是读");
        assert_eq!(r["repeated_start"], true, "第二帧应以重复起始开头");
        assert_eq!(r["bytes"], json!([0x01, 0x2C]), "读回两个字节");

        assert_eq!(v["trustworthy"], true);
    }

    #[test]
    fn i2c_decode_reports_a_nack() {
        // README 给 P3 定的验收场景就是「抓一次 I2C 写时序并告诉我**为什么
        // NACK**」，而在此之前模拟器**造不出 NACK** —— `transaction()` 把
        // 应答位写死在每个字节后面。这条测试锁住新补的场景。
        let mut s = Session::new();
        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "i2c_nack" }),
        ))
        .unwrap();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s.i2c_decode(&args(json!({ "capture_id": 1 }))).unwrap();

        let f = &v["frames"][0];
        assert_eq!(
            f["address"]["acked"], true,
            "地址被应答了 —— 从机在总线上、地址也对"
        );
        assert_eq!(f["nack"], true, "但这一笔写被 NACK 了");
        assert_eq!(
            f["bytes"],
            json!([0x00]),
            "被 NACK 的那个字节仍然出现在总线上，解码器应当照实报出来"
        );
        // `complete` 说的是「有正常 STOP 收尾」，**不是**「没有 NACK」——
        // 被 NACK 的帧照样会以 STOP 正常结束。`false` 只意味着被采集窗截断。
        // 这两个概念混起来读，会把「器件拒绝了你」误读成「数据不完整」。
        assert_eq!(f["complete"], true, "这一帧是正常收尾的，只是内容被拒了");
        // 从机在，地址对，但这一笔它不收 —— 解码器不该因此判定整条总线不可信
        assert_eq!(v["trustworthy"], true);
    }

    /// `scope_decode(protocol=uart)`：单通道场景解出真值序列。
    /// 这是「协议分析在真机上成立」的那条路径 —— 真机只有 1 个通道。
    #[test]
    fn scope_decode_uart_decodes_the_sim_scenario() {
        let mut s = Session::new();
        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "uart_115k" }),
        ))
        .unwrap();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s
            .decode(&args(json!({
                "capture_id": 1,
                "protocol": "uart",
                "uart": { "baud": 115200 },
            })))
            .unwrap();

        assert_eq!(v["protocol"], "uart");
        assert_eq!(v["baud"], 115200);
        assert_eq!(v["schema"], "decode/uart-8n1");
        let n = v["frame_count"].as_u64().unwrap();
        assert!(n >= 3, "1024 点应至少解出几帧，实际 {n}");
        // 首字节必须是真值 0x55（锁定态：采集窗从起始位开始）。
        assert_eq!(v["bytes"][0], 0x55, "首帧字节：{v}");
        assert_eq!(v["quality"]["frame_errors"], 0, "正确波特率下零错帧：{v}");
        assert!(
            v["quality"]["sample_margin"].as_f64().unwrap() >= 4.0,
            "欠采样门内必须有 sample_margin"
        );
    }

    /// 别名等价：`scope_decode(protocol=i2c)` 与 `scope_i2c_decode`
    /// 对同一采集给出一致的帧数与字节。
    #[test]
    fn scope_decode_i2c_is_a_real_alias() {
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();

        let via_alias = s.i2c_decode(&args(json!({ "capture_id": 1 }))).unwrap();
        let via_unified = s
            .decode(&args(json!({
                "capture_id": 1,
                "protocol": "i2c",
            })))
            .unwrap();

        assert_eq!(
            via_alias["frame_count"], via_unified["frame_count"],
            "别名与统一入口的帧数必须一致"
        );
        let a = &via_alias["frames"];
        let u = &via_unified["frames"];
        assert_eq!(a, u, "帧列表（含地址、ACK/NACK、字节）必须逐项一致");
    }

    /// 互斥参数显式报错，不静默忽略 —— 混用几乎一定是调用方的错。
    #[test]
    fn scope_decode_rejects_mixed_protocol_params() {
        let mut s = Session::new();
        let e = s
            .decode(&args(json!({
                "capture_id": 1,
                "protocol": "uart",
                "i2c": { "scl_channel": 0 },
            })))
            .unwrap_err();
        assert!(e.message.contains("协议不匹配"), "实测：{}", e.message);

        let e = s
            .decode(&args(json!({
                "capture_id": 1,
                "protocol": "i2c",
                "uart": { "baud": 9600 },
            })))
            .unwrap_err();
        assert!(e.message.contains("协议不匹配"), "实测：{}", e.message);
    }

    #[test]
    fn i2c_decode_on_a_single_channel_capture_says_why() {
        let mut s = Session::new();
        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "sine_1k_3v3" }),
        ))
        .unwrap();
        let _ = s.capture(&args(json!({}))).unwrap();
        let e = s.i2c_decode(&args(json!({ "capture_id": 1 }))).unwrap_err();
        assert!(e.message.contains("两条线"), "实测 {}", e.message);
        assert!(e.hint.is_some());
    }

    #[test]
    fn i2c_decode_refuses_a_half_specified_channel_pair() {
        // 回归：从前的 `_ =>` 分支把「只指定了 SCL」变成「两个都忽略、
        // 退回自动判定」。自动判定多半能猜对，于是返回的帧列表看着完全
        // 正常 —— 这个 bug 几乎不可能被顺路径测试发现。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let e = s
            .i2c_decode(&args(json!({ "capture_id": 1, "scl_channel": 0 })))
            .unwrap_err();
        assert!(e.message.contains("要么都给"), "实测 {}", e.message);
        assert!(
            e.hint.as_deref().unwrap_or("").contains("SCL="),
            "提示里要顺带告诉它自动判定会怎么判，实测 {:?}",
            e.hint
        );
        // 两个都给就正常。但「正常」本身说明不了显式通道被采用了 ——
        // 自动判定多半也判成 (0,1)，静默忽略同样能拿到这份结果。
        // 所以判据是**把两条线对调**：结果必须跟着变。
        let normal = s
            .i2c_decode(&args(json!({
                "capture_id": 1, "scl_channel": 0, "sda_channel": 1
            })))
            .unwrap();
        assert!(normal["frame_count"].as_u64().unwrap() > 0);

        let swapped = s
            .i2c_decode(&args(json!({
                "capture_id": 1, "scl_channel": 1, "sda_channel": 0
            })))
            .unwrap();
        assert_eq!(swapped["scl_channel"], 1, "回显的应当是显式给的那个通道");
        assert_ne!(
            normal["signal_quality"]["scl_edges"], swapped["signal_quality"]["scl_edges"],
            "把 SCL/SDA 对调之后边沿统计没变 —— 显式通道其实没被采用？"
        );
    }

    #[test]
    fn i2c_decode_rejects_a_level_that_would_wrap_around() {
        // 65536 as u16 == 0。一个「门限 = 0」的判决带能让整条总线看起来
        // 全是高电平，而解码器照样返回一份像是成功的结果。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        for bad in [65536u32, 4096, u32::MAX] {
            let e = s
                .i2c_decode(&args(json!({ "capture_id": 1, "vih_lsb": bad })))
                .unwrap_err();
            assert!(
                e.message.contains(&bad.to_string()),
                "vih_lsb={bad} 应当被拒绝，实测 {}",
                e.message
            );
        }
        // 4095 是边界内，应当通过
        assert!(s
            .i2c_decode(&args(
                json!({ "capture_id": 1, "vih_lsb": 4095, "vil_lsb": 100 })
            ))
            .is_ok());
    }

    #[test]
    fn configure_reports_the_quantized_rate() {
        // SET_* 必须回显实际生效值 —— 定时器只能出整数分频的档位。
        //
        // 回归：这条测试**曾经用 857143**（比上限大 1）来触发量化。改成按
        // schema 声明的上界拒绝越界值之后，那个输入直接撞在参数校验上，
        // 测的就不再是量化逻辑了。这里换成范围内、但落在两个定时器档位
        // 之间的值（72 MHz / 100001 不是整数分频）。
        let mut s = connected();
        let v = s
            .configure(&args(json!({ "sample_rate_hz": 100_001 })))
            .unwrap();
        let actual = v["applied"]["sample_rate_hz"].as_u64().unwrap();
        assert_ne!(actual, 100_001, "100001 Hz 不是可达到的档位，应当被量化");
        let w = v["warnings"].as_array().unwrap();
        assert!(!w.is_empty(), "量化了就要说");
        assert!(
            w[0].as_str().unwrap().contains(&actual.to_string()),
            "量化提示里要写清实际值，实测 {:?}",
            w[0]
        );
    }

    #[test]
    fn configure_rejects_a_rate_above_the_single_adc_limit() {
        // schema 里写的是 `max = MAX_SAMPLE_RATE_HZ`(857142，单 ADC)，
        // 而 `MAX_INTERLEAVED_HZ`(1714285) 是双 ADC 交织的**理论上限**、
        // 文档明说只用于显示。两者不能混用 ——
        // 回归：收窄当初用的是交织上限，于是 1000000 能过这一关再撞到核心层，
        // 而错误消息说「上限 1714285 Hz」，与 schema 自相矛盾。
        let mut s = connected();
        for (v, ok) in [(857_142u64, true), (857_143, false), (1_000_000, false)] {
            let r = s.configure(&args(json!({ "sample_rate_hz": v })));
            assert_eq!(r.is_ok(), ok, "sample_rate_hz={v} 的接受性不对：{r:?}");
            if !ok {
                let e = r.unwrap_err();
                assert!(
                    e.message.contains("857142"),
                    "错误里要说清真实上界，实测 {}",
                    e.message
                );
            }
        }
    }

    #[test]
    fn configure_echoes_the_channel_it_received() {
        // `channel` 从前只回显在一条警告的文本里（「暂未接线上层」），
        // 现在它是真的下发并回读的 —— 回显也变成了 `applied.channel` 里的
        // 结构化字段，Agent 不必去解析中文。
        let mut s = connected();
        let v = s
            .configure(&args(json!({
                "channel": { "ch": 0, "enable": false, "coupling": "ac", "offset_lsb": -100 }
            })))
            .unwrap();
        let a = &v["applied"]["channel"];
        assert_eq!(a["ch"], 0);
        assert_eq!(a["enable"], false);
        assert_eq!(a["coupling"], "ac");
        assert_eq!(a["offset_lsb"], -100);
    }

    #[test]
    fn configure_reports_the_trigger_it_applied_in_canonical_form() {
        let mut s = connected();
        let v = s
            .configure(&args(json!({
                "trigger": { "mode": "auto", "edge": "falling", "level_lsb": 1000, "source": 0 }
            })))
            .unwrap();
        assert_eq!(v["applied"]["trigger"]["mode"], "auto");
        assert_eq!(v["applied"]["trigger"]["edge"], "falling");
        assert_eq!(v["applied"]["trigger"]["level_lsb"], 1000);
    }

    #[test]
    fn configured_trigger_survives_a_capture() {
        // 这条才是「configure 真的下发到设备了」的判据 ——
        // 上面那条只看回显，而回显是照着请求拼出来的，**发错电平它也不会红**。
        //
        // 而且必须**中间夹一次采集**：采集路径（`core/acquire.rs`）从前会
        // 无条件重发 `set_trigger(1, 0, 0, ..)` 与 `set_acq(0, .., 0, 1)`，
        // 把 configure 配好的触发模式/边沿/源与采集格式/抽点整个推回默认。
        // 配置在 `scope_status` 里「看着生效」，真正决定数据的那一次却没用上。
        let mut s = connected();
        // ⚠ 每个值都要**与 acquire 从前的硬编码不同**，否则测不出覆盖。
        // `source` 曾经配 0 —— 而硬编码的 source 也是 0，于是「source 被覆盖」
        // 这个变异体它能照样通过（第一版就是这么写的，被验证抓出来了）。
        // 硬编码是 (mode=1 normal, source=0 CH1, edge=0 rising) + (format=0, decim=1)。
        s.configure(&args(json!({
            "decimation": 8,
            "format": "minmax",
            "trigger": { "mode": "auto", "edge": "falling", "level_lsb": 1000, "source": 1 }
        })))
        .unwrap();

        let before = s.status().unwrap()["config"].clone();
        assert_eq!(before["trigger_mode"], 0, "configure 之后设备侧就该是 auto");
        assert_eq!(
            before["trigger_source"], 1,
            "configure 之后设备侧就该是 CH2"
        );
        assert_eq!(before["format"], 2, "configure 之后设备侧就该是 minmax");

        let _ = s.capture(&args(json!({}))).unwrap();

        let after = s.status().unwrap()["config"].clone();
        // 五个字段都要比 —— 少了 `trigger_level_lsb` 的话，
        // 「acquire 把电平钉成 2048」这个变异体不会被发现。
        for k in [
            "trigger_mode",
            "trigger_edge",
            "trigger_source",
            "trigger_level_lsb",
            "format",
            "decimation",
        ] {
            assert_eq!(
                after[k], before[k],
                "采集之后 {k} 被改回了默认值 —— acquire 又在硬编码 set_trigger/set_acq？"
            );
        }
    }

    #[test]
    fn configure_omitted_trigger_fields_inherit_from_the_device() {
        // schema 里写着「省略的子项沿用设备当前值」，这条测试就是那句话的判据。
        // 回归：从前实现一律 `unwrap_or(默认)`，于是 Agent 只想改个电平，
        // 却把刚配好的 auto / 下降沿 / CH1 全部重置回 normal / rising / CH0 ——
        // 与同函数里采集三件套（它们**确实**沿用现值）各说各话。
        let mut s = connected();
        s.configure(&args(json!({
            "trigger": { "mode": "auto", "edge": "falling", "level_lsb": 1000, "source": 1 }
        })))
        .unwrap();

        // 只改电平，别的都不给
        let v = s
            .configure(&args(json!({ "trigger": { "level_lsb": 1500 } })))
            .unwrap();
        assert_eq!(v["applied"]["trigger"]["level_lsb"], 1500);
        assert_eq!(v["applied"]["trigger"]["mode"], "auto", "模式应当沿用");
        assert_eq!(v["applied"]["trigger"]["edge"], "falling", "边沿应当沿用");
        // `source` 也要查 —— 少了它，`None => 0` 那个变异体照样绿
        assert_eq!(v["applied"]["trigger"]["source"], 1, "触发源应当沿用");

        // 省略 level 时也该沿用现值，不是跳回 2048
        let v = s
            .configure(&args(json!({ "trigger": { "mode": "normal" } })))
            .unwrap();
        assert_eq!(v["applied"]["trigger"]["level_lsb"], 1500, "电平应当沿用");
        assert_eq!(v["applied"]["trigger"]["source"], 1, "触发源应当沿用");
    }

    #[test]
    fn measure_reports_honest_units() {
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s.measure(&args(json!({ "capture_id": 1 }))).unwrap();
        let ch0 = &v["measurements"]["ch0"];
        // 幅值必须是正的 —— 回归：曾经把幅值当电平换算，得到负数
        assert!(ch0["vpp_v"].as_f64().unwrap() > 0.0);
        assert!(ch0["ac_rms_v"].as_f64().unwrap() >= 0.0);
        // 真 RMS 也必须有出口 —— 从前只有 AC-RMS，真 RMS 完全消失
        assert!(ch0["rms_v"].as_f64().unwrap() >= 0.0);
        // 统一分析结果对象的三个键
        assert_eq!(v["schema"], "analysis/v1");
        assert_eq!(v["provenance"]["source"]["kind"], "device");
        assert_eq!(v["provenance"]["source"]["simulated"], true);
        // 每个量都有判决；频率第一版至少是「拒答」，不能凭空 Ok
        assert_eq!(ch0["quality"]["verdicts"]["edges"]["kind"], "ok");
        assert_eq!(ch0["quality"]["verdicts"]["freq_hz"]["kind"], "refused");
        // A3：MCP 出口与同一个 AnalysisReport 的标量逐字段一致
        let cap = s.store.get(1).expect("采集应还在 store 里");
        let scales = s.calib_view.clone();
        let report = scope_core::AnalysisReport::from_capture(cap, &scales, "sim", true);
        let ca = report.channel(0).expect("报告应有通道 0");
        assert_eq!(ch0["vpp_v"].as_f64().unwrap(), ca.measurements.vpp);
        assert_eq!(ch0["rms_v"].as_f64().unwrap(), ca.measurements.rms);
        assert_eq!(
            ch0["freq_hz"].as_f64().unwrap(),
            ca.measurements.freq_hz.unwrap()
        );
        // SCL 是 100 kHz
        let f = ch0["freq_hz"].as_f64().unwrap();
        assert!((f - 100_000.0).abs() < 500.0, "实测 {f} Hz");
    }

    /// 标定状态要**结构化**地给出来，不能只写在一句中文 note 里 ——
    /// 模型得读懂那句话才知道要打折，而一个布尔字段它不会漏掉。
    ///
    /// 这条同时钉住 `Session::new()` **不碰磁盘**：测试没有注入标定，
    /// 所以无论开发机 `%APPDATA%` 里有什么，这里都必须是 false。
    #[test]
    fn the_session_reports_an_uncalibrated_voltage_by_default() {
        let mut s = connected();
        let c = s
            .connect(&args(
                json!({ "transport": "sim", "sim_scenario": "i2c_100k" }),
            ))
            .unwrap();
        assert!(c["device"]["uid"].is_string(), "uid 要暴露给 Agent");
        assert_eq!(c["calibration"]["record_found"], false);
        assert_eq!(c["calibration"]["calibrated"], false);

        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s.measure(&args(json!({ "capture_id": 1 }))).unwrap();
        assert_eq!(v["voltage_calibrated"], false);
    }

    /// 注入标定后，**数值必须真的随标定缩放**。
    ///
    /// 只断言布尔字段是不够的 —— 那只证明标志位传对了，不证明换算真的
    /// 走到了标定值。所以这里拿同一场景的未标定基线，比一个**可预测的比值**。
    #[test]
    fn an_injected_calibration_scales_the_numbers() {
        // 未标定基线
        let mut base = connected();
        let _ = base.capture(&args(json!({}))).unwrap();
        let b = base.measure(&args(json!({ "capture_id": 1 }))).unwrap();
        assert_eq!(b["voltage_calibrated"], false, "基线必须是未标定");
        let base_vpp = b["measurements"]["ch0"]["vpp_v"].as_f64().unwrap();

        // 模拟器自身的 uid（`sim/src/device.rs:422`），标定表按它索引
        let uid = scope_core::decode_uid("5eed00010203040506070809").unwrap();
        // 占位值是 3.3/4096 ≈ 0.00080566；取 0.001 让比值恰为 1.241212…
        let per_lsb = 1.0e-3;
        let mut store = CalibrationStore::empty();
        for ch in 0..2 {
            store
                .set_channel(
                    uid,
                    ch,
                    scope_core::ChannelScale {
                        volts_per_lsb: per_lsb,
                        zero_lsb: 2048.0,
                    },
                    None,
                    0,
                )
                .unwrap();
        }

        let mut s = Session::new().with_calibration(store);
        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "i2c_100k" }),
        ))
        .unwrap();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s.measure(&args(json!({ "capture_id": 1 }))).unwrap();

        assert_eq!(v["voltage_calibrated"], true, "注入了标定就该报 true");
        let vpp = v["measurements"]["ch0"]["vpp_v"].as_f64().unwrap();
        let ratio = per_lsb / (3.3 / 4096.0);
        let want = base_vpp * ratio;
        assert!(
            (vpp - want).abs() < want * 0.02,
            "Vpp 应随标定缩放：基线 {base_vpp:.4} V × {ratio:.6} = {want:.4} V，实测 {vpp:.4} V"
        );
    }

    #[test]
    fn read_waveform_survives_an_absurd_start_sample() {
        // 回归：start + want 曾溢出 → debug 下 panic → **整个 server 进程死掉**，
        // 后续请求（含在途的）全部丢失。一个合法 JSON 整数就能远程打挂它。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let v = s
            .read_waveform(&args(json!({
                "capture_id": 1,
                "start_sample": u64::MAX,
                "count": 1
            })))
            .expect("越界的 start_sample 应返回空页，而不是崩掉");
        assert_eq!(v["count"], 0);
        assert_eq!(v["has_more"], false);
    }

    #[test]
    fn reconnecting_clears_the_capture_store() {
        // 回归：换设备后 capture_id 从 1 重新分配，而 store 按 id 取数 ——
        // 旧采集会**遮蔽**同 id 的新采集，measure/decode/read/save 全部
        // 静默返回过期数据。
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        assert_eq!(
            s.list_captures().unwrap()["captures"]
                .as_array()
                .unwrap()
                .len(),
            1
        );

        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "sine_1k_3v3" }),
        ))
        .unwrap();
        assert_eq!(
            s.list_captures().unwrap()["captures"]
                .as_array()
                .unwrap()
                .len(),
            0,
            "换连接后历史应清空，否则旧采集会遮蔽同 id 的新采集"
        );
        let _ = s.capture(&args(json!({}))).unwrap();
        let caps = s.list_captures().unwrap();
        let arr = caps["captures"].as_array().unwrap();
        assert_eq!(arr.len(), 1);
        assert_eq!(arr[0]["channels"], 1, "留下的应是新场景（单通道）的采集");
    }

    /// `list_captures` 要能让**另一个进程**重建出同一个 `Capture`。
    ///
    /// 用途是「GUI 起 scope-mcp 子进程、自己当显示端点」那条路：
    /// AI 在子进程里采，GUI 要把那一窗画出来。重建方**只有这个接口**，
    /// 缺一个字段就意味着重建出来的东西与子进程里那份不同 —— 而且**静默**。
    ///
    /// 最要命的是 `expected_samples`：缺了它，`report.rs` 的
    /// 「实际点数少于期望」告警永远不触发，等于把「传输有缺口」这件事静音了。
    ///
    /// 这条测试逐字段对照 store 里的原件 —— **任何字段被删掉都会红**，
    /// 所以它同时守住了「旧字段一个不少」。
    #[test]
    fn list_captures_exposes_everything_needed_to_rebuild_a_capture() {
        let mut s = connected();
        let summary = s.capture(&args(json!({}))).unwrap();
        let id = summary["capture_id"].as_u64().unwrap() as u16;
        let stored = s.store.get(id).expect("刚采的应当在 store 里");

        let caps = s.list_captures().unwrap();
        let row = caps["captures"]
            .as_array()
            .unwrap()
            .iter()
            .find(|c| c["capture_id"].as_u64() == Some(id as u64))
            .expect("刚采的应当在列表里");

        // —— 原有的字段 ——
        assert_eq!(
            row["rate_hz"].as_u64(),
            Some(stored.rate_hz as u64),
            "rate_hz"
        );
        assert_eq!(
            row["sample_count"].as_u64(),
            Some(stored.len() as u64),
            "sample_count"
        );
        assert_eq!(
            row["channels"].as_u64(),
            Some(stored.channels.len() as u64),
            "channels"
        );
        assert_eq!(row["overrun"].as_bool(), Some(stored.overrun), "overrun");
        match stored.trigger_index {
            Some(i) => assert_eq!(
                row["trigger_index"].as_u64(),
                Some(i as u64),
                "trigger_index"
            ),
            None => assert!(row["trigger_index"].is_null(), "trigger_index 应为 null"),
        }

        // —— 重建 Capture 必需的三个字段 ——
        assert_eq!(
            row["device_tick_us"].as_u64(),
            Some(stored.device_tick_us as u64),
            "device_tick_us 缺了就只能填 0"
        );
        assert_eq!(
            row["wall_time"].as_u64(),
            Some(stored.wall_time),
            "wall_time 缺了会被重建方填成本地 now —— 静默不同"
        );
        assert_eq!(
            row["expected_samples"].as_u64(),
            Some(stored.expected_samples as u64),
            "expected_samples 缺了 → report.rs 的缺口告警永久静音"
        );
    }

    #[test]
    fn unknown_sim_scenario_is_rejected_not_silently_replaced() {
        // 静默换成默认场景会让 Agent 以为连的是自己指定的波形。
        // 现在这一关在**反序列化**时就守住了，错误里直接列出可选清单。
        let e = try_args::<ConnectArgs>(json!({
            "transport": "sim", "sim_scenario": "i2c_999k"
        }))
        .unwrap_err();
        let msg = e.to_string();
        assert!(msg.contains("未知场景"), "实测 {msg}");
        assert!(
            msg.contains("i2c_100k"),
            "错误里应当列出可选场景，实测 {msg}"
        );
    }

    #[test]
    fn capture_samples_is_validated_not_truncated() {
        let mut s = connected();
        // 65537 as u16 == 1：静默回绕会让「采 65537 点」变成「采 1 点」
        let e = s
            .configure(&args(json!({ "capture_samples": 65537 })))
            .unwrap_err();
        assert!(e.message.contains("65537"), "实测 {}", e.message);
        // 0 点会让每次采集都「成功但空」，状态还会持续污染历史
        assert!(s.configure(&args(json!({ "capture_samples": 0 }))).is_err());
    }

    #[test]
    fn save_capture_refuses_formats_it_cannot_write() {
        // schema 曾宣称支持 npy / bin，实际一律写 CSV —— 现在 `SaveFormat`
        // 只有一个变体，非 csv 在参数解析那关就被挡下。
        let e = try_args::<SaveCaptureArgs>(json!({
            "capture_id": 1, "path": "x.npy", "format": "npy"
        }))
        .unwrap_err();
        assert!(e.to_string().contains("npy"), "实测 {e}");
    }

    #[test]
    fn measure_metrics_filter_and_reject_typos() {
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();

        let v = s
            .measure(&args(
                json!({ "capture_id": 1, "metrics": ["vpp", "freq"], "channel": 0 }),
            ))
            .unwrap();
        let m = &v["measurements"];
        assert!(m.get("ch1").is_none(), "指定 channel=0 时不该回 ch1");
        let ch0 = &m["ch0"];
        assert!(ch0.get("vpp_v").is_some() && ch0.get("freq_hz").is_some());
        assert!(ch0.get("duty_pct").is_none(), "没要的指标不该回");

        // 拼错的名字必须报错 —— 静默忽略会让 Agent 以为测过了。
        // 现在这份清单**就是 `MetricKind` 这个类型**，不再有两份要靠人眼对齐。
        let e = try_args::<MeasureArgs>(json!({ "capture_id": 1, "metrics": ["freqency"] }))
            .unwrap_err();
        let msg = e.to_string();
        assert!(msg.contains("freqency"), "实测 {msg}");
        assert!(msg.contains("freq"), "错误里应当列出合法取值，实测 {msg}");
    }

    /// 新增的五个指标（fall/overshoot/undershoot/high/edges）要能取到，
    /// 且**不请求时不得出现** —— 这一条防的是「变体加了、输出行忘了」
    /// 的静默错：schema 里列着指标，返回却永远是空的。
    ///
    /// 变异（杀掉）：把 session.rs 里五个 `if has(...)` 行删掉任意一条 ——
    /// 这条的对应断言红。
    #[test]
    fn the_new_metrics_are_served_only_when_asked() {
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();

        let v = s
            .measure(&args(json!({
                "capture_id": 1, "channel": 0,
                "metrics": ["fall", "overshoot", "undershoot", "high", "edges"]
            })))
            .unwrap();
        let ch0 = &v["measurements"]["ch0"];
        for key in [
            "fall_ns",
            "overshoot_pct",
            "undershoot_pct",
            "high_ns",
            "edges",
        ] {
            assert!(ch0.get(key).is_some(), "请求了却没回：{key}");
        }

        let v2 = s
            .measure(&args(
                json!({ "capture_id": 1, "metrics": ["vpp"], "channel": 0 }),
            ))
            .unwrap();
        let ch0 = &v2["measurements"]["ch0"];
        assert!(
            ch0.get("fall_ns").is_none() && ch0.get("edges").is_none(),
            "没要的指标不该回"
        );
    }

    #[test]
    fn watch_refuses_to_run_without_a_connection() {
        // 回归：初版把一切错误计进 gaps，于是「还没连设备」伪装成
        // 「1 秒里 100 万次没触发」，还返回 isError=false。
        let mut s = Session::new();
        let e = s.watch(&args(json!({ "duration_ms": 50 }))).unwrap_err();
        assert!(e.message.contains("尚未连接"), "实测 {}", e.message);
    }

    #[test]
    fn watch_does_not_flood_the_capture_store() {
        let mut s = connected();
        let before = s.list_captures().unwrap()["captures"]
            .as_array()
            .unwrap()
            .len();
        let v = s.watch(&args(json!({ "duration_ms": 400 }))).unwrap();
        let after = s.list_captures().unwrap()["captures"]
            .as_array()
            .unwrap()
            .len();
        assert!(
            v["captures"].as_u64().unwrap() > 1,
            "这段时间应采到不止一次"
        );
        assert_eq!(
            after,
            before + 1,
            "watch 只该入库一次（供后续 measure/read），而不是每次都 push"
        );
    }

    #[test]
    fn save_capture_writes_the_file() {
        let mut s = connected();
        let _ = s.capture(&args(json!({}))).unwrap();
        let path = std::env::temp_dir().join("scope-mcp-test.csv");
        let p = path.to_string_lossy().into_owned();
        let v = s
            .save_capture(&args(json!({ "capture_id": 1, "path": p })))
            .unwrap();
        assert!(v["bytes"].as_u64().unwrap() > 0, "文件应有内容");
        assert_eq!(v["format"], "csv");
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(
            text.starts_with("# rate_hz="),
            "CSV 首行应是速率注释（时间列 9 位小数反推不回精确速率）"
        );
        assert!(text.contains("Time(s)"), "CSV 表头不对");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn imported_capture_is_usable_by_measure_and_decode() {
        // C5：导入的采集必须真的进了 store，且下游工具给出与原件一致的结论。
        // 走的是**文件**这条路（导出 → 另起会话导入），不是内存里换引用。
        let mut src = Session::new();
        src.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "uart_115k" }),
        ))
        .unwrap();
        let summary = src.capture(&args(json!({}))).unwrap();
        let src_rate = summary["rate_hz"].as_u64().unwrap();
        // 导入只认工作目录内的 .csv，所以不能写系统临时目录。
        let path = std::env::current_dir()
            .unwrap()
            .join("scope-mcp-test-import-roundtrip.csv");
        let p = path.to_string_lossy().into_owned();
        src.save_capture(&args(json!({ "capture_id": 1, "path": p })))
            .unwrap();

        let mut dst = Session::new();
        let v = dst.import_capture(&args(json!({ "path": p }))).unwrap();
        let _ = std::fs::remove_file(&path);
        let id = v["capture_id"].as_u64().unwrap();
        assert!(
            (IMPORT_ID_BASE as u64..IMPORT_ID_LIMIT as u64).contains(&id),
            "导入采集应落在专用编号段，实际 {id}"
        );
        assert_eq!(v["rate_hz"].as_u64(), Some(src_rate), "速率来自首行注释");
        assert_eq!(v["rate_source"], "header");

        let m = dst.measure(&args(json!({ "capture_id": id }))).unwrap();
        assert_eq!(m["schema"], "analysis/v1");
        assert!(
            !m["measurements"]["ch0"]["vpp_v"].is_null(),
            "导入的采集应能测量：{m}"
        );

        let d = dst
            .decode(&args(json!({
                "capture_id": id,
                "protocol": "uart",
                "uart": { "baud": 115200 },
            })))
            .unwrap();
        assert_eq!(d["schema"], "decode/uart-8n1");
        assert!(
            d["frame_count"].as_u64().unwrap() >= 3,
            "导入的采集应能解码：{d}"
        );
        assert_eq!(d["bytes"][0], 0x55);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn import_only_accepts_csv_inside_the_working_directory() {
        let mut s = Session::new();

        // 1) 扩展名不对 —— 在读文件之前就该拒绝。
        let e = s
            .import_capture(&args(json!({ "path": "capture.txt" })))
            .unwrap_err();
        assert!(e.message.contains(".csv"), "实测：{}", e.message);

        // 2) 存在、但是真实路径在 cwd 之外（系统临时目录）—— 必须拒绝。
        let outside = std::env::temp_dir().join("scope-mcp-outside.csv");
        std::fs::write(
            &outside,
            "# rate_hz=1000000\nTime(s),CH1V\n0.000000000,0.000000\n",
        )
        .unwrap();
        let e = s
            .import_capture(&args(json!({ "path": outside.to_string_lossy() })))
            .unwrap_err();
        let _ = std::fs::remove_file(&outside);
        assert!(e.message.contains("不在工作目录内"), "实测：{}", e.message);
        assert!(e.hint.is_some(), "越界路径要给出「怎么办」");
    }

    // ── 模拟器：seed 与 inject ────────────────────────────────────
    //
    // 这两个字段从前是**声明了但没人读**：传了照样返回成功，而工具说明
    // 写着「注入故障」。Agent 会拿着干净数据下结论，以为自己刚注入过。

    // ── 通道配置：从「只回显」到「真的生效」────────────────────────

    #[test]
    fn configure_channel_actually_reaches_the_device() {
        // 回归：这个分支从前**一条命令都不发**，只推一条「暂未接线上层」的
        // 警告，而工具描述写着「返回实际生效值」。
        let mut s = connected();
        let v = s
            .configure(&args(json!({ "channel": { "ch": 0, "enable": false } })))
            .unwrap();
        assert_eq!(v["applied"]["channel"]["enable"], false);

        // 判据是**回读设备侧**，不是看回显 —— 回显是照着请求拼的
        let st = s.status().unwrap();
        assert_eq!(st["config"]["ch0_enable"], 0, "SET_CHANNEL 没真的下发");
    }

    #[test]
    fn a_disabled_channel_actually_comes_back_flat() {
        // 「存下来」不等于「生效」。这条测的是**可观察效果** ——
        // 从前模拟器把 `ch0_enable` 存下来并在 GET_CONFIG 里回显，
        // 但**没有任何代码用它**：关掉一个通道，采集数据一字不变。
        let mut s = connected();
        let before = s.capture(&args(json!({}))).unwrap();
        assert!(
            before["channels"][0]["pp_lsb"].as_i64().unwrap() > 1000,
            "默认场景 CH1 应当是有信号的"
        );

        s.configure(&args(json!({ "channel": { "ch": 0, "enable": false } })))
            .unwrap();
        let after = s.capture(&args(json!({}))).unwrap();
        assert_eq!(
            after["channels"][0]["pp_lsb"], 0,
            "关掉的通道应当是一条平线"
        );
        assert!(
            after["channels"][1]["pp_lsb"].as_i64().unwrap() > 1000,
            "另一个通道不该受影响"
        );
    }

    #[test]
    fn channel_offset_actually_shifts_the_trace() {
        // 用**恒定**场景（dc）：触发点会随电平移动，用变化的波形比较均值
        // 会掺进「这次采到的是哪一段」的影响 —— 第一版就是这样差了 2 LSB。
        let mut s = Session::new();
        s.connect(&args(json!({ "transport": "sim", "sim_scenario": "dc" })))
            .unwrap();

        let base = s.capture(&args(json!({}))).unwrap()["channels"][0]["mean_lsb"]
            .as_f64()
            .unwrap();

        s.configure(&args(json!({ "channel": { "ch": 0, "offset_lsb": 200 } })))
            .unwrap();
        let shifted = s.capture(&args(json!({}))).unwrap()["channels"][0]["mean_lsb"]
            .as_f64()
            .unwrap();

        assert!(
            (shifted - base - 200.0).abs() < 0.5,
            "偏移应当真的加到样点上：{base} → {shifted}"
        );
    }

    #[test]
    fn configuring_channel_1_does_not_touch_channel_0() {
        // 回归：模拟器的 `cmd_set_channel` 从前无论 `ch` 是几都往 `ch0_*` 里写 ——
        // 配 CH2 会**改掉 CH1 的配置**。
        let mut s = connected();
        s.configure(&args(json!({ "channel": { "ch": 1, "enable": false } })))
            .unwrap();

        let st = s.status().unwrap();
        assert_eq!(st["config"]["ch0_enable"], 1, "配 CH2 不该动 CH1");

        let cap = s.capture(&args(json!({}))).unwrap();
        assert_eq!(cap["channels"][1]["pp_lsb"], 0, "CH2 应当被关掉");
        assert!(
            cap["channels"][0]["pp_lsb"].as_i64().unwrap() > 1000,
            "CH1 应当没事"
        );
    }

    #[test]
    fn mechanical_switch_fields_are_flagged_not_silently_accepted() {
        // 本板的耦合与量程是手拨机械开关，AI 无法程控。设备会记录并回显
        // 下发值，但**实际开关位置以硬件为准** —— 不说的话 Agent 会以为
        // 耦合真的切到 AC 了，然后拿一份和它以为的不一样的波形下结论。
        let mut s = connected();
        let v = s
            .configure(&args(json!({
                "channel": { "ch": 0, "coupling": "ac", "range_idx": 1 }
            })))
            .unwrap();
        let w = v["warnings"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|x| x.as_str())
            .collect::<Vec<_>>()
            .join(" | ");
        assert!(w.contains("SW2"), "耦合是机械开关，要说清：{w}");
        assert!(w.contains("SW3"), "量程是机械开关，要说清：{w}");
    }

    #[test]
    fn status_reports_real_link_counters() {
        // 回归：core 的 `get_status` 从前**只读第一个字节**就把 33 字节的
        // payload 丢了 —— 溢出计数、CRC 错误计数一路都到不了 Agent。
        // 模拟器那边更早：这几个字段直接写死为 0。
        let mut s = connected();

        let st = s.status().unwrap();
        assert_eq!(st["link_health"]["clean"], true, "刚连上应当是干净的：{st}");
        assert_eq!(st["link_health"]["overrun_samples"], 0);

        // 注入一次溢出 → 计数必须真的涨
        s.sim_set_scenario(&args(json!({
            "scenario": "i2c_100k",
            "inject": { "force_overrun": true }
        })))
        .unwrap();
        let _ = s.capture(&args(json!({}))).unwrap();

        let st = s.status().unwrap();
        assert!(
            st["link_health"]["overrun_samples"].as_u64().unwrap() > 0,
            "溢出计数应当真的涨上去：{st}"
        );
        assert_eq!(st["link_health"]["clean"], false);
        assert_eq!(st["link_health"]["err_flags"], 1, "bit0 表示发生过溢出");
    }

    #[test]
    fn status_counter_survives_a_reconnect_reset() {
        // 计数器属于设备，不属于会话 —— 重连之后应当重新从新设备读。
        // （这条同时守住「store 清空」与「计数器不清空」不会互相干扰。）
        let mut s = connected();
        let st = s.status().unwrap();
        assert!(st["link_health"]["clean"].as_bool().unwrap_or(false));
        s.connect(&args(json!({ "transport": "sim", "sim_scenario": "dc" })))
            .unwrap();
        let st = s.status().unwrap();
        assert_eq!(
            st["link_health"]["overrun_samples"], 0,
            "新设备应当是干净的"
        );
    }

    #[test]
    fn sim_set_scenario_applies_the_seed() {
        let mut s = Session::new();
        s.connect(&args(
            json!({ "transport": "sim", "sim_scenario": "noise" }),
        ))
        .unwrap();

        let shot = |s: &mut Session, seed: u64| -> Value {
            s.sim_set_scenario(&args(json!({ "scenario": "noise", "seed": seed })))
                .unwrap();
            let c = s.capture(&args(json!({}))).unwrap();
            c["preview"]["y_min"].clone()
        };

        // **同一个种子两次必须逐点相同** —— 只有这条能证明 `set_seed` 真的接线了。
        //
        // 回归：这条测试的前一版比的是「种子 1 与种子 2 的波形不同」。
        // 那个断言在 `set_seed` **完全空转**时照样成立 —— 两次采集之间
        // 生成器本来就在推进，波形天然不同。它证明不了任何事。
        // （`noise` 是纯 rng 场景，不看样点计数，所以同种子必定同波形。）
        let a1 = shot(&mut s, 42);
        let a2 = shot(&mut s, 42);
        assert_eq!(a1, a2, "同一个种子两次跑出不同波形 —— set_seed 没接线？");

        assert_ne!(a1, shot(&mut s, 7777), "换了种子波形应当不同");

        // **相邻**种子也必须给出不同波形。
        // 回归：`Rng::new` 从前是 `seed | 1`，把每一对相邻种子折成同一个
        // （2 与 3 都变成 3），于是「换个种子试试」有一半概率拿到同一条波形。
        assert_ne!(
            shot(&mut s, 2),
            shot(&mut s, 3),
            "相邻种子折成了同一个 —— Rng::new 又在 |1 了？"
        );
    }

    #[test]
    fn sim_set_scenario_applies_clears_and_keeps_faults() {
        let mut s = connected();

        // 1) 注入生效
        let v = s
            .sim_set_scenario(&args(json!({
                "scenario": "i2c_100k",
                "inject": { "force_overrun": true }
            })))
            .unwrap();
        assert_eq!(v["faults_clean"], false, "注入了故障就必须说");
        let cap = s.capture(&args(json!({}))).unwrap();
        assert_eq!(cap["overrun"], true, "注入的 force_overrun 应当真的生效");

        // 2) **不带 `inject` 时故障必须原样留着** —— 这才是「省略 ≠ 清空」的判据。
        //    回归：前一版把这条写成「先清空、再不带 inject 调用、断言还是干净」，
        //    而那时故障本来就已经干净，断言恒真 —— 一个空转的测试。
        let v = s
            .sim_set_scenario(&args(json!({ "scenario": "i2c_100k" })))
            .unwrap();
        assert_eq!(v["faults_clean"], false, "省略 inject = 不动故障，不是清空");
        let cap = s.capture(&args(json!({}))).unwrap();
        assert_eq!(cap["overrun"], true, "省略 inject 之后故障应当还在");

        // 3) `inject: {}` 的语义才是「清除全部故障」
        let v = s
            .sim_set_scenario(&args(json!({ "scenario": "i2c_100k", "inject": {} })))
            .unwrap();
        assert_eq!(v["faults_clean"], true);
        let cap = s.capture(&args(json!({}))).unwrap();
        assert_eq!(cap["overrun"], false, "清掉故障后不该再溢出");
    }

    #[test]
    fn sim_set_scenario_rejects_bad_injection_through_the_tool_layer() {
        // ⚠ 判据必须**经过 `Session::sim_set_scenario`**，不能直接调
        // `InjectArgs::into_faults`。
        //
        // 回归：第一版只有 params.rs 里一条直接调 `into_faults` 的测试。
        // 把 session.rs 里那句 `into_faults().map_err(..)?` 改成
        // `unwrap_or_default()`，**63 条测试全绿** —— 负概率又变回
        // 「静默清掉故障」，而响应还说 `faults_clean: true`。内层测到了、
        // 工具层零覆盖，等于没测。
        let mut s = connected();

        // 先注入一个**合法**故障，好证明不合法的输入不会把它悄悄清掉
        s.sim_set_scenario(&args(json!({
            "scenario": "i2c_100k",
            "inject": { "force_overrun": true }
        })))
        .unwrap();

        for bad in [
            json!({ "latency_spike_probability": -3.0 }),
            json!({ "latency_spike_probability": 2.5 }),
            json!({ "latency_spike_ms": defaults::MAX_LATENCY_SPIKE_MS + 1 }),
            json!({ "latency_spike_ms": u64::MAX }),
        ] {
            let e = s
                .sim_set_scenario(&args(json!({ "scenario": "i2c_100k", "inject": bad })))
                .unwrap_err();
            assert!(
                e.hint.is_some(),
                "参数错误也要给「怎么办」，实测 {}",
                e.message
            );
            // 关键：**故障必须原样留着**，不能被静默清掉
            let cap = s.capture(&args(json!({}))).unwrap();
            assert_eq!(
                cap["overrun"], true,
                "非法注入被拒绝时不该动已有故障（{bad}），实测 {}",
                e.message
            );
        }
    }

    #[test]
    fn capture_and_watch_reject_an_absurd_timeout() {
        // 回归：`check_timeout` 在 `capture` 与 `watch` 里各调一次，但
        // **MCP 层一条测试都没有** —— 核心层有（`acquire.rs` 的
        // `an_absurd_timeout_is_rejected...`），端到端我手跑过，
        // 可是 CI 里没有一条会拦住有人删掉那两行 `check_timeout(...)?`。
        //
        // 这两个输入从前会让 `serve()` 永久挂住（单线程，之后任何请求都不
        // 再被处理，只能杀进程），所以这条测试守的是一个**致命**行为，
        // 不是参数洁癖。
        let mut s = connected();
        for bad in [u64::MAX, MAX_TIMEOUT_MS + 1] {
            let e = s.capture(&args(json!({ "timeout_ms": bad }))).unwrap_err();
            assert!(e.message.contains("timeout_ms"), "实测 {}", e.message);
            assert!(
                e.message.contains(&MAX_TIMEOUT_MS.to_string()),
                "要说清上界"
            );
            assert!(e.hint.is_some(), "错误必须给出「怎么办」");

            let e = s.watch(&args(json!({ "duration_ms": bad }))).unwrap_err();
            assert!(e.message.contains("duration_ms"), "实测 {}", e.message);
        }

        // 0 也要拒 —— schema 里 minimum 是 1
        assert!(s.capture(&args(json!({ "timeout_ms": 0 }))).is_err());
        assert!(s.watch(&args(json!({ "duration_ms": 0 }))).is_err());

        // 边界值本身合法（差一错误会让「上限」变成「上限减一」）
        let v = s.capture(&args(
            json!({ "timeout_ms": MAX_TIMEOUT_MS, "mode": "single" }),
        ));
        // 会真的等到触发或超时；用 Dc + auto 场景不会等满 10 分钟才回来 ——
        // 这里只要求「不是被参数校验挡下的」。
        if let Err(e) = v {
            assert!(
                !e.message.contains("超出范围"),
                "正好等于上界不该被拒，实测 {}",
                e.message
            );
        }
    }

    #[test]
    fn preview_point_caps_reject_zero() {
        // 0 会让 `Capture::preview` 返回 `None` —— 一次「成功但没有预览」
        // 的采集，Agent 很容易读成「没采到数据」。schema 里 minimum 是 1。
        let mut s = connected();
        for e in [
            s.capture(&args(json!({ "max_preview_points": 0 })))
                .unwrap_err(),
            s.watch(&args(json!({ "max_points": 0 }))).unwrap_err(),
        ] {
            assert!(e.message.contains("至少为 1"), "实测 {}", e.message);
        }
    }

    #[test]
    fn an_absurd_latency_spike_cannot_hang_the_server() {
        // 回归：`latency_spike_ms` 会被原样交给 `sim` 的 `thread::sleep`，
        // 而 `serve()` 是单线程 —— u64::MAX 毫秒＝永久睡死，之后任何调用
        // 都不再被处理。这跟 `timeout_ms` / `duration_ms` 是同一个洞，
        // 从「故障注入」这个门又漏了回来。
        let mut s = connected();
        let e = s
            .sim_set_scenario(&args(json!({
                "scenario": "i2c_100k",
                "inject": { "latency_spike_probability": 1.0, "latency_spike_ms": u64::MAX }
            })))
            .unwrap_err();
        assert!(e.message.contains("latency_spike_ms"), "实测 {}", e.message);
        // 而且拒绝之后必须还能正常干活（没被半应用弄坏）
        assert!(s.status().is_ok());
        assert!(s.capture(&args(json!({}))).is_ok());
    }

    #[test]
    fn debug_raw_rejects_a_command_code_that_would_wrap_around() {
        // 回归：`as u16` 会把 65537 变成 1 —— 一个越界的输入
        // 变成了一条真实存在的命令。
        let mut s = connected();
        let e = s.debug_raw(&args(json!({ "cmd": 65537 }))).unwrap_err();
        assert!(e.message.contains("65537"), "实测 {}", e.message);
        // 65535 在范围内，但不是一个已知命令码
        let e = s.debug_raw(&args(json!({ "cmd": 65535 }))).unwrap_err();
        assert!(e.message.contains("未知命令码"), "实测 {}", e.message);
    }
}
