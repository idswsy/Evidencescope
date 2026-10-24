//! # scope-mcp —— MCP Server（P3 阶段）
//!
//! 把示波器能力以 **16 个粗粒度工具**暴露给 AI Agent。
//!
//! ## 状态
//!
//! 16 个工具**全部实现**（见 `session.rs`）。三种用法：
//! 1. 默认：跑 stdio 上的 MCP server（换行分隔的 JSON-RPC 2.0）
//! 2. `--list-tools`：打印工具清单与**由参数类型生成的** JSON Schema
//! 3. `--selftest`：走一遍模拟器，确认命令层可用
//!
//! `docs/08-agent-walkthrough.md` 是一份真实的 Agent 使用实录，
//! 生成程序在 `tools/agent_demo/`。
//!
//! ## 设计约束（为什么是"粗粒度"）
//!
//! 16 个工具 ≈ 16 个**Agent 意图**，而不是 32 个逐命令工具。
//! 后者会让工具列表爆炸，且一个意图要多次往返。
//!
//! ## 三层 token 防护（硬性，不可协商）
//!
//! 1. `scope_capture` / `scope_watch` 默认只回**统计量 + ≤256 点 minmax 预览**
//! 2. `scope_read_waveform` 分页、默认 512 点、**硬顶 4096**、超出拒绝并提示存盘
//! 3. **全量数据只进 capture store 和磁盘，永不进上下文**
//!
//! 一次 4096 点采集 = 8192 字节 ≈ 4096 个 JSON 数字 ≈ 上万 token。
//! 把它塞给 LLM 既贵又没用。

#![deny(clippy::all)]

mod jsonrpc;
mod params;
mod session;

use anyhow::{Context, Result};
use params::*;
use scope_core::{CaptureStore, CommandBus};
use scope_sim::{Scenario, SimDevice};
use serde_json::{json, Value};

/// MCP 工具注解（`tools/list` 里的 `annotations` 对象）。
///
/// 规范里这四个字段**都是提示，不强制**，且**缺省值是刻意悲观的**：
/// 不写就等于 `readOnlyHint=false` + `destructiveHint=true`。也就是说
/// 客户端会把「你没声明」理解成「这个工具可能有害」，连 `scope_status`
/// 这种纯读的也要走确认。所以每个工具都必须显式声明。
///
/// 字段名按规范用驼峰（`readOnlyHint` 等），由 `rename_all` 统一转换。
#[derive(Debug, Clone, Copy, serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct ToolAnnotations {
    /// 不给客户端任何「可免确认」的余地，除非真的只读。
    read_only_hint: bool,
    /// 可能毁掉设备或主机上的数据。只有逃生门为真。
    destructive_hint: bool,
    /// 同样的参数再调一次，不会产生额外后果。
    idempotent_hint: bool,
    /// 会与设备之外的系统交互（扫串口、写任意路径）。
    open_world_hint: bool,
}

/// 注解的几种典型组合。**在工具的声明处引用它们**，而不是各写各的 ——
/// 一处改动就能让同类工具一起变，也便于审查「哪些工具是只读的」。
#[allow(dead_code)] // 常量按语义成组给出，未必每一个都被用到
impl ToolAnnotations {
    /// 纯读：不改设备状态，不碰主机文件。
    const READ_ONLY: Self = Self {
        read_only_hint: true,
        destructive_hint: false,
        idempotent_hint: true,
        open_world_hint: false,
    };
    /// 只读，但要扫系统（枚举串口）。
    const READ_ONLY_OPEN: Self = Self {
        read_only_hint: true,
        destructive_hint: false,
        idempotent_hint: true,
        open_world_hint: true,
    };
    /// 改设备状态：连接、配置、采集、流。可重复调用，但每次都有新后果。
    const CHANGES_DEVICE: Self = Self {
        read_only_hint: false,
        destructive_hint: false,
        idempotent_hint: false,
        open_world_hint: false,
    };
    /// 写主机文件，路径由调用方给定。
    const WRITES_HOST: Self = Self {
        read_only_hint: false,
        destructive_hint: false,
        idempotent_hint: true,
        open_world_hint: true,
    };
    /// 逃生门：能发任意命令码，含 `MEM_WRITE`。
    const ESCAPE_HATCH: Self = Self {
        read_only_hint: false,
        destructive_hint: true,
        idempotent_hint: false,
        open_world_hint: false,
    };
}

/// 一个 MCP 工具的声明。
struct ToolSpec {
    name: &'static str,
    /// 一句话说明 —— 这是 LLM 决定要不要调用它的主要依据。
    description: &'static str,
    /// 输入参数的 JSON Schema —— **由参数类型生成**，不是手写的。
    schema: fn() -> Value,
    /// 对应哪些协议命令。
    protocol_cmds: &'static str,
    /// 是否只在某些条件下注册。
    condition: &'static str,
    /// 给 MCP 客户端的注解（只读 / 破坏性等）。见 [`ToolAnnotations`]。
    annotations: ToolAnnotations,
    /// 实现。内部会把 `arguments` 解成**生成 `schema` 的那个类型**。
    handler: fn(&mut session::Session, &Value) -> R,
}

/// 参数解析的结果类型（与 `session.rs` 里的一致）。
type R = std::result::Result<Value, session::ToolError>;

/// 声明一个工具。
///
/// `$args` 在整个宏体里**只写一次**，而展开后它既是 schema 的来源、
/// 又是解析的目标。这就是「声明的参数」与「实现读的参数」不可能不一致的
/// **全部机制** —— 想让它们分家，你得先让同一个宏参数同时是两个类型。
///
/// 从前 `input_schema` 是一段手写的 JSON 字符串，与 `session.rs` 里
/// `p.get("字段名")` 的读取之间没有任何联系。上一轮对抗性验证找出的 8 类
/// P1 里有 4 类是「schema 声明了、实现忽略」——`scope_capture.mode`、
/// `scope_read_waveform.format`、`scope_sim_set_scenario` 的 `seed` 与
/// `inject` 全都是传了不报错、静默走默认值。
macro_rules! tool {
    (
        $name:literal, $desc:literal, $cmds:literal, $cond:literal,
        $ann:expr,
        $args:ty, |$s:ident, $p:ident| $body:expr
    ) => {
        ToolSpec {
            name: $name,
            description: $desc,
            protocol_cmds: $cmds,
            condition: $cond,
            annotations: $ann,
            schema: schema_of::<$args>,
            handler: |$s: &mut session::Session, __args: &Value| {
                parse_then::<$args, _>(__args, |$p| $body)
            },
        }
    };
}

/// 从一个参数类型生成 JSON Schema。
///
/// schemars 生成的 schema 必定能转成 JSON —— 转不动是代码错误，应当当场
/// 炸掉。**从前这里是 `unwrap_or_else(|_| json!({"type":"object"}))`**：
/// 手写的 schema 字符串里多一个逗号，那个工具的参数说明就整个消失，
/// 客户端看到一个「不需要参数」的工具，而服务端一声不吭。
fn schema_of<T: schemars::JsonSchema>() -> Value {
    Value::from(schemars::schema_for!(T))
}

/// 把 `arguments` 解成 `T`，成功就交给 `f`。
///
/// 失败时给一条**指明字段**的错误，外加「本工具接受哪些参数」——后者直接
/// 从 `T` 自己的 schema 里读出来，所以不可能和实现说的不是一回事。
fn parse_then<T, F>(args: &Value, f: F) -> R
where
    T: serde::de::DeserializeOwned + schemars::JsonSchema,
    F: FnOnce(T) -> R,
{
    match serde_json::from_value::<T>(args.clone()) {
        Ok(t) => f(t),
        Err(e) => Err(session::ToolError {
            message: format!("参数不合法：{e}"),
            hint: Some(accepted_params::<T>()),
        }),
    }
}

/// 「本工具接受哪些参数」——从类型自己的 schema 里读。
fn accepted_params<T: schemars::JsonSchema>() -> String {
    let s = schema_of::<T>();
    let Some(props) = s.get("properties").and_then(|v| v.as_object()) else {
        return "本工具不接受任何参数；arguments 请留空或省略".into();
    };
    if props.is_empty() {
        return "本工具不接受任何参数；arguments 请留空或省略".into();
    }
    let required: Vec<&str> = s
        .get("required")
        .and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|v| v.as_str()).collect())
        .unwrap_or_default();
    let list: Vec<String> = props
        .iter()
        .map(|(k, v)| {
            let ty = type_label(v, &s);
            if required.contains(&k.as_str()) {
                format!("{k}:{ty}（必填）")
            } else {
                format!("{k}:{ty}")
            }
        })
        .collect();
    format!(
        "本工具接受的参数：{}。用 tools/list 可以看每一项的完整说明",
        list.join(" / ")
    )
}

/// 一个属性的 `type` 名单（可能是字符串，也可能是数组）。
fn type_names(s: &Value) -> Vec<&str> {
    match s.get("type") {
        Some(Value::String(x)) => vec![x.as_str()],
        Some(Value::Array(a)) => a.iter().filter_map(|v| v.as_str()).collect(),
        _ => Vec::new(),
    }
}

/// 把一个属性的 schema 解成「真身」+「是否可空」。
///
/// schemars 1.x 生成的是 JSON Schema 2020-12，同一个意思有好几种写法，
/// **三种都要认**：
///
/// | 写法 | 什么时候出现 |
/// |---|---|
/// | `"type": "integer"` | 必填的基本类型 |
/// | `"type": ["integer","null"]` | **可选**的基本类型 |
/// | `"anyOf": [<真身>, {"type":"null"}]` | 可选的自定义类型（枚举、结构体） |
/// | `"$ref": "#/$defs/X"` | 自定义类型本身被抽到了 `$defs` |
///
/// 外加派生枚举生成的是 `oneOf: [{"const": ...}]` 而不是 `enum`。
///
/// 抽出来共用是因为**这个坑会以另一种面貌重现在任何「读 schema 的地方」**：
/// 第一版 `type_label` 只认字符串，于是所有可选参数在提示里都显示成
/// 「任意」；而测试里按 schema 造值的 `sample_for` 犯了同一个错，于是
/// 造出了 `null` 去喂必填字段。两处各写一份，就会修一处漏一处。
fn resolve_prop<'a>(schema: &'a Value, root: &'a Value) -> (&'a Value, bool) {
    // 1. 拆 anyOf
    let (inner, nullable) = match schema.get("anyOf").and_then(|v| v.as_array()) {
        Some(arms) => {
            let is_null = |a: &Value| a.get("type").and_then(|t| t.as_str()) == Some("null");
            (
                arms.iter().find(|a| !is_null(a)).unwrap_or(schema),
                arms.iter().any(is_null),
            )
        }
        None => (schema, false),
    };

    // 2. 解 `$ref`（`#/$defs/X` → JSON Pointer 要去掉开头的 `#`）
    let resolved = match inner.get("$ref").and_then(|v| v.as_str()) {
        Some(r) => root
            .pointer(r.strip_prefix('#').unwrap_or(r))
            .unwrap_or(inner),
        None => inner,
    };

    (
        resolved,
        nullable || type_names(schema).contains(&"null") || type_names(resolved).contains(&"null"),
    )
}

/// 把 schema 的类型描述压缩成一句，用在参数清单里。
///
/// 例：`整数`、`整数?`（可选）、`serial|sim?`（可选枚举）、`数组?`。
///
/// 需要 `root` 才能解 `$ref` —— 见 [`resolve_prop`]。
fn type_label(schema: &Value, root: &Value) -> String {
    let (resolved, nullable) = resolve_prop(schema, root);

    let label = if let Some(e) = resolved.get("enum").and_then(|v| v.as_array()) {
        e.iter()
            .filter_map(|v| v.as_str())
            .collect::<Vec<_>>()
            .join("|")
    } else if let Some(o) = resolved.get("oneOf").and_then(|v| v.as_array()) {
        // 自定义枚举生成的是 `oneOf: [{"const":"serial"}, ...]`
        o.iter()
            .filter_map(|arm| arm.get("const").and_then(|c| c.as_str()))
            .collect::<Vec<_>>()
            .join("|")
    } else {
        type_names(resolved)
            .iter()
            .filter(|n| **n != "null")
            .map(|n| match *n {
                "string" => "字符串",
                "integer" => "整数",
                "number" => "数字",
                "boolean" => "布尔",
                "array" => "数组",
                "object" => "对象",
                other => other,
            })
            .collect::<Vec<_>>()
            .join("|")
    };

    match (label.is_empty(), nullable) {
        (true, _) => "任意".into(),
        // 「可选」用问号表示，不必在类型里重复说一遍 null
        (false, true) => format!("{label}?"),
        (false, false) => label,
    }
}

/// 工具清单。
///
/// 工具名与说明的**人读镜像**在 `docs/05-roadmap.md` 的「MCP 工具清单」一节
/// （`docs/03-protocol.md` 里没有 §MCP 这一节 —— 这里曾经指向它）。
/// 那边是手抄的，CI 有一条检查卡住工具名不许分家。
///
/// 每个条目的最后一个参数是参数类型：schema 由它生成，`arguments` 也解成它。
/// **不要在这里手写 schema 字符串** —— 见 [`tool!`] 的说明。
const TOOLS: &[ToolSpec] = &[
    tool!(
        "scope_list_devices",
        "列出系统中可用的串口。不指定 port 连接时用内置模拟器，无需硬件",
        "GET_INFO",
        "总是",
        ToolAnnotations::READ_ONLY_OPEN,
        NoArgs,
        |s, _p| s.list_devices()
    ),
    tool!(
        "scope_connect",
        "连接设备。不指定 port 时使用模拟器",
        "GET_INFO + GET_CONFIG",
        "总是",
        ToolAnnotations::CHANGES_DEVICE,
        ConnectArgs,
        |s, p| s.connect(&p)
    ),
    tool!(
        "scope_disconnect",
        "断开连接并停止流",
        "STOP",
        "总是",
        ToolAnnotations::CHANGES_DEVICE,
        NoArgs,
        |s, _p| s.disconnect()
    ),
    tool!(
        "scope_status",
        "设备当前状态与生效配置（含链路描述）。任何状态可调",
        "GET_STATUS",
        "总是",
        ToolAnnotations::READ_ONLY,
        NoArgs,
        |s, _p| s.status()
    ),
    tool!(
        "scope_configure",
        "一次性配置采样率/采集/触发/通道。返回实际生效值与警告",
        "SET_SAMPLE_RATE / SET_TRIGGER / SET_ACQ / SET_CHANNEL",
        "总是",
        ToolAnnotations::CHANGES_DEVICE,
        ConfigureArgs,
        |s, p| s.configure(&p)
    ),
    tool!(
        "scope_capture",
        "采集一次并等待触发完成。默认只返回统计量与 ≤256 点 minmax 预览，不含全量波形。⚠ 返回的 capture_id 只在**当前连接的本次会话**里有效 —— 之后引用它的调用必须接着发，不能另起一个进程",
        "ARM + EVENT_TRIGGER + READ_BUFFER",
        "总是",
        ToolAnnotations::CHANGES_DEVICE,
        CaptureArgs,
        |s, p| s.capture(&p)
    ),
    tool!(
        "scope_read_waveform",
        "按需分页拉取波形样点。硬顶 4096 点；要全量请用 scope_save_capture。capture_id 来自本次会话里的 scope_capture",
        "READ_BUFFER",
        "总是",
        ToolAnnotations::READ_ONLY,
        ReadWaveformArgs,
        |s, p| s.read_waveform(&p)
    ),
    tool!(
        "scope_measure",
        "对指定采集做测量：频率/峰峰值/均值/RMS/占空比/上升与下降时间/过冲/下冲/脉宽/跃变数，另附瞬态（振铃/阻尼/调节/RC/压摆/runt）与 TIE（σ/周期抖动）。每个量带 quality 判决；返回 schema/provenance/quality，单位与语义不变",
        "主机侧计算（精度最高）；流模式可走 MEASURE",
        "总是",
        ToolAnnotations::READ_ONLY,
        MeasureArgs,
        |s, p| s.measure(&p)
    ),
    tool!(
        "scope_decode",
        "统一解码入口：protocol=i2c 解 I2C 帧序列（需 2 通道），protocol=uart 解 UART 8N1 字节流（需 1 通道，真机单通道可用）。波特率必须显式确认（默认 115200）",
        "主机侧解码（基于 READ_BUFFER 数据）",
        "总是",
        ToolAnnotations::READ_ONLY,
        DecodeArgs,
        |s, p| s.decode(&p)
    ),
    tool!(
        "scope_i2c_decode",
        "把采集解码为 I2C 帧序列（START/地址/ACK/数据/STOP），返回帧列表与信号质量评估。等价于 scope_decode(protocol=i2c)，保留为兼容别名",
        "主机侧解码（基于 READ_BUFFER 数据）",
        "总是",
        ToolAnnotations::READ_ONLY,
        I2cDecodeArgs,
        |s, p| s.i2c_decode(&p)
    ),
    tool!(
        "scope_list_captures",
        "列出主机侧保存的最近采集，供引用而不必重抓",
        "无（本地 capture store）",
        "总是",
        ToolAnnotations::READ_ONLY,
        NoArgs,
        |s, _p| s.list_captures()
    ),
    tool!(
        "scope_save_capture",
        "把全量波形落盘为 CSV。全量数据不进 LLM 上下文",
        "无（本地写文件）",
        "总是",
        ToolAnnotations::WRITES_HOST,
        SaveCaptureArgs,
        |s, p| s.save_capture(&p)
    ),
    tool!(
        "scope_import_capture",
        "从 CSV 导入一次采集到本会话（不连设备）。首行 # rate_hz= 优先；两个电压参数都不给时按当前标定窗口换算，导入本项目导出文件可逐点还原。路径限工作目录内的 .csv。返回的 capture_id 可直接用于 scope_measure / scope_decode",
        "无（本地读文件 + capture store）",
        "总是",
        ToolAnnotations::WRITES_HOST,
        ImportCaptureArgs,
        |s, p| s.import_capture(&p)
    ),
    tool!(
        "scope_watch",
        "在一段时间里连续采集，返回采集次数与缺口统计，外加第一次采集（供后续测量/解码）。设备侧流模式尚未实现",
        "ARM(stream) + 分片推送 + STOP",
        "总是",
        ToolAnnotations::CHANGES_DEVICE,
        WatchArgs,
        |s, p| s.watch(&p)
    ),
    tool!(
        "scope_sim_set_scenario",
        "切换模拟器波形场景 / 换随机种子 / 注入故障（仅模拟器模式）",
        "无（模拟器内部）",
        "仅 transport=sim",
        ToolAnnotations::CHANGES_DEVICE,
        SimSetScenarioArgs,
        |s, p| s.sim_set_scenario(&p)
    ),
    tool!(
        "scope_debug_raw",
        "开发者逃生门：发任意命令码与 payload。默认不向 LLM 暴露",
        "任意（含 MEM_READ / MEM_WRITE）",
        "仅 SCOPE_MCP_DEBUG=1",
        ToolAnnotations::ESCAPE_HATCH,
        DebugRawArgs,
        |s, p| s.debug_raw(&p)
    ),
];

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    if args.iter().any(|a| a == "--list-tools") {
        print_tools();
        return Ok(());
    }
    if args.iter().any(|a| a == "--selftest") {
        return selftest();
    }

    // 默认：跑 stdio 上的 MCP server
    serve()
}

// ══════════════════════════════════════════════════════════════
// stdio 上的 MCP server
// ══════════════════════════════════════════════════════════════

/// 我们声明支持的最新协议版本。
///
/// `2025-03-26` 相对 `2024-11-05` 的关键增量是**工具注解**（`annotations`），
/// `tools/list` 里正在用它 —— 停在旧版本号上却发新字段，是不诚实的。
const PROTOCOL_VERSION: &str = "2025-03-26";

/// 还认得、并能按其语义服务的旧版本。客户端请求其中之一时按它回。
const SUPPORTED_PROTOCOL_VERSIONS: [&str; 2] = ["2025-03-26", "2024-11-05"];

/// `scope_debug_raw` 的注册条件：`SCOPE_MCP_DEBUG` **精确等于 `"1"`**。
///
/// 这里刻意不用 `is_ok()`（只判变量是否存在）：`scope_debug_raw` 是逃生门，
/// 能发任意命令码，含 `MEM_WRITE`。只判存在的话，`SCOPE_MCP_DEBUG=0`
/// 或空串这类「看起来是关掉」的误设，反而会把它暴露给模型 ——
/// 暴露条件必须是一次**显式**的 `=1`。
fn debug_tools_enabled(flag: Option<&str>) -> bool {
    flag == Some("1")
}

/// 跑 stdio 循环：一行一条 JSON-RPC 消息，读到 EOF 就退出。
fn serve() -> Result<()> {
    use std::io::{BufRead, Write};

    let flag = std::env::var("SCOPE_MCP_DEBUG").ok();
    let debug_tools = debug_tools_enabled(flag.as_deref());
    let mut session = session::Session::new();
    // 只在这里读磁盘 ——  必须保持不碰文件系统，
    // 否则每个 MCP 测试都会依赖开发机 %APPDATA% 里有什么。
    session.load_calibration_default();
    let stdin = std::io::stdin();
    let mut stdout = std::io::stdout();

    for line in stdin.lock().lines() {
        let line = line.context("读 stdin 失败")?;
        if line.trim().is_empty() {
            continue;
        }
        // IO 循环只做两件事：读一行、写一条。全部判断都在 `handle_line` 里 ——
        // 那边不碰 stdout，所以协议层的边界可以直接单测。
        if let Some(resp) = handle_line(&line, &mut session, debug_tools) {
            writeln!(stdout, "{}", serde_json::to_string(&resp)?)?;
            stdout.flush()?;
        }
    }
    Ok(())
}

/// 处理一条消息。
///
/// 返回 `None` 表示这是**通知**，按 JSON-RPC 规矩不回响应。
///
/// 不碰 stdout —— 协议层的边界（通知怎么判、错误码怎么选、校验的次序）
/// 全在这里，可以单测。之前这些逻辑和 IO 混在一个循环里，只能靠手工
/// 灌 stdin 验证。
fn handle_line(
    line: &str,
    session: &mut session::Session,
    debug_tools: bool,
) -> Option<jsonrpc::Response> {
    use jsonrpc::{code, Response};

    // 先宽松解析成 Value 再收紧成 Request —— 这样「**是**合法 JSON 但
    // 不**是**合法请求对象」能报 -32600 并**回显 id**，而不是一律
    // -32700 且把 id 丢成 null。客户端靠 id 匹配响应，丢了它只能干等。
    let raw: Value = match serde_json::from_str(line) {
        Ok(v) => v,
        Err(e) => {
            // 这一步才是真的解析失败：连 id 都无从得知，按规范用 null
            return Some(Response::err(
                Value::Null,
                code::PARSE_ERROR,
                format!("JSON 解析失败: {e}"),
            ));
        }
    };
    let req: jsonrpc::Request = match serde_json::from_value(raw.clone()) {
        Ok(r) => r,
        Err(e) => {
            let id = raw.get("id").cloned().unwrap_or(Value::Null);
            return Some(Response::err(
                id,
                code::INVALID_REQUEST,
                format!("不是合法的 JSON-RPC 请求对象: {e}"),
            ));
        }
    };

    // --- 请求级校验：先确定「该不该回」，再确定「回什么」 ---
    //
    // 错误响应也要尽量带上 id（能拿到就带），客户端靠 id 匹配响应。
    let id_for_err = || raw.get("id").cloned().unwrap_or(Value::Null);

    // `jsonrpc` 是必填字段。之前「缺失或空串」被当作合法请求照常处理 ——
    // 一条根本不是 JSON-RPC 的消息也能拿到成功响应。
    if req.jsonrpc != "2.0" {
        return Some(Response::err(
            id_for_err(),
            code::INVALID_REQUEST,
            format!("jsonrpc 必须是 \"2.0\"，收到 {:?}", req.jsonrpc),
        ));
    }

    // 通知的判据是「原始 JSON 里**有没有 id 这个键**」，而不是
    // 「id 解析出来是不是 None」。`{"id":null}` 是合法请求（id 为 null），
    // 必须回响应 —— 之前 serde 把显式 null 折叠成 None，这类请求被当成
    // 通知静默丢弃，客户端永远等不到回复。
    //
    // `?` 在这里就是「没有 id 键 → 直接返回 None（通知，不回）」。
    raw.get("id")?;
    let id = req.id.clone().unwrap_or(Value::Null);

    // `params` 若给，必须是对象（命名参数）。给了数组会让后面的
    // `params.get("name")` 静默取到 None，冒出一个与实际不符的「缺少 name」。
    let params = req.params.clone();
    if !params.is_null() && !params.is_object() {
        return Some(Response::err(id, code::INVALID_PARAMS, "params 必须是对象"));
    }

    match req.method.as_str() {
        "initialize" => {
            // 客户端声明它支持的版本。**认得就按它的回** —— 回一个比客户端新的版本，
            // 会让它拿不认识的语义去解释后面的响应。不认得才回我们最新的。
            let asked = params.get("protocolVersion").and_then(|v| v.as_str());
            let agreed = match asked {
                Some(v) if SUPPORTED_PROTOCOL_VERSIONS.contains(&v) => v,
                _ => PROTOCOL_VERSION,
            };
            Some(Response::ok(
                id,
                json!({
                    "protocolVersion": agreed,
                    "capabilities": { "tools": {} },
                    "serverInfo": {
                        "name": "scope-mcp",
                        "version": scope_core::VERSION,
                    },
                }),
            ))
        }

        "ping" => Some(Response::ok(id, json!({}))),

        "tools/list" => Some(Response::ok(id, json!({ "tools": tool_list(debug_tools) }))),

        "tools/call" => {
            let name = params.get("name").and_then(|v| v.as_str()).unwrap_or("");
            let args = match params.get("arguments") {
                None => json!({}),
                Some(a) if a.is_object() => a.clone(),
                Some(_) => {
                    return Some(Response::err(
                        id,
                        code::INVALID_PARAMS,
                        "arguments 必须是对象",
                    ))
                }
            };
            if name.is_empty() {
                Some(Response::err(
                    id,
                    code::INVALID_PARAMS,
                    "tools/call 缺少 name",
                ))
            } else if !tool_enabled(name, debug_tools) {
                Some(Response::err(
                    id,
                    code::INVALID_PARAMS,
                    format!("工具 {name} 在当前会话里不可用（未注册）"),
                ))
            } else if !TOOLS.iter().any(|t| t.name == name) {
                // 未知工具名与「未注册」走同一类错误 —— 两条路径报不同
                // 种类的错会让客户端难以统一处理。
                Some(Response::err(
                    id,
                    code::INVALID_PARAMS,
                    format!("未知工具 {name}"),
                ))
            } else {
                Some(Response::ok(id, dispatch(session, name, &args)))
            }
        }

        other => Some(Response::err(
            id,
            code::METHOD_NOT_FOUND,
            format!("未知方法 {other}"),
        )),
    }
}

/// 这个工具在 `tools/list` 里出不出现。
///
/// **只按启动时定死的东西过滤**，不看连接状态。理由：MCP 客户端一般在启动时
/// `tools/list` 一次就缓存了，之后连接模拟器并不会让它重新拉列表 ——
/// 按 `sim` 过滤的话，`scope_sim_set_scenario` 在连上模拟器后依然看不见。
/// 所以它总是列出，真连了串口再调用时给一句明确的错误。
///
/// `scope_debug_raw` 是例外：它由环境变量决定，进程启动后就定死了，
/// 不会中途变化，过滤掉是安全的。
fn tool_enabled(name: &str, debug: bool) -> bool {
    match name {
        "scope_debug_raw" => debug,
        _ => true,
    }
}

/// 工具清单 —— 按会话状态过滤后返回。
///
/// `inputSchema` 现在是**现场从参数类型生成的**，不再是一段手写字符串 ——
/// 所以不存在「字符串写坏了，工具的参数说明整个消失」这种事。
fn tool_list(debug: bool) -> Vec<Value> {
    TOOLS
        .iter()
        .filter(|t| tool_enabled(t.name, debug))
        .map(|t| {
            json!({
                "name": t.name,
                "description": t.description,
                "inputSchema": (t.schema)(),
                // 规范的缺省值是刻意悲观的（不写 = 可能有害），所以每个工具都显式给，
                // 让客户端能把 list_devices / status 这类纯读的放行。
                "annotations": serde_json::to_value(t.annotations)
                    .expect("注解只有 bool，必定可序列化"),
            })
        })
        .collect()
}

/// 把一次 `tools/call` 派发到具体实现，并把结果包成 MCP 的 content 形状。
///
/// 查表和调用**是同一张表**（[`TOOLS`]）—— 从前这里是一个 14 分支的 match，
/// 与上面那张表各写一遍工具名，改名时只改一处就能让两个工具错位。
fn dispatch(session: &mut session::Session, name: &str, args: &Value) -> Value {
    use jsonrpc::{tool_error, tool_result};

    // `handle_line` 已经先查过一遍并给出 JSON-RPC 错误码，正常走不到这里；
    // 留着是为了让这个函数自己也是完整的。
    let Some(spec) = TOOLS.iter().find(|t| t.name == name) else {
        return tool_error(format!("未知工具 {name}"), None);
    };

    match (spec.handler)(session, args) {
        Ok(v) => tool_result(&v, false),
        Err(e) => tool_error(e.message, e.hint),
    }
}

fn print_tools() {
    println!("# scope-mcp 工具清单（{} 个）", TOOLS.len());
    println!();
    println!("三层 token 防护：");
    println!("  1. capture / watch 默认只回统计量 + ≤256 点 minmax 预览");
    println!("  2. read_waveform 分页、默认 512 点、硬顶 4096、超出拒绝");
    println!("  3. 全量数据只进 capture store 和磁盘，永不进上下文");
    println!();

    for t in TOOLS {
        println!("── {}", t.name);
        println!("   说明   : {}", t.description);
        println!("   协议   : {}", t.protocol_cmds);
        println!("   注册条件: {}", t.condition);
        // 给人工核对用：一眼看出哪些工具是只读的（客户端会据此免确认）
        println!(
            "   注解   : 只读={} 破坏性={} 幂等={} 开放世界={}",
            t.annotations.read_only_hint,
            t.annotations.destructive_hint,
            t.annotations.idempotent_hint,
            t.annotations.open_world_hint
        );
        // 美化输出：这是给人核对用的，紧凑 JSON 读起来太费劲
        println!(
            "   schema : {}",
            serde_json::to_string_pretty(&(t.schema)()).unwrap_or_else(|_| "(无法序列化)".into())
        );
        println!();
    }
}

/// 把工具的 `ToolError` 转成 `anyhow::Error`，供自检用。
///
/// `selftest` 走的是 CLI 风格的 `anyhow::Result`，而工具实现返回
/// `Result<Value, ToolError>` —— 直接 `?` 会要求 `ToolError: StdError`。
/// 这里把「怎么办」一并带上，自检失败时仍然能照着改。
fn tool_err(e: session::ToolError) -> anyhow::Error {
    match e.hint {
        Some(h) => anyhow::anyhow!("{}（{h}）", e.message),
        None => anyhow::anyhow!("{}", e.message),
    }
}

/// 跑一遍完整链路，确认命令层真的可用。
fn selftest() -> Result<()> {
    println!("== scope-mcp 自检 ==");
    println!();

    let mut bus = CommandBus::new(SimDevice::new(Scenario::I2c100k));
    let info = bus.connect()?;
    println!(
        "[1/7] 连接成功: {} 通道, 上限 {} Hz",
        info.ch_count, info.rate_max_hz
    );

    let actual = bus.set_sample_rate(857_143)?;
    println!("[2/7] 采样率: 请求 857143 → 实际 {actual} Hz（已量化）");

    bus.set_trigger(1, 0, 0, 2048, 1024, 1000)?;
    bus.set_acq(0, 1024, 0, 1)?;
    println!("[3/7] 触发与采集参数已配置");

    bus.arm()?;
    let ev = bus
        .wait_trigger(std::time::Duration::from_millis(2000))?
        .ok_or_else(|| anyhow::anyhow!("模拟器未产生触发事件"))?;
    let capture_id = u16::from_le_bytes([ev[0], ev[1]]);
    println!("[4/7] 触发完成: capture_id={capture_id}");

    let mut store = CaptureStore::default();
    let mut cap = scope_core::Capture::new(capture_id, actual, 2, 1024);

    for ch in 0..2usize {
        let payload = bus.read_buffer(capture_id, 0, 1024, 0, ch as u8)?;
        let hdr = scope_proto::ChunkHeader::decode(&payload).unwrap();
        for pair in payload[scope_proto::CHUNK_HEADER_LEN..].chunks_exact(2) {
            cap.channels[ch].push(u16::from_le_bytes([pair[0], pair[1]]));
        }
        println!(
            "      CH{}: {} 点, format={}, decimation={}",
            ch + 1,
            hdr.count,
            hdr.format,
            hdr.decimation
        );
    }
    store.push(cap);

    let c = store.latest().unwrap();
    for ch in 0..2usize {
        if let Some(s) = c.summary(ch) {
            println!(
                "      CH{} 摘要: pp={} LSB, 上升沿 {}",
                ch + 1,
                s.pp_lsb,
                s.rising_edges
            );
        }
    }

    let preview = c.preview(0, 32).unwrap();
    println!(
        "[5/7] minmax 预览: {} 个桶 (每桶 {} 样点) — 这就是 Agent 默认能看到的粒度",
        preview.y_min.len(),
        preview.bucket
    );

    // ── UART 解码（单通道协议 —— 真机可用的那条解码路径）──
    let mut uart_bus = CommandBus::new(SimDevice::new(Scenario::Uart115k));
    uart_bus.connect()?;
    let uart_rate = uart_bus.set_sample_rate(857_143)?;
    // UART 空闲是高、起始位是低 → auto 触发即可（起始位下降沿本身就能触发）
    uart_bus.set_trigger(0, 0, 0, 2048, 0, 1000)?;
    uart_bus.set_acq(0, 1024, 0, 1)?;
    uart_bus.arm()?;
    let ev = uart_bus
        .wait_trigger(std::time::Duration::from_millis(2000))?
        .ok_or_else(|| anyhow::anyhow!("UART 场景未产生触发事件"))?;
    let uart_id = u16::from_le_bytes([ev[0], ev[1]]);

    let payload = uart_bus.read_buffer(uart_id, 0, 1024, 0, 0)?;
    let mut uart_cap = scope_core::Capture::new(uart_id, uart_rate, 1, 1024);
    for pair in payload[scope_proto::CHUNK_HEADER_LEN..].chunks_exact(2) {
        uart_cap.channels[0].push(u16::from_le_bytes([pair[0], pair[1]]));
    }
    let req = scope_core::decoder::DecodeRequest {
        protocol: scope_core::decoder::Protocol::Uart,
        channels: vec![0],
        params: scope_core::decoder::DecodeParams::Uart(scope_core::decoder::UartConfig::default()),
    };
    let decoded = scope_core::decoder::decode(&uart_cap, &req)?;
    let bytes = decoded.uart_bytes().unwrap_or_default();
    let head: Vec<String> = bytes.iter().take(3).map(|b| format!("{b:02X}")).collect();
    println!(
        "[6/7] UART 解码: {} 帧（{} 帧 ok），首字节 {}，每比特 {:.2} 样点",
        decoded.frame_count(),
        decoded.quality.frame_ok,
        head.join(" "),
        decoded.quality.sample_margin.unwrap_or(0.0),
    );
    if decoded.frame_count() == 0 {
        anyhow::bail!("UART 解码一帧都没解出 —— 解码链路坏了");
    }

    // ── CSV 导入回环（不连设备的那条路径）──
    // 用刚采到的采集写出 CSV，再另起一个 Session 导入：验证的正是
    // 「导出 → 导入」这条跨进程路径，而不是复用内存里的对象。
    let scales = scope_core::ScaleSet::uncalibrated(c.channels.len());
    let csv = c.to_csv(scales.as_slice());
    // 导入工具只认工作目录内的 .csv（见 `session::csv_in_cwd`），
    // 所以临时文件必须放在这里，而不是系统临时目录。
    let path = std::env::current_dir()?.join("scope-mcp-selftest-import.csv");
    std::fs::write(&path, &csv)?;
    let mut imp = session::Session::new();
    let v = imp
        .import_capture(&ImportCaptureArgs {
            path: path.display().to_string(),
            rate_hz: None,
            full_scale_v: None,
            offset_v: None,
            raw: None,
            normalize_t0: None,
        })
        .map_err(tool_err)?;
    let _ = std::fs::remove_file(&path);
    let imported_id = v["capture_id"].as_u64().unwrap_or(0);
    let m = imp
        .measure(&MeasureArgs {
            capture_id: imported_id,
            metrics: None,
            channel: None,
        })
        .map_err(tool_err)?;
    println!(
        "[7/7] CSV 导入: capture_id={imported_id}, {} Hz, {} 点, vpp={:.3} V",
        v["rate_hz"],
        v["samples"],
        m["measurements"]["ch0"]["vpp_v"].as_f64().unwrap_or(0.0),
    );
    if v["rate_hz"].as_u64() != Some(actual as u64) {
        anyhow::bail!("导入后的采样率与导出前不一致：{v}");
    }
    if v["samples"].as_u64() != Some(c.len() as u64) {
        anyhow::bail!("导入后的点数与导出前不一致：{v}");
    }
    if m["schema"] != "analysis/v1" {
        anyhow::bail!("导入的采集上做测量没走到统一分析对象：{m}");
    }
    println!();
    println!("自检通过。命令层与模拟器均可用。");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use jsonrpc::code;

    /// 走一遍协议层，返回 (响应, 是不是通知)。
    fn call(line: &str) -> (Option<jsonrpc::Response>, bool) {
        let mut s = session::Session::new();
        let r = handle_line(line, &mut s, false);
        let is_notification = r.is_none();
        (r, is_notification)
    }

    fn err_code(line: &str) -> Option<i32> {
        call(line).0?.error.map(|e| e.code)
    }

    #[test]
    fn unparseable_json_is_a_parse_error_with_a_null_id() {
        let (r, _) = call("这不是 JSON");
        let r = r.expect("解析失败也要回响应");
        assert_eq!(r.error.unwrap().code, code::PARSE_ERROR);
        assert_eq!(r.id, Value::Null, "连 id 都无从得知，按规范用 null");
    }

    #[test]
    fn valid_json_that_is_not_a_request_echoes_the_id() {
        // 回归：曾经一律报 -32700 并把 id 丢成 null，客户端靠 id 匹配响应，
        // 丢了它只能干等。
        let (r, _) = call(r#"{"jsonrpc":"2.0","id":42}"#);
        let r = r.unwrap();
        assert_eq!(r.error.unwrap().code, code::INVALID_REQUEST);
        assert_eq!(r.id, json!(42), "能拿到 id 就要回显");
    }

    #[test]
    fn jsonrpc_field_is_required() {
        // 回归：缺 jsonrpc 的裸消息曾经被当作合法请求照常处理并回成功
        assert_eq!(
            err_code(r#"{"id":1,"method":"ping"}"#),
            Some(code::INVALID_REQUEST)
        );
        assert_eq!(
            err_code(r#"{"jsonrpc":"1.0","id":1,"method":"ping"}"#),
            Some(code::INVALID_REQUEST)
        );
    }

    #[test]
    fn explicit_null_id_is_a_request_not_a_notification() {
        // `{"id":null}` 是合法请求（id 为 null），必须回响应。
        // 回归：serde 把显式 null 折叠成 None，这类请求被当成通知静默丢弃。
        let (r, is_notification) = call(r#"{"jsonrpc":"2.0","id":null,"method":"ping"}"#);
        assert!(!is_notification, "显式 null 的 id 仍要回");
        assert!(r.unwrap().error.is_none());
    }

    #[test]
    fn a_request_without_the_id_key_is_a_notification() {
        let (_, is_notification) =
            call(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#);
        assert!(is_notification, "没有 id 键才是通知");
    }

    #[test]
    fn container_types_are_validated() {
        // params 给数组 → 否则后面 params.get("name") 静默取到 None，
        // 冒出一个与实际不符的「缺少 name」
        assert_eq!(
            err_code(r#"{"jsonrpc":"2.0","id":1,"method":"ping","params":[1,2]}"#),
            Some(code::INVALID_PARAMS)
        );
        assert_eq!(
            err_code(
                r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"scope_status","arguments":[1]}}"#
            ),
            Some(code::INVALID_PARAMS)
        );
    }

    #[test]
    fn unknown_method_and_unknown_tool_are_distinct_but_consistent() {
        assert_eq!(
            err_code(r#"{"jsonrpc":"2.0","id":1,"method":"没这个方法"}"#),
            Some(code::METHOD_NOT_FOUND)
        );
        assert_eq!(
            err_code(
                r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"没这个工具"}}"#
            ),
            Some(code::INVALID_PARAMS)
        );
        // 未注册的工具与未知工具同一类错误 —— 两条路径报不同种类的错
        // 会让客户端难以统一处理
        assert_eq!(
            err_code(
                r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"scope_debug_raw"}}"#
            ),
            Some(code::INVALID_PARAMS)
        );
    }

    #[test]
    fn initialize_reports_the_protocol_and_server() {
        let (r, _) = call(r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}"#);
        let v = r.unwrap().result.unwrap();
        assert_eq!(v["protocolVersion"], PROTOCOL_VERSION);
        assert_eq!(v["serverInfo"]["name"], "scope-mcp");
        assert!(v["capabilities"]["tools"].is_object());
    }

    #[test]
    fn tools_list_matches_the_tools_constant() {
        let (r, _) = call(r#"{"jsonrpc":"2.0","id":1,"method":"tools/list"}"#);
        let tools = r.unwrap().result.unwrap()["tools"].clone();
        let arr = tools.as_array().unwrap();
        // 不带 SCOPE_MCP_DEBUG 时少一个 scope_debug_raw
        assert_eq!(arr.len(), TOOLS.len() - 1);
        // 每个工具都要有 name / description / inputSchema / annotations
        for t in arr {
            assert!(t["name"].is_string(), "缺 name: {t}");
            assert!(t["description"].is_string(), "缺 description: {t}");
            assert!(t["inputSchema"].is_object(), "schema 不是对象: {t}");
            // 注解缺省值是刻意悲观的 —— 漏发一个，客户端就把那个工具当成可能有害的。
            let a = &t["annotations"];
            assert!(a.is_object(), "缺 annotations: {t}");
            for k in [
                "readOnlyHint",
                "destructiveHint",
                "idempotentHint",
                "openWorldHint",
            ] {
                assert!(a[k].is_boolean(), "annotations 缺 {k} 或不是布尔: {t}");
            }
        }
    }

    /// 钉住注解的**语义**，不只是「字段在」。
    ///
    /// 这两条是注解存在的全部理由：只读的能被客户端免确认放行，
    /// 逃生门不能被当成普通工具。把它们写反，等于给了模型一把没上锁的枪。
    #[test]
    fn annotations_mark_read_only_tools_and_the_escape_hatch() {
        let find = |name: &str| {
            TOOLS
                .iter()
                .find(|t| t.name == name)
                .unwrap_or_else(|| panic!("没有这个工具：{name}"))
                .annotations
        };

        // 纯读的：只读、非破坏
        for name in [
            "scope_status",
            "scope_read_waveform",
            "scope_measure",
            "scope_i2c_decode",
            "scope_list_captures",
            "scope_list_devices",
        ] {
            let a = find(name);
            assert!(a.read_only_hint, "{name} 应声明为只读");
            assert!(!a.destructive_hint, "{name} 不该声明为破坏性");
        }

        // 改设备状态的：不能声明只读（否则客户端会免确认，而它真的在动设备）
        for name in [
            "scope_connect",
            "scope_disconnect",
            "scope_configure",
            "scope_capture",
            "scope_watch",
            "scope_sim_set_scenario",
        ] {
            assert!(
                !find(name).read_only_hint,
                "{name} 改了设备状态，不能声明只读"
            );
        }

        // 逃生门：唯一一个破坏性 + 非只读的
        let esc = find("scope_debug_raw");
        assert!(
            esc.destructive_hint && !esc.read_only_hint,
            "逃生门必须声明为破坏性且非只读"
        );
        let destructive: Vec<&str> = TOOLS
            .iter()
            .filter(|t| t.annotations.destructive_hint)
            .map(|t| t.name)
            .collect();
        assert_eq!(
            destructive,
            vec!["scope_debug_raw"],
            "只有逃生门该声明破坏性；多标了会让正常工具也走确认"
        );
    }

    /// 回显客户端认识的版本，不认识才回我们最新的。
    ///
    /// 回一个比客户端更新的版本，它会拿不认识的语义解释后面的响应 ——
    /// 而这次新增的 `annotations` 正是版本相关的字段。
    #[test]
    fn initialize_echoes_a_known_protocol_version() {
        let ask = |v: &str| {
            let (r, _) = call(&format!(
                r#"{{"jsonrpc":"2.0","id":1,"method":"initialize","params":{{"protocolVersion":"{v}"}}}}"#
            ));
            r.unwrap().result.unwrap()["protocolVersion"]
                .as_str()
                .unwrap()
                .to_string()
        };
        assert_eq!(ask("2024-11-05"), "2024-11-05", "认识的旧版本要照回");
        assert_eq!(ask("2025-03-26"), "2025-03-26");
        assert_eq!(ask("1999-01-01"), PROTOCOL_VERSION, "不认识的回最新");
    }

    #[test]
    fn tool_names_are_unique() {
        let mut seen = std::collections::HashSet::new();
        for t in TOOLS {
            assert!(seen.insert(t.name), "工具名重复：{}", t.name);
        }
    }

    /// 钉住「显式 `=1` 才暴露逃生门」。
    ///
    /// 这条测试存在的理由：实现原本是 `is_ok()`（判存在），于是
    /// `SCOPE_MCP_DEBUG=0` 和空串都会把 `scope_debug_raw` 注册出去 ——
    /// 与 `--list-tools` 自己打印的「仅 SCOPE_MCP_DEBUG=1」相反。
    /// 把 `flag == Some("1")` 改回 `flag.is_some()`，前两条断言立刻变红。
    #[test]
    fn debug_tools_require_an_exact_one() {
        assert!(!debug_tools_enabled(None), "未设置时不得注册");
        assert!(!debug_tools_enabled(Some("0")), "=0 是关，不是开");
        assert!(!debug_tools_enabled(Some("")), "空串不得注册");
        assert!(
            !debug_tools_enabled(Some("true")),
            "只认 1，不认别的真值写法"
        );
        assert!(debug_tools_enabled(Some("1")));
    }

    /// 走一遍协议层调用工具，返回**工具自己的响应体**（已解掉 MCP 的
    /// content 信封）。
    fn tool_call(session: &mut session::Session, name: &str, args: Value) -> Value {
        let line = json!({
            "jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": { "name": name, "arguments": args },
        })
        .to_string();
        let r = handle_line(&line, session, true).expect("tools/call 不是通知");
        let r = r.result.expect("tools/call 不该产生协议级错误");
        let text = r["content"][0]["text"].as_str().expect("应当是文本块");
        serde_json::from_str(text).expect("工具响应应当是 JSON")
    }

    /// 这条响应是不是「参数没通过解析」，而不是工具自身业务上的失败。
    ///
    /// [`tool_call`] 已经剥掉了 MCP 的信封，所以这里判的是**内层**形状：
    /// 工具失败时内层是 `{"error": ..., "hint": ...}`。带 `error` 键的
    /// 内层对象只有失败路径会产生。
    ///
    /// 判据是 [`parse_then`] 给参数错误加的那句前缀。
    fn is_param_error(r: &Value) -> bool {
        r.get("error")
            .and_then(|v| v.as_str())
            .map(|s| s.starts_with("参数不合法"))
            .unwrap_or(false)
    }

    /// 按 schema 造一个合法值 —— 只覆盖本项目用到的类型。
    ///
    /// 解析部分一律走 [`resolve_prop`]，不另起炉灶：第一版这里自己读
    /// `s["type"].as_str()`，于是对 `"type":["integer","null"]` 的字段
    /// 造出了 `null`，对一个**必填**字段也一样 —— 造出来的值根本不合 schema。
    fn sample_for(s: &Value, root: &Value) -> Value {
        let (real, _) = resolve_prop(s, root);

        // 取值优先：`enum` 或派生枚举的 `oneOf: [{"const": ...}]`
        if let Some(first) = real["enum"].as_array().and_then(|e| e.first()) {
            return first.clone();
        }
        if let Some(c) = real["oneOf"]
            .as_array()
            .and_then(|o| o.first())
            .and_then(|arm| arm.get("const"))
        {
            return c.clone();
        }

        // `type` 可能是数组（可选字段），挑一个非 null 的来造值
        match type_names(real).into_iter().find(|t| *t != "null") {
            // 空串而不是 "x"：有些字段（payload_hex、port）用空串会走到
            // 「业务上的失败」，而 "x" 可能撞上别的错误分支。
            Some("string") => json!(""),
            Some("integer") => json!(1),
            Some("number") => json!(0.5),
            Some("boolean") => json!(true),
            Some("array") => json!([]),
            // 嵌套对象：把它自己的属性也填满 —— 这样 `configure.trigger`、
            // `configure.channel`、`sim_set_scenario.inject` 里面的字段
            // 也在覆盖范围内，而那些正是嵌套声明最容易和实现分家的地方。
            Some("object") => match real["properties"].as_object() {
                Some(p) => Value::Object(
                    p.iter()
                        .map(|(k, v)| (k.clone(), sample_for(v, root)))
                        .collect(),
                ),
                None => json!({}),
            },
            _ => json!(null),
        }
    }

    /// **本文件最重要的那条测试。**
    ///
    /// 它证明：`tools/list` 里为每个工具声明的参数，与 `tools/call` 真正
    /// 解析的参数，是同一份。做法是两头夹：
    ///
    /// 1. schema 里出现的每个属性，照着造一个合法值发过去 —— 不许出现
    ///    「参数不合法」。TOOLS 的 schema 与 dispatch 用的类型一旦分家，
    ///    这里就会炸。
    /// 2. 反过来，发一个 schema 里没声明的属性 —— **必须**被拒绝。
    ///
    /// 两条合起来即 `properties(schema) == fields(解析目标类型)`。
    ///
    /// 之所以需要这条，是因为上一轮对抗性验证找出的 8 类 P1 里有 4 类是
    /// 「schema 声明了、实现忽略」：`scope_capture.mode`、
    /// `scope_read_waveform.format`、`scope_sim_set_scenario` 的 `seed` 与
    /// `inject`，全都是**传了不报错、静默走默认值**。这类 bug 顺路径测试
    /// 永远发现不了，因为测试用的字段名是对的。
    #[test]
    fn every_tool_accepts_exactly_the_parameters_its_schema_declares() {
        for t in TOOLS {
            let schema = (t.schema)();
            let empty = serde_json::Map::new();
            let props = schema["properties"].as_object().unwrap_or(&empty);

            // (1) 把 schema 声明的参数全填上，必须被接受
            let full: Value = Value::Object(
                props
                    .iter()
                    .map(|(k, v)| (k.clone(), sample_for(v, &schema)))
                    .collect(),
            );
            let mut s = session::Session::new();
            let got = tool_call(&mut s, t.name, full.clone());
            assert!(
                !is_param_error(&got),
                "{} 的 schema 声明了这些参数 {full}，但实现不认：{got}",
                t.name
            );

            // (2) schema 没声明的参数，必须被拒绝 —— 否则「字段名拼错」
            //     又会退化成静默走默认值
            let mut s = session::Session::new();
            let mut bogus = full.clone();
            bogus["__nope__"] = json!(1);
            let got = tool_call(&mut s, t.name, bogus);
            assert!(
                is_param_error(&got),
                "{} 接受了 schema 里没声明的参数（字段名拼错会被静默忽略）：{got}",
                t.name
            );
            assert!(
                got["error"].as_str().unwrap_or("").contains("__nope__"),
                "{} 的错误里要点名是哪个参数不对：{got}",
                t.name
            );
        }
    }

    #[test]
    fn every_tool_schema_is_a_closed_object() {
        // MCP 要求 inputSchema 是对象；`additionalProperties: false` 是
        // `deny_unknown_fields` 在 schema 上的投影 —— 有了它，客户端在
        // 发出去之前就知道多余字段会被拒，而不是发了才撞墙。
        for t in TOOLS {
            let schema = (t.schema)();
            assert_eq!(schema["type"], json!("object"), "{}", t.name);
            assert_eq!(
                schema["additionalProperties"],
                json!(false),
                "{} 的 schema 没有声明 additionalProperties: false",
                t.name
            );
        }
    }

    #[test]
    fn section_names_match_the_serialized_field_names() {
        // 字段名是 Agent 唯一能用的接口。抽查几条：schema 里的名字必须
        // 就是实现读的那个名字。
        let connect = (TOOLS
            .iter()
            .find(|t| t.name == "scope_connect")
            .expect("有 scope_connect")
            .schema)();
        let props = connect["properties"].as_object().unwrap();
        for k in ["port", "baud", "transport", "sim_scenario"] {
            assert!(props.contains_key(k), "scope_connect 的 schema 缺 {k}");
        }
        // 报告里曾经写过 `scl_chanel` 这种拼错的名字，靠人眼是看不出来的
        assert!(
            !props.contains_key("sim_scenerio"),
            "schema 里出现了拼错的字段名"
        );
    }

    #[test]
    fn bad_parameters_come_back_with_a_field_list_and_a_hint() {
        // 错误要给「怎么办」—— 这里给的是「本工具接受哪些参数」，
        // 直接从参数类型自己的 schema 里读出来。
        let mut s = session::Session::new();
        let got = tool_call(
            &mut s,
            "scope_i2c_decode",
            json!({ "capture_id": 1, "scl_chanel": 0 }),
        );
        assert!(is_param_error(&got), "实测 {got}");
        let hint = got["hint"].as_str().unwrap_or("");
        assert!(
            hint.contains("capture_id"),
            "提示要列出可用参数，实测 {hint}"
        );
        assert!(
            hint.contains("scl_channel"),
            "提示要列出可用参数，实测 {hint}"
        );
        assert!(hint.contains("必填"), "提示要标出哪些是必填的，实测 {hint}");
    }

    #[test]
    fn bad_transport_is_rejected_with_the_valid_values() {
        // `TransportArg` 只认 schema 里那两个名字 —— 别名（simulator /
        // uart / port）留给给人用的 CLI，MCP 这层要对齐 schema。
        let mut s = session::Session::new();
        let got = tool_call(&mut s, "scope_connect", json!({ "transport": "bogus" }));
        assert!(is_param_error(&got), "实测 {got}");
        let msg = got["error"].as_str().unwrap_or("");
        assert!(msg.contains("bogus"), "实测 {msg}");
        assert!(msg.contains("serial"), "错误里要列出合法取值，实测 {msg}");
        assert!(msg.contains("sim"), "错误里要列出合法取值，实测 {msg}");
    }

    #[test]
    fn missing_required_arguments_name_the_field() {
        let mut s = session::Session::new();
        let got = tool_call(&mut s, "scope_save_capture", json!({ "capture_id": 1 }));
        assert!(is_param_error(&got), "实测 {got}");
        assert!(
            got["error"].as_str().unwrap_or("").contains("path"),
            "实测 {got}"
        );
    }

    #[test]
    fn param_type_labels_handle_both_schema_shapes() {
        // 回归：只按字符串读 `type` 时，**所有可选参数**都会显示成「任意」——
        // 因为 schemars 为 `Option<T>` 生成的是 `"type": ["integer","null"]`。
        // 一条把每个可选参数都写成「任意」的提示，等于没有提示。
        let schema_of = |tool: &str| {
            (TOOLS
                .iter()
                .find(|t| t.name == tool)
                .unwrap_or_else(|| panic!("有 {tool}"))
                .schema)()
        };

        let label = |root: &Value, field: &str| type_label(&root["properties"][field], root);

        let decode = schema_of("scope_i2c_decode");
        // 可选、无 enum → 类型名 + ?   （`"type": ["integer","null"]` 那种）
        assert_eq!(label(&decode, "debounce_ns"), "整数?");
        // 必填 → 不加问号
        assert_eq!(label(&decode, "capture_id"), "整数");

        // 可选**自定义枚举**走的是 `anyOf: [$ref, null]`，
        // 而枚举本身是 `oneOf: [{const}, ...]` —— 三层都要解对
        let connect = schema_of("scope_connect");
        assert_eq!(label(&connect, "transport"), "serial|sim?");
        assert_eq!(
            label(&connect, "sim_scenario"),
            "sine_1k_3v3|square_50k|pulse_glitch|noise|dc|am|i2c_100k|i2c_400k|i2c_nack|\
             pwm_1k_25|pwm_1k_75|pwm_1k_5|step_ring|rc_charge|uart_115k|\
             slow_edge_noisy|ring_alias?"
        );

        // 可选数组、可选字符串
        assert_eq!(label(&schema_of("scope_measure"), "metrics"), "数组?");
        assert_eq!(label(&connect, "port"), "字符串?");
    }

    #[test]
    fn the_parameter_hint_names_every_field_with_its_type_and_requiredness() {
        let mut s = session::Session::new();
        let got = tool_call(&mut s, "scope_connect", json!({ "nope": 1 }));
        let hint = got["hint"].as_str().unwrap_or("").to_string();
        for want in ["port:字符串?", "baud:整数?", "transport:serial|sim?"] {
            assert!(hint.contains(want), "提示里应当有 {want}，实测 {hint}");
        }
        // 没有必填项时不该乱标
        assert!(!hint.contains("必填"), "实测 {hint}");
    }
}
