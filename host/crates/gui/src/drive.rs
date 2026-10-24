//! 「让 AI 自己配置并采集」—— GUI 把设备让给 `scope-mcp` 子进程，自己当 MCP 客户端。
//!
//! # 为什么是子进程而不是进程内
//!
//! 真机串口**独占**（`docs/07-dev-env.md:232`：GUI 占着 COM7 时 `scope-mcp`
//! 打不开同一个口），模拟器则每个进程各建一台（`Transport::sim()`）。
//! 所以同一时刻只能有一个设备持有者 —— 要么 GUI，要么子进程。
//!
//! 选子进程，是因为它**复用整套已测过的 MCP 工具层**（15 个工具、schema、
//! 越界校验、三层 token 防护）。进程内重做要先把这个 bin-only 的 crate 拆成 lib，
//! 而且很容易长出第二份策略 —— 那正是本项目反复打的病。
//!
//! # 这个模块管什么
//!
//! 三件事，按依赖顺序：
//!
//! 1. **找 `scope-mcp` 可执行文件** —— 按 `current_exe()` 同级找，
//!    不按 cwd（GUI 的 cwd 与仓库根不同）
//! 2. **stdio JSON-RPC 客户端** —— 起进程、发请求、读响应，外加把所有
//!    「看似可运行但会卡死」的地方堵上（见 [`McpProcess`] 的说明）
//! 3. **工具循环** —— 移植自 `tools/agent_demo/agent.py`，那是一个已验证可用的
//!    参考实现；但它有几处做法**不能照抄**，逐条写在 [`run_session`] 上面
//!
//! # 显示由 GUI 自己镜像，不用 AI 的读数
//!
//! AI 可能只看统计量**根本不读样点**（那屏幕就是空白的）；真读时 4096 点的
//! JSON 又会被 [`RESULT_CHAR_CAP`] 截断，半截样点喂进上下文 ——
//! **既烧 token 又误导模型**。
//!
//! 所以镜像由 GUI 自己做：每轮工具执行完调一次 `scope_list_captures`，
//! 对没见过的编号自己发 `scope_read_waveform` 拉全通道，重建 [`Capture`] 上屏。
//! **这些调用绝不进 LLM 的消息。**

use std::collections::BTreeSet;
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use scope_core::Capture;
use serde_json::{json, Value};

use crate::ai::AiError;
use crate::config::AiConfig;

/// 单次工具结果喂给模型时的字符上限。
///
/// 与 `tools/agent_demo/agent.py` 的 `RESULT_CHAR_CAP` 一致。
///
/// ⚠ 截断**必须带说明**。静默截断会让模型以为「数据就这么多」，
/// 而它可能正据此下一个「没有异常」的结论。
pub const RESULT_CHAR_CAP: usize = 12_000;

/// 工具循环的轮数上限。与 `agent.py` 的 `MAX_TURNS` 一致。
pub const MAX_TURNS: usize = 24;

// 回复长度上限**不在这里定义** —— 从前的回归正是「单发」与「驱动会话」
// 各写一个值（2048 / 8192），没有任何东西守着它们不分家。
// 唯一出处是 `crate::ai::MAX_TOKENS`，用之前先看那里的说明。

/// 单次 MCP 请求的等待上限。
///
/// ⚠ **`agent.py` 这里是没有超时的**（阻塞死等）。照抄的话，子进程一旦卡住
/// 就再也回不来 —— 而它跑在一条专用线程上，用户只能看着界面转圈。
pub const RPC_TIMEOUT: Duration = Duration::from_secs(30);

/// 单次 LLM 请求的超时。
pub const LLM_TIMEOUT: Duration = Duration::from_secs(180);

/// 子进程优雅退出的等待上限；超过就 kill。
pub const SHUTDOWN_GRACE: Duration = Duration::from_secs(3);

/// 找 `scope-mcp` 可执行文件。
///
/// **按 `current_exe()` 同级找，不按 cwd** —— GUI 的 cwd 是仓库根，
/// 而可执行文件在 `target/debug/`（`run.sh` 还可能把它重定向到别处）。
/// `agent.py` 是靠列一串候选路径解决的，那在 GUI 里没必要：
/// 两个 exe 是 `cargo` 一起放进同一个目录的。
pub fn find_scope_mcp() -> Result<PathBuf, AiError> {
    let me = std::env::current_exe().map_err(|e| {
        AiError::new(
            format!("无法获取自身可执行文件的位置（{e}）"),
            "这是程序缺陷，请提交 issue",
        )
    })?;
    let dir = me.parent().ok_or_else(|| {
        AiError::new(
            "自身可执行文件没有上级目录".to_string(),
            "这是程序缺陷，请提交 issue",
        )
    })?;

    let name = if cfg!(windows) {
        "scope-mcp.exe"
    } else {
        "scope-mcp"
    };

    // 找两处：
    //   1. 同目录 —— 正常运行时的情况（`cargo` 把两个 exe 放进同一个目录）
    //   2. **上一级** —— 测试二进制跑在 `target/debug/deps/` 下，
    //      它的同目录只有测试产物，真正的 exe 在上一级
    //
    // 第 2 条不是补丁：`--drive` 这条命令行入口正是靠它才能在
    // `cargo test` 里被验到（否则只能手工起窗口试）。
    let mut tried = Vec::new();
    for d in [Some(dir), dir.parent()].into_iter().flatten() {
        let candidate = d.join(name);
        if candidate.is_file() {
            return Ok(candidate);
        }
        tried.push(candidate);
    }

    Err(AiError::new(
        format!(
            "找不到 {}",
            tried
                .iter()
                .map(|p| p.display().to_string())
                .collect::<Vec<_>>()
                .join(" 或 ")
        ),
        "与本程序放在同一目录即可（通常是 host/target/debug/）—— \
         由 ./host/run.sh run -p scope-gui 启动时自动就位",
    ))
}

// ══════════════════════════════════════════════════════════════════
// 纯函数（CI 可测）
// ══════════════════════════════════════════════════════════════════

/// 解析子进程的一行输出。
///
/// 返回 `None` 表示**这行不是 JSON，应当跳过**。
///
/// ⚠ `agent.py` 不跳 —— 它直接 `json.loads`，于是子进程往 stdout 打一行日志
/// 就会抛 JSONDecodeError 把整个会话打断。而「往 stdout 打日志」是子进程
/// 完全可能做的事（本项目就有 `SCOPE_MCP_DEBUG`）。
pub fn parse_line(line: &str) -> Option<Value> {
    let t = line.trim();
    if t.is_empty() {
        return None;
    }
    serde_json::from_str::<Value>(t).ok()
}

/// 校验响应里的 `id` 是否为当前等待的那一条。
///
/// 没有 `id` 的（通知）直接跳过；有 `id` 但对不上的**报错** ——
/// 本实现一次只发送一条请求，出现别的 id 就是协议出问题了，静默收下会让
/// 调用方拿着别人的结果当自己的。
pub fn check_id(resp: &Value, want: u64) -> Result<bool, AiError> {
    match resp.get("id") {
        None => Ok(false), // 通知，跳过
        Some(v) => {
            if v.as_u64() == Some(want) {
                Ok(true)
            } else {
                Err(AiError::new(
                    format!("响应 id 对不上：等 {want}，收到 {v}"),
                    "子进程与客户端已不同步 —— 终止本次会话后重新发起",
                ))
            }
        }
    }
}

/// 把工具结果截到 [`RESULT_CHAR_CAP`]，**带说明**。
pub fn truncate_for_llm(text: &str) -> String {
    let n = text.chars().count();
    if n <= RESULT_CHAR_CAP {
        return text.to_string();
    }
    let head: String = text.chars().take(RESULT_CHAR_CAP).collect();
    format!("{head}\n…（已截断：原文 {n} 字符，此处保留前 {RESULT_CHAR_CAP} 字符）")
}

/// 从 `scope_list_captures` 的一条记录重建 `Capture` 的**非样点部分**。
///
/// 样点由 `scope_read_waveform` 另外拉 —— 那是全量数据，不进模型上下文。
pub fn rebuild_capture(meta: &Value, channels: Vec<Vec<u16>>) -> Result<Capture, AiError> {
    let bad = |what: &str| {
        AiError::new(
            format!("list_captures 的记录里缺 {what} —— 无法重建采集"),
            "这是程序缺陷或版本不匹配，请提交 issue",
        )
    };
    let id = meta["capture_id"]
        .as_u64()
        .ok_or_else(|| bad("capture_id"))? as u16;
    let rate_hz = meta["rate_hz"].as_u64().ok_or_else(|| bad("rate_hz"))? as u32;
    let expected = meta["expected_samples"]
        .as_u64()
        .ok_or_else(|| bad("expected_samples"))? as u32;

    let mut cap = Capture::new(id, rate_hz, channels.len().max(1), expected);
    cap.channels = channels;
    cap.trigger_index = meta["trigger_index"].as_u64().map(|v| v as u32);
    cap.device_tick_us = meta["device_tick_us"].as_u64().unwrap_or(0) as u32;
    cap.wall_time = meta["wall_time"].as_u64().unwrap_or(0);
    cap.overrun = meta["overrun"].as_bool().unwrap_or(false);
    Ok(cap)
}

// ══════════════════════════════════════════════════════════════════
// 子进程 + JSON-RPC
// ══════════════════════════════════════════════════════════════════

/// 一个 `scope-mcp` 子进程，加一条 stdio JSON-RPC 通道。
///
/// # 与 `agent.py` 的三处不同（都是「看似可运行但会卡死」的坑）
///
/// 1. **stderr 要排空。** `agent.py` 把它接进管道却从不读 —— 子进程往 stderr
///    写够一个管道缓冲就**永久阻塞**。这里单开一条线程读掉。
/// 2. **读要有超时。** `agent.py` 是阻塞死等。这里把 stdout 交给一条读线程，
///    请求方 `recv_timeout` —— 子进程卡住时能报错而不是陪着一起卡。
/// 3. **非法行要跳过、id 要校验**（见 [`parse_line`] / [`check_id`]）。
pub struct McpProcess {
    child: Arc<Mutex<Child>>,
    stdin: Arc<Mutex<ChildStdin>>,
    lines: Receiver<String>,
    next_id: u64,
}

impl McpProcess {
    /// 起一个子进程。
    pub fn spawn(exe: &PathBuf) -> Result<McpProcess, AiError> {
        let fail = |e: std::io::Error| {
            AiError::new(
                format!("无法启动 scope-mcp（{e}）"),
                "确认它与本程序在同一目录、且有执行权限",
            )
        };

        let mut child = Command::new(exe)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(fail)?;

        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| fail(std::io::Error::other("no stdin")))?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| fail(std::io::Error::other("no stdout")))?;
        let stderr = child
            .stderr
            .take()
            .ok_or_else(|| fail(std::io::Error::other("no stderr")))?;

        // stdout → 通道。读线程随子进程退出而结束（read_line 返回 0）。
        let (tx, lines) = mpsc::channel::<String>();
        std::thread::Builder::new()
            .name("scope-mcp-stdout".into())
            .spawn(move || {
                let mut r = BufReader::new(stdout);
                loop {
                    let mut s = String::new();
                    match r.read_line(&mut s) {
                        Ok(0) | Err(_) => break, // EOF 或出错：子进程没了
                        Ok(_) => {
                            if tx.send(s).is_err() {
                                break; // 接收端没了
                            }
                        }
                    }
                }
            })
            .map_err(fail)?;

        // stderr **必须读掉**，否则子进程写满管道缓冲就永久阻塞。
        // 内容丢弃（那是诊断信息，界面上不显示；真要看可以开 SCOPE_MCP_DEBUG）。
        std::thread::Builder::new()
            .name("scope-mcp-stderr".into())
            .spawn(move || {
                let mut r = BufReader::new(stderr);
                let mut s = String::new();
                while matches!(r.read_line(&mut s), Ok(n) if n > 0) {
                    s.clear();
                }
            })
            .map_err(fail)?;

        Ok(McpProcess {
            child: Arc::new(Mutex::new(child)),
            stdin: Arc::new(Mutex::new(stdin)),
            lines,
            next_id: 1,
        })
    }

    /// 子进程句柄 —— 给 UI 线程用来「立刻终止」。
    ///
    /// 必须是共享的 `Arc<Mutex<_>>`：取消时本线程多半正阻塞在 LLM 的 HTTP
    /// （最长 [`LLM_TIMEOUT`]）或管道读上，**它自己没有机会去看取消标志**。
    pub fn child_handle(&self) -> Arc<Mutex<Child>> {
        Arc::clone(&self.child)
    }

    fn send(&mut self, msg: &Value) -> Result<(), AiError> {
        let mut line = serde_json::to_string(msg).map_err(|e| {
            AiError::new(format!("序列化请求失败：{e}"), "属程序缺陷，请提交 issue")
        })?;
        line.push('\n');
        let mut w = self.stdin.lock().unwrap();
        w.write_all(line.as_bytes())
            .and_then(|_| w.flush())
            .map_err(|e| {
                AiError::new(
                    format!("写给 scope-mcp 失败：{e}"),
                    "子进程可能已经退出了 —— 终止本次会话重来",
                )
            })
    }

    /// 发一条请求并等它的响应。
    pub fn call(&mut self, method: &str, params: Value) -> Result<Value, AiError> {
        let id = self.next_id;
        self.next_id += 1;
        self.send(&json!({
            "jsonrpc": "2.0",
            "id": id,
            "method": method,
            "params": params,
        }))?;

        let deadline = Instant::now() + RPC_TIMEOUT;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let line = match self.lines.recv_timeout(left) {
                Ok(l) => l,
                Err(RecvTimeoutError::Timeout) => {
                    return Err(AiError::new(
                        format!("等 {method} 的响应超时（{} 秒）", RPC_TIMEOUT.as_secs()),
                        "子进程无响应 —— 终止本次会话后重新发起",
                    ));
                }
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(AiError::new(
                        "scope-mcp 没有响应就退出了".to_string(),
                        "它可能崩溃了 —— 终止本次会话后重试",
                    ));
                }
            };

            let Some(resp) = parse_line(&line) else {
                continue; // 不是 JSON —— 跳过（子进程可能打了日志）
            };
            if !check_id(&resp, id)? {
                continue; // 通知
            }
            if let Some(err) = resp.get("error") {
                return Err(AiError::new(
                    format!("{method} 返回错误：{err}"),
                    "协议层出错 —— 终止本次会话重来",
                ));
            }
            return Ok(resp.get("result").cloned().unwrap_or(Value::Null));
        }
    }

    /// 优雅收场：让子进程自己断开设备，再关 stdin 等它退出，超时才 kill。
    ///
    /// **顺序不能反**：先杀的话，真机上设备可能停在 Armed ——
    /// 而 GUI 随后要把它连回来。
    ///
    /// **kill 之后必须 `wait()`**：Windows 上进程句柄没释放就重连，
    /// 会偶发「端口被占用」。
    pub fn shutdown(mut self) {
        // 1) 请它自己断开（它内部会 bus.stop()，真机回到 Idle）
        let _ = self.call(
            "tools/call",
            json!({"name": "scope_disconnect", "arguments": {}}),
        );

        // 2) 关 stdin —— 子进程读到 EOF 会自己退出
        if let Ok(mut w) = self.stdin.lock() {
            let _ = w.flush();
        }
        drop(self.stdin);

        // 3) 等它退出；超时才杀
        let child = Arc::clone(&self.child);
        let Ok(mut c) = child.lock() else { return };
        let deadline = Instant::now() + SHUTDOWN_GRACE;
        loop {
            match c.try_wait() {
                Ok(Some(_)) => return, // 自己退了
                Ok(None) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(50));
                }
                _ => {
                    let _ = c.kill();
                    let _ = c.wait(); // ← 必须等，否则句柄没释放
                    return;
                }
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════
// 镜像：GUI 自己把 AI 采的波拉回来上屏
// ══════════════════════════════════════════════════════════════════

/// 已经镜像过的采集编号 —— 用来判断「这一窗是不是新的」。
#[derive(Default)]
pub struct Mirrored {
    seen: BTreeSet<u16>,
}

impl Mirrored {
    /// 扫一遍子进程的采集列表，把**没见过的**拉回样点、重建成 `Capture`。
    ///
    /// 返回这一轮新出现的采集（可能为空）。
    ///
    /// **这些调用绝不进 LLM 的消息** —— 它们是「给人看的」，
    /// 让模型看见只会白白消耗 token，而且 4096 点还会被截断成半截。
    pub fn poll(&mut self, mcp: &mut McpProcess) -> Result<Vec<Capture>, AiError> {
        let list = mcp.call(
            "tools/call",
            json!({"name": "scope_list_captures", "arguments": {}}),
        )?;
        let text = tool_text(&list);

        // ⚠ 解析失败**必须报错**，不能返回空列表。
        //
        // 第一版这里是 `return Ok(Vec::new())` —— 于是「拉不到波形」与
        // 「确实还没有采集」表现完全一样：屏幕空白，而没有任何线索。
        // 静默降级成「什么也没有」正是本项目反复打的那类病。
        let parsed = parse_tool_json(&text).ok_or_else(|| {
            AiError::new(
                format!("list_captures 返回的不是 JSON：{}", first_line(&text)),
                "子进程与客户端版本可能不匹配 —— 重新构建后重试",
            )
        })?;
        let rows = parsed["captures"].as_array().ok_or_else(|| {
            AiError::new(
                format!(
                    "list_captures 的返回里没有 captures 数组：{}",
                    first_line(&text)
                ),
                "子进程与客户端版本可能不匹配 —— 重新构建后重试",
            )
        })?;

        let mut fresh = Vec::new();
        for meta in rows {
            let Some(id) = meta["capture_id"].as_u64() else {
                continue;
            };
            let id = id as u16;
            if self.seen.contains(&id) {
                continue;
            }
            let nch = meta["channels"].as_u64().unwrap_or(1).max(1) as usize;
            let mut channels = Vec::with_capacity(nch);
            for ch in 0..nch {
                let r = mcp.call(
                    "tools/call",
                    json!({
                        "name": "scope_read_waveform",
                        "arguments": { "capture_id": id, "channel": ch, "count": 4096 },
                    }),
                )?;
                channels.push(read_samples(&tool_text(&r)));
            }
            self.seen.insert(id);
            fresh.push(rebuild_capture(meta, channels)?);
        }
        Ok(fresh)
    }
}

/// `tools/call` 的返回形如 `{content:[{type:"text",text:"…"}]}`，取其中文本。
pub fn tool_text(result: &Value) -> String {
    result
        .get("content")
        .and_then(|c| c.as_array())
        .map(|blocks| {
            blocks
                .iter()
                .filter(|b| b.get("type").and_then(|t| t.as_str()) == Some("text"))
                .filter_map(|b| b.get("text").and_then(|t| t.as_str()))
                .collect::<Vec<_>>()
                .join("\n")
        })
        .unwrap_or_default()
}

/// 工具返回的文本里嵌着一层 JSON，解出来。
pub fn parse_tool_json(text: &str) -> Option<Value> {
    serde_json::from_str::<Value>(text.trim()).ok()
}

/// 从 `scope_read_waveform` 的返回里取样点。
pub fn read_samples(text: &str) -> Vec<u16> {
    parse_tool_json(text)
        .and_then(|v| {
            v.get("samples").and_then(|s| s.as_array()).map(|arr| {
                arr.iter()
                    .filter_map(|x| x.as_u64())
                    .map(|x| x.min(4095) as u16)
                    .collect()
            })
        })
        .unwrap_or_default()
}

// ══════════════════════════════════════════════════════════════════
// 会话
// ══════════════════════════════════════════════════════════════════

/// 交给 AI 的一次会话。
pub struct DriveJob {
    /// 用户的自然语言需求。
    pub task: String,
    /// 密钥 / 端点 / 模型（与单发分析共用一份配置）。
    pub cfg: AiConfig,
    /// **钉死的** `scope_connect` 参数。
    ///
    /// AI 调 `scope_connect` 时，参数由这里覆盖 —— 不让它自己挑设备。
    /// 否则它可能连到别的口上去，而 GUI 手上那份「交接快照」就对不上了。
    pub connect_args: Value,
}

/// 会话过程中报给界面的事件。
///
/// ⚠ **不是聊天记录**，是动作轨迹。用户要能看出「AI 改了哪个参数、采了哪几窗」——
/// 因为在示波器上选错时基/触发电平会得到一份**看起来合理**的新波形，
/// 错误是沉默的。
#[derive(Debug)]
pub enum DriveEvent {
    /// 阶段变化（起进程、握手、跑循环…）。
    Phase(String),
    /// AI 决定调用某个工具。
    ToolCall {
        /// 工具名。
        name: String,
        /// 参数的紧凑描述（给人看）。
        args: String,
    },
    /// 工具执行完了。
    ToolResult {
        /// 工具名。
        name: String,
        /// 业务上成功了没有。
        ok: bool,
        /// 一句话摘要（给人看）。
        summary: String,
    },
    /// 工具返回里带的警告（例如采样率被量化、SW2/SW3 是手拨开关）。
    Warnings(Vec<String>),
    /// 子进程采到了新的一窗 —— GUI 已经替它拉回样点并重建。
    Capture(Box<Capture>),
    /// 正常结束。
    Finished {
        /// 最终回复。
        text: String,
        /// 实际跑了几轮。
        turns: usize,
        /// 是否因达到轮数上限而停止。
        hit_limit: bool,
    },
    /// 出错结束。
    ///
    /// ⚠ **必须有这一条，不能只发一条「失败」文案。**
    ///
    /// 回归：第一版出错时只往轨迹里塞了一句 `✗ …`，而没有终止信号 ——
    /// 于是收尾那一步（把设备连回来）根本不会执行，
    /// **界面永远停在「AI 正在操作设备」，而设备谁也拿不回来**。
    /// 是「`DrivePhase::Failed` 从没被构造」这个死代码警告把它顶出来的。
    Failed(AiError),
}

/// 执行一次完整会话。**阻塞**，只能在专用线程里调。
///
/// # 与 `tools/agent_demo/agent.py` 的关系
///
/// 循环的骨架是从它移植的（那是唯一一个已验证可用的 MCP 工具循环参考实现），
/// 但有几处**故意不同**：
///
/// | `agent.py` | 这里 | 为什么 |
/// |---|---|---|
/// | stderr 接了不读 | 单开线程排空 | 不排空子进程会阻塞在写 stderr 上 |
/// | 无读超时 | [`RPC_TIMEOUT`] | 卡住要能报错，不能陪着一起卡 |
/// | 不校验 id、不跳非 JSON 行 | 都做 | 子进程打一行日志就会把会话打断 |
/// | 结果由模型决定读什么 | **GUI 自己镜像** | 见模块头 —— 模型可能不读样点 |
///
/// 轮数的终止条件仍是「有没有 `tool_use`」—— 与它一致（**不看 `stop_reason`**）。
pub fn run_session(
    job: &DriveJob,
    mcp: &mut McpProcess,
    emit: &mut dyn FnMut(DriveEvent),
) -> Result<(), AiError> {
    emit(DriveEvent::Phase("正在初始化 MCP 会话".into()));

    // 与 agent.py 一致：只 initialize + tools/list，**不发 initialized 通知**
    // （服务端不要求，且发了也没人读）。
    mcp.call(
        "initialize",
        json!({
            "protocolVersion": "2024-11-05",
            "capabilities": {},
            "clientInfo": { "name": "scope-gui", "version": scope_core::VERSION },
        }),
    )?;

    let tools_resp = mcp.call("tools/list", json!({}))?;
    let tools = to_llm_tools(&tools_resp);
    let tool_count = tools.as_array().map(|a| a.len()).unwrap_or(0);

    // 连设备。**参数由 GUI 钉死** —— 不让 AI 自己挑目标。
    emit(DriveEvent::Phase("正在连接设备".into()));
    mcp.call(
        "tools/call",
        json!({ "name": "scope_connect", "arguments": job.connect_args }),
    )?;

    emit(DriveEvent::Phase(format!(
        "开始（{tool_count} 个工具可用）"
    )));

    let mut messages = vec![json!({ "role": "user", "content": job.task })];
    let mut mirrored = Mirrored::default();

    for turn in 0..MAX_TURNS {
        let body = json!({
            "model": job.cfg.model,
            "max_tokens": crate::ai::MAX_TOKENS,
            // ⚠ **必须带 system**。回归：第一版这里没有它 —— 模型既不知道
            // 输出该收敛成什么形状，也不知道界面上的通道是 CH1/CH2，
            // 于是把接口的 0 起编号原样转述给了用户。
            "system": drive_system_prompt(),
            "tools": tools,
            "messages": messages,
        });
        let resp = post_llm(&job.cfg, &body)?;
        let blocks = resp
            .get("content")
            .and_then(|c| c.as_array())
            .cloned()
            .unwrap_or_default();

        // assistant 的 content 块**原样回传** —— thinking 块带的 signature
        // 必须一起带回去，模型才认得出自己的思考。
        messages.push(json!({ "role": "assistant", "content": blocks }));

        let calls: Vec<&Value> = blocks
            .iter()
            .filter(|b| b.get("type").and_then(|t| t.as_str()) == Some("tool_use"))
            .collect();

        if calls.is_empty() {
            // 没有工具调用 = 它答完了
            let text = blocks
                .iter()
                .filter(|b| b.get("type").and_then(|t| t.as_str()) == Some("text"))
                .filter_map(|b| b.get("text").and_then(|t| t.as_str()))
                .collect::<Vec<_>>()
                .join("\n");

            // 「没有工具调用」也可能是**话没说完就没预算了**：整轮的 token
            // 全烧在思考块上，正文与 tool_use 一个都没轮到。
            //
            // 不区分的话，这条路会以 `Finished { text: "" }` 收场，界面显示
            // 「无分析结果」—— 与「模型认为没什么可说」长得一模一样，而两者
            // 的处置完全相反（前者要缩短证据包重试，后者不必）。这一次会话
            // 的钱已经花掉了，至少要让用户知道它为什么是空的。
            if text.trim().is_empty()
                && resp.get("stop_reason").and_then(|s| s.as_str()) == Some("max_tokens")
            {
                return Err(AiError::new(
                    "会话在最后一轮用光了 token 预算，未留下任何结论".to_string(),
                    "整个预算被思考块消耗。减少采集点数后重试，或把任务描述得更具体",
                ));
            }

            emit(DriveEvent::Finished {
                text,
                turns: turn + 1,
                hit_limit: false,
            });
            return Ok(());
        }

        let mut results = Vec::new();
        for c in calls {
            let name = c
                .get("name")
                .and_then(|n| n.as_str())
                .unwrap_or("?")
                .to_string();
            let call_id = c
                .get("id")
                .and_then(|i| i.as_str())
                .unwrap_or("")
                .to_string();
            let mut arguments = c.get("input").cloned().unwrap_or(json!({}));

            // 把 scope_connect 钉死在 GUI 交接的那个目标上
            if name == "scope_connect" {
                arguments = job.connect_args.clone();
            }

            emit(DriveEvent::ToolCall {
                name: name.clone(),
                args: summarize_args(&arguments),
            });

            let (text, is_err) = match mcp.call(
                "tools/call",
                json!({ "name": name, "arguments": arguments }),
            ) {
                Ok(r) => {
                    let t = tool_text(&r);
                    let e = r.get("isError").and_then(|v| v.as_bool()).unwrap_or(false);
                    (t, e)
                }
                Err(e) => (format!("{}｜{}", e.message, e.hint), true),
            };

            // 工具返回里可能带 warnings（采样率量化、手拨开关…）—— 必须上屏
            if let Some(w) = parse_tool_json(&text).and_then(|v| {
                v.get("warnings").and_then(|w| w.as_array()).map(|a| {
                    a.iter()
                        .filter_map(|x| x.as_str().map(String::from))
                        .collect()
                })
            }) {
                emit(DriveEvent::Warnings(w));
            }

            emit(DriveEvent::ToolResult {
                name: name.clone(),
                ok: !is_err,
                summary: first_line(&text),
            });

            // —— GUI 自己镜像：把新采的那一窗拉回来上屏 ——
            // 这一步的花费**不进** messages。
            match mirrored.poll(mcp) {
                Ok(caps) => {
                    for cap in caps {
                        emit(DriveEvent::Capture(Box::new(cap)));
                    }
                }
                Err(e) => {
                    // 镜像失败不该打断 AI 的活 —— 记一句，继续
                    emit(DriveEvent::ToolResult {
                        name: "镜像".into(),
                        ok: false,
                        summary: format!("拉回波形失败：{}", e.message),
                    });
                }
            }

            results.push(json!({
                "type": "tool_result",
                "tool_use_id": call_id,
                "content": truncate_for_llm(&text),
                "is_error": is_err,
            }));
        }

        // 一次调用的全部结果**并入同一条 user 消息**（与 agent.py 一致）
        messages.push(json!({ "role": "user", "content": results }));
    }

    emit(DriveEvent::Finished {
        text: String::new(),
        turns: MAX_TURNS,
        hit_limit: true,
    });
    Ok(())
}

/// 把 MCP 的 `tools/list` 结果转成 LLM 要的形状。
///
/// **只改字段名** `inputSchema` → `input_schema`，其余原样 ——
/// 与 `agent.py` 的做法一致，那条路已经验证可用。
///
/// ⚠ **`annotations` 刻意不转发。** 这个函数的输出**直接进 Anthropic Messages
/// 请求体**，而那边对工具对象的键是白名单的 —— 多一个它不认识的键，**整个请求
/// 会被拒**（`Extra inputs are not permitted`）。注解是给 **MCP 宿主**看的提示
/// （据此决定要不要免确认），不是给模型看的，模型不需要它。
/// `tools_list_is_renamed_not_reshaped` 里钉了这条。
pub fn to_llm_tools(resp: &Value) -> Value {
    let tools: Vec<Value> = resp
        .get("tools")
        .and_then(|t| t.as_array())
        .map(|arr| {
            arr.iter()
                .map(|t| {
                    json!({
                        "name": t.get("name").cloned().unwrap_or(Value::Null),
                        "description": t.get("description").cloned().unwrap_or(Value::Null),
                        "input_schema": t.get("inputSchema").cloned().unwrap_or(json!({"type":"object"})),
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    Value::Array(tools)
}

/// 驱动会话的系统提示。
///
/// = 单发那一份（不复述、三节、模拟器声明、不得杜撰）
/// **+ 通道编号的换算规矩** —— 那是这一条路特有的。
///
/// # 为什么必须有
///
/// 工具返回里的通道号是**接口的内部编号（0 起）**：
///
/// ```text
/// scope_capture    → channels[].channel = 0, 1
/// scope_i2c_decode → scl_channel = 0, sda_channel = 1
/// ```
///
/// 而用户界面上显示的是 **CH1 / CH2（1 起）**。模型如实转述接口编号，
/// 用户看到的就是「CH0 = SCL」—— **对不上屏幕上那两条泳道**。
///
/// 这是「同一件事两个编号」的老毛病，但两个编号各有理由：
/// 一个是数组下标，一个是给人看的标签。**不通信才是缺陷** ——
/// 所以规矩写在这里，而不是去改接口的编号。
///
/// 回归：第一版这条路上**根本没发 system prompt**，模型无从知道界面怎么编号。
fn drive_system_prompt() -> String {
    format!(
        "{}\n\n\
         ## 通道编号的换算（这一条务必遵守）\n\n\
         工具参数与工具返回里的通道号（`channel` / `scl_channel` / `sda_channel`）\
         **从 0 起**，那是接口的内部编号。\n\n\
         用户界面上显示的是 **CH1 / CH2（从 1 起）**。**向用户表述时必须换算**：\n\n\
         | 接口里的编号 | 界面上叫 |\n|---|---|\n\
         | 0 | CH1 |\n| 1 | CH2 |\n\n\
         直接把接口编号写进回答，用户会对不上屏幕上的波形。\
         自己推理时用哪个都行，但**写给人看的文字一律用 CH1 / CH2**。",
        crate::ai::SYSTEM_PROMPT
    )
}

/// 发一次 LLM 请求。形状与 `ai.rs` 的单发一致，多一个 `tools`。
fn post_llm(cfg: &AiConfig, body: &Value) -> Result<Value, AiError> {
    if !cfg.has_key() {
        return Err(AiError::new(
            "未配置 API 密钥",
            "请在面板的「设置」中填写密钥",
        ));
    }
    let text = serde_json::to_string(body)
        .map_err(|e| AiError::new(format!("构造请求体失败：{e}"), "属程序缺陷，请提交 issue"))?;

    let result = ureq::post(cfg.base_url.trim())
        .header("content-type", "application/json")
        .header("x-api-key", cfg.api_key.trim())
        .header("anthropic-version", "2023-06-01")
        .config()
        .http_status_as_error(false)
        .timeout_connect(Some(Duration::from_secs(10)))
        .timeout_global(Some(LLM_TIMEOUT))
        .build()
        .send(text.as_str());

    match result {
        Ok(mut resp) => {
            let status = resp.status().as_u16();
            let body = resp.body_mut().read_to_string().unwrap_or_default();
            if !(200..=299).contains(&status) {
                return Err(crate::ai::http_error_for_drive(status, &body));
            }
            serde_json::from_str::<Value>(&body).map_err(|e| {
                AiError::new(
                    format!("响应不是 JSON：{e}"),
                    "端点可能配错了 —— 确认 base_url 指向 messages 接口",
                )
            })
        }
        Err(e) => Err(crate::ai::transport_error_for_drive(&e)),
    }
}

/// 参数的紧凑描述（给人看，进轨迹）。
fn summarize_args(v: &Value) -> String {
    let s = v.to_string();
    if s.chars().count() <= 120 {
        s
    } else {
        format!("{}…", s.chars().take(120).collect::<String>())
    }
}

/// 取第一行非空内容当摘要。
fn first_line(text: &str) -> String {
    let line = text.lines().find(|l| !l.trim().is_empty()).unwrap_or("");
    if line.chars().count() <= 120 {
        line.to_string()
    } else {
        format!("{}…", line.chars().take(120).collect::<String>())
    }
}

// ══════════════════════════════════════════════════════════════════
// 会话句柄：UI 侧的那一半
// ══════════════════════════════════════════════════════════════════

/// 子进程句柄的「信箱」。
///
/// 线程 spawn 出子进程之后把句柄放进来，UI 点「终止」时直接拿它去 kill ——
/// 线程可能正阻塞在 LLM 的 HTTP 上，它自己没有机会看取消标志。
type ChildSlot = Arc<Mutex<Option<Arc<Mutex<Child>>>>>;

/// 一次会话在 UI 侧的句柄。
pub struct DriveHandle {
    /// 线程报上来的事件。
    events: Receiver<DriveEvent>,
    /// 子进程句柄（起好之后才有）。
    child: ChildSlot,
    /// 世代号 —— 迟到的会话事件靠它丢弃。
    ///
    /// 用户终止后又发起一次时，旧线程可能还会吐几条事件出来；
    /// 没有这个号，它们会混进新会话的轨迹里。
    pub generation: u64,
}

impl DriveHandle {
    /// 排空事件。**先写入 Vec 再处理** —— 与 `worker.rs` / `ai.rs` 同一套纪律，
    /// 免得在闭包里可变借走 `App`。
    pub fn drain(&self, mut f: impl FnMut(DriveEvent)) {
        while let Ok(e) = self.events.try_recv() {
            f(e);
        }
    }

    /// 立刻终止：杀掉子进程。
    ///
    /// **不走优雅收场** —— 用户点了终止就该立刻停，而不是再等 3 秒。
    /// 代价是真机可能停在 Armed，重连后要如实提示（见 `App` 那侧的处理）。
    pub fn terminate(&self) {
        if let Ok(slot) = self.child.lock() {
            if let Some(c) = slot.as_ref() {
                if let Ok(mut child) = c.lock() {
                    let _ = child.kill();
                    // 必须 wait：句柄不释放，GUI 重连会偶发「端口被占用」
                    let _ = child.wait();
                }
            }
        }
    }
}

/// 起一条线程跑完整会话，返回 UI 侧的句柄。
///
/// 线程里做四件事：找 exe → spawn → 跑循环 → 优雅收场。
/// 所有进展都经 [`DriveEvent`] 报回 UI。
pub fn spawn_session(job: DriveJob, generation: u64, ctx: egui::Context) -> DriveHandle {
    let (tx, events) = mpsc::channel::<DriveEvent>();
    let child: ChildSlot = Arc::new(Mutex::new(None));
    let slot = Arc::clone(&child);

    std::thread::Builder::new()
        .name("scope-drive".into())
        .spawn(move || {
            let mut emit = |e: DriveEvent| {
                if tx.send(e).is_ok() {
                    ctx.request_repaint();
                }
            };

            // 起不来的两种情况都走同一个出口：报 `Failed`。
            // ⚠ 不能只发一条文案就 `return` —— 那样 UI 侧收不到终止信号，
            // 会永远停在「AI 正在操作设备」。
            let exe = match find_scope_mcp() {
                Ok(p) => p,
                Err(e) => {
                    emit(DriveEvent::Failed(e));
                    return;
                }
            };

            let mut mcp = match McpProcess::spawn(&exe) {
                Ok(m) => m,
                Err(e) => {
                    emit(DriveEvent::Failed(e));
                    return;
                }
            };

            // 把句柄交给 UI —— 在这之前用户点终止是杀不到东西的（还没起）
            if let Ok(mut s) = slot.lock() {
                *s = Some(mcp.child_handle());
            }

            if let Err(e) = run_session(&job, &mut mcp, &mut emit) {
                emit(DriveEvent::Failed(e));
            }

            // 优雅收场（子进程可能已经被终止掉了 —— 那时这里是空转）
            mcp.shutdown();
        })
        .expect("起不了 drive 线程");

    DriveHandle {
        events,
        child,
        generation,
    }
}

// ══════════════════════════════════════════════════════════════════
// UI 侧状态
// ══════════════════════════════════════════════════════════════════

/// 会话处在哪一步。
///
/// **用枚举而不是一个 u8 计数器**（`demo_stage` 那种）：这次的边界多得多
/// ——断开、起进程、跑循环、收场、重连，每一步失败都有不同的收尾动作。
/// 隐式状态机读不懂，也测不了。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum DrivePhase {
    /// 没在跑。设备归 GUI。
    #[default]
    Idle,
    /// 正在让 GUI 断开 —— 要把串口让出去。
    Disconnecting,
    /// 子进程在跑，AI 在操作设备。
    Running,
    /// AI 干完了（或出错了），正在把设备连回来。
    ///
    /// **失败不走单独的状态**：一样要先把设备收回来，收回来之后
    /// 「失败」这件事已经记在轨迹与日志里了，再挂一个状态只会多一处收尾要维护。
    Reconnecting,
}

impl DrivePhase {
    /// 设备此刻是不是**不在** GUI 手上。
    ///
    /// 界面上一堆控件要靠它来决定禁不禁用 —— 而这**不是 UI 的偏好，
    /// 是串口独占的必然**：设备已经被让出去了，按钮点下去也连不上。
    pub fn device_is_elsewhere(&self) -> bool {
        !matches!(self, DrivePhase::Idle)
    }

    /// 会话还在进行中（可以终止）。
    pub fn is_busy(&self) -> bool {
        matches!(self, DrivePhase::Disconnecting | DrivePhase::Running)
    }
}

/// 交出去那一刻的设备与链路快照。
///
/// 两个用途，都不是可有可无的：
///
/// 1. **provenance** —— AI 期间 GUI 处于断开状态，[`crate::app::App::link_description`]
///    会输出「serial @ 921600」、`simulated` 会变成 false。**模拟器会被说成串口。**
///    有这个快照，锚点行与证据包才说得出真话。
/// 2. **「恢复交接前的设置」** —— 用户手动触发后即按其重新配置。
#[derive(Debug, Clone)]
pub struct Handoff {
    /// 链路的一句话描述（给人看，也进证据包）。
    pub link: String,
    /// 交出去时是不是模拟器。
    pub simulated: bool,
    /// 交出去时的设备配置回显。`None` = 当时没读到。
    pub config: Option<scope_core::DeviceConfig>,
    /// 重连用的目标 —— 与交给 AI 的**是同一个**。
    pub transport: crate::msg::TransportKind,
    /// 串口名。
    pub port: String,
    /// 波特率。
    pub baud: u32,
    /// 模拟器场景。
    pub scenario: scope_sim::Scenario,
}

impl Handoff {
    /// 交给子进程（以及重连时给 worker）的 `scope_connect` 参数。
    ///
    /// **同一个快照同时用于「交给 AI」和「收回来」** —— 两边各写一份的话，
    /// 迟早有一处不一样，而那种错的表现是「AI 采的」与「你屏幕上看到的」
    /// 是两台设备。
    pub fn connect_args(&self) -> Value {
        match self.transport {
            crate::msg::TransportKind::Serial => json!({
                "transport": "serial",
                "port": self.port,
                "baud": self.baud,
            }),
            crate::msg::TransportKind::Sim => json!({
                "transport": "sim",
                "sim_scenario": self.scenario.name(),
            }),
        }
    }
}

/// 轨迹里的一条。
///
/// ⚠ **这不是聊天记录，是动作清单。** 用户要能看出「AI 改了哪个参数、
/// 采了哪几窗」—— 因为在示波器上选错时基或触发电平会得到一份
/// **看起来完全合理**的新波形，错误是沉默的。
#[derive(Debug, Clone)]
pub enum TrajectoryItem {
    /// 阶段变化。
    Phase(String),
    /// AI 决定调用某个工具。
    Call {
        /// 工具名。
        name: String,
        /// 参数的紧凑描述。
        args: String,
    },
    /// 工具执行完。
    Result {
        /// 工具名。
        name: String,
        /// 业务上成功了没有。
        ok: bool,
        /// 一句话摘要。
        summary: String,
    },
    /// 工具返回里带的警告 —— **必须显式展示**。
    ///
    /// 采样率被量化、SW2/SW3 是手拨开关这类事，AI 与用户可能都不知道，
    /// 而它们直接影响「AI 配的到底生效了没有」。
    Warning(String),
    /// 镜像回来的新采集。
    Capture {
        /// 采集编号。
        id: u16,
        /// 点数。
        points: usize,
        /// 通道数。
        channels: usize,
    },
}

/// 「把 AI 改过的设置改回去」那个按钮的文案。
///
/// # 为什么单独成常量
///
/// 它有**四处引用**：按钮本身、设备收回时的那行日志、以及 [`Handoff`] 与
/// [`DriveState::begin`] 的文档。而它们此前**各不相同** ——
/// 「恢复我原来的设置」「恢复我交出去之前的设置」两种写法并存，
/// 按钮后来改了名，日志里还指着旧名字，**用户照着提示去找会找不到**。
///
/// 这正是本项目最熟的那类病：同一件事有多个名字，而且没有任何东西守着它们一致。
/// 程序里能共用的部分抽成常量；文档注释里仍写全名（那里引用不了常量值），
/// 但它只有一处，改的时候不会漏。
pub const RESTORE_LABEL: &str = "恢复交接前的设置";

/// 轨迹最多留多少条。
///
/// 24 轮 × 每条几项，不设上限能涨到几百条 —— 那是给人扫一眼用的，
/// 不是归档。与 `App::note` 的 200 行同一个道理。
pub const TRAJECTORY_CAP: usize = 200;

/// 「让 AI 自己配置并采集」这一摊状态。
///
/// 单独成组而不是往 `App` 上再摊七八个字段：这块与设备 worker **互斥**
/// （跑的时候设备不在 GUI 手上），收在一起才看得出这一点。
#[derive(Default)]
pub struct DriveState {
    /// 现在到哪一步了。
    pub phase: DrivePhase,
    /// 正在跑的会话（有的话）。UI 靠它收事件、也靠它终止。
    pub handle: Option<DriveHandle>,
    /// 用户输入的需求（与单发分析的输入框共用同一个字段）。
    pub task: String,
    /// 动作轨迹 —— **给人看的，不是聊天记录**。
    pub trajectory: Vec<TrajectoryItem>,
    /// 交出去那一刻的设备与链路快照。
    pub handoff: Option<Handoff>,
    /// AI 的最终结论。
    pub final_text: Option<String>,
    /// 世代号：迟到的会话事件靠它丢弃。
    generation: u64,
}

impl DriveState {
    /// 开一次新会话要用的世代号。
    pub fn next_generation(&mut self) -> u64 {
        self.generation += 1;
        self.generation
    }

    /// 现在这条事件是不是本次会话的。
    ///
    /// 用户终止后又发起一次时，旧线程还可能吐几条出来 —— 没有这道闸，
    /// 它们会混进新会话的轨迹里，而**表现与「AI 真的这么做了」完全一样**。
    pub fn accepts(&self, generation: u64) -> bool {
        generation == self.generation
    }

    /// 记一条轨迹。
    pub fn push(&mut self, item: TrajectoryItem) {
        if self.trajectory.len() >= TRAJECTORY_CAP {
            self.trajectory.remove(0);
        }
        self.trajectory.push(item);
    }

    /// 开始一次新会话：清轨迹、清结论、丢掉旧句柄。
    ///
    /// ⚠ **丢掉旧句柄之前要先终止它** —— 调用方负责（见 `App::start_drive`）。
    /// 这里只负责把自己这一摊清干净。
    ///
    /// **不动 `handoff`** —— 那是「恢复交接前的设置」的依据，
    /// 要活到设备收回来之后。
    pub fn begin(&mut self) {
        self.trajectory.clear();
        self.final_text = None;
        self.handle = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── 行解析 ───────────────────────────────────────────────────────

    #[test]
    fn non_json_lines_are_skipped_not_fatal() {
        // 回归风险：agent.py 直接 json.loads，子进程打一行日志就整个会话完蛋。
        assert!(parse_line("").is_none());
        assert!(parse_line("   ").is_none());
        assert!(parse_line("[scope-mcp] 正在启动…").is_none());
        assert!(parse_line("{\"partial\": ").is_none());

        // 正常的一行要能解出来
        assert!(parse_line(r#"{"jsonrpc":"2.0","id":1,"result":{}}"#).is_some());
        // 前后有空白也要认得
        assert!(parse_line("  {\"id\":1} \n").is_some());
    }

    #[test]
    fn a_mismatched_id_is_an_error_not_silently_accepted() {
        let want = 7;

        // 对得上
        assert!(check_id(&json!({"id": 7}), want).unwrap());
        // 通知（没有 id）→ 跳过，不算错
        assert!(!check_id(&json!({"method": "notifications/x"}), want).unwrap());
        // 对不上 → 报错。静默收下会让调用方拿着别人的结果当自己的。
        let e = check_id(&json!({"id": 8}), want).unwrap_err();
        assert!(
            e.message.contains("7") && e.message.contains("8"),
            "{}",
            e.message
        );
        assert!(!e.hint.trim().is_empty(), "错误必须带怎么办");
    }

    // ── 截断 ─────────────────────────────────────────────────────────

    #[test]
    fn truncation_always_says_so() {
        let short = "x".repeat(100);
        assert_eq!(truncate_for_llm(&short), short, "没超限不该动");

        let long = "x".repeat(RESULT_CHAR_CAP + 500);
        let out = truncate_for_llm(&long);
        assert!(out.contains("已截断"), "截断必须带说明");
        assert!(
            out.contains(&format!("{}", RESULT_CHAR_CAP + 500)),
            "要说清原文多长"
        );
        assert!(
            out.chars().count() <= RESULT_CHAR_CAP + 100,
            "截完还是太长：{}",
            out.chars().count()
        );
    }

    // ── 重建 Capture ─────────────────────────────────────────────────

    fn meta(id: u16) -> Value {
        json!({
            "capture_id": id, "rate_hz": 857_142, "sample_count": 4,
            "channels": 2, "duration_ms": 0.005,
            "trigger_index": 2, "overrun": true,
            "device_tick_us": 12345, "wall_time": 1_700_000_000,
            "expected_samples": 4096,
        })
    }

    #[test]
    fn a_rebuilt_capture_matches_every_field() {
        let cap = rebuild_capture(&meta(3), vec![vec![1, 2, 3, 4], vec![5, 6, 7, 8]]).unwrap();

        assert_eq!(cap.id, 3);
        assert_eq!(cap.rate_hz, 857_142);
        assert_eq!(cap.channels.len(), 2);
        assert_eq!(cap.channels[0], vec![1, 2, 3, 4]);
        assert_eq!(cap.trigger_index, Some(2));
        assert_eq!(cap.device_tick_us, 12345);
        assert_eq!(cap.wall_time, 1_700_000_000);
        assert!(cap.overrun);
        // ⚠ 这一条最要紧：拿不到它，report.rs 的缺口告警就永久静音了
        assert_eq!(
            cap.expected_samples, 4096,
            "expected_samples 必须是设备报的期望值，不是实际长度"
        );
        assert_ne!(
            cap.expected_samples,
            cap.channels[0].len() as u32,
            "测试数据要能区分「期望」与「实际」—— 否则这条断言是空的"
        );
    }

    #[test]
    fn missing_metadata_is_reported_not_defaulted() {
        let mut m = meta(1);
        m.as_object_mut().unwrap().remove("expected_samples");
        let e = rebuild_capture(&m, vec![vec![1]]).unwrap_err();
        assert!(e.message.contains("expected_samples"), "{}", e.message);
        assert!(!e.hint.trim().is_empty());
    }

    #[test]
    fn trigger_index_none_round_trips() {
        let mut m = meta(1);
        m["trigger_index"] = Value::Null;
        assert_eq!(
            rebuild_capture(&m, vec![vec![1]]).unwrap().trigger_index,
            None
        );
    }

    // ── 驱动会话的系统提示 ───────────────────────────────────────────

    /// **驱动会话必须告诉模型界面上的通道编号。**
    ///
    /// 回归：这条路第一版**根本没发 system prompt**。模型只能看到工具返回里的
    /// `channel: 0` / `scl_channel: 0`（接口的内部编号），于是把它原样写进回答 ——
    /// 用户看到「CH0 = SCL」，**对不上屏幕上那两条泳道（CH1 / CH2）**。
    ///
    /// 这不是模型算错：界面用 1 起、接口用 0 起，**而没人告诉过它这件事**。
    #[test]
    fn the_drive_prompt_teaches_the_display_channel_numbering() {
        let p = drive_system_prompt();

        assert!(
            p.contains("CH1") && p.contains("CH2"),
            "必须点名界面上用的两个编号：{p}"
        );
        assert!(
            p.contains("0 起"),
            "必须说清工具返回里的编号是从 0 起的：{p}"
        );
        assert!(p.contains("必须换算"), "必须明确要求换算：{p}");

        // 单发那份纪律也要带上 —— 不然这条路会退化成「只报数字」
        assert!(p.contains("不要复述"), "应复用单发那份的反复述纪律：{p}");
        assert!(p.contains("模拟器"), "应复用「数据源须声明」那条：{p}");
    }

    // ── 工具表转换 ───────────────────────────────────────────────────

    #[test]
    fn tools_list_is_renamed_not_reshaped() {
        let resp = json!({
            "tools": [{
                "name": "scope_capture",
                "description": "采集一次",
                "inputSchema": { "type": "object", "properties": { "count": { "type": "integer" } } },
                // 真实响应里现在带这个字段 —— 放进输入，才能验它不被转发
                "annotations": { "readOnlyHint": false, "destructiveHint": false },
            }]
        });
        let out = to_llm_tools(&resp);
        let t = &out.as_array().unwrap()[0];

        assert_eq!(t["name"], "scope_capture");
        assert_eq!(t["description"], "采集一次");
        assert_eq!(t["input_schema"]["type"], "object");
        assert_eq!(
            t["input_schema"]["properties"]["count"]["type"], "integer",
            "只改字段名，schema 内容必须原样"
        );
        assert!(t.get("inputSchema").is_none(), "旧名字不该留");
        // 这条是**防回归**，不是防手滑：annotations 一旦漏进请求体，
        // Anthropic API 会用「不认识的键」把整个请求打回，而不是忽略它。
        assert!(
            t.get("annotations").is_none(),
            "annotations 是给 MCP 宿主看的，不能进 LLM 请求体"
        );
    }

    #[test]
    fn an_empty_tool_list_does_not_panic() {
        assert_eq!(to_llm_tools(&json!({})).as_array().unwrap().len(), 0);
        assert_eq!(
            to_llm_tools(&json!({"tools": []}))
                .as_array()
                .unwrap()
                .len(),
            0
        );
    }

    // ── 工具返回的解析 ───────────────────────────────────────────────

    #[test]
    fn tool_text_joins_text_blocks_only() {
        let r = json!({ "content": [
            {"type": "thinking", "thinking": "内部推理"},
            {"type": "text", "text": "第一段"},
            {"type": "text", "text": "第二段"},
        ]});
        let t = tool_text(&r);
        assert!(t.contains("第一段") && t.contains("第二段"));
        assert!(!t.contains("内部推理"), "thinking 不该混进来");
    }

    #[test]
    fn samples_are_clamped_to_12_bits() {
        let text = r#"{"samples":[0, 4095, 99999]}"#;
        assert_eq!(read_samples(text), vec![0, 4095, 4095]);
    }

    #[test]
    fn unparsable_samples_give_an_empty_vec_not_a_panic() {
        assert!(read_samples("不是 JSON").is_empty());
        assert!(read_samples("{}").is_empty());
        assert!(read_samples(r#"{"samples":"字符串"}"#).is_empty());
    }

    // ── 参数摘要 ─────────────────────────────────────────────────────

    #[test]
    fn arg_summary_is_bounded() {
        let big = json!({ "k": "x".repeat(500) });
        assert!(summarize_args(&big).chars().count() <= 121);
        assert_eq!(summarize_args(&json!({"a":1})), r#"{"a":1}"#);
    }

    // ── 与真子进程对接的两条，默认不跑 ───────────────────────────────

    /// **交接的往返：设备让出去，再完好地收回来。**
    ///
    /// 这条验的是路线 B 的**核心机制** ——
    ///
    /// ```text
    /// ① GUI 侧本来能采
    /// ② 让出去（断开）
    /// ③ 子进程接同一目标、采一窗
    /// ④ 子进程收场
    /// ⑤ GUI 侧重新连上、还能采
    /// ```
    ///
    /// ⑤ 是重点：**交接失败最坏的形态不是报错，而是「设备回不来了」**。
    ///
    /// ⚠ 在**模拟器**上验，说明力有限：模拟器没有「端口独占」这回事，
    /// 两个进程各建一台本来就互不干扰。所以这条能证明的是
    /// 「子进程起得来、收得干净、收完之后 GUI 侧的连接与采集照常」；
    /// **真正的串口独占要靠真机验**（见 HANDOFF 的未验证项）。
    ///
    /// ```text
    /// ./host/run.sh test -p scope-gui -- --ignored handoff_round_trip --nocapture
    /// ```
    #[test]
    #[ignore = "要起 scope-mcp 子进程；手动跑"]
    fn handoff_round_trip_gives_the_device_back() {
        use scope_core::{AcquireParams, CommandBus};
        use scope_device::Transport;

        let connect = || {
            let mut bus = CommandBus::new(Transport::sim(scope_sim::Scenario::I2c100k));
            bus.connect().expect("连模拟器失败");
            bus
        };
        let capture = |bus: &mut CommandBus<Transport>| {
            scope_core::acquire(
                bus,
                &AcquireParams {
                    samples: 4096,
                    rate_hz: scope_core::f103::MAX_SAMPLE_RATE_HZ,
                    trigger_level_lsb: 2048,
                    timeout: Duration::from_millis(2000),
                },
            )
            .expect("采集失败")
        };

        // ── ① GUI 侧本来是好的 ──
        let mut bus = connect();
        let before = capture(&mut bus);
        println!(
            "交出去之前：采集 #{} · {} 点",
            before.id,
            before.channels[0].len()
        );
        assert_eq!(before.channels[0].len(), 4096);

        // ── ② 让出去 ──
        drop(bus);

        // ── ③ 子进程接管同一目标 ──
        let exe = find_scope_mcp().expect("找不到 scope-mcp —— 先 ./host/run.sh build");
        let mut mcp = McpProcess::spawn(&exe).expect("起不了子进程");
        mcp.call(
            "initialize",
            json!({"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "test", "version": "0"}}),
        )
        .expect("握手失败");

        let tool = |mcp: &mut McpProcess, name: &str, args: Value| {
            let r = mcp
                .call("tools/call", json!({"name": name, "arguments": args}))
                .unwrap_or_else(|e| panic!("{name} 传输失败：{}｜{}", e.message, e.hint));
            assert!(
                !r.get("isError").and_then(|v| v.as_bool()).unwrap_or(false),
                "{name} 被拒：{}",
                tool_text(&r)
            );
            r
        };

        tool(
            &mut mcp,
            "scope_connect",
            json!({"transport": "sim", "sim_scenario": "i2c_100k"}),
        );
        tool(&mut mcp, "scope_capture", json!({}));
        let mut mirrored = Mirrored::default();
        let got = mirrored.poll(&mut mcp).expect("镜像失败");
        assert_eq!(got.len(), 1, "子进程那一侧应当采到一窗");
        println!(
            "AI 接管期间：采集 #{} · {} 点",
            got[0].id,
            got[0].channels[0].len()
        );

        // ── ④ 收场，并确认子进程**真的被回收了** ──
        let killer = mcp.child_handle();
        mcp.shutdown();
        let reaped = killer.lock().unwrap().try_wait().expect("try_wait 出错");
        assert!(
            reaped.is_some(),
            "子进程没有被回收 —— 这就是「关窗留下孤儿占着 COM7」那个形态"
        );
        println!(
            "子进程已回收（退出码 {}）",
            reaped.unwrap().code().unwrap_or(-1)
        );

        // ── ⑤ 设备收回来，照常能采 ──
        let mut bus = connect();
        let after = capture(&mut bus);
        println!(
            "收回来之后：采集 #{} · {} 点",
            after.id,
            after.channels[0].len()
        );
        assert_eq!(
            after.channels[0].len(),
            4096,
            "交接之后设备必须还能正常采集"
        );
        assert!(
            after.channels[0].iter().any(|&v| v > 2048),
            "收回来之后采到的波形不该是空的"
        );
    }

    // ── 与真子进程对接的那一条，默认不跑 ─────────────────────────────

    /// **镜像这条路通不通 —— 对着真的 `scope-mcp` 子进程验，但不碰 LLM。**
    ///
    /// 这条覆盖的是「AI 采了一窗，GUI 自己把它拉回来画出来」：
    /// 起子进程 → 连模拟器 → 采一窗 → [`Mirrored::poll`] 重建 `Capture`。
    ///
    /// 为什么值得单独有一条：真实执行一次完整会话要花费 LLM 的费用，而镜像**不经过
    /// LLM** —— 把它单独拎出来验，既便宜又能定位问题（会话没出图时，
    /// 一眼就能分清是镜像失效，还是模型压根未读取样点）。
    ///
    /// ```text
    /// ./host/run.sh test -p scope-gui -- --ignored mirror_pulls_a_capture --nocapture
    /// ```
    #[test]
    #[ignore = "要起 scope-mcp 子进程；手动跑"]
    fn mirror_pulls_a_capture_from_a_real_subprocess() {
        let exe = find_scope_mcp().expect("找不到 scope-mcp —— 先 ./host/run.sh build");
        let mut mcp = McpProcess::spawn(&exe).expect("起不了子进程");

        mcp.call(
            "initialize",
            json!({"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "test", "version": "0"}}),
        )
        .expect("握手失败");

        // ⚠ **必须检查 `isError`**，不能只看协议层有没有报错。
        //
        // 第一版这里只 `unwrap_or_else` 处理传输错误，于是「工具被拒」
        // （例如传了 schema 里没有的参数）与「调用成功」表现完全一样 ——
        // 为此多跑了两轮才看出 store 是空的。
        let tool = |mcp: &mut McpProcess, name: &str, args: Value| {
            let r = mcp
                .call("tools/call", json!({"name": name, "arguments": args}))
                .unwrap_or_else(|e| panic!("{name} 传输失败：{}｜{}", e.message, e.hint));
            if r.get("isError").and_then(|v| v.as_bool()).unwrap_or(false) {
                panic!("{name} 被拒：{}", tool_text(&r));
            }
            r
        };

        tool(
            &mut mcp,
            "scope_connect",
            json!({"transport": "sim", "sim_scenario": "i2c_100k"}),
        );
        // `scope_capture` 的参数只有 mode / timeout_ms / max_preview_points
        // —— 没有 count（那是 `scope_read_waveform` 的）
        tool(&mut mcp, "scope_capture", json!({}));

        // 先看一眼原始返回 —— poll 失败时这是唯一的线索
        let raw = tool(&mut mcp, "scope_list_captures", json!({}));
        let raw_text = tool_text(&raw);
        println!(
            "list_captures 原文（前 400 字）：\n{}",
            raw_text.chars().take(400).collect::<String>()
        );

        let mut mirrored = Mirrored::default();
        let caps = mirrored.poll(&mut mcp).expect("镜像失败");
        assert_eq!(caps.len(), 1, "应当拉回恰好一窗");

        let c = &caps[0];
        println!(
            "镜像回来：采集 #{} · {} 点 × {} 通道 · {} Hz · 触发点 {:?} · 溢出 {}",
            c.id,
            c.channels[0].len(),
            c.channels.len(),
            c.rate_hz,
            c.trigger_index,
            c.overrun
        );

        assert_eq!(c.channels.len(), 2, "i2c_100k 是双通道");
        assert_eq!(c.channels[0].len(), 4096, "应当拿到全窗（上限就是 4096）");
        assert!(c.rate_hz > 0, "采样率不该是 0");
        assert!(
            c.channels[0].iter().any(|&v| v > 2048) && c.channels[0].iter().any(|&v| v <= 2048),
            "样点应当跨越中点 —— 全 0 或全满说明拉空了"
        );
        assert!(
            c.expected_samples > 0,
            "expected_samples 没拿到 = report.rs 的缺口告警会永久静音"
        );

        // 再 poll 一次不该重复拉（同一个编号只镜像一次）
        let again = mirrored.poll(&mut mcp).expect("第二次镜像失败");
        assert!(again.is_empty(), "同一个采集不该被镜像两次");

        mcp.shutdown();
    }
}
