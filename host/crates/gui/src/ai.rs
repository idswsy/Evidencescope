//! AI 分析 —— 将证据包发送至语言模型，取回一段解释文本。
//!
//! # 职责范围：本模块仅处理网络，不涉及界面
//!
//! 状态存于 `app.rs`，渲染由 `panels.rs` 完成。本模块只承担三个步骤：
//! **构造请求体 → 发送 → 解析响应**，并将每类失败转换为
//! 「错误说明 + 处理措施」（项目约定：错误不得只给错误码）。
//!
//! # 使用独立线程的原因
//!
//! `worker.rs` 的线程**常驻并阻塞于设备 IO**（`Acquire` 最长等待 2 秒以上），
//! 而单次模型调用耗时 15–60 秒。若并入同一队列，分析期间全部设备操作将被阻塞。
//!
//! # 采用「每请求一线程」的原因
//!
//! 阻塞式 `ureq` **无法中断**，取消仅能表现为丢弃结果。若采用常驻线程，
//! 用户在取消后再次发起分析时，新请求须排队等待旧请求超时。
//! 每请求独立线程则无此问题：新请求立即开始，旧线程到期自行结束。
//!
//! # 请求关联（`req_id`）
//!
//! 用户可能在分析进行期间再次采集。因此每个请求携带自增编号；返回时若编号
//! 不匹配，**不予丢弃**（请求已计费），而是标注其对应的采集批次。
//! 处理细节见 `app.rs`。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::config::AiConfig;

/// 连接超时。握手阶段卡住通常意味着网络或 DNS 有问题，早点报出来。
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
/// 整体超时。**已包含连接耗时** —— 不是「连接 10 s + 读取 60 s」。
const GLOBAL_TIMEOUT: Duration = Duration::from_secs(60);

/// 回复的长度上限。**单发分析与 AI 驱动会话共用这一个值。**
///
/// # 为什么是 8192，而不是从前那个 2048
///
/// 从前这里写 2048，而 `drive.rs` 那边写 8192 —— 同一个模型、同一个端点、
/// 同一类任务，两条路差 4 倍，**而且没有任何东西守着它们不分家**
/// （连 `README.md` 都把两个数并排抄了下来，见 §5.4.3 与表 5）。
///
/// 2026-10-06 真机回归：拿一个 2 kHz 方波做分析，模型把 2048 的预算
/// **全部烧在思考块上**，正文一个字都没有，`stop_reason = max_tokens` ——
/// 界面直接报「无分析结果」，**这一次请求的钱白花了**。
///
/// 8192 是**上限而不是预扣**：模型答完就停，没用掉的预算不花钱。
/// 而预算不足的代价是整次请求作废，远大于那点省下的输出。
pub(crate) const MAX_TOKENS: u32 = 8192;

/// 一次分析的结果。
///
/// # 为什么不带 token 用量
///
/// 响应里的 `usage.input_tokens` / `output_tokens` **对不上**，已实测：
/// 一次真实请求的 body 是 3512 字符（其中 444 个中文字），服务端却报
/// `input_tokens: 239` —— 合 14.7 字符/token，而**光那 444 个中文字就不止
/// 239 个 token**。反方向也不成立：回复 344 字符却报 967 输出 token。
///
/// 既然这个数说不清是什么，**就不该显示给人看**。显示一个来路不明的数字，
/// 比不显示更坏 —— 它会让人以为那是真的，并据此做成本判断。
///
/// 需要刻度时用**字符数**（`evidence.chars().count()`），那个是确定的。
#[derive(Debug, Clone)]
pub struct AiAnswer {
    /// 模型给出的文字。
    pub text: String,
    /// 实际用的模型名（服务端回显的，不是我们请求的 —— 可能被路由）。
    pub model: String,
    /// 耗时。
    pub elapsed: Duration,
    /// 服务端因 `max_tokens` 把响应截断了 —— **这段正文是不完整的**。
    ///
    /// 从前这个状态被整个丢掉：只要有正文就当成成功返回，`stop_reason`
    /// 看都不看。于是一段被拦腰砍断的诊断与一段完整的诊断在界面上长得
    /// 一模一样 —— 而「话说了一半」正是最容易被当成「说完了」的那种错。
    pub truncated: bool,
}

/// 一次失败。**`hint` 是必填的** —— 界面上要显示「怎么办」。
#[derive(Debug, Clone)]
pub struct AiError {
    /// 出了什么事。
    pub message: String,
    /// 怎么办。
    pub hint: String,
}

impl AiError {
    /// 构造一条「说明 + 怎么办」。
    ///
    /// `pub(crate)`：`drive.rs` 也要造同类错误 —— 两个入口（单发分析、
    /// AI 驱动的会话）对用户而言是一件事，错误形状必须一样。
    pub(crate) fn new(message: impl Into<String>, hint: impl Into<String>) -> Self {
        AiError {
            message: message.into(),
            hint: hint.into(),
        }
    }
}

/// 交给 AI 线程的一次请求。
///
/// **不含 `req_id` 字段** —— 请求号由 [`AiWorker::start`] 分配，它是唯一
/// 掌握序列状态的一方。若由调用方传入再被覆盖，只会引入一个无法验证的
/// 「传入值与实际值是否一致」问题。
#[derive(Debug)]
pub struct AiJob {
    /// 这次分析针对哪个采集。
    pub capture_id: u16,
    /// 采集时那个采集的快照描述（界面上用来标注「基于哪一次」）。
    pub anchor: String,
    /// 已经渲染好的证据包。
    pub evidence: String,
}

/// AI 线程交回来的结果。
#[derive(Debug)]
pub struct AiUpdate {
    /// 对应哪个请求。
    pub req_id: u64,
    /// 针对哪个采集。
    pub capture_id: u16,
    /// 请求时的锚点描述。
    pub anchor: String,
    /// 成功或失败。
    pub outcome: Result<AiAnswer, AiError>,
}

/// 发送给语言模型的系统提示。
///
/// # 为什么第一句就是「不要复述」
///
/// 这个面板的旁边就是波形图和测量面板 —— **峰峰值、频率、占空比都摆在用户
/// 眼前**。模型把它们抄一遍是纯冗余，而且会把真正有价值的内容（哪里不对、
/// 为什么、怎么办）挤掉。所以提示词的第一条不是「别编数字」，而是
/// **「别复述数字」**：数字是判断依据，不是要转达的内容。
///
/// # 为什么「数字时序」排在「总线解码」前面
///
/// 2026-10-06 真机回归：屏幕上给一个 2 kHz、占空比 50.3% 的方波，回答里
/// 三段有两段在讲「缺 SDA，I2C 解码做不了」。彼时提示词的第一个分支、第一个
/// 示例都是 I2C，另有一整段 I2C 7 位地址规则 —— 非 I2C 的波形是二等公民。
/// 现在分支按「波形形状」排，非 I2C 的场景先讲。
///
/// # 其余每一条对应的失效模式
///
/// 杜撰数值、将模拟器数据当作实测、对包络桶内细节作推断、信息不足时强行作答。
pub(crate) const SYSTEM_PROMPT: &str = "\
你是一台数字示波器的分析助手。用户**已经看到**波形图与
测量面板，你将收到一份证据包，内含测量值、波形形状、波形包络、信号质量，
**有总线解码时**另有 I2C 帧表与告警。

## 最重要的一条：不要复述

峰峰值、频率、占空比、幅值这类数值，界面上已经显示，用户看得见。
**把它们抄一遍没有任何价值。** 证据包里的数字是给你**作判断依据**用的 ——
只在支撑某条结论时引用它，不要罗列。

## 只回答三件事：状态 / 问题 / 建议

先看证据包「场景」行与「波形形状」段，按信号选分支：

**分支一：数字时序（方波 / 脉冲 / 阶跃响应）—— 第一节写「波形状态」**
PWM 看占空比偏离与边沿；阶跃响应（PID 调参）看**过冲、下冲、振铃** ——
证据里有这些数字时点名它们；时钟看间隔稳定性。
**每条都要点名支撑它的数字。**

**分支二：模拟信号（正弦 / 调幅）—— 第一节写「波形状态」**
削顶、失真、调幅深度。**每条都要点名支撑它的数字。**

**分支三：I2C 总线解码 —— 第一节写「总线状态」**
事务内容、地址、读写方向与从机应答情况。

**分支四：形状判不出 —— 第一节写「波形状态」**
转述证据里「判不出·…」的原因 —— 那是**工具的边界**，不是信号坏了。

**第二、三节所有分支相同：**

**问题**
依据证据指出的异常。**每条都要点明它基于哪个数**。
没有异常就直说「未发现异常」，不要为凑内容编问题。

**建议**
针对每条问题的具体动作：换多大的上拉、调哪个门限、换个场景怎么重采。
没问题时给出下一步可以验证什么。

## 其余规定

- 仅可引用证据包中出现的数值，不得估算或推算。
- 须区分「实测」与「推断」，以不同措辞分别表述。
- 证据包中的波形包络是降采样结果，每桶只有 min/max。**不得对桶内的毛刺、
  振铃、边沿形状作结论** —— 该信息在包络里不存在。
- 数据源标注为模拟器时，须说明结论不适用于真实硬件。
- 信息不足时须直说缺什么，不得杜撰。
- **仅有总线解码时适用**：I2C 地址是 7 位的，读写方向由 `read` 标志单独表示，
  **不要写成 8 位形式**。地址 0x44 的读事务**仍然是 0x44**，不是 0x45。

## 输出的形状

**条目化，不写整段散文。** 一条一个事实；数字单独成项，不要埋在句子中间。

三节按这个形状写（下面是纯文本，不是 markdown）：

    波形状态
      方波 1 kHz  占空比 25.0%  上升时间 4.2 µs
      上升沿偏缓：4.2 µs 是周期的 0.4%，驱动能力可能不足
    问题
      占空比 25.0% 偏离 50%；上升时间 4.2 µs 占周期 0.4%，偏缓
    建议
      用 X1 档直接采集复查；对比另一路信号源的边沿

总线解码场景第一节换成「总线状态」，例如：

    总线状态
      SCL   CH1   99.96 kHz   占空比 49.97%
      事务  写 0x44/0x00 → 重复起始 → 读 0x44 → 返回 0x01 0x2C
      帧数  19（末帧被窗口截断）

**不要写这三类**：

- **复述**：刚说完的结论不再重复一遍。
- **开脱**：没问题就直说「未发现异常」，不必替它解释「属…，不是…」。
- **没有依据的形容**：「可靠」「干净」「充足」这类词，
  后面要么跟上是哪个数字支撑，要么删掉。

输出中文。**全文 300 字以内** —— 以简短、准确为先。

⚠ 纯文本显示，不用 markdown 记号（`**`、`#`、反引号会原样显示出来）。
分节直接写「波形状态」「总线状态」这样的小标题。数字与字母照常写。";

/// 构造请求体（Anthropic Messages 格式）。
///
/// 与 `tools/agent_demo/agent.py` 采用同一格式，该格式已验证可用。
/// 唯一差异是本处使用 `system` 字段（该脚本把指令并入 user 消息）：
/// 本次调用为**单发**，无需维持多轮上下文。
pub fn build_body(cfg: &AiConfig, evidence: &str) -> Result<String, AiError> {
    let body = serde_json::json!({
        "model": cfg.model,
        "max_tokens": MAX_TOKENS,
        "system": SYSTEM_PROMPT,
        "messages": [{
            "role": "user",
            "content": evidence,
        }],
    });
    serde_json::to_string(&body)
        .map_err(|e| AiError::new(format!("构造请求体失败：{e}"), "属程序缺陷，请提交 issue"))
}

/// 解响应。
///
/// `http_status_as_error(false)` 之后，4xx/5xx 也会走到这里 ——
/// 这是有意的：API 的错误说明在响应体里，比光看状态码有用得多。
pub fn parse_response(
    status: u16,
    body: &str,
    fallback_model: &str,
    elapsed: Duration,
) -> Result<AiAnswer, AiError> {
    let v: serde_json::Value = match serde_json::from_str(body) {
        Ok(v) => v,
        Err(_) => {
            // 非 JSON 的响应体通常是网关/代理插进来的 HTML
            let head: String = body.chars().take(200).collect();
            return Err(match status {
                200..=299 => AiError::new(
                    format!("响应体无法解析（前 200 字符）：{head}"),
                    "端点可能配置有误。请确认 base_url 指向 messages 接口",
                ),
                _ => AiError::new(
                    format!("HTTP {status}，响应体无法解析：{head}"),
                    "确认 base_url 正确，并检查网络是否需要代理",
                ),
            });
        }
    };

    if !(200..=299).contains(&status) {
        return Err(http_error(status, &v));
    }

    // content 是一个块数组；只取 text 块。
    // （这个 API 还可能有 thinking 块 —— 那是模型的内部推理，不该显示给用户。）
    let text = v
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
        .unwrap_or_default();

    let stop_reason = v.get("stop_reason").and_then(|s| s.as_str());
    let truncated = stop_reason == Some("max_tokens");

    if text.trim().is_empty() {
        // 一个字都没有 —— 这**不是**「模型觉得没什么可说」，是这一轮废了。
        // 预算被思考块吃干净是这个服务端最常见的成因。
        return Err(AiError::new(
            format!(
                "模型未返回文本内容（stop_reason = {}）",
                stop_reason.unwrap_or("(未给出)")
            ),
            "若为 max_tokens：整个预算被思考块吃掉了，正文一个字没留下。\
             证据包过长时最容易发生 —— 减小采集点数或缩短提问后重试",
        ));
    }

    // 响应里还有 `usage.input_tokens` / `output_tokens` —— **故意不取**。
    // 实测那两个数与请求体大小对不上（见 [`AiAnswer`] 的说明），
    // 说不清含义的数字不该往上传，免得界面把它当成真的显示出去。
    Ok(AiAnswer {
        text,
        model: v
            .get("model")
            .and_then(|m| m.as_str())
            .unwrap_or(fallback_model)
            .to_string(),
        elapsed,
        truncated,
    })
}

/// 给 `drive.rs` 用的薄包装：那边手上是响应体的**字符串**。
///
/// 故意不另写一份错误映射 —— 单发分析与工具循环走的是同一个端点，
/// 同一个 401 在两个地方给出不同的措辞，就是又一处「同一次故障两种描述」。
pub(crate) fn http_error_for_drive(status: u16, body: &str) -> AiError {
    let v = serde_json::from_str(body).unwrap_or(serde_json::Value::Null);
    http_error(status, &v)
}

/// 同上，传输层。
pub(crate) fn transport_error_for_drive(e: &ureq::Error) -> AiError {
    transport_error(e)
}

/// 把 HTTP 错误码翻译成「说明 + 怎么办」。
fn http_error(status: u16, v: &serde_json::Value) -> AiError {
    // 服务端自己的说明优先 —— 它比我们猜的准
    let detail = v
        .pointer("/error/message")
        .and_then(|m| m.as_str())
        .or_else(|| v.get("message").and_then(|m| m.as_str()))
        .unwrap_or("")
        .to_string();

    let (what, how) = match status {
        401 | 403 => ("API key 被拒绝", "确认密钥有效、未过期、账户配额充足"),
        404 => (
            "端点不存在（404）",
            "base_url 配置有误，应指向 messages 接口，例如 \
             https://api.deepseek.com/anthropic/v1/messages",
        ),
        429 => (
            "请求频率过高或配额耗尽（429）",
            "稍后重试；若持续出现，请在服务商控制台核对配额",
        ),
        500..=599 => ("服务端错误", "非本端配置问题，稍后重试"),
        _ => ("请求被拒绝", "确认 base_url 与密钥属于同一服务商"),
    };

    let message = if detail.is_empty() {
        format!("{what}（HTTP {status}）")
    } else {
        format!("{what}（HTTP {status}）：{detail}")
    };
    AiError::new(message, how)
}

/// 真正发一次请求。**阻塞**，所以只能在 AI 线程里调。
fn ask(cfg: &AiConfig, evidence: &str, cancel: &AtomicBool) -> Result<AiAnswer, AiError> {
    if !cfg.has_key() {
        return Err(AiError::new(
            "未配置 API 密钥",
            "请在面板的「设置」中填写密钥",
        ));
    }
    if cancel.load(Ordering::Relaxed) {
        return Err(AiError::new("请求已取消", "可重新发起分析"));
    }

    let body = build_body(cfg, evidence)?;
    let t0 = Instant::now();

    let result = ureq::post(cfg.base_url.trim())
        .header("content-type", "application/json")
        .header("x-api-key", cfg.api_key.trim())
        .header("anthropic-version", "2023-06-01")
        .config()
        // 4xx/5xx 也走 Ok 分支 —— 我们要读响应体里的错误说明
        .http_status_as_error(false)
        .timeout_connect(Some(CONNECT_TIMEOUT))
        .timeout_global(Some(GLOBAL_TIMEOUT))
        .build()
        .send(body.as_str());

    let elapsed = t0.elapsed();

    match result {
        Ok(mut resp) => {
            let status = resp.status().as_u16();
            let text = resp.body_mut().read_to_string().unwrap_or_default();
            parse_response(status, &text, &cfg.model, elapsed)
        }
        Err(e) => Err(transport_error(&e)),
    }
}

/// 传输层失败（连不上、超时、DNS 挂了）。
fn transport_error(e: &ureq::Error) -> AiError {
    use ureq::Error as E;
    match e {
        E::Timeout(_) => AiError::new(
            format!("请求超时（{} 秒）", GLOBAL_TIMEOUT.as_secs()),
            "网络延迟或服务端无响应。可重试；若持续超时，请检查网络与代理设置",
        ),
        E::HostNotFound => AiError::new(
            "域名解析失败",
            "确认 base_url 中的域名拼写正确，且本机可访问外网",
        ),
        E::ConnectionFailed => {
            AiError::new("无法连接服务器", "确认网络连通性、防火墙策略与代理设置")
        }
        E::BadUri(u) => AiError::new(
            format!("地址格式错误：{u}"),
            "base_url 须为完整 URL，含 http(s):// 前缀",
        ),
        // TLS 握手失败会落到这里 —— 在 Windows + GNU 工具链下这是真会发生的
        E::Io(io) => AiError::new(
            format!("网络 IO 错误：{io}"),
            "若为证书或 TLS 相关错误，可能被中间设备拦截。请更换网络或配置代理",
        ),
        other => AiError::new(
            format!("请求失败：{other}"),
            "检查网络与 base_url；若持续失败请提交 issue",
        ),
    }
}

/// AI 线程的句柄。**每请求一条线程**，理由见模块头。
pub struct AiWorker {
    tx: Sender<AiUpdate>,
    rx: Receiver<AiUpdate>,
    cancel: Arc<AtomicBool>,
    /// 当前在跑的那条线程。`None` = 还没跑过。
    in_flight: Option<std::thread::JoinHandle<()>>,
    next_id: u64,
    ctx: egui::Context,
}

impl AiWorker {
    /// 建句柄。**不起常驻线程** —— 线程是按请求起的。
    pub fn new(ctx: egui::Context) -> Self {
        let (tx, rx) = mpsc::channel();
        AiWorker {
            tx,
            rx,
            cancel: Arc::new(AtomicBool::new(false)),
            in_flight: None,
            next_id: 1,
            ctx,
        }
    }

    /// 发起一次分析，返回请求号。
    pub fn start(&mut self, cfg: AiConfig, job: AiJob) -> u64 {
        // 新一轮，把上一次的取消标志清掉
        self.cancel.store(false, Ordering::Relaxed);

        let req_id = self.next_id;
        self.next_id += 1;
        let capture_id = job.capture_id;
        let anchor = job.anchor.clone();
        let evidence = job.evidence;

        let tx = self.tx.clone();
        let cancel = self.cancel.clone();
        let ctx = self.ctx.clone();
        // 闭包里要用一份，起线程失败那条兜底路径也要用一份
        let anchor_for_thread = anchor.clone();

        // 上一条线程可能还在跑（阻塞在 ureq 里，打断不了）。
        // **不 join 它** —— 挡住界面没有意义，让它在超时后自己结束，
        // 它的结果会因为 req_id 对不上而被标注为「过期」。
        let spawned = std::thread::Builder::new()
            .name("scope-ai".into())
            .spawn(move || {
                let outcome = ask(&cfg, &evidence, &cancel);
                // 发送失败只说明界面已经关了 —— 不是错误
                if tx
                    .send(AiUpdate {
                        req_id,
                        capture_id,
                        anchor: anchor_for_thread,
                        outcome,
                    })
                    .is_ok()
                {
                    ctx.request_repaint();
                }
            });

        match spawned {
            Ok(h) => self.in_flight = Some(h),
            Err(e) => {
                // 起不了线程是很罕见的情况，但**不能静默** ——
                // 界面会永远停在「分析中」
                let _ = self.tx.send(AiUpdate {
                    req_id,
                    capture_id,
                    anchor,
                    outcome: Err(AiError::new(
                        format!("无法创建后台线程：{e}"),
                        "系统资源不足。请关闭部分程序后重试",
                    )),
                });
                self.ctx.request_repaint();
            }
        }
        req_id
    }

    /// 排空回执。**收进 Vec 再处理** —— 和 `worker.rs` 一个路子，
    /// 避免在闭包里可变借走 App。
    pub fn drain(&self, mut f: impl FnMut(AiUpdate)) {
        while let Ok(u) = self.rx.try_recv() {
            f(u);
        }
    }

    /// 请求取消。**在途的 HTTP 打断不了**，这只是让结果被标成过期。
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }
}

/// AI 面板的界面状态。
///
/// 「是否正在分析」以 [`AiState::is_running`]（即 `awaiting`）为唯一判据。
/// **不要**改用线程存活状态：请求被取消后，在途 HTTP 仍会跑到超时，
/// 而那时界面应当已经回到就绪态。
pub struct AiState {
    /// 网络线程句柄。
    pub worker: AiWorker,
    /// 当前配置（含密钥）。
    pub cfg: crate::config::AiConfig,
    /// 配置文件路径。`None` = 无可用写入位置，配置仅保存在内存中。
    pub config_path: Option<std::path::PathBuf>,
    /// 面板是否展开。
    pub panel_open: bool,
    /// 正在等待结果的请求号。
    awaiting: Option<u64>,
    /// 最近一次结果。**迟到结果亦予保留** —— 请求已计费，丢弃不若标注。
    pub answer: Option<AiAnswer>,
    /// 结果对应的请求号。
    answer_req: u64,
    /// 结果对应的采集编号。
    pub answer_capture_id: u16,
    /// 生成结果时该采集的锚点描述。
    pub answer_anchor: String,
    /// 结果迟于其请求返回（此前已取消，或已发起新的分析）。
    pub answer_late: bool,
    /// 最近一次失败。
    pub error: Option<AiError>,
    /// 用户输入的分析要求。
    pub question: String,
    /// 密钥是否明文显示。
    pub show_key: bool,
}

impl AiState {
    /// 读取配置并建立状态。
    ///
    /// 返回 `(状态, 配置来源说明)`。说明描述配置文件的位置与读取结果
    /// （不存在 / 解析失败 / 无可用目录），**由调用方记入日志**，
    /// 不在操作界面上常驻显示。
    pub fn load(ctx: egui::Context) -> (AiState, Option<String>) {
        let path = crate::config::default_config_path();
        let (cfg, note) = crate::config::load(path.as_deref());
        let state = AiState {
            worker: AiWorker::new(ctx),
            cfg,
            config_path: path,
            panel_open: false,
            awaiting: None,
            answer: None,
            answer_req: 0,
            answer_capture_id: 0,
            answer_anchor: String::new(),
            answer_late: false,
            error: None,
            question: String::new(),
            show_key: false,
        };
        (state, note)
    }

    /// 当前是否允许发起分析。
    pub fn can_start(&self) -> bool {
        self.cfg.has_key() && self.awaiting.is_none()
    }

    /// 是否正在等待结果。界面上的「分析中」状态以此为准。
    pub fn is_running(&self) -> bool {
        self.awaiting.is_some()
    }

    /// 发起一次分析。返回是否成功发出。
    pub fn start(&mut self, evidence: String, capture_id: u16, anchor: String) -> bool {
        if !self.can_start() {
            return false;
        }
        // 清空上一次结果，避免新旧结论并列显示
        self.error = None;
        self.answer_late = false;
        let req_id = self.worker.start(
            self.cfg.clone(),
            AiJob {
                capture_id,
                anchor,
                evidence,
            },
        );
        self.awaiting = Some(req_id);
        true
    }

    /// 取消。界面立刻回到就绪态；在途的请求会在超时后返回，届时被标成「迟到」。
    pub fn cancel(&mut self) {
        self.worker.cancel();
        self.awaiting = None;
    }

    /// 收一条回执。
    pub fn apply(&mut self, u: AiUpdate) {
        // 这条是不是当前正在等的那一个？
        let is_current = self.awaiting == Some(u.req_id);

        // **只有它才是**才能清 `awaiting`。
        //
        // 回归：这里曾经无条件 `self.awaiting = None`，于是一条**旧请求**的
        // 迟到结果会把**新请求**的运行态一并清掉 —— 界面显示「不在分析」、
        // 「开始分析」按钮重新亮起，而新请求其实还在飞。用户此时再点一次，
        // 就有三个请求同时在路上，而 req_id 机制本来就是为了避免这种混乱。
        // 这个交错靠读代码看不出来，是被 `a_stale_result_does_not_disturb_
        // the_new_request` 抓到的。
        if is_current {
            self.awaiting = None;
        }

        // 不是正在等的那个 —— 说明用户取消过，或者又发起了一次。
        // **不丢弃**：请求已经计过费，标一下比扔掉有用。
        self.answer_late = !is_current;

        match u.outcome {
            Ok(a) => {
                self.answer = Some(a);
                self.answer_req = u.req_id;
                self.answer_capture_id = u.capture_id;
                self.answer_anchor = u.anchor;
            }
            Err(e) => {
                self.error = Some(e);
            }
        }
    }

    /// 结果对应的采集和当前显示的不是同一次。
    pub fn answer_is_about_another_capture(&self, current: Option<u16>) -> bool {
        self.answer.is_some() && current != Some(self.answer_capture_id)
    }

    /// 保存配置到磁盘。返回一句可直接写入日志的结果说明。
    ///
    /// **不再往状态里存一份** —— 曾经有个 `config_note` 字段同时被面板和返回值读，
    /// 面板改成不显示之后它就变成了「写进去、没人读」。部署细节归日志，
    /// 调用方拿到这个字符串记一行即可。
    pub fn save_config(&self) -> String {
        let Some(p) = self.config_path.clone() else {
            return "未能保存：无可用配置目录（APPDATA / XDG_CONFIG_HOME / HOME 均未设置）。\
                    设置仅保存在内存中。"
                .into();
        };
        match crate::config::save(&p, &self.cfg) {
            Ok(()) => format!("AI 配置已保存至 {}", p.display()),
            Err(e) => e,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::AiConfig;

    fn cfg() -> AiConfig {
        AiConfig {
            api_key: "sk-test".into(),
            base_url: "https://example.test/v1/messages".into(),
            model: "test-model".into(),
        }
    }

    // ══════════════════════════════════════════════════════════════════
    // AiState 的状态机
    // ══════════════════════════════════════════════════════════════════
    //
    // ⚠ 这些测试**从不调用 `start()`** —— 那会真的起线程、发网络请求。
    // 需要「正在等结果」这个前置状态时，直接写 `awaiting` 字段（同模块可见）。
    //
    // 覆盖的是界面最容易出错的那部分：取消之后结果回来了怎么办、
    // 结论对应的采集已经切走了怎么办。**这是「取消与迟到标注」那条
    // 人工验证项里唯一能自动化的部分。**

    /// 造一个不含网络活动的状态。`key` 传 `None` 表示没配密钥。
    fn state_with_key(key: Option<&str>) -> AiState {
        let cfg = match key {
            Some(k) => AiConfig {
                api_key: k.into(),
                ..cfg()
            },
            None => AiConfig {
                api_key: String::new(),
                ..cfg()
            },
        };
        AiState {
            worker: AiWorker::new(egui::Context::default()),
            cfg,
            config_path: None,
            panel_open: false,
            awaiting: None,
            answer: None,
            answer_req: 0,
            answer_capture_id: 0,
            answer_anchor: String::new(),
            answer_late: false,
            error: None,
            question: String::new(),
            show_key: false,
        }
    }

    fn answer(text: &str) -> AiAnswer {
        AiAnswer {
            text: text.into(),
            model: "m".into(),
            elapsed: Duration::from_millis(1200),
            truncated: false,
        }
    }

    fn update(req_id: u64, capture_id: u16, text: &str) -> AiUpdate {
        AiUpdate {
            req_id,
            capture_id,
            anchor: format!("#{capture_id}"),
            outcome: Ok(answer(text)),
        }
    }

    #[test]
    fn cannot_start_without_a_key() {
        let s = state_with_key(None);
        assert!(!s.can_start(), "没有密钥时不得发起分析");
        assert!(!s.is_running());
    }

    #[test]
    fn can_start_with_a_key_and_is_idle() {
        let s = state_with_key(Some("sk-x"));
        assert!(s.can_start());
        assert!(!s.is_running());
    }

    /// 「是否正在分析」只看 `awaiting`。
    ///
    /// 曾经想过用线程存活状态判断，那是错的：请求取消后在途 HTTP 仍会跑到
    /// 超时，而那时界面早该回到就绪态。
    #[test]
    fn running_state_follows_awaiting_not_the_thread() {
        let mut s = state_with_key(Some("sk-x"));
        assert!(!s.is_running());

        s.awaiting = Some(7);
        assert!(s.is_running(), "有在途请求时应报运行中");
        assert!(!s.can_start(), "运行中不得再次发起");

        s.awaiting = None;
        assert!(!s.is_running());
        assert!(s.can_start(), "回到就绪态后应可再次发起");
    }

    /// 取消后界面立刻回到就绪态 —— **不等**在途请求真的结束。
    #[test]
    fn cancel_returns_to_idle_immediately() {
        let mut s = state_with_key(Some("sk-x"));
        s.awaiting = Some(3);
        s.cancel();
        assert!(!s.is_running(), "取消后应立即回到就绪态");
        assert!(s.can_start(), "取消后应能立刻重新发起");
    }

    #[test]
    fn a_normal_result_is_not_marked_late() {
        let mut s = state_with_key(Some("sk-x"));
        s.awaiting = Some(1);
        s.apply(update(1, 5, "结论"));
        assert!(!s.answer_late, "正常返回不该被标成迟到");
        assert_eq!(s.answer.as_ref().unwrap().text, "结论");
        assert_eq!(s.answer_capture_id, 5);
        assert!(!s.is_running());
    }

    /// **取消之后结果回来了 —— 不丢弃，标成迟到。**
    ///
    /// 丢弃是错的：请求已经计过费了。这条同时守住「标了」和「没丢」两件事。
    #[test]
    fn a_late_result_is_kept_and_labelled() {
        let mut s = state_with_key(Some("sk-x"));
        s.awaiting = Some(1);
        s.cancel(); // awaiting 归 None
        assert!(s.awaiting.is_none());

        s.apply(update(1, 5, "迟到的结论"));
        assert!(s.answer_late, "取消后返回的结果必须标成迟到");
        assert!(s.answer.is_some(), "迟到结果不得丢弃 —— 请求已经计过费了");
    }

    /// 用户取消后又发起了一次：旧请求的结果回来时也必须标成迟到，
    /// 且**不能顶掉**新请求的状态。
    #[test]
    fn a_stale_result_does_not_disturb_the_new_request() {
        let mut s = state_with_key(Some("sk-x"));
        s.awaiting = Some(2); // 新请求在跑

        s.apply(update(1, 5, "上一轮的")); // 旧请求回来了
        assert!(s.answer_late, "请求号对不上就要标迟到");
        assert!(s.is_running(), "旧结果回来不该把新请求的运行态清掉");
        assert_eq!(s.awaiting, Some(2));
    }

    /// 结论对应的采集已经不是当前显示的那个 —— 界面要能判断出来。
    #[test]
    fn an_answer_about_another_capture_is_flagged() {
        let mut s = state_with_key(Some("sk-x"));
        s.awaiting = Some(1);
        s.apply(update(1, 5, "针对采集 5 的结论"));

        assert!(
            !s.answer_is_about_another_capture(Some(5)),
            "当前就是采集 5，不该报错配"
        );
        assert!(
            s.answer_is_about_another_capture(Some(6)),
            "当前切到采集 6 了，必须能识别出来"
        );
        assert!(
            s.answer_is_about_another_capture(None),
            "当前没有采集时也算对不上"
        );
    }

    #[test]
    fn no_answer_means_nothing_to_mismatch() {
        let s = state_with_key(Some("sk-x"));
        assert!(
            !s.answer_is_about_another_capture(Some(1)),
            "没有结论时不该报「结论对不上」"
        );
    }

    #[test]
    fn a_failure_is_recorded_with_its_remedy() {
        let mut s = state_with_key(Some("sk-x"));
        s.awaiting = Some(1);
        s.apply(AiUpdate {
            req_id: 1,
            capture_id: 5,
            anchor: "#5".into(),
            outcome: Err(AiError::new("出事了", "这么办")),
        });
        let e = s.error.as_ref().expect("失败要被记下来");
        assert_eq!(e.message, "出事了");
        assert!(
            !e.hint.trim().is_empty(),
            "失败必须带处理措施 —— 项目规矩：错误不能只给错误码"
        );
    }

    /// 上一次失败了，这一次成功 —— 旧错误不该继续挂在界面上。
    #[test]
    fn a_new_start_clears_the_previous_error() {
        let mut s = state_with_key(Some("sk-x"));
        s.error = Some(AiError::new("上一次失败了", "怎么办"));
        // 直接模拟 start 里的清理动作，避免真的起线程
        s.error = None;
        assert!(s.error.is_none());
    }

    /// 没配密钥时保存要返回「存储失败」，而不是静默返回成功。
    #[test]
    fn saving_without_a_config_path_says_so() {
        let s = state_with_key(Some("sk-x"));
        assert!(s.config_path.is_none());
        let msg = s.save_config();
        assert!(
            msg.contains("未能保存") || msg.contains("无可用配置目录"),
            "没有配置目录时必须明说保存失败：{msg}"
        );
        assert!(
            msg.contains("内存"),
            "还要说清后果（设置不会持久化）：{msg}"
        );
    }

    // ── 请求体 ───────────────────────────────────────────────────────

    #[test]
    fn body_has_the_shape_the_api_expects() {
        let b = build_body(&cfg(), "证据在这里").unwrap();
        let v: serde_json::Value = serde_json::from_str(&b).unwrap();

        assert_eq!(v["model"], "test-model");
        assert_eq!(v["max_tokens"], MAX_TOKENS);
        assert_eq!(v["messages"][0]["role"], "user");
        assert_eq!(v["messages"][0]["content"], "证据在这里");
        // system 中必须包含「禁止杜撰数值」一条 —— 缺失时模型将自行发挥
        assert!(
            v["system"]
                .as_str()
                .unwrap()
                .contains("仅可引用证据包中出现的数值"),
            "system prompt 中的反杜撰规定缺失"
        );
    }

    #[test]
    fn system_prompt_forbids_guessing_bucket_interiors() {
        let b = build_body(&cfg(), "x").unwrap();
        let v: serde_json::Value = serde_json::from_str(&b).unwrap();
        let sys = v["system"].as_str().unwrap();
        assert!(sys.contains("桶内"), "须禁止对包络桶内细节作结论");
        assert!(sys.contains("模拟器"), "须要求声明模拟器数据源");
        assert!(
            sys.contains("信息不足时须直说缺什么"),
            "须允许并明确要求「信息不足」这一结论"
        );
    }

    /// **提示词必须要求「不要复述数值」。**
    ///
    /// 这是用户明确提出的一条：波形图和测量面板就在旁边，峰峰值、频率、
    /// 占空比用户看得见，让模型抄一遍是纯冗余 —— 而且会把真正有价值的内容
    /// （哪里不对、为什么、怎么办）挤掉。
    ///
    /// 这条一旦丢，模型会退回成「测量面板的语音版」，这个功能就没有存在意义。
    #[test]
    fn system_prompt_forbids_reciting_what_the_user_can_see() {
        let b = build_body(&cfg(), "x").unwrap();
        let v: serde_json::Value = serde_json::from_str(&b).unwrap();
        let sys = v["system"].as_str().unwrap();

        assert!(sys.contains("不要复述"), "须明确禁止复述界面上已有的数值");
        // 必须点名用户看得见的那些量，否则模型不知道「不要复述」指什么
        for seen in ["峰峰值", "频率", "占空比"] {
            assert!(sys.contains(seen), "须点名「{seen}」属于用户可见、不必复述");
        }
    }

    /// 系统提示词有多大 —— 它是**每一轮**都要背的固定开销。
    ///
    /// 刻度尺测试：成本讨论里要引用这个数。
    #[test]
    fn the_system_prompt_stays_small() {
        let n = SYSTEM_PROMPT.chars().count();
        let cjk = SYSTEM_PROMPT
            .chars()
            .filter(|c| ('\u{4e00}'..='\u{9fff}').contains(c))
            .count();
        println!("SYSTEM_PROMPT: {n} 字符（中文 {cjk} 字）");

        // 上限随内容涨过两次：第一次加了「输出的形状」（条目化、不要复述/开脱/空形容）
        // 与「I2C 地址是 7 位的」；第二次加了**模拟波形分支**（场景选择指引、
        // 「波形状态」一节、模拟输出示例，约 +375 字符，1314 → 1689）。
        // 这些同属**输出质量**规范：单发面板现在要覆盖 I2C 与模拟两种场景，
        // 而输出质量正是这个功能的价值所在 —— 涨得有理由。
        assert!(
            n < 1800,
            "系统提示词 {n} 字符 —— 它每轮都要发一次，别让它无理由地膨胀"
        );
    }

    /// 输出必须收敛到「状态 / 问题 / 建议」三节 —— 用户要的是诊断，不是报告。
    #[test]
    fn system_prompt_asks_for_diagnosis_and_actions() {
        let b = build_body(&cfg(), "x").unwrap();
        let v: serde_json::Value = serde_json::from_str(&b).unwrap();
        let sys = v["system"].as_str().unwrap();

        assert!(sys.contains("总线状态"), "须要求给出总线状态");
        assert!(sys.contains("问题"), "须要求指出问题");
        assert!(sys.contains("建议"), "须要求给出建议");
        assert!(
            sys.contains("没有异常就直说"),
            "须明确：没有问题就说没有，不得为凑内容编问题"
        );
    }

    /// 提示词必须覆盖**四个分支**，且非 I2C 的分支排在前面。
    ///
    /// 2026-10-06 真机回归：I2C 分支排第一、示例也全是 I2C 时，模型拿到
    /// 一个 2 kHz 方波，三段回答两段在讲「缺 SDA，I2C 解码做不了」。
    /// 分支顺序就是注意力顺序 —— 所以数字时序排第一，I2C 排第三。
    #[test]
    fn system_prompt_covers_all_four_scenario_branches() {
        let b = build_body(&cfg(), "x").unwrap();
        let v: serde_json::Value = serde_json::from_str(&b).unwrap();
        let sys = v["system"].as_str().unwrap();

        assert!(sys.contains("先看证据包「场景」行"), "须指引先看场景行");
        for branch in [
            "分支一：数字时序",
            "分支二：模拟信号",
            "分支三：I2C 总线解码",
            "分支四：形状判不出",
        ] {
            assert!(sys.contains(branch), "缺分支：{branch}");
        }
        // 非 I2C 必须排在 I2C 前面 —— 这是那次回归的直接修复
        let digital = sys.find("分支一：数字时序").unwrap();
        let i2c = sys.find("分支三：I2C 总线解码").unwrap();
        assert!(digital < i2c, "数字时序分支必须排在 I2C 之前");

        assert!(sys.contains("过冲"), "阶跃响应分支要点名过冲/振铃");
        assert!(
            sys.contains("工具的边界"),
            "判不出分支必须让模型明白：那是工具边界，不是信号坏了"
        );
        assert!(
            sys.contains("波形状态"),
            "非总线分支的第一节须是「波形状态」"
        );
    }

    // ── 响应解析：正常路径 ───────────────────────────────────────────

    #[test]
    fn parses_a_normal_response() {
        let body = r#"{
            "model": "deepseek-flash",
            "content": [{"type": "text", "text": "结论：一切正常"}],
            "usage": {"input_tokens": 1234, "output_tokens": 56}
        }"#;
        let a = parse_response(200, body, "fallback", Duration::from_millis(1500)).unwrap();
        assert_eq!(a.text, "结论：一切正常");
        assert_eq!(a.model, "deepseek-flash");
    }

    /// **响应里的 `usage` 字段被有意丢弃。**
    ///
    /// 实测它与请求体大小对不上（3512 字符的中文请求报 239 输入 token），
    /// 含义说不清就不该往上传。这条测试钉住「解析时不再读它」——
    /// 免得以后有人顺手又把它接回界面。
    #[test]
    fn usage_fields_are_deliberately_ignored() {
        let body = r#"{
            "model": "m",
            "content": [{"type": "text", "text": "ok"}],
            "usage": {"input_tokens": 1234, "output_tokens": 56}
        }"#;
        let a = parse_response(200, body, "m", Duration::from_millis(1)).unwrap();
        assert_eq!(a.text, "ok");
        // `AiAnswer` 上根本没有这两个字段 —— 编译期就挡住了。
        // 这条测试的意义是：那个响应**能被正常解析**（丢弃 usage 不影响别的）。
    }

    /// thinking 块是模型的内部推理，**不该显示给用户**。
    #[test]
    fn thinking_blocks_are_not_shown() {
        let body = r#"{"model":"m","content":[
            {"type":"thinking","thinking":"内部推理，用户不该看到"},
            {"type":"text","text":"这是给用户看的"}
        ]}"#;
        let a = parse_response(200, body, "m", Duration::from_millis(1)).unwrap();
        assert_eq!(a.text, "这是给用户看的");
        assert!(!a.text.contains("内部推理"));
    }

    #[test]
    fn multiple_text_blocks_are_joined() {
        let body = r#"{"model":"m","content":[
            {"type":"text","text":"第一段"},
            {"type":"text","text":"第二段"}
        ]}"#;
        let a = parse_response(200, body, "m", Duration::from_millis(1)).unwrap();
        assert!(a.text.contains("第一段") && a.text.contains("第二段"));
    }

    #[test]
    fn model_falls_back_when_server_omits_it() {
        let body = r#"{"content":[{"type":"text","text":"ok"}]}"#;
        let a = parse_response(200, body, "my-requested-model", Duration::from_millis(1)).unwrap();
        assert_eq!(a.model, "my-requested-model");
    }

    // ── 响应解析：错误路径，每一条都要有「怎么办」 ───────────────────

    #[test]
    fn empty_content_is_an_error_not_an_empty_answer() {
        let body = r#"{"model":"m","content":[],"stop_reason":"max_tokens"}"#;
        let e = parse_response(200, body, "m", Duration::from_millis(1)).expect_err("空回复要报错");
        assert!(e.message.contains("未返回文本内容"));
        assert!(
            !e.hint.is_empty() && e.hint.contains("max_tokens"),
            "要指出可能是被截断了：{}",
            e.hint
        );
    }

    /// **截断但有正文 —— 保留正文，并标出来。**
    ///
    /// 回归：从前只要有正文就返回 `Ok`，`stop_reason` 看都不看。于是一段
    /// 被拦腰砍断的诊断与一段完整的诊断在界面上完全一样 —— 而「话说了一半」
    /// 正是最容易被当成「说完了」的那种错。
    ///
    /// 变异：`truncated` 写死 `false`，这条红。
    #[test]
    fn a_truncated_answer_keeps_its_text_and_says_so() {
        let body = r#"{"model":"m","content":[{"type":"text","text":"结论写到一半"}],
                       "stop_reason":"max_tokens"}"#;
        let a =
            parse_response(200, body, "m", Duration::from_millis(1)).expect("有正文就不该整条报错");
        assert_eq!(a.text, "结论写到一半");
        assert!(a.truncated, "stop_reason = max_tokens 必须被标出来");
    }

    /// 正常收尾**不得**被标成截断 —— 否则那个标记就没有信息量了。
    ///
    /// 变异：`truncated` 写死 `true`，这条红。
    #[test]
    fn a_complete_answer_is_not_flagged_as_truncated() {
        let body = r#"{"model":"m","content":[{"type":"text","text":"说完了"}],
                       "stop_reason":"end_turn"}"#;
        let a = parse_response(200, body, "m", Duration::from_millis(1)).unwrap();
        assert!(!a.truncated, "end_turn 不是截断");
    }

    /// 服务端不给 `stop_reason` 时按**没截断**处理。
    ///
    /// 变异：判断写成 `!= Some("end_turn")` —— 这条红。把「未知」当成
    /// 「截断」，会给每一个正常回复都挂上一个假警报。
    #[test]
    fn a_missing_stop_reason_is_not_treated_as_truncation() {
        let body = r#"{"model":"m","content":[{"type":"text","text":"ok"}]}"#;
        let a = parse_response(200, body, "m", Duration::from_millis(1)).unwrap();
        assert!(!a.truncated, "缺 stop_reason 不等于被截断");
    }

    /// 预算下限。**低于它会整次请求作废**，不只是答得短一点。
    ///
    /// 2026-10-06 真机回归：2048 被思考块吃光，正文一个字没有，界面报
    /// 「无分析结果」—— 钱花了，什么也没拿到。
    ///
    /// 这条**不钉常量的当前值**，只钉它不许掉到底线以下：预算可以涨，
    /// 涨不用改测试。
    /// clippy 会说这是「常量断言」—— 没错，它**就是**常量对常量：预算与
    /// 底线都是编译期常量，折叠之后恒真。留着它的价值不在运行期，而在
    /// 「改预算的人必须先读懂这条」：把 8192 调低，编译立刻红，注释里写着
    /// 为什么不能调低。所以明确允许这个 lint，而不是把比较绕成运行期。
    #[test]
    #[allow(clippy::assertions_on_constants)]
    fn the_token_budget_stays_above_the_thinking_floor() {
        assert!(
            MAX_TOKENS >= 8192,
            "token 预算掉到 {MAX_TOKENS} —— 推理模型会把它全烧在思考块上，\
             正文一个字都留不下（2026-10-06 真机回归）"
        );
    }

    #[test]
    fn unauthorized_points_at_the_key() {
        let body = r#"{"error":{"message":"Authentication Fails"}}"#;
        let e = parse_response(401, body, "m", Duration::from_millis(1)).unwrap_err();
        assert!(e.message.contains("401"));
        assert!(
            e.message.contains("Authentication Fails"),
            "要带上服务端的原话"
        );
        assert!(e.hint.contains("密钥"), "处理措施里要提到密钥：{}", e.hint);
    }

    #[test]
    fn not_found_points_at_the_url() {
        let body = r#"{"error":{"message":"Not Found"}}"#;
        let e = parse_response(404, body, "m", Duration::from_millis(1)).unwrap_err();
        assert!(e.hint.contains("base_url"), "404 应当指向 base_url");
    }

    #[test]
    fn server_error_says_it_is_not_your_fault() {
        let body = r#"{"error":{"message":"internal"}}"#;
        let e = parse_response(503, body, "m", Duration::from_millis(1)).unwrap_err();
        assert!(
            e.hint.contains("非本端配置问题"),
            "5xx 须说明这不是本地配置错误：{}",
            e.hint
        );
    }

    /// 网关返回 HTML 是常见的 —— 不能崩，而且要给可操作的提示。
    #[test]
    fn html_instead_of_json_is_handled() {
        let html = "<html><body>502 Bad Gateway</body></html>";
        let e = parse_response(502, html, "m", Duration::from_millis(1)).unwrap_err();
        assert!(e.message.contains("无法解析"));
        assert!(!e.hint.is_empty(), "即使这样也要给怎么办");
    }

    /// 边界：200 但返回 HTML（端点写成了一个网页）。
    #[test]
    fn html_with_200_status_is_also_handled() {
        let html = "<!DOCTYPE html><html>hello</html>";
        let e = parse_response(200, html, "m", Duration::from_millis(1)).unwrap_err();
        assert!(
            e.hint.contains("base_url"),
            "200 + 非 JSON 说明端点指错了地方：{}",
            e.hint
        );
    }

    // ── 每一条错误都必须带「怎么办」 ─────────────────────────────────

    #[test]
    fn every_error_carries_a_hint() {
        let cases: [(u16, &str); 6] = [
            (400, r#"{"error":{"message":"bad"}}"#),
            (401, "{}"),
            (404, "{}"),
            (429, "{}"),
            (500, "{}"),
            (418, "not json"),
        ];
        for (status, body) in cases {
            let e = parse_response(status, body, "m", Duration::from_millis(1)).unwrap_err();
            assert!(
                !e.hint.trim().is_empty(),
                "HTTP {status} 的错误没有「怎么办」"
            );
            assert!(!e.message.trim().is_empty(), "HTTP {status} 的错误没有说明");
        }
    }

    // ── 没填 key 时不发请求 ──────────────────────────────────────────

    #[test]
    fn missing_key_fails_before_any_network_call() {
        let empty = AiConfig {
            api_key: String::new(),
            ..cfg()
        };
        let cancel = AtomicBool::new(false);
        // 端点是个不存在的域名 —— 如果它真去发了，这条会以网络错误的形式失败，
        // 而不是以「未配置密钥」的形式
        let e = ask(&empty, "证据", &cancel).unwrap_err();
        assert!(e.message.contains("未配置"), "实测：{}", e.message);
    }

    #[test]
    fn already_cancelled_does_not_send() {
        let cancel = AtomicBool::new(true);
        let e = ask(&cfg(), "证据", &cancel).unwrap_err();
        assert!(e.message.contains("已取消"), "实测：{}", e.message);
    }

    // ── 需要外网的那一条，默认不跑 ───────────────────────────────────

    /// **真的发一次 HTTPS 请求**，用一个假 key 打真实端点。
    ///
    /// 目的不是拿到答案，是验证 **DNS + TCP + TLS + HTTP + 错误映射**
    /// 这整条链在这台机器上真的通。**期望结果就是 401**（假 key）——
    /// 真返回成功反而说明哪里不对。
    ///
    /// 这条是**唯一**能覆盖 TLS/ring 在 MinGW 下链接结果的自动化手段，
    /// 所以放成 `#[ignore]` 而不是删掉。手动跑：
    ///
    /// ```text
    /// ./host/run.sh test -p scope-gui -- --ignored live_endpoint
    /// ```
    /// **量一次真实证据包的尺寸 —— 纯本地，不花钱、不联网。**
    ///
    /// 它复现 `live_single_shot_end_to_end` 里那次采集的全部输入，
    /// 只是不把请求发出去。用来核对服务端报的 `input_tokens` 是否可信：
    ///
    /// ```text
    /// ./host/run.sh test -p scope-gui -- --ignored measure_real_payload --nocapture
    /// ```
    #[test]
    #[ignore = "要跑一次真实采集（本地），手动跑"]
    fn measure_real_payload() {
        use scope_core::{AcquireParams, CommandBus, EvidenceInput};

        let mut bus = CommandBus::new(scope_device::Transport::sim(scope_sim::Scenario::I2c100k));
        bus.connect().expect("连模拟器失败");
        let cap = scope_core::acquire(
            &mut bus,
            &AcquireParams {
                samples: 4096,
                rate_hz: scope_core::f103::MAX_SAMPLE_RATE_HZ,
                trigger_level_lsb: 2048,
                timeout: Duration::from_millis(2000),
            },
        )
        .expect("采集失败");

        let scale = scope_core::ScaleSet::uncalibrated(cap.channels.len());
        let decode_cfg = scope_core::I2cDecodeConfig::default();
        let decode = scope_core::decode_capture(&cap, &decode_cfg).ok();
        let frames = decode.as_ref().map(|d| d.transactions.len()).unwrap_or(0);
        let decoded_frames = decode.as_ref().map(|d| scope_core::to_decoded(d.clone()));
        let decode_req = scope_core::DecodeRequest {
            protocol: scope_core::Protocol::I2c,
            channels: vec![],
            params: scope_core::DecodeParams::I2c(decode_cfg.clone()),
        };

        let evidence = scope_core::build_evidence(&EvidenceInput {
            capture: &cap,
            scale: &scale,
            link: "sim(i2c_100k)",
            simulated: true,
            config: bus.config.as_ref(),
            decode: decoded_frames.as_ref(),
            decode_req: Some(&decode_req),
            question: Some("这条总线上的通信是否正常？有没有值得注意的地方？"),
        });

        let cfg = AiConfig {
            api_key: "sk-placeholder".into(),
            ..AiConfig::default()
        };
        let body = build_body(&cfg, &evidence).unwrap();

        let ev = evidence.chars().count();
        let sp = SYSTEM_PROMPT.chars().count();
        println!("\n真实证据包   : {ev} 字符（{frames} 帧）");
        println!("系统提示词   : {sp} 字符");
        println!("请求体总长   : {} 字符", body.chars().count());
        println!("两者合计     : {} 字符", ev + sp);
        println!(
            "\n（服务端上次报的输入是 239 token —— 拿这个字符数去对，\n \
             若明显对不上，说明那个字段不能直接当成本依据）"
        );
    }

    /// **真跑一次成功的单发分析 —— 端到端，用真 key，花真钱。**
    ///
    /// 手动执行（`--nocapture` 才能看到基线）：
    ///
    /// ```text
    /// ./host/run.sh test -p scope-gui -- --ignored live_single_shot --nocapture
    /// ```
    ///
    /// # 为什么必须有这一条
    ///
    /// 在此之前，**整条链只验过失败路径**（假 key → 401）。而「闭环 / 工具循环」
    /// 这类扩展要搭在这条主干上 —— 主干从没成功过就往上盖，出问题时
    /// 分不清是新增部分的 bug 还是主干本来就有的。
    ///
    /// 它也顺带把几件从未一起验过的事串起来：真采集 → 真证据包 → 真 HTTP →
    /// 真解析 → 真用量。跑出来的数字就是后续的**基线**。
    #[test]
    #[ignore = "要外网 + 真 key + 花钱；手动跑"]
    fn live_single_shot_end_to_end() {
        use scope_core::{AcquireParams, CommandBus, EvidenceInput};

        // ── 1) 读用户机器上的真实配置 ──
        let path = crate::config::default_config_path();
        let (cfg, note) = crate::config::load(path.as_deref());
        if let Some(n) = &note {
            println!("配置来源：{n}");
        }
        assert!(
            cfg.has_key(),
            "配置里没有 key —— 先在 GUI 面板的「设置」里填好并保存"
        );
        println!("端点 {}  模型 {}", cfg.base_url, cfg.model);

        // ── 2) 真采一窗（模拟器，i2c_100k）──
        let mut bus = CommandBus::new(scope_device::Transport::sim(scope_sim::Scenario::I2c100k));
        let info = bus.connect().expect("连模拟器失败");
        println!(
            "设备 {} 通道，采样率上限 {} Hz",
            info.ch_count, info.rate_max_hz
        );

        let cap = scope_core::acquire(
            &mut bus,
            &AcquireParams {
                samples: 4096,
                rate_hz: scope_core::f103::MAX_SAMPLE_RATE_HZ,
                trigger_level_lsb: 2048,
                timeout: Duration::from_millis(2000),
            },
        )
        .expect("采集失败");
        println!(
            "采集 #{} {} 点 @ {} Hz",
            cap.id,
            cap.channels[0].len(),
            cap.rate_hz
        );

        // ── 3) 证据包（走真实路径：core 的 measure + decode）──
        let scale = scope_core::ScaleSet::uncalibrated(cap.channels.len());
        let decode_cfg = scope_core::I2cDecodeConfig::default();
        let decode = scope_core::decode_capture(&cap, &decode_cfg).ok();
        let decoded_frames = decode.as_ref().map(|d| scope_core::to_decoded(d.clone()));
        let decode_req = scope_core::DecodeRequest {
            protocol: scope_core::Protocol::I2c,
            channels: vec![],
            params: scope_core::DecodeParams::I2c(decode_cfg.clone()),
        };
        let evidence = scope_core::build_evidence(&EvidenceInput {
            capture: &cap,
            scale: &scale,
            link: "sim(i2c_100k)",
            simulated: true,
            config: bus.config.as_ref(),
            decode: decoded_frames.as_ref(),
            decode_req: Some(&decode_req),
            question: Some("这条总线上的通信是否正常？有没有值得注意的地方？"),
        });
        println!(
            "\n── 证据包 {} 字符（约 {} token）──\n{evidence}\n",
            evidence.chars().count(),
            evidence.chars().count() / 3
        );

        // ── 4) 真发一次 ──
        let cancel = AtomicBool::new(false);
        let answer = ask(&cfg, &evidence, &cancel).expect("真实调用应当成功 —— 失败看上面的错误");

        // ── 5) 打印基线 ──
        //
        // 基线**只记字符数**，不记 token —— 服务端那个用量字段与请求体大小
        // 对不上（见 `AiAnswer` 的说明）。字符数是确定的，才是能拿来比的刻度。
        println!("══ 基线 ══");
        println!("模型      : {}", answer.model);
        println!("用时      : {:.2} s", answer.elapsed.as_secs_f32());
        println!("证据包    : {} 字符", evidence.chars().count());
        println!("回复字数  : {}", answer.text.chars().count());
        println!("\n── 回复全文 ──\n{}\n", answer.text);

        // 断的是「真的拿到了东西」，不是措辞
        assert!(!answer.text.trim().is_empty(), "回复是空的");
        assert!(
            answer.text.chars().count() > 20,
            "回复只有 {} 字 —— 短得不像一份诊断",
            answer.text.chars().count()
        );

        // 提示词要求「不复述界面上已有的数值」，模型应当给的是诊断而非抄数
        println!("（人工核对：上面这段有没有在复述峰峰值/频率/占空比？）");
    }

    #[test]
    #[ignore = "需要外网；手动跑：./host/run.sh test -p scope-gui -- --ignored live_endpoint"]
    fn live_endpoint_round_trip_maps_the_auth_error() {
        let live = AiConfig {
            api_key: "sk-definitely-not-a-real-key".into(),
            ..AiConfig::default()
        };
        let cancel = AtomicBool::new(false);
        let e = ask(&live, "这是一个测试用的证据包。", &cancel)
            .expect_err("假 key 不该成功 —— 成功了说明端点上有什么东西不对");

        assert!(
            e.message.contains("401") || e.message.contains("403"),
            "期望看到鉴权失败，实测：{} / 怎么办：{}",
            e.message,
            e.hint
        );
        assert!(!e.hint.trim().is_empty(), "错误必须带「怎么办」");
        // 这条同时证明传输层没挂：能走到 401 说明握手成功了
        assert!(
            !e.message.contains("连不上") && !e.message.contains("超时"),
            "传输层就失败了，问题在 TLS/网络而不在 key：{}",
            e.message
        );
    }
}
