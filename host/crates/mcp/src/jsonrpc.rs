//! JSON-RPC 2.0 的最小实现 + stdio 上的换行分隔编解码。
//!
//! # 为什么是换行分隔，不是 Content-Length
//!
//! MCP 的 stdio 传输规定：**一条消息一行 JSON**，用 `\n` 分隔。
//! （LSP 那套 `Content-Length:` 头是另一个协议，不要混。）
//! 好处是不必自己写帧解析，坏处是 JSON 里不能有裸换行 ——
//! `serde_json::to_string` 本来就不产生换行，所以直接写就行。
//!
//! # 只实现用得上的部分
//!
//! 没有批量请求（JSON-RPC 允许数组形式的批请求，MCP 不用），
//! 没有命名参数以外的参数形式。少写少错。

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

/// JSON-RPC 标准错误码。
pub mod code {
    /// 解析失败：收到的不是合法 JSON。
    pub const PARSE_ERROR: i32 = -32700;
    /// 请求对象不合法（缺字段、类型不对）。
    pub const INVALID_REQUEST: i32 = -32600;
    /// 方法不存在。
    pub const METHOD_NOT_FOUND: i32 = -32601;
    /// 参数不合法。
    pub const INVALID_PARAMS: i32 = -32602;
}

// 注：JSON-RPC 还有个 -32603 INTERNAL_ERROR，这里用不上 ——
// 工具失败走的是 `isError: true` 的**工具级**错误（MCP 的约定），
// 不是协议级错误。两者的区别对客户端很重要：前者是「工具已执行但失败」，
// 后者是「这条请求本身有问题」。

/// 收到的请求。
///
/// `id` 为 `None` 表示这是一条**通知** —— 按 JSON-RPC 规矩**不回响应**。
#[derive(Debug, Deserialize)]
pub struct Request {
    /// 协议版本，固定 `"2.0"`。收到别的值属于客户端 bug，按规范仍应处理。
    #[serde(default)]
    pub jsonrpc: String,
    /// 请求 id。缺省即通知。
    #[serde(default)]
    pub id: Option<Value>,
    /// 方法名。
    pub method: String,
    /// 命名参数。
    #[serde(default)]
    pub params: Value,
}

/// 响应体。
#[derive(Debug, Serialize)]
pub struct Response {
    /// 固定 `"2.0"`。
    pub jsonrpc: &'static str,
    /// 回显请求 id。
    pub id: Value,
    /// 成功时的结果。
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    /// 失败时的错误。
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<RpcError>,
}

/// 错误对象。
#[derive(Debug, Serialize)]
pub struct RpcError {
    /// 错误码。
    pub code: i32,
    /// 一句话说明。
    pub message: String,
    /// 附加上下文。
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
}

impl Response {
    /// 成功响应。
    pub fn ok(id: Value, result: Value) -> Response {
        Response {
            jsonrpc: "2.0",
            id,
            result: Some(result),
            error: None,
        }
    }

    /// 失败响应。
    pub fn err(id: Value, code: i32, message: impl Into<String>) -> Response {
        Response {
            jsonrpc: "2.0",
            id,
            result: None,
            error: Some(RpcError {
                code,
                message: message.into(),
                data: None,
            }),
        }
    }
}

/// `tools/call` 的返回体 —— MCP 规定工具结果放在 `content` 数组里。
///
/// 本服务的工具返回的都是结构化数据，统一序列化成 JSON 文本放在一个 text 块里。
/// `is_error` 让客户端知道「工具执行失败」与「协议错误」是两回事。
pub fn tool_result(value: &Value, is_error: bool) -> Value {
    // 用紧凑 JSON 而不是 pretty —— 缩进和换行对 LLM 是**纯浪费**：
    // 一次 capture 的响应从 7879 字符降到 ~4500，省下的是真金白银的 token。
    // 人要看的话，GUI 那边有的是带格式的视图。
    json!({
        "content": [{ "type": "text", "text": serde_json::to_string(value).unwrap_or_default() }],
        "isError": is_error,
    })
}

/// 工具失败的统一形状。
///
/// 带 `hint` —— 项目纪律：错误要给「怎么办」，不能只给错误码。
pub fn tool_error(message: impl Into<String>, hint: Option<String>) -> Value {
    let mut v = json!({ "error": message.into() });
    if let Some(h) = hint {
        v["hint"] = Value::String(h);
    }
    tool_result(&v, true)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_request_with_params() {
        let r: Request =
            serde_json::from_str(r#"{"jsonrpc":"2.0","id":7,"method":"tools/list","params":{}}"#)
                .unwrap();
        assert_eq!(r.id, Some(json!(7)));
        assert_eq!(r.method, "tools/list");
    }

    #[test]
    fn a_request_without_id_is_a_notification() {
        let r: Request =
            serde_json::from_str(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#)
                .unwrap();
        assert!(r.id.is_none(), "没有 id 就是通知，不该回响应");
    }

    #[test]
    fn response_omits_the_unused_field() {
        let ok = serde_json::to_value(Response::ok(json!(1), json!({"a":1}))).unwrap();
        assert!(ok.get("result").is_some());
        assert!(ok.get("error").is_none(), "成功响应里不该有 error 字段");

        let err = serde_json::to_value(Response::err(json!(1), code::METHOD_NOT_FOUND, "无此方法"))
            .unwrap();
        assert!(err.get("error").is_some());
        assert!(err.get("result").is_none(), "失败响应里不该有 result 字段");
    }

    #[test]
    fn tool_result_is_one_text_block() {
        let v = tool_result(&json!({"n": 1}), false);
        let arr = v["content"].as_array().unwrap();
        assert_eq!(arr.len(), 1);
        assert_eq!(arr[0]["type"], "text");
        assert_eq!(v["isError"], false);
    }

    #[test]
    fn tool_error_carries_a_hint() {
        let v = tool_error("炸了", Some("重试一次".into()));
        assert_eq!(v["isError"], true);
        let text = v["content"][0]["text"].as_str().unwrap();
        assert!(text.contains("hint"), "提示应该进到文本里：{text}");
    }
}
