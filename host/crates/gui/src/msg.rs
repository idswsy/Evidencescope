//! UI 线程 ↔ worker 线程的消息协议。
//!
//! 规矩只有一条：**每个 [`Request`] 恰好产生一个终态 [`Update`]**
//! （`Acquired` / `ConfigApplied` / `Connected` / `Disconnected` / `Failed`），
//! 中间过程用非终态消息（`Busy` / `StateChanged`）。
//!
//! 这条规矩让 UI 侧的 busy 态可以简单地用「发请求时置位、收到终态时复位」维护，
//! 不需要超时兜底，也不会出现永远转圈的按钮。

use scope_core::{Capture, DeviceConfig, DeviceInfo, State};
use scope_sim::{FaultInjection, Scenario};

// 连接方式的定义在共享的设备层里（GUI 与 MCP 共用一份）
pub use scope_device::TransportKind;

/// UI → worker。
#[derive(Debug, Clone)]
pub enum Request {
    /// 枚举串口。
    ListPorts,
    /// 连接。`transport` 决定走模拟器还是串口。
    Connect {
        /// 传输类型。
        transport: TransportKind,
        /// 串口名（`transport = Serial` 时有效）。
        port: String,
        /// 波特率（`transport = Serial` 时有效）。
        baud: u32,
        /// 模拟器场景（`transport = Sim` 时有效）。
        scenario: Scenario,
    },
    /// 断开并停流。
    Disconnect,
    /// 一次性下发配置。内部会先读回显、再更新缓存。
    ApplyConfig {
        /// 请求的采样率。设备会量化，实际值以回显为准。
        rate_hz: u32,
        /// 采集点数。
        samples: u16,
        /// 触发模式：`0=auto` / `1=normal` / `2=single`。
        trigger_mode: u8,
        /// 触发源通道。
        trigger_source: u8,
        /// 触发边沿：`0=上升` / `1=下降`。
        trigger_edge: u8,
        /// 触发电平（ADC LSB）。
        trigger_level_lsb: u16,
        /// 预触发点数。
        pre_samples: u16,
        /// 触发保持（µs）。
        holdoff_us: u32,
    },
    /// 采一次。走 [`scope_core::acquire`] 的完整编排。
    Acquire {
        /// 采样点数。
        samples: u16,
        /// 请求采样率。
        rate_hz: u32,
        /// 触发电平（ADC LSB）。
        trigger_level_lsb: u16,
        /// 等待触发的超时（ms）。
        timeout_ms: u64,
    },
    /// 切换模拟器场景（仅模拟器连接有效）。
    SetScenario(Scenario),
    /// 设置模拟器故障注入（仅模拟器连接有效）。
    SetFaults(Box<FaultInjection>),
    /// `RESET` —— 从 `Fault` 态里出来的唯一办法。
    Reset,
    /// 让 worker 收尾退出。
    Shutdown,
}

/// worker → UI。
#[derive(Debug, Clone)]
pub enum Update {
    /// 串口枚举结果：`(端口名, 描述, 是否疑似目标)`。
    Ports(Vec<(String, String, bool)>),
    /// 连接成功。**`state` 是补发一次 `GET_STATUS` 获得的真值** ——
    /// `connect()` 自己不发 `GET_STATUS`，缺了它 `guard_config_allowed`
    /// 就没有判据，配置按钮在"已武装"时也不会被拦住。
    Connected {
        /// 设备能力。
        info: DeviceInfo,
        /// 设备当前配置。
        config: Option<DeviceConfig>,
        /// 设备当前状态。
        state: State,
        /// 背后是不是模拟器 —— 决定「场景 / 故障注入」面板要不要出现。
        simulated: bool,
    },
    /// 已断开。
    Disconnected,
    /// 配置已生效（值来自设备回显）。
    ConfigApplied(Box<DeviceConfig>),
    /// 采集完成。
    Acquired(Box<Capture>),
    /// 设备状态发生变化。
    StateChanged(State),
    /// 某个长操作开始/结束。
    Busy {
        /// 操作名（显示在按钮上）。
        op: &'static str,
        /// 是否正在进行。
        active: bool,
    },
    /// 失败。**已在这一侧把 `ScopeError` 拆成可显示的两段** ——
    /// `ScopeError` 不是 `Clone`，且 UI 需要的是文案 + 自救提示。
    Failed {
        /// 一句话说明。
        text: String,
        /// 「怎么办」。来自 `DeviceError::hint()` 等，没有则为 `None`。
        hint: Option<String>,
    },
}

/// 把错误拆成 UI 能用的两段：说明 + 自救提示。
///
/// 实现在 core 里（`ScopeError::summary()` / `hint()`）—— GUI 与 MCP 共用一份，
/// 避免两侧的措辞漂移。
pub fn describe_error(e: &scope_core::ScopeError) -> (String, Option<String>) {
    (e.summary(), e.hint())
}
