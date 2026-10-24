//! 一次完整采集的编排：配置 → 武装 → 等触发 → 分片拉取。
//!
//! # 为什么这必须是**一份**实现
//!
//! 这段流程里有三处极易写错、且错了很难查的地方：
//!
//! 1. `EVENT_TRIGGER` 的 18 字节 payload 必须**手工按小端解析** ——
//!    `capture_id` 只存在于这里，`GET_STATUS` 拿不回来，`READ_BUFFER` 又必须要它。
//! 2. 分片上限是 `(MAX_PAYLOAD_TX - CHUNK_HEADER_LEN) / 2`，**不是**
//!    设备上报的 `preferred_chunk_samples` —— 固件报的值若大于物理上限，
//!    设备端会回 `BAD_PARAM`。
//! 3. 「没有触发点」的哨兵 `0xFFFF_FFFF` 必须映射成 `None`。
//!
//! CLI 有一份、MCP 有一份简化版、GUI 再写一份就是三份。
//! 所以它在这里，三个消费方共用。
//!
//! # 与 `ARMED` 态的交互（重要）
//!
//! 等待触发期间**绝不能发送任何其他命令**。`CommandBus::await_response` 对带
//! `EVENT` 标志的帧只更新状态、**丢弃 payload** —— 所以一旦在武装期间插入一次
//! `GET_STATUS`（比如状态栏轮询），这次采集的元数据就永久丢失，只能重新 ARM。
//!
//! 本模块的结构性保证：从 `arm()` 到拿到 payload 之间，只调用
//! [`CommandBus::wait_trigger`]，而它只读帧、不发命令。

use crate::capture::Capture;
use crate::command::CommandBus;
use crate::device::DevicePort;
use crate::error::{Result, ScopeError};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// `EVENT_TRIGGER` payload 的长度（见 `docs/03-protocol.md`）。
const EVENT_TRIGGER_LEN: usize = 18;

/// 「没有触发点」的哨兵值。
///
/// 设备在软触发 / 未找到触发点时发这个值。`Capture::trigger_index` 是
/// `Option<u32>` 正是为此 —— **不映射的话，触发点会落在 42 亿号样点上**，
/// 任何按它定位的 UI 都会画到天边去。
const NO_TRIGGER: u32 = u32::MAX;

/// 等待触发时每次切片的长度。
///
/// 不整段等待的原因：`wait_trigger` 是阻塞的，整段 2 s 会把 worker 独占，
/// 而用户在「武装中」最想按的就是停止。切片之后每 80 ms 就有一次机会
/// 响应取消和强制触发。
const WAIT_SLICE: Duration = Duration::from_millis(80);

/// 一次采集等待触发的上限（ms）。
///
/// 这条上界的存在理由很具体：`wait_trigger_sliced` 算的是
/// `Instant::now() + total`，`total` 来自调用方。给一个 `u64::MAX` 毫秒
/// （约 5.8 亿年，LLM 常拿大整数当「无限」）之后，**整个调用会永久挂住
/// 且不报错** —— 实测 MCP server 从此不再响应任何请求，只能杀进程重启，
/// 在途请求连同后续的全部丢失。
///
/// 10 分钟是「人还愿意等」的上限；比这更长的观测应该用 `scope_watch`
/// 那种分段采集，而不是把一次等待拉长。
pub const MAX_TIMEOUT_MS: u64 = 600_000;

/// `SET_TRIGGER.mode` 的取值（见 `proto/protocol.h`）。
const TRIG_MODE_NORMAL: u8 = 1;
/// `SET_ACQ.mode` 的取值。
const ACQ_SINGLE: u8 = 0;

/// `READ_BUFFER.format` / 分片头的 `wf_format_t` 取值（见 `proto/protocol.h`）。
const CHUNK_FMT_RAW16: u8 = 0;
const CHUNK_FMT_PACK12: u8 = 1;

/// 一次采集的参数。
#[derive(Debug, Clone, Copy)]
pub struct AcquireParams {
    /// 采样点数。
    pub samples: u16,
    /// 请求的采样率（Hz）。**实际生效值会被设备量化**，时间轴以回显为准。
    pub rate_hz: u32,
    /// 触发电平（ADC LSB）。
    pub trigger_level_lsb: u16,
    /// 等待触发的总超时。
    pub timeout: Duration,
}

impl Default for AcquireParams {
    fn default() -> Self {
        AcquireParams {
            samples: crate::f103::MAX_CAPTURE_SAMPLES as u16,
            rate_hz: crate::f103::MAX_SAMPLE_RATE_HZ,
            trigger_level_lsb: 2048, // 12-bit 中点
            timeout: Duration::from_millis(2000),
        }
    }
}

/// 解析后的 `EVENT_TRIGGER`。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerEvent {
    /// 设备分配的采集编号。
    pub capture_id: u16,
    /// 触发点在窗内的相对样点号。**`None` 表示软触发或未找到触发点。**
    pub trigger_index: Option<u32>,
    /// 触发时刻的设备时钟（µs）。
    pub device_tick_us: u32,
    /// 设备实际生效的采样率。**时间轴以它为准。**
    pub rate_hz: u32,
    /// 本次采集的样点数。
    pub n_samples: u32,
}

impl TriggerEvent {
    /// 从 `EVENT_TRIGGER` 的 payload 解析（小端）。
    pub fn decode(p: &[u8]) -> Result<TriggerEvent> {
        if p.len() < EVENT_TRIGGER_LEN {
            return Err(ScopeError::Unsupported(format!(
                "EVENT_TRIGGER payload 只有 {} 字节，至少要 {EVENT_TRIGGER_LEN}",
                p.len()
            )));
        }
        let raw = u32::from_le_bytes([p[2], p[3], p[4], p[5]]);
        Ok(TriggerEvent {
            capture_id: u16::from_le_bytes([p[0], p[1]]),
            trigger_index: (raw != NO_TRIGGER).then_some(raw),
            device_tick_us: u32::from_le_bytes([p[6], p[7], p[8], p[9]]),
            rate_hz: u32::from_le_bytes([p[10], p[11], p[12], p[13]]),
            n_samples: u32::from_le_bytes([p[14], p[15], p[16], p[17]]),
        })
    }
}

/// `READ_BUFFER` 分片 payload（**不含** 12 B 分片头）→ 样点。
///
/// 按**设备回显**的 `format` 解码 —— 这是本函数的全部意义：主机请求
/// PACK12，设备有权按自己的情况回别的格式（未来还可能回 MINMAX），
/// 按请求值解就是把两种布局当成同一种读，出来的波形全是垃圾。
///
/// 只产出 `count` 个样点：PACK12 奇数时最后一对里有一个填充值，
/// 超出 `count` 的部分必须丢掉。payload 容量不足以容纳 `count` 个样点 → 报错
/// （设备声明与实际不符），**不**静默返回半段数据。
fn decode_chunk_samples(format: u8, payload: &[u8], count: u16) -> Result<Vec<u16>> {
    let count = count as usize;
    let mut samples = Vec::with_capacity(count);

    match format {
        CHUNK_FMT_RAW16 => {
            for pair in payload.chunks_exact(2).take(count) {
                samples.push(u16::from_le_bytes([pair[0], pair[1]]));
            }
        }
        CHUNK_FMT_PACK12 => {
            // 一对 3 字节解出 2 个；取到 count 需要的那一对为止。
            // 奇数 count 时最后一对多出的第 count+1 个值由 truncate 丢掉。
            for trio in payload.chunks_exact(3).take(count.div_ceil(2)) {
                let (s0, s1) = scope_proto::unpack12_pair([trio[0], trio[1], trio[2]]);
                samples.push(s0);
                samples.push(s1);
            }
            samples.truncate(count);
        }
        other => {
            return Err(ScopeError::Unsupported(format!(
                "未知的分片格式 {other}（仅支持 RAW16=0 / PACK12=1）"
            )));
        }
    }

    if samples.len() < count {
        return Err(ScopeError::InvalidParam {
            field: "chunk.count",
            value: count.to_string(),
            reason: format!(
                "分片只解出 {} 个样点（format={format}，payload {} 字节）",
                samples.len(),
                payload.len()
            ),
        });
    }
    Ok(samples)
}

/// 采集一次，不响应取消。
///
/// 等价于传一个永远为假的取消标志。给 CLI 这类一次性调用用。
pub fn acquire<P: DevicePort>(bus: &mut CommandBus<P>, params: &AcquireParams) -> Result<Capture> {
    static NEVER: AtomicBool = AtomicBool::new(false);
    acquire_cancellable(bus, params, &NEVER)
}

/// 采集一次，等待期间可被取消。
///
/// `cancel` 一旦置位，会在下一个切片边界（≤80 ms）停下来并发 `STOP`，
/// 返回 [`ScopeError::Cancelled`]。
pub fn acquire_cancellable<P: DevicePort>(
    bus: &mut CommandBus<P>,
    params: &AcquireParams,
    cancel: &AtomicBool,
) -> Result<Capture> {
    // 超时上界 —— 见 [`MAX_TIMEOUT_MS`] 的说明。放在**这里**而不是只放在
    // MCP 层：CLI 与 GUI 走的是同一条路径，一处漏掉就有一处能卡死。
    if params.timeout > Duration::from_millis(MAX_TIMEOUT_MS) {
        return Err(ScopeError::InvalidParam {
            field: "timeout",
            value: format!("{} ms", params.timeout.as_millis()),
            reason: format!("超过上限 {MAX_TIMEOUT_MS} ms"),
        });
    }

    // ── 1) 采样率：以**回显值**建时间轴（ADR 纪律，绝不用请求值）──
    let actual_hz = bus.set_sample_rate(params.rate_hz)?;

    // ── 2) 触发与采集参数 ──
    //
    // 触发模式/边沿/源与采集格式/抽点**沿用设备当前配置**，不写死。
    //
    // 回归：这里从前是 `set_trigger(1, 0, 0, ..)` 与 `set_acq(0, .., 0, 1)`
    // —— 固定 normal / ch0 / 上升沿 / RAW16 / 不抽点。于是
    // `scope_configure` 配好的东西会被**下一次采集**整个推回默认，
    // 而 configure 的响应还回显着「已生效」、`GET_STATUS` 也确认过。
    // 最刺眼的一处：采集超时的错误提示写着「改用 auto 模式」，而按提示
    // 配好的 auto 会在 ARM 之前被这几行改回 normal —— 提示是条死路。
    //
    // 沿用现值的语义：设备上电默认（`Config::default()` 与
    // `firmware/App/acq.c` 的 `acq_init()`，两者必须一致）是
    // `trigger_mode=auto / source=0 / edge=rising / format=RAW16 / decimation=1`。
    //
    // ⚠ **默认是 auto 而不是 normal，这是 2026-10-05 改的。** 边沿触发要求
    // 信号先跌破 `level-16` 再升破 `level+16`，所以**直流信号在任何电平下都
    // 不可能触发** —— 默认 normal 会让「接上板子、没接信号、点采集」必然等满
    // 超时。auto 约 200 ms 无触发即强制完成，先给出波形。
    let cfg = bus.config.as_ref();
    let (t_mode, t_src, t_edge) = cfg
        .map(|c| (c.trigger_mode, c.trigger_source, c.trigger_edge))
        .unwrap_or((TRIG_MODE_NORMAL, 0, 0));
    let (a_mode, a_fmt, a_dec) = cfg
        .map(|c| (c.acq_mode, c.format, c.decimation))
        .unwrap_or((ACQ_SINGLE, 0, 1));
    // 抽点倍数 0 是非法值（协议规定 1..=256），`set_acq` 会直接拒绝 ——
    // 于是一个把 `decimation` 报成 0 的设备会让**每一次采集都失败**。
    // 从前这里写死 1，恰好兜住了这种情况；改成沿用例值之后必须显式兜底。
    let a_dec = if a_dec == 0 { 1 } else { a_dec };

    let pre = (params.samples / 2).min(2048);
    bus.set_trigger(t_mode, t_src, t_edge, params.trigger_level_lsb, pre, 1000)?;
    bus.set_acq(a_mode, params.samples, a_fmt, a_dec)?;

    bus.arm()?;

    // ── 3) 等触发（切片，可取消）──
    let payload = match wait_trigger_sliced(bus, params.timeout, cancel)? {
        Some(p) => p,
        None => {
            let _ = bus.stop();
            return Err(ScopeError::NoTrigger(
                params.timeout.as_millis() as u64,
                format!(
                    "触发电平当前为 {} LSB；可降低电平、改用 auto 模式，或确认信号接在通道 0",
                    params.trigger_level_lsb
                ),
            ));
        }
    };

    let ev = TriggerEvent::decode(&payload)?;

    // ── 4) 分片拉取 ──
    let (ch_count, preferred_chunk) = match &bus.info {
        Some(i) => (i.ch_count.max(1) as usize, i.preferred_chunk_samples),
        None => (1, crate::PREFERRED_CHUNK as u16),
    };

    // 时间轴以设备回显为准。`EVENT_TRIGGER` 里带的 rate 是**本次采集**的实际值，
    // 比 `SET_SAMPLE_RATE` 的回显更贴近这一刻；设备若没填（0）就退回回显值。
    // 两个都不用请求值 —— 这是 ADR 纪律。
    let capture_rate_hz = if ev.rate_hz > 0 {
        ev.rate_hz
    } else {
        actual_hz
    };

    let mut capture = Capture::new(ev.capture_id, capture_rate_hz, ch_count, ev.n_samples);
    capture.trigger_index = ev.trigger_index;
    capture.device_tick_us = ev.device_tick_us;

    // 分片实际能用多大：payload 上限减掉 12 B 分片头，再按 u16 折半。
    let max_chunk_samples =
        ((scope_proto::MAX_PAYLOAD_TX - scope_proto::CHUNK_HEADER_LEN) / 2) as u16;
    let chunk = preferred_chunk
        .min(max_chunk_samples)
        .min(params.samples)
        .max(1);

    for ch in 0..ch_count {
        let mut offset: u32 = 0;
        while (offset as usize) < ev.n_samples as usize {
            if cancel.load(Ordering::Relaxed) {
                let _ = bus.stop();
                return Err(ScopeError::Cancelled("拉取波形分片"));
            }

            let want = chunk.min((ev.n_samples - offset) as u16);
            // 请求 PACK12：2 样点挤进 3 字节，省 25% 线路时间。
            // 设备有权回别的格式（回显在 hdr.format）—— 解码一律按回显值，
            // 绝不按这里的请求值（见 decode_chunk_samples）。
            let payload =
                bus.read_buffer(ev.capture_id, offset, want, CHUNK_FMT_PACK12, ch as u8)?;

            let hdr = crate::ChunkHeader::decode(&payload).ok_or_else(|| {
                ScopeError::Unsupported(format!("分片头解析失败（payload {} 字节）", payload.len()))
            })?;
            if !hdr.is_valid() {
                // 溢出/无效标志 —— 该数据不能按完整数据处理，须如实标记
                capture.overrun = true;
            }

            let samples = decode_chunk_samples(
                hdr.format,
                &payload[scope_proto::CHUNK_HEADER_LEN..],
                hdr.count,
            )?;
            capture.channels[ch].extend_from_slice(&samples);

            offset += hdr.count as u32;

            if hdr.is_last() {
                break;
            }
            if hdr.count == 0 {
                break; // 防死循环：设备回了 0 长度又不带 LAST
            }
        }
    }

    let _ = bus.stop();
    Ok(capture)
}

/// 切片等待 `EVENT_TRIGGER`。
///
/// 每片之间检查取消标志。`Ok(None)` 表示总超时用完。
fn wait_trigger_sliced<P: DevicePort>(
    bus: &mut CommandBus<P>,
    total: Duration,
    cancel: &AtomicBool,
) -> Result<Option<Vec<u8>>> {
    let deadline = Instant::now() + total;
    loop {
        if cancel.load(Ordering::Relaxed) {
            let _ = bus.stop();
            return Err(ScopeError::Cancelled("等待触发"));
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Ok(None);
        }
        if let Some(p) = bus.wait_trigger(remaining.min(WAIT_SLICE))? {
            return Ok(Some(p));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 造一个合法的 18 字节 EVENT_TRIGGER payload。
    fn payload(capture_id: u16, trigger_index: u32, rate: u32, n: u32) -> Vec<u8> {
        let mut p = Vec::with_capacity(EVENT_TRIGGER_LEN);
        p.extend_from_slice(&capture_id.to_le_bytes());
        p.extend_from_slice(&trigger_index.to_le_bytes());
        p.extend_from_slice(&1234u32.to_le_bytes());
        p.extend_from_slice(&rate.to_le_bytes());
        p.extend_from_slice(&n.to_le_bytes());
        p
    }

    #[test]
    fn parses_a_normal_trigger_event() {
        let ev = TriggerEvent::decode(&payload(7, 512, 857_142, 4096)).unwrap();
        assert_eq!(ev.capture_id, 7);
        assert_eq!(ev.trigger_index, Some(512));
        assert_eq!(ev.rate_hz, 857_142);
        assert_eq!(ev.n_samples, 4096);
        assert_eq!(ev.device_tick_us, 1234);
    }

    /// 一个什么都不做的传输。
    ///
    /// 超时检查发生在**任何设备 I/O 之前**（见 [`acquire_cancellable`] 的
    /// 第一步），所以这里的实现永远不会被调用 —— 它存在的唯一理由是给
    /// `CommandBus` 一个具体类型。要测真实交互请去 `scope-device` 或
    /// `scope-mcp`（那边有模拟器；core 不依赖 sim，这是**对的**依赖方向）。
    struct NullPort;

    impl crate::DevicePort for NullPort {
        fn write_all(&mut self, _bytes: &[u8]) -> std::result::Result<(), crate::LinkError> {
            Ok(())
        }
        fn read_some(&mut self) -> std::result::Result<Vec<u8>, crate::LinkError> {
            Ok(Vec::new())
        }
        fn describe(&self) -> String {
            "null".into()
        }
    }

    #[test]
    fn an_absurd_timeout_is_rejected_before_it_becomes_a_deadline() {
        // 回归：`wait_trigger_sliced` 算的是 `Instant::now() + total`。
        // 给一个 u64::MAX 毫秒（约 5.8 亿年）之后**调用永久挂住且不报错** ——
        // 实测一次就能让整个 MCP server 从此不再响应任何请求（`serve()` 是
        // 单线程），只能杀进程重启。这里守住的是 CLI / GUI / MCP 三条路径
        // 的共同入口。
        let mut bus = CommandBus::new(NullPort);

        for bad in [
            Duration::from_millis(MAX_TIMEOUT_MS + 1),
            Duration::from_millis(u64::MAX),
        ] {
            let params = AcquireParams {
                timeout: bad,
                ..Default::default()
            };
            match acquire(&mut bus, &params).unwrap_err() {
                ScopeError::InvalidParam {
                    field,
                    value,
                    reason,
                } => {
                    assert_eq!(field, "timeout");
                    assert!(!value.is_empty(), "错误里要带上实际请求的值");
                    assert!(
                        reason.contains(&MAX_TIMEOUT_MS.to_string()),
                        "理由里要写清上界，实测 {reason}"
                    );
                }
                other => panic!("越界超时应当被明确拒绝，实测 {}", other.summary()),
            }
        }
    }

    #[test]
    fn the_timeout_boundary_itself_is_not_rejected() {
        // 正好等于上界必须放行 —— 否则「上限」就成了「上限减一」，
        // 而那是个看不出来的差一错误。
        let mut bus = CommandBus::new(NullPort);
        let params = AcquireParams {
            timeout: Duration::from_millis(MAX_TIMEOUT_MS),
            ..Default::default()
        };
        // 会往后走到设备 I/O 上失败（NullPort 不响应），但不能是超时那条错误
        let e = acquire(&mut bus, &params).unwrap_err();
        assert!(
            !matches!(&e, ScopeError::InvalidParam { field, .. } if *field == "timeout"),
            "正好等于上界不该被拒，实测 {}",
            e.summary()
        );
    }

    #[test]
    fn sentinel_trigger_index_becomes_none() {
        // 回归：哨兵 0xFFFF_FFFF 必须映射成 None。
        // CLI 曾经把它原样包成 Some(4294967295) 存进 Capture ——
        // 任何按 trigger_index 定位的 UI 都会把触发标记画到天边去。
        let ev = TriggerEvent::decode(&payload(1, u32::MAX, 857_142, 1024)).unwrap();
        assert_eq!(
            ev.trigger_index, None,
            "哨兵值必须映射成 None，而不是 Some(4294967295)"
        );
    }

    #[test]
    fn trigger_index_zero_is_a_real_index_not_none() {
        // 边界：0 是**合法**的触发点（触发发生在第一个样点），不能和哨兵混淆
        let ev = TriggerEvent::decode(&payload(1, 0, 857_142, 1024)).unwrap();
        assert_eq!(ev.trigger_index, Some(0));
    }

    #[test]
    fn short_payload_is_rejected_not_panicking() {
        let e = TriggerEvent::decode(&[0u8; 10]);
        assert!(e.is_err(), "过短的 payload 应报错而不是越界读");
    }

    #[test]
    fn pack12_chunk_decodes_to_the_same_samples_as_raw16() {
        // 同一批 7 个样点，按两种格式编码 → 解码必须逐点相等。
        // 这条断言是可证伪的：把 3 字节按 2 字节解、或把高低半字节拼反，
        // 解出来的值都不会等于原样点。
        let samples: Vec<u16> = vec![0x000, 0xFFF, 0x123, 0xABC, 0x001, 0x800, 0x7FF];

        let mut raw = Vec::new();
        for s in &samples {
            raw.extend_from_slice(&s.to_le_bytes());
        }

        // PACK12：奇数末样点与 0 凑对（与固件 cmd_read_buffer 的约定一致）
        let mut packed = Vec::new();
        for pair in samples.chunks(2) {
            let s1 = pair.get(1).copied().unwrap_or(0);
            packed.extend_from_slice(&scope_proto::pack12_pair(pair[0], s1));
        }
        assert_eq!(
            packed.len(),
            12,
            "7 个样点应当打成 4 对 12 字节，实测 {}",
            packed.len()
        );

        let got_raw = decode_chunk_samples(0, &raw, samples.len() as u16).unwrap();
        let got_packed = decode_chunk_samples(1, &packed, samples.len() as u16).unwrap();

        assert_eq!(got_raw, samples, "RAW16 解码往返");
        assert_eq!(
            got_packed, samples,
            "PACK12 解码往返（第 7 点与 0 凑对，解出的第 8 个值必须截掉）"
        );
        assert_eq!(
            got_packed.len(),
            7,
            "PACK12 解出的长度应当按 count 截断到 7"
        );
    }

    #[test]
    fn unsupported_chunk_format_is_an_error_not_garbage() {
        // MINMAX(2) 的 payload 布局完全不同 —— 对未知格式硬解就是把两种
        // 字节当同一种读，出来的「波形」全是垃圾。必须明确报错。
        let e = decode_chunk_samples(2, &[0u8; 16], 4).unwrap_err();
        assert!(
            matches!(e, ScopeError::Unsupported(_)),
            "未知格式必须是明确的错误，实测 {}",
            e.summary()
        );
        // summary() 对 Unsupported 是固定文案（见 error.rs），细节在 hint() 里。
        let hint = e.hint().expect("Unsupported 应当带说明");
        assert!(
            hint.contains("格式"),
            "错误说明要指出是格式问题，实测 {hint}"
        );
    }
}
