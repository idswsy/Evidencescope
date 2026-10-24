//! # scope-sim —— 模拟器，一等公民
//!
//! 没有硬件时，Agent 逻辑、MCP 工具、I2C 解码器、测量算法全都能开发与回归测试。
//! 这不是「顺便做的辅助工具」，而是让三个人不必抢一块板子的**基础设施**。
//!
//! ## 复刻了哪些真实行为
//!
//! - 采样率量化（只提供定时器分频档位，回显 `actual_hz`）
//! - 状态机约束（ARMED 下发配置 → `BUSY`）
//! - auto 触发约 200 ms 超时强制完成（**没有这条 Agent 会永久阻塞**）
//! - seq 单槽去重缓存（`ARM` 重试安全）
//! - 按 64 字节分片输出，覆盖真实的粘包/拆包路径
//!
//! ## 故障注入
//!
//! 真实链路的可靠性只能靠「制造故障」来验证：
//!
//! ```no_run
//! # use scope_sim::{SimDevice, FaultInjection, Scenario};
//! let mut dev = SimDevice::new(Scenario::Sine1k3v3);
//! dev.faults = FaultInjection {
//!     drop_every_n_frames: 50,       // 每 50 帧丢一个
//!     crc_err_every_n_frames: 37,    // 每 37 帧坏一个
//!     no_trigger: false,
//!     ..Default::default()
//! };
//! ```

#![deny(clippy::all)]
#![warn(missing_docs)]

pub mod device;
pub mod waveform;

pub use device::{CapsMode, FaultInjection, SimDevice};
pub use waveform::{Rng, Scenario, WaveformGen, FULL_SCALE_LSB, MID_LSB};

#[cfg(test)]
mod tests {
    use super::*;
    use scope_core::{CaptureStore, CommandBus, DevicePort, State};
    use scope_proto::Cmd;

    #[test]
    fn oversized_host_payload_is_rejected_locally() {
        // 回归：主机→设备方向的上限是 512 B，比设备发来的 2060 B 小得多。
        // 不在本地拦下的话，超出设备解析缓冲（524 B）容量，会把帧逐字节削掉，
        // 主机只能看到三次超时，完全不知道是自己发太大了。
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();

        let too_big = vec![0u8; scope_proto::MAX_PAYLOAD_RX + 1];
        let err = bus
            .transaction(Cmd::Echo, too_big, scope_core::TIMEOUT_CONTROL)
            .unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("512"), "错误信息应说明真实上限: {msg}");

        // 恰好 512 应当被接受
        let ok = vec![0u8; scope_proto::MAX_PAYLOAD_RX];
        assert!(
            bus.transaction(Cmd::Echo, ok, scope_core::TIMEOUT_CONTROL)
                .is_ok(),
            "512 字节应当被接受"
        );
    }

    #[test]
    fn connect_and_read_device_info() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Sine1k3v3));
        let info = bus.connect().expect("连接设备失败");

        assert_eq!(info.adc_bits, 12);
        assert_eq!(
            info.rate_max_hz, 857_142,
            "F103 上限必须是 857142 —— 不是 1000000（72MHz 系统下 ADCCLK 只能到 12MHz）"
        );
        assert_eq!(info.capture_max_samples, 4096);
        assert_eq!(info.max_rx_payload, 512);
        // 2060 = 12 B 分片头 + 1024 样点 × u16。
        // 注意不是 2048 —— 分片头是算在 payload 之内的。
        assert_eq!(info.max_tx_payload, 2060);
    }

    #[test]
    fn ping_echoes_payload() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();
        let (echoed, _rtt) = bus.ping(&[1, 2, 3, 4]).unwrap();
        assert_eq!(echoed, vec![1, 2, 3, 4]);
    }

    #[test]
    fn sample_rate_is_quantized_and_echoed() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();

        let actual = bus.set_sample_rate(857_143).unwrap();
        assert_eq!(actual, 857_142, "设备必须回显量化后的实际值");

        // 请求超出上限时，本地就应该拦下来
        let err = bus.set_sample_rate(2_000_000).unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("857142"), "错误信息应说明真实上限: {msg}");
    }

    #[test]
    fn full_capture_cycle_produces_waveform() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Sine1k3v3));
        bus.connect().unwrap();
        bus.set_sample_rate(857_142).unwrap();
        bus.set_trigger(1, 0, 0, 2048, 2048, 1000).unwrap();
        bus.set_acq(0, 1024, 0, 1).unwrap();

        assert_eq!(bus.get_status().unwrap().state, State::Idle);
        bus.arm().unwrap();

        // 等触发事件
        let ev = bus
            .wait_trigger(scope_core::TIMEOUT_TRIGGER)
            .expect("等待触发事件失败")
            .expect("应产生触发事件");
        assert!(ev.len() >= 18, "EVENT_TRIGGER payload 应至少 18 字节");

        let capture_id = u16::from_le_bytes([ev[0], ev[1]]);

        // 拉一片
        let chunk = bus.read_buffer(capture_id, 0, 512, 0, 0).unwrap();
        assert!(chunk.len() > 12, "应返回分片头 + 样点");

        let hdr = scope_proto::ChunkHeader::decode(&chunk).unwrap();
        assert_eq!(hdr.capture_id, capture_id);
        assert_eq!(hdr.start_sample, 0);
        assert!(hdr.count > 0);

        // 样点应是 12-bit
        let samples = &chunk[12..];
        for pair in samples.chunks_exact(2) {
            let v = u16::from_le_bytes([pair[0], pair[1]]);
            assert!(v <= 4095, "样点越界: {v}");
        }
    }

    /// 直接往设备塞一个裸帧，返回设备回的第一帧。
    fn raw_exchange(dev: &mut SimDevice, bytes: &[u8]) -> scope_proto::Frame {
        dev.write_all(bytes).unwrap();
        let mut reader = scope_core::FrameReader::new();
        reader
            .next_frame(dev, std::time::Duration::from_millis(200))
            .expect("设备应有响应")
    }

    fn raw_frame(seq: u16, cmd: Cmd) -> Vec<u8> {
        scope_proto::Frame {
            header: scope_proto::Header {
                ver: scope_proto::VER,
                flags: 0,
                seq,
                cmd_raw: cmd as u16,
                len: 0,
            },
            payload: Vec::new(),
        }
        .encode()
    }

    #[test]
    fn duplicate_seq_replays_cached_response() {
        // 复刻真机的 seq 去重缓存：同一 (seq, cmd) 重发必须**回放**上次响应，
        // 而不是重新执行。这是 ARM 这类非幂等命令重试安全的唯一保障。
        let mut dev = SimDevice::new(Scenario::Dc);
        let arm = raw_frame(7, Cmd::Arm);

        let r1 = raw_exchange(&mut dev, &arm);
        assert_eq!(r1.header.seq, 7);
        assert_eq!(r1.header.cmd_raw, Cmd::Arm as u16);
        assert_eq!(dev.state(), State::Armed);

        // 主机超时后按原 seq 重发
        let r2 = raw_exchange(&mut dev, &arm);

        assert_eq!(
            r2.encode(),
            r1.encode(),
            "同一 seq 重发必须逐字节回放，不能产生新的副作用"
        );
    }

    #[test]
    fn arm_with_new_seq_while_armed_returns_busy() {
        // 协议规定：新 seq 在已 ARMED 状态下回 BUSY（先 STOP）
        let mut dev = SimDevice::new(Scenario::Dc);
        raw_exchange(&mut dev, &raw_frame(1, Cmd::Arm));

        let r = raw_exchange(&mut dev, &raw_frame(2, Cmd::Arm));
        assert!(r.header.is_error(), "应回错误帧");
        assert_eq!(r.header.cmd_raw, Cmd::Error as u16);
        assert_eq!(
            u16::from_le_bytes([r.payload[0], r.payload[1]]),
            0x0004,
            "错误码应为 BUSY"
        );
    }

    #[test]
    fn command_bus_rejects_second_arm_locally() {
        // 上层用 CommandBus 时，第二次 ARM 在本地就被拦下 —— 比等设备回 BUSY 更快，
        // 也避免了一次无谓的往返。
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();
        bus.arm().unwrap();

        let err = bus.arm().unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("ARM"), "错误应指出是 ARM: {msg}");
        assert!(msg.contains("STOP"), "提示应告诉用户怎么办: {msg}");
    }

    #[test]
    fn config_rejected_while_armed() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();
        bus.set_trigger(1, 0, 0, 2048, 2048, 1000).unwrap();
        bus.arm().unwrap();

        // 采集进行中改采样率 → 本地就该拦住
        let err = bus.set_sample_rate(100_000).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("ARMED") || msg.contains("STOP"),
            "提示应指出怎么办: {msg}"
        );
    }

    #[test]
    fn capture_store_keeps_history() {
        let mut store = CaptureStore::with_capacity(4);
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();
        bus.set_acq(0, 256, 0, 1).unwrap();

        for _ in 0..3 {
            bus.arm().unwrap();
            bus.force_trigger().unwrap();
        }

        store.push(scope_core::Capture::new(1, 857_142, 1, 256));
        assert_eq!(store.len(), 1);
        assert!(store.latest().is_some());
    }

    #[test]
    fn no_trigger_fault_does_not_hang_forever() {
        let mut dev = SimDevice::new(Scenario::Dc);
        dev.faults.no_trigger = true;
        let mut bus = CommandBus::new(dev);
        bus.connect().unwrap();
        bus.set_trigger(1, 0, 0, 2048, 2048, 1000).unwrap(); // mode=1 即 normal
        bus.arm().unwrap();

        // 永不触发的情况下，wait_trigger 必须在超时后干净地返回 None，
        // 而不是永远卡住 —— 否则 Agent 会死等
        let t0 = std::time::Instant::now();
        let got = bus
            .wait_trigger(std::time::Duration::from_millis(300))
            .unwrap();
        assert!(got.is_none());
        assert!(t0.elapsed() < std::time::Duration::from_secs(2));
    }

    #[test]
    fn auto_mode_completes_even_when_signal_never_triggers() {
        // 回归：tick() 里 5ms 的"模拟触发"分支曾经没有排除 auto 模式，
        // 导致 auto 在约 5ms 就完成、200ms 超时分支成了死代码；
        // 而 no_trigger 又在最前面直接 return，把 auto 的超时也一起屏蔽了。
        //
        // 真机的 auto 模式就是会超时完成的，Agent 正是靠这一条才不永久阻塞。
        // 模拟器必须如实复刻 —— 否则"对着模拟器开发"这个前提就不成立。
        let mut dev = SimDevice::new(Scenario::Dc);
        dev.faults.no_trigger = true; // 信号永不触发
        let mut bus = CommandBus::new(dev);
        bus.connect().unwrap();
        bus.set_trigger(0, 0, 0, 2048, 2048, 1000).unwrap(); // mode=0 即 auto
        bus.arm().unwrap();

        let t0 = std::time::Instant::now();
        let got = bus
            .wait_trigger(std::time::Duration::from_millis(1500))
            .expect("等待不应报错")
            .expect("auto 模式必须在超时后强制完成一次采集");
        let elapsed = t0.elapsed();

        assert!(
            elapsed >= std::time::Duration::from_millis(150),
            "auto 应等到约 200ms 才强制完成，实际只用了 {elapsed:?}（说明 5ms 分支没排除 auto）"
        );
        assert!(
            elapsed < std::time::Duration::from_millis(1200),
            "auto 不应拖过 200ms 太久，实际 {elapsed:?}"
        );
        assert!(!got.is_empty(), "EVENT_TRIGGER payload 不应为空");
    }

    #[test]
    fn normal_mode_completes_promptly_without_fault_injection() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();
        bus.set_trigger(1, 0, 0, 2048, 2048, 1000).unwrap(); // normal
        bus.arm().unwrap();

        let t0 = std::time::Instant::now();
        assert!(bus
            .wait_trigger(std::time::Duration::from_millis(1000))
            .unwrap()
            .is_some());
        assert!(
            t0.elapsed() < std::time::Duration::from_millis(500),
            "normal 模式应很快完成，实际 {:?}",
            t0.elapsed()
        );
    }

    #[test]
    fn unknown_command_returns_error_frame() {
        let mut bus = CommandBus::new(SimDevice::new(Scenario::Dc));
        bus.connect().unwrap();

        // 0x7F7F 是保留区里的未分配命令
        let err = bus.transaction(
            Cmd::try_from_u16(0x0101).unwrap(),
            Vec::new(),
            scope_core::TIMEOUT_CONTROL,
        );
        assert!(err.is_ok(), "GET_INFO 应该正常");

        // 直接用裸码发未知命令：用 port 手工组帧
        let frame = scope_proto::Frame {
            header: scope_proto::Header {
                ver: scope_proto::VER,
                flags: 0,
                seq: 4242,
                cmd_raw: 0x7F7F,
                len: 0,
            },
            payload: Vec::new(),
        };
        let bytes = frame.encode();
        bus.port_mut().write_all(&bytes).unwrap();

        let mut reader = scope_core::FrameReader::new();
        let resp = reader
            .next_frame(bus.port_mut(), std::time::Duration::from_millis(200))
            .expect("应回错误帧");
        assert_eq!(resp.header.seq, 4242);
        assert!(resp.header.is_error(), "应带 ERROR 标志");
        assert_eq!(resp.header.cmd_raw, Cmd::Error as u16);
        assert_eq!(
            u16::from_le_bytes([resp.payload[0], resp.payload[1]]),
            0x0001
        );
    }

    #[test]
    fn device_description_marks_simulation() {
        let dev = SimDevice::new(Scenario::I2c100k);
        assert!(dev.is_simulated());
        assert!(dev.describe().contains("sim"));
        assert_eq!(dev.channel_count(), 2, "I2C 场景是双通道");
    }
}
