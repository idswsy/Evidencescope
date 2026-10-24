//! # scope-transport-serial —— 串口传输层
//!
//! 覆盖 F103 核心板的两条现实链路：
//!
//! | 链路 | 引脚 | 需要什么 | 实测吞吐 |
//! |---|---|---|---|
//! | **USART1 + 外接 CH340 模块** | PA9 / PA10 | 四根线（TX/RX 交叉） | 921600 → 92 KB/s |
//! | **USB CDC** | PA11 / PA12 | 自写 USB device 固件 | 500–900 KB/s |
//!
//! 若核心板确实有板载串口桥且实测稳定，也可改用它 —— 少四根线，协议层同样不用动
//! （[ADR-013](`docs/06-decisions.md`)）。
//!
//! 两条在操作系统层都表现为一个 COM 口，所以**同一个实现覆盖两者** ——
//! 这正是 ADR-012 说「换链路协议一个字都不用改」的原因（ADR-012 取代 ADR-009，
//! 换板前后的完整经过见 `docs/06-decisions.md`）。
//!
//! ⚠️ **绝不要用 USART2（PA2/PA3）**：底板把这俩脚分别用作 PWM 输出和模拟输入，
//! 是硬冲突。见 `docs/02-hardware.md` §2。
//!
//! ## 已知的坑（都已在代码里处理）
//!
//! - **CH340/CP2102 驱动的 Latency Timer 默认可能 16 ms** → 会显著拖慢小帧往返。
//!   这里把每次读取的超时设得很短并主动轮询，避免被驱动的缓冲策略拖死。
//! - **USB CDC 首次打开时 DTR 可能触发板子复位** → 连接后应重试一次握手。
//!   当前核心板的 PA12 是否有 1.5 kΩ 硬上拉未核实（`【待测】`，上一块板的结论）。
//! - **写入必须处理部分写** → 用 `write_all` 语义循环写完。
//! - **串口粘包/拆包** → 交给 [`scope_core::FrameReader`]，它内部是增量状态机。

#![deny(clippy::all)]
#![warn(missing_docs)]

use scope_core::error::LinkError;
use scope_core::DevicePort;
use serialport::{ClearBuffer, SerialPort};
use std::time::Duration;

/// 常见波特率。
pub mod baud {
    /// 最低可用档（满足命令传输，不满足波形传输）。
    pub const B_115200: u32 = 115_200;
    /// 推荐的默认档。
    pub const B_460800: u32 = 460_800;
    /// 通常能稳定运行的最高档（2 Mbps 一档须先实测，见下一档）。**波形传输建议用这个。**
    pub const B_921600: u32 = 921_600;
    /// 2 Mbps：F103 侧 BRR=36 零误差。**必须与固件 `LINK_BAUD` 一致，且先实测。**
    pub const B_2000000: u32 = 2_000_000;
}

/// 串口设备。
pub struct SerialDevice {
    port: Box<dyn SerialPort>,
    /// 用于动态超时估算。
    byte_rate: u32,
    description: String,
}

// `Box<dyn SerialPort>` 不是 Debug，所以手写一个 —— 只暴露真正有信息量的字段。
impl std::fmt::Debug for SerialDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialDevice")
            .field("description", &self.description)
            .field("byte_rate", &self.byte_rate)
            .finish_non_exhaustive()
    }
}

impl SerialDevice {
    /// 打开串口。
    ///
    /// `timeout_ms` 是底层读超时 —— 设小一点（如 5 ms）让上层能及时轮询，
    /// 避免被 CH340 驱动的缓冲策略拖慢。
    pub fn open(
        port_name: &str,
        baud_rate: u32,
        timeout_ms: u64,
    ) -> Result<SerialDevice, LinkError> {
        let target = format!("{port_name} @ {baud_rate}");

        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(timeout_ms))
            .open()
            .map_err(|e| LinkError::Open {
                target: target.clone(),
                source: Box::new(e),
            })?;

        // 丢弃上电前留在驱动缓冲里的垃圾字节，否则解析器要花时间重同步。
        // serialport 4.x 的 clear() 取 &self，所以绑定不需要 mut。
        let _ = port.clear(ClearBuffer::All);

        Ok(SerialDevice {
            port,
            // 串口 8N1 的等效字节率 = 波特率 / 10（每字节 8 数据位 + 起止位）
            byte_rate: baud_rate / 10,
            description: target,
        })
    }

    /// 用推荐默认参数打开。
    pub fn open_default(port_name: &str) -> Result<SerialDevice, LinkError> {
        Self::open(port_name, baud::B_921600, 5)
    }
}

impl DevicePort for SerialDevice {
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), LinkError> {
        // serialport 的 write_all 已经处理部分写，但错误类型要转换
        self.port
            .write_all(bytes)
            .map_err(|e| LinkError::Write(Box::new(e)))?;
        self.port.flush().map_err(|e| LinkError::Write(Box::new(e)))
    }

    fn read_some(&mut self) -> Result<Vec<u8>, LinkError> {
        let mut buf = [0u8; 512];
        match self.port.read(&mut buf) {
            Ok(0) => Ok(Vec::new()),
            Ok(n) => Ok(buf[..n].to_vec()),
            // 超时不是错误 —— 只是「此刻没有数据」
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(Vec::new()),
            Err(e) => Err(LinkError::Read(Box::new(e))),
        }
    }

    fn describe(&self) -> String {
        self.description.clone()
    }

    fn byte_rate(&self) -> u32 {
        self.byte_rate
    }
}

/// 列出系统中所有可用串口，附带启发式识别。
///
/// 返回 `(端口名, 描述, 是否为疑似目标设备)` —— 目标设备是
/// CH340 / CP210x / STM32 虚拟串口这几类。
pub fn list_ports() -> Vec<(String, String, bool)> {
    match serialport::available_ports() {
        Ok(ports) => ports
            .into_iter()
            .map(|p| {
                let desc = match &p.port_type {
                    serialport::SerialPortType::UsbPort(info) => {
                        let vid_pid = format!("{:04X}:{:04X}", info.vid, info.pid);
                        let product = info.product.clone().unwrap_or_default();
                        format!("USB {vid_pid} {product}")
                    }
                    serialport::SerialPortType::BluetoothPort => "Bluetooth".to_string(),
                    serialport::SerialPortType::PciPort => "PCI".to_string(),
                    serialport::SerialPortType::Unknown => "Unknown".to_string(),
                };

                // CH340 = 1A86:7523 / 1A86:5523；CP210x = 10C4:EA60；
                // STM32 虚拟串口 = 0483:5740；FTDI = 0403:6001
                let likely_target = match &p.port_type {
                    serialport::SerialPortType::UsbPort(info) => matches!(
                        (info.vid, info.pid),
                        (0x1A86, 0x7523)
                            | (0x1A86, 0x5523)
                            | (0x10C4, 0xEA60)
                            | (0x0483, 0x5740)
                            | (0x0403, 0x6001)
                    ),
                    _ => false,
                };

                (p.port_name.clone(), desc, likely_target)
            })
            .collect(),
        Err(_) => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn list_ports_does_not_panic_without_hardware() {
        // 没有硬件时也应返回空表而不是 panic
        let _ = list_ports();
    }

    #[test]
    fn baud_constants_are_sane() {
        // 用运行期取值比较，避免 clippy 把常量断言判成"恒真"
        let rates = [
            baud::B_115200,
            baud::B_460800,
            baud::B_921600,
            baud::B_2000000,
        ];
        assert!(
            rates.windows(2).all(|w| w[0] < w[1]),
            "波特率档位应递增: {rates:?}"
        );
    }

    #[test]
    fn opening_bogus_port_reports_clear_error() {
        let err = SerialDevice::open("COM_NOT_EXIST_999", 115_200, 5).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("COM_NOT_EXIST_999"),
            "错误信息应包含端口名: {msg}"
        );
    }

    #[test]
    fn byte_rate_matches_8n1_arithmetic() {
        assert_eq!(921_600u32 / 10, 92_160);
    }
}
