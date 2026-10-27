# 13 · 阶段性总结（2026-10-27）

> 本文是截至 2026-10-27 的阶段性快照：系统框架、各部分功能、MCU 引脚占用
> 与真机验证状态。所有数字均可回溯到仓库内的代码、测试或本文标注的出处；
> 引脚表以 [02-hardware.md](02-hardware.md) §2 为唯一依据（已与
> `Core/Src/gpio.c` / `tim.c` / `usart.c` 逐项核对）。

---

## 1. 项目定位

**EvidenceScope**（原 F103 Agent Scope）—— 一台**能被 AI Agent 直接操控**的
示波器 / I2C / UART 总线分析仪。两条设计主线：

1. **Agent 单条指令完成采集 → 解码 → 测量 → 判决**（MCP 16 个工具）；
2. **每个测量量附带可信度判决**：31 个判决键中只允许 4 个 `Ok`
   （3 个整数计数 + σ_TIE），无不确定度模型的量一律 `Refused`，
   不输出看似精确的无依据数值。

不是计量仪器：不承诺绝对精度，承诺的是**证据可审计**。

---

## 2. 系统框架

```
外部 Agent（Claude Code 等）        桌面 GUI（scope-gui）
      │  MCP over stdio             │ AI 面板 + 「交给 AI 采集」
      ▼                             ▼
┌─────────────┐              ┌──────────────┐
│  scope-mcp  │              │  scope-gui   │──AI 分析（HTTPS，Anthropic 格式）
│  16 个工具  │              │  egui/eframe │
└──────┬──────┘              └──────┬───────┘
       └────────────┬───────────────┘
                    ▼
        ┌───────────────────────┐
        │ scope-core 命令与分析层 │  CommandBus / acquire / measure / analysis
        └───────────┬───────────┘
                    │ DevicePort（trait，唯一解耦点）
        ┌───────────┼─────────────┐
        ▼           ▼             ▼
  UART 921600   模拟器 sim     （CSV 导入）
        └───── AA55 帧 / CRC16 / 分片 ─────┐
                    ▼                      │
        ┌─────────────────────┐   CSV ◄───┘
        │ STM32F103C8T6 固件  │
        │  双通路：ADC + TIM  │
        └─────────────────────┘
```

三条「通往模型」的路径：外部 Agent 走 MCP；GUI AI 面板解释当前一窗；
GUI「交给 AI 采集」把设备让给 `scope-mcp` 子进程、GUI 充当 MCP 客户端。
真机串口独占，三路不能同时连同一设备。

---

## 3. 各部分功能

### 3.1 固件（`firmware/`，C，Keil MDK 工程）

| 层 | 内容 | 状态 |
|---|---|---|
| `Core/` | CubeMX 生成的外设初始化（ADC1+DMA、TIM2/3/4、USART1、SPI1、GPIO） | 在用 |
| `Hardware/` | 外设驱动：ADC-DMA 环形缓冲、TFT 驱动（显示已**冻结**：`display.c` 移出构建）、比较器输入捕获 | 在用 |
| `App/` | 协议任务 `proto_task.c`（AA55 帧 / CRC16 / seq / 超时）、采集 `acq.c`（状态机 + 触发搜索）、触发 `trigger.c`、本机输入 `local_*.c` | 在用 |
| `tests/` | App 层 PC 端测试：**7 个程序 / 252 项断言**（gcc 直接编译，不需要板子） | ✅ 全过 |

固件侧 `measure.c`、`i2c_decode.c`、`ui.c` **未实现**（测量与解码在主机侧完成）。
App 层禁 `#include` HAL（ADR-008），单帧栈预算 512 B（CI 用 `-fstack-usage` 钉住）。

### 3.2 协议（`proto/`，唯一真相源）

`protocol.h`（C 头）+ `protocol.c`（纯 C 编解码）+ `tests/vectors.json`
（黄金向量）。**改协议 = 同步四处**（docs/03、protocol.h、`scope-proto`
lib.rs、黄金向量），CI 的 `proto-c` job 盯着漂移。C 端测试 50 项全过。

### 3.3 主机（`host/`，Rust workspace，8 个 crate）

| crate | 功能 | 要点 |
|---|---|---|
| `scope-proto` | 线协议 Rust 实现 | 与 C 端逐字节等价，同一份黄金向量 |
| `scope-core` | **分析唯一来源**：`measure`（14 量）→ `analysis`（AnalysisReport / 31 键判决 / `verdict_line` 唯一判决文案）→ `report`（证据包 ≤8000 字符/64 帧）；`decoder`（I2C / UART，**极性自检**）；`acquire`（采集编排：SET_SAMPLE_RATE → SET_ACQ → SET_TRIGGER → ARM）；`calib`（标定表）/ `persist`（写入闸门）/ `import`（CSV） | 双电平平台门限（F7）、极性双遍裁决（G4） |
| `scope-transport-serial` | 串口链路（`SerialDevice`，COM 口抽象） | 921600 实测稳定 |
| `scope-sim` | 模拟器：18 个场景、6 个故障注入字段、`CapsMode`（默认与真机 DEV_CAPS 逐位一致） | Agent 逻辑切 sim 零改动 |
| `scope-device` | `DevicePort` trait + `CommandBus`（编码 / seq / 超时重试 / 状态缓存） | 唯一解耦点 |
| `scope-cli` | 命令行：`ports / sim / serial / import / cal`，capture / decode（I2C/UART）/ measure | 测量行带可信度判决（F8） |
| `scope-mcp` | MCP Server：**16 个工具**（默认 15 + `scope_debug_raw` 需环境变量），三层 token 防护 | P3 验收场景可复现 |
| `scope-gui` | egui 桌面：设备 / 配置 / 波形 / 总线解码（I2C+UART）/ 历史测量（判决行 + 瞬态与 TIE + CSV 导出）/ AI 分析（含导出 .md）/ AI 自采集 / 故障注入 / 日志 / 帮助 | 88 项测试 |

### 3.4 支撑

- `tools/agent_demo/`：零依赖 MCP 参考客户端（`docs/08` 实录的生成程序）
- `.github/workflows/ci.yml`：**5 个 job**（C 向量 / Rust / GUI / 固件分层纪律 /
  端到端冒烟），当前全绿（#50）
- `docs/00–12`：设计文档（每个数字有出处）

---

## 4. MCU 引脚占用与功能（F103C8T6）

依据 [02-hardware.md](02-hardware.md) §2，已与固件初始化代码核对。
「真机状态」指 2026-10-25/26 G4 首日实测。

| 引脚 | 外设 / 信号 | 功能 | 方向 | 真机状态 |
|---|---|---|---|---|
| **PA3** | ADC12_IN3 | 模拟输入：TL072 → ADC，**857 142 Hz** 采样（TIM4_CC4 触发 + DMA1_Ch1 进 8 KB 环） | 输入 | ✅ 已实测（DC 工作点 ≈2.66 V、UART 波形采集成功） |
| **PA6** | TIM3_CH1 输入捕获 | 数字通路：LM393 比较器输出 → 13.9 ns 边沿时间戳（协议解码用） | 输入 | ⬜ 待真机验证（必测第 10 项） |
| **PA2** | TIM2_CH3 PWM | 底板函数发生器输出：**1 kHz / 50% / 3.3 V**（PSC=71、ARR=999、Pulse=500）——可当内置已知信号源 | 输出 | ✅ 已实测（G4 首日即此信号） |
| **PA9 / PA10** | USART1 TX / RX | 协议链路（AA55 帧 / CRC16），经外接 CH340 模块，921600 8N1 | 双向 | ✅ 已实测（first light + 4096 点采集 + 解码全通） |
| PA5 / PA7 | SPI1 SCK / MOSI | TFT 屏 SPI0（未引出 MISO） | 输出 | 屏幕已冻结（`display.c` 不在构建） |
| PB5 / PB6 / PB7 / PB8 | GPIO | TFT RES / DC / CS / BL | 输出 | 同上（默认拉高） |
| PB3 / PB4 / PB9 | EXTI / GPIO | EC11 编码器 B 相 / A 相 / 按下（本机 UI 输入） | 输入 | 冻结（`SCOPE_LOCAL_UI=0`） |
| PB13 / PB14 / PB15 | GPIO | 三个独立按键（接地侧，软件消抖） | 输入 | 冻结 |
| PC14 / PC15 | GPIO | 底板 LED2 / LED3（**低电平点亮**） | 输出 | 冻结 |
| PA13 / PA14 | SWDIO / SWCLK | DAP-Link 烧录调试 | 双向 | ✅ 烧录通道 |
| 🚫 **USART2** | PA2 / PA3 | **硬冲突禁用**（PA2=PWM、PA3=ADC） | — | — |

内部资源（无引脚）：ADC1 + DMA1_Ch1（采样搬运）、TIM4_CC4（ADC 外部触发，
运行时按量化采样率重设 ARR/CCR4）、TIM3 中断（PA6 边沿时间戳）。

---

## 5. 阶段成果（截至 2026-10-27）

### 5.1 完成度声明

**代码与模拟器维度：项目已完成。** 固件（采集 + 协议）、主机 8 crate、
四个入口（CLI / GUI / MCP / 参考客户端）、质量门（476 + 252 + 50 + CI 5 job）
全部就绪且相互一致；模拟器能力可全量声明。

**声明边界**（按判决纪律，不声明未验证的能力）：
- ✅ 可声明：模拟器全量能力；真机**采集链路成立**（921600 链路、
  857 142 Hz 时基、CSV 契约、静默噪声 pp=6 LSB——G4 首日实测）；
  **真机 UART 解码成立**（2026-10-27：48 帧 / frame_errors = 0 /
  字节逐个递增与发送序列一致，自同步图样 [递增字节,FF,FF]）；
  解码逻辑含真机反馈修复（极性 / 计数器 / 触发顺序三处分歧）。
- ⬜ 待闭环后声明：真机 I2C 解码（需 ADR-010 第二路 ADC）；
  LM393 数字通路（PA6）真机核对；必测清单其余项（docs/02 §9）。

### 5.2 成果数字

| 维度 | 数字 |
|---|---|
| 主机测试 | **476 passed / 5 ignored**（CI #50 全绿） |
| 固件 PC 断言 | 252 项（7 程序）/ 0 Error 0 Warning / Flash 19.02 KB |
| C 协议向量 | 50 项 |
| 模拟器 | 18 场景（含 `clock_62k` 数值 TIE）、6 类故障注入 |
| MCP | 16 工具 / 三层 token 防护 / schema 与实现一致性有测试钉住 |
| CI | 5 job 全绿（含端到端冒烟） |

**真机验证（G4）进度**：first light 完成（链路 / 时基 / CSV 契约 / 静默噪声
pp=6 LSB）；**当场抓到并修复三个 sim/真机分歧**——SET_TRIGGER 顺序
（`16eeb94`）、UART 前端反相（解码极性双遍裁决）、反相流计数器下溢
（`22f2954`）；解码器已锁帧，卡点在 **TL072 容性负载自激**（鳄鱼夹长引线，
处方：短线 / 串 1 kΩ / 短同轴线，详见 HANDOFF §2.4）。

## 6. 边界与下一步

- 能：I2C 解码 100k/400k/1M（数字通路）、UART 8N1（模拟通路，每比特 ≥4 样点）、
  模拟观察 ≤100 kHz、双通道 857 kSPS、单次 4096 点。
- 不能：I2C 时序合规性 ns 级判定、THD/SFDR、自动量程、1 MSPS 与 USB 并存。
- 下一步：① 排除引线自激后完成 G4 UART 验收（`0x55 0x0F 0xA5` +
  `frame_errors=0`）；② 十二项实板必测清单回填 docs/02；③ ADR-010
  （第二路 ADC 脚位）拍板后开启 I2C 双通道真机演示。
