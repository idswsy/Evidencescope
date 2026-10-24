# 01 · 系统架构

## 分层总览

```
┌─────────────────────────────────────────────────────────────────────┐
│  L5  Agent 层           Claude / 任意 LLM        GUI 内嵌 AI 面板   │
│                          ↕ MCP (JSON-RPC)         ↕ HTTPS（单发）    │
│  L4a MCP Server         14 个粗粒度工具，         L4b scope-gui     │
│                          schema 由参数类型生成     （人操作的界面）  │
│                          └──────────┬──────────────────┘            │
│                                     ↕ 直接调用（**阻塞**，无 async） │
│  L3  命令层 scope-core  CommandBus：编码 / seq / 超时重试 / 状态缓存│
│                                     ↕ DevicePort (trait)            │
│  L2  传输层             UART(serialport) │ sim                      │
│                          ↕ AA55 帧 / CRC16 / 分片                   │
│  L1  设备层             STM32F103C8T6 固件                          │
└─────────────────────────────────────────────────────────────────────┘
```

**解耦点只有一个**：`DevicePort`（6 个方法）。L3 以上完全不关心背后是真硬件还是模拟器 ——
换一种链路 = 实现这一个 trait，其余一行不动。

**L4 是两个同级消费方**，共用 L3：

| | 谁主导 | 通向模型的协议 | 何时用 |
|---|---|---|---|
| `scope-mcp` | **AI**（它决定采什么、怎么采） | MCP | 外部 AI 客户端接入 |
| `scope-gui` 的 AI 面板 | **人**（点按钮，AI 只解释） | HTTPS 单发 | 分析屏幕上正在看的那一窗 |

> ⚠ 真机串口是**独占**的，二者不能同时连同一台设备。所以「让 AI 分析你屏幕上
> 这一窗」**只能在 GUI 进程内做** —— 不是二选一，是那条路走不通。

> 全栈是**同步阻塞**的：`CommandBus` 的每个命令调用都会等到设备回包或超时。
> 没有 async runtime，`Cargo.toml` 里也没有 tokio。GUI 那边之所以要 worker 线程，
> 是因为绘制不能阻塞，不是为了并发 IO。**AI 面板另起第三条线程**同理 ——
> 一次模型调用是 15–60 秒，塞进 worker 会把设备操作全部卡住。

---

## F103 固件内部分层

```
firmware/
├─ App/            应用层（与硬件无关的逻辑，可 PC 上单元测试）
│   ├─ hal.h            硬件接口 —— 纯函数指针，无寄存器
│   ├─ proto_task       帧解析状态机 → 命令分发 → 响应编码
│   ├─ acq              采集状态机（IDLE / ARMED / DONE）
│   ├─ trigger          触发搜索（迟滞电平比较）
│   └─ waveform         采集窗 → 屏幕一帧（上下沿 / Vpp / 频率）
├─ Hardware/       硬件抽象（唯一碰寄存器的地方）
│   ├─ hal_impl         装配层：把硬件实现填进 `hal_t`
│   ├─ adc_dma          TIM4_CC4 → ADC1 → DMA1Ch1 → 8 KB 环
│   ├─ link_uart        USART1 收发环
│   ├─ display          TFT 渲染 + 分片刷新调度（见 ADR-014）
│   ├─ tft / tft_init   上游的 ST7735S 驱动（原样并入，见 NOTICE.md §2）
│   └─ scope_ui         上游那套本机 UI 的存档，`#if SCOPE_LOCAL_UI` 默认关
├─ Core/           CubeMX 生成：时钟、GPIO、外设初始化、中断向量
├─ Drivers/        ST HAL + CMSIS
├─ MDK-ARM/        Keil 工程（含 `.map`，构建后最值得读的文件）
├─ tests/          App 层测试（不需要板子）
└─ run_tests.sh
```

**规则**：`App/` 不许 `#include` 任何 HAL / 寄存器头文件（ADR-008）。
这样 `trigger.c`、`acq.c`、`proto_task.c`、`waveform.c` 可以在 PC 上用 gcc
加 mock 编译测试 —— 这是本项目在没有硬件时也能推进的关键。

`firmware/run_tests.sh` 覆盖 `App/`；`Hardware/` 与 `Core/` 由 CI 的
「硬件层语法检查」按 `-DSCOPE_LOCAL_UI=0` 与 `=1` 各编一遍兜住语法。

> **还没写的**：`App/measure.c`（定点测量）与 `App/i2c_decode.c`。
> 前者不阻塞任何主机功能 —— 主机侧的测量走 `core::measure()` 本地算，
> 从不调用设备的 `CMD_MEASURE`；后者卡在 ADR-010（本板只有一路比较器）。

---

## 数据通路：为什么是双通路

这是本项目最重要的架构决策，来自一次定量分析（见 [`04-performance.md`](04-performance.md)）。

### 硬约束

I2C 解码要求：**SCL 与 SDA 必须同时刻采样**，且每比特至少 2 个采样点才能判定 START/STOP（判据 `S ≥ 2/tHIGH`）。

| I2C 速率 | tHIGH | 需要采样率 | F103 单 ADC 857 kSPS 够吗 |
|---|---|---|---|
| 100 kHz | 4.0 µs | 0.5 MSPS | ✅ 够（8.6 点/bit，2× 裕量） |
| 400 kHz | 0.6 µs | 3.33 MSPS | ❌ 差 3.9 倍 |
| 1 MHz | 0.26 µs | 7.69 MSPS | ❌ 差 9 倍 |

而且 400 kHz 时 `857k/400k ≈ 15/7`，采样相位只有 **15 个离散值**（间隔 1/15 个比特周期）；两颗晶振 40 ppm 差会让解码**以 ~62 ms 为周期时好时坏地闪** —— 这不是 bug，是采样相位拍频，软件补不了。

### 解法：把「解码」和「看波形」拆开

```
                    ┌──────────────────────────────────────┐
   BNC ──► TL072 ──►│ ADC1/ADC2 (同步模式, 857 kSPS/路)     │──► 模拟通路
                    │   → 电平裕量 / 上拉强度 / 振铃 / 边沿形状 │
                    └──────────────────────────────────────┘
                    ┌──────────────────────────────────────┐
   输入 ──► LM393 ──►│ TIM 输入捕获 (72 MHz, 13.9 ns)        │──► 协议通路
                    │   → START/地址/ACK/数据/STOP 解码      │
                    └──────────────────────────────────────┘
```

**板上现成的 LM393 滞回比较器**（原设计只用来测频率）接输入捕获后：

- 边沿时间戳精度 **13.9 ns**（等效 72 MSPS）
- 边沿驱动解码 → **免疫混叠、免疫采样相位问题、免疫时钟拉伸**
- 100 kHz / 400 kHz / 1 MHz **全部 100% 可靠**

两条通路共用一个触发，时间对齐后同时呈现「协议对不对」+「信号好不好」。

> **代价**：数字通路只有 0/1 二值，丢失电平裕量信息 —— 正好由 ADC 通路补上。这就是为什么两条都要。

---

## 一次典型采集的时序

```
主机                                    设备
 │  SET_SAMPLE_RATE(857143)               │  量化到 TIM4 ARR=83 → 857142 Hz
 │ ◄──── ACTUAL(857142)  ← 必须回显实际值  │
 │  SET_TRIGGER(mode=normal, edge=↑, lvl) │
 │  ARM ─────────────────────────────────►│  状态 IDLE → ARMED
 │                                        │  TIM4_CC4 → ADC → DMA 循环写入 8KB 环
 │                                        │  HT/TC 中断每 2048 点发布一个半区
 │                                        │  主循环在已发布半区上搜索触发点（迟滞）
 │                                        │  找到 T → 继续采 T+post+512 余量 → 停 DMA
 │  ◄──── EVENT_TRIGGER{trigger_index}    │  状态 → DONE
 │  READ_BUFFER(cap_id, 0, 1024, RAW16) ─►│
 │  ◄──── 1024 样点 + 12B 分片头           │
 │  READ_BUFFER(cap_id, 1024, 1024, ...)─►│
 │  ◄──── ...                             │
 │  上位机侧：minmax 降采样 → 存 capture store → Agent 只拿到统计量+预览
```

---

## 命令语义三端一致

同一条命令在三端各是什么：

| 命令 | Agent 侧（MCP 工具） | 上位机侧（Rust） | 固件侧（C） |
|---|---|---|---|
| `SET_SAMPLE_RATE` | `scope_configure(sample_rate_hz=…)` | 编码 → `0x0201`，**以回显值建时间轴** | 量化到 TIM 实际分频 → 回显 `actual_hz` |
| `READ_BUFFER` | `scope_read_waveform(capture_id, …)` | 分片循环 + 超时重拉 | 按绝对样点号切片回发 |
| `MEASURE` | `scope_measure(capture_id, metrics=[…])` | 默认主机侧算（精度最高） | 可选，流模式快算 |

**关键纪律**：
- 主机的时间轴**永远**由 `(capture_id, start_sample, actual_rate_hz)` 推导，**绝不用分片到达时刻**。
- 一律 `SET_*` 回显实际生效值，主机以回显为准。
- 控制链路**永不出现 `f32`**（F103 无 FPU，软浮点慢且线上格式易被两端误解）—— 电平用 ADC LSB 整数，测量结果用带缩放字段名的定点（`mean_x256`、`duty_x10000`）。

> **送给模型的证据包**（`core/src/report.rs`）也守同一条纪律：它**只转述**
> `core::measure()`、`core::signal::classify()` 与 `I2cDecode` 已经算出的结果，
> 不重算任何信号处理。信号分类（波形形状段）是 2026-10-06 加入的 ——
> 渲染层只格式化 `classify` 返回的「判断 + 证据」，判错了读者能对着证据行看出来。
> 另注：GUI 用的是**用户手选**的 SCL/SDA 通道，MCP 才自动检测 ——
> 证据包里必须如实标注来源，否则会与界面上显示的解码结果不一致。

---

## 协议真相源的跨语言问题

固件 C、上位机 Rust，**编译器帮不上忙**。所以：

```
proto/
├─ protocol.h          ← C 端定义（#pragma pack + static_assert 锁偏移）
├─ protocol.c          ← 纯 C 编解码，不依赖 HAL，PC 可 gcc 编译
└─ tests/vectors.json  ← 唯一契约：黄金测试向量
```

同一份 `vectors.json` 被两端执行：

| 执行方 | 命令 | CI |
|---|---|---|
| C 端 | `make -C proto test`（gcc + 纯 C） | ✅ |
| Rust 端 | `cargo test -p scope-proto` | ✅ |

任何一端改了编码但没改另一处 → **测试立刻红**。

---

## 无硬件开发的支撑

| 组件 | 作用 |
|---|---|
| `host/crates/sim` | 实现同一 `DevicePort`，内含 F103 真实档位表、ARM/触发/环缓冲语义、状态机约束（`ARMED` 下发配置回 `BUSY`）、seq 去重缓存 |
| 故障注入 | [`FaultInjection`](../host/crates/sim/src/device.rs) 六个字段：`drop_every_n_frames` / `crc_err_every_n_frames` / `latency_spike_probability` / `latency_spike_ms` / `no_trigger` / `force_overrun` |
| `host/crates/device` | `Transport` enum：运行时在模拟器与串口之间切换（`Box<dyn DevicePort>` 不行的原因见该 crate 的文档头） |
| C 端纯逻辑测试 | `App/` 层不依赖 HAL → PC 上 gcc 编译跑同一批黄金向量 |

两种传输（`serialport` 串口 / `sim`）即插即用，切到 `transport='sim'` 后 **Agent 逻辑零改动**。

> **模拟器不受波特率限制。** `SimDevice::byte_rate()` 返回 `u32::MAX`，
> 它不会按波特率节流 —— 那样只会让测试变慢。`byte_rate` 的唯一用途是让
> 命令层**估算超时**（波特率越低，等一个分片要越久）。
