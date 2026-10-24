# F103 Agent Scope

[![CI](https://github.com/idswsy/f103-agent-scope/actions/workflows/ci.yml/badge.svg)](https://github.com/idswsy/f103-agent-scope/actions/workflows/ci.yml)

> 一台可由 AI Agent 直接操控的数字示波器与 I2C 总线分析仪。
> 硬件：立创开源《简易数字示波器设计（入门版）》插件底板 +
> STM32F103C8T6 最小系统板。

---

## 1 概述

### 1.1 项目定位

本项目实现一台可由 AI Agent 直接操控的调试仪器：Agent 通过 MCP 工具调用完成
波形采集、协议解码与信号质量判定，无需人工逐项配置。

适用场合为低频信号观察与总线协议调试（电源纹波、音频级信号、I2C/UART/SPI）。
本项目不作为计量仪器使用，不提供绝对精度指标。

### 1.2 双通路设计

I2C 解码的瓶颈是边沿时间精度，不是采样率。857 kSPS 的 ADC 解 400 kHz I2C
需要 3.33 MSPS，且存在采样相位拍频问题（主从两侧晶振 40 ppm 频差导致解码
周期性失效），属采样体制的固有限制，软件层无法补偿。

底板上的 LM393 滞回比较器接至定时器输入捕获后，直接输出 72 MHz / 13.9 ns
的边沿时间戳（等效约 72 MSPS），不占用 ADC 与内存带宽。表 1 给出两条通路的分工。

**表 1. 通路分工**

| 通路 | 硬件路径 | 用途 | 能力 |
|---|---|---|---|
| 协议通路（权威） | LM393 → TIM 输入捕获 | I2C 帧解码、时序、时钟拉伸 | 100 kHz / 400 kHz / 1 MHz 全部可靠，13.9 ns 分辨率 |
| 模拟通路（差异化） | TL072 → ADC | 电平裕量、上拉强度、振铃、边沿形状 | 100 kHz 保证 / 400 kHz 有条件 / 1 MHz 不支持 |

两条通路互补：低成本逻辑分析仪只给出协议是否正确，本项目同时给出协议是否正确
与信号是否良好。

### 1.3 硬件平台

**表 2. 系统部件**

| 部件 | 型号 | 说明 |
|---|---|---|
| 插件底板 | 立创开源《简易数字示波器设计（入门版）》 | 模拟前端、比较器、TFT、编码器、按键 |
| 核心板 | STM32F103C8T6 最小系统板 | 64 KB Flash / 20 KB SRAM；串口经外接 CH340 模块 |

平台的硬件约束（无以太网、无 FSMC、2 个 ADC）与选型过程见
[docs/00-origin.md](docs/00-origin.md)、[docs/02-hardware.md](docs/02-hardware.md)。

---

## 2 能力与边界

完整推导见 [docs/04-performance.md](docs/04-performance.md)。表 3 为摘要。

**表 3. 能力与边界**

| 指标 | 支持 | 不支持 |
|---|---|---|
| 采样率 | 单通道 **857.14 kSPS**（12 bit）；单通道交织 **1.7143 MSPS**（仅显示，不作解码依据） | 1 MSPS 与 USB 并存；双通道同步 857 kSPS/路（需 P4 改板，见 7.2 节） |
| 存储深度 | 单次 **4096 点**（8 KB 环） | 深存储、外扩 SRAM（C8 无 FSMC） |
| I2C 解码 | **100 kHz / 400 kHz / 1 MHz 全部可靠**（数字通路） | 时序合规性验证（t<sub>SU;DAT</sub> / t<sub>r</sub> 的 ns 级判定） |
| 模拟带宽 | ≤ 100 kHz 保证 | > 0.5 MHz |
| 上传 | 单次 8 KB：USB 约 12 ms / UART 921600 约 91 ms | 1.71 MB/s 原始数据连续流 |
| 测量 | Vpp / 频率 / 占空比 / RMS / 上升与下降时间 / 过冲与下冲 / 高电平脉宽 / 边沿数（定点） | THD / SFDR / ENOB；12 bit 绝对精度 |
| 波形分类 | 直流 / 方波 / 脉冲 / 阶跃响应 / 正弦 / 调幅 / 噪声。判不出时给出**原因**（未检出周期、采样密度不足…），不硬猜 | 三角 / 锯齿（波形因数与正弦只差 0.065，要 rise/fall 不对称才能分开）；抖动（857 kSPS 下测不出） |
| 量程切换 | — | 自动量程（SW2/SW3 为机械开关，见 7.3 节） |

> 采样率上限为 857.14 kSPS，**不得写作 1 MSPS**。推导：PCLK2 = 72 MHz 时
> ADCCLK 经 /6 得 12 MHz（/2、/4 超出 14 MHz 上限），12 bit 最短转换时间
> 1.5 + 12.5 = 14 个 ADCCLK，即 1.1667 µs/点。

---

## 3 系统架构

```
  外部 Agent（Claude Code 等）        桌面 GUI（scope-gui）
        │                                  │          │
        │  MCP over stdio            HTTPS + 密钥      │ 启动子进程
        ▼                                  ▼          ▼
  ┌──────────────┐              ┌──────────────┐  ┌──────────────┐
  │  scope-mcp   │              │  AI 分析面板  │  │  scope-mcp   │
  │  MCP Server  │              │  单次提问     │  │  AI 自采集    │
  └──────┬───────┘              └──────┬───────┘  └──────┬───────┘
         └──────────────┬──────────────┴─────────────────┘
                        ▼
              ┌──────────────────┐
              │  scope-core 命令层 │  CommandBus：编码 / seq / 超时重试 / 状态缓存
              └────────┬─────────┘
                       │  DevicePort（trait，唯一解耦点，6 个方法）
          ┌────────────┼────────────┐
          ▼            ▼            ▼
   UART 921600     USB CDC      模拟器 sim
          └────────────┴────────────┘
                       │  AA55 帧 / CRC16 / 分片
              ┌────────▼─────────┐
              │ STM32F103C8T6 固件 │
              │  ├ 模拟通路：TIM4_CC4 → ADC1 → DMA1Ch1 → 8 KB 环
              │  └ 协议通路：LM393 → TIM 输入捕获（13.9 ns）
              └──────────────────┘
```

通往模型共三条路径，适用场合不同：

1. **外部 Agent 走 MCP**（`scope-mcp`）—— Agent 主导，自行决定采什么。
2. **GUI「AI 分析」面板** —— 人工主导，模型解释当前显示的一窗。
3. **GUI「交给 AI 采集」** —— GUI 将设备让给 `scope-mcp` 子进程并充当
   MCP 客户端，由 Agent 自行配置、采集、分析；详见 5.4.4 节。

真机串口为独占资源，三条路径不能同时连接同一台设备。

---

## 4 仓库结构

```
F103/
├─ docs/          设计文档 00~09（起源 / 架构 / 硬件 / 协议 / 性能边界 /
│                 路线图 / 决策记录 / 开发环境 / Agent 工作流实录 / 实时显示）
├─ proto/         协议唯一真相源：C 头文件 + 纯 C 编解码 + 双端共用的黄金测试向量
├─ firmware/      STM32F103 固件（C，分层 App/Hardware）
├─ host/          Rust workspace，8 个 crate：
│                   proto · core · transport-serial · sim · device · cli · gui · mcp
├─ hardware/      立创底板资料与改板设计
├─ tools/         独立工具：agent_demo
└─ .github/       CI：5 个 job（C 向量 / Rust / GUI / 固件分层检查 / 端到端冒烟）
```

**协议真相源策略。** 固件用 C、上位机用 Rust，跨语言一致性无法由编译器保证。
[`proto/tests/vectors.json`](proto/tests/vectors.json) 是唯一契约：同一份黄金向量
由 Rust `#[test]` 与不依赖 HAL 的纯 C 测试两端执行，漂移在测试期暴露。
协议变更需同时修改四处（`docs/03-protocol.md`、`proto/protocol.h`、
`host/crates/proto/src/lib.rs`、黄金向量），见
[docs/07-dev-env.md](docs/07-dev-env.md) §5。

---

## 5 快速开始

全部命令在仓库根目录执行。构建一律使用 `./host/run.sh`，不使用裸 `cargo`：
该脚本把 `CARGO_TARGET_DIR` 指向纯 ASCII 路径（MinGW 的 `ld.exe` 无法打开
含中文的路径；详见 [docs/07-dev-env.md](docs/07-dev-env.md) 坑 #1），并统一构建参数。

> ⚠ `run.sh` 内部会 `cd` 到 `host/`，因此 `-o` 的相对路径相对 `host/` 解析。
> 需要落在仓库根目录时写 `-o ../wave.csv` 或使用绝对路径。

环境要求与安装命令见 [docs/07-dev-env.md](docs/07-dev-env.md) §1。

### 5.1 无硬件验证

以下四条全部通过即表示环境就绪，不需要硬件。

```bash
# 1. 协议测试：C 与 Rust 两端跑同一份黄金向量
./proto/run_tests.sh            # C 端，50 项
./host/run.sh test              # Rust 端：要求所有 test result 为 ok（条数不写死）
./firmware/run_tests.sh         # 固件 App 层，7 个测试程序 / 252 项断言，在 PC 上运行

# 2. 对模拟器采集一次波形
./host/run.sh run -p scope-cli -- sim capture --scenario sine_1k_3v3 -n 1024 -o wave.csv

# 3. 查看 I2C 双通道解码
./host/run.sh run -p scope-cli -- sim capture --scenario i2c_100k -n 2048 -o i2c.csv
./host/run.sh run -p scope-cli -- sim decode --protocol i2c --scenario i2c_100k -n 4096

# 3c. UART 8N1 单通道解码（真机 1 通道可用的那条路径）
./host/run.sh run -p scope-cli -- sim decode --protocol uart --scenario uart_115k -n 4096

# 3b. 非 I2C 波形（共 15 个场景）：PWM 占空比 / 阶跃响应（过冲+振铃）/ RC / UART
./host/run.sh run -p scope-cli -- measure --scenario pwm_1k_25
./host/run.sh run -p scope-cli -- measure --scenario step_ring

# 4. MCP 工具链自检
./host/run.sh run -p scope-mcp -- --selftest
```

模拟器实现同一套 `DevicePort`，内含 F103 真实档位表、ARM/触发语义、状态机约束、
seq 去重缓存，以及 6 个故障注入字段（丢帧 / CRC 错 / 延迟尖峰概率 / 尖峰时长 /
永不触发 / 强制溢出）。CLI、GUI、MCP 切至 `sim` 后 Agent 逻辑无需改动。
模拟器不按波特率节流；链路的实际速率只影响命令层的超时估算。

**模拟器场景（15 个）**

`--scenario <名称>` 的取值。场景名同时是 CLI 参数、MCP schema 的 `sim_scenario`
枚举、GUI 下拉项与证据包里的链路描述 —— 同一份清单，三处不手抄。

| 名称 | 波形 | 用途 |
|---|---|---|
| `sine_1k_3v3` | 1 kHz 正弦，3.3 Vpp | 最基本的「能看到东西」 |
| `square_50k` | 50 kHz 方波 | 高频下的混叠与边沿 |
| `pulse_glitch` | 带单样点毛刺的脉冲串 | minmax 预览是否保住尖峰 |
| `noise` | 白噪声 | 质量标志与「不要回垃圾数」 |
| `dc` | 纯直流 | FORCE_TRIGGER 与 auto 模式 |
| `am` | 20 kHz 载波 / 200 Hz 包络调幅 | RMS / 包络类测量 |
| `i2c_100k` | 100 kHz I2C（SCL + SDA，**双通道**） | 解码器回归的主场景 |
| `i2c_400k` | 400 kHz I2C（双通道） | 「模拟通路解不了、数字通路能解」的边界 |
| `i2c_nack` | 含一个 NACK 的 I2C 总线（双通道） | 验收场景「告诉我为什么 NACK」 |
| `pwm_1k_25` | 1 kHz、25% 方波 | 占空比测量（偏离 50% 才有意义） |
| `pwm_1k_75` | 1 kHz、75% 方波 | 上一条的镜像：只测窄脉冲抓不到「100−duty」这类错 |
| `pwm_1k_5` | 1 kHz、5% 窄脉冲 | 双电平检测最危险的退化情形（平台样点数差 19 倍） |
| `step_ring` | 500 Hz 设定方波经二阶欠阻尼系统 | **PID / 控制系统**：过冲 ≈31%、12 kHz 衰减振铃 |
| `rc_charge` | 一阶 RC，τ = 150 µs | 指数曲线；同时验证「不是方波就别硬叫方波」 |
| `uart_115k` | 115200 baud，三字节循环 + 空闲位 | 数字时序但非 I2C；帧边界在波形上可见 |

> `step_ring` 的过冲是 `exp(-πζ/√(1-ζ²))` 在 ζ = 0.35 下的值 —— 场景文档、
> 实现与测试断言引用的是同一个解析式，不是互相抄。
>
> ⚠ 857.14 kSPS 采样下 `uart_115k` 每位只有 **7.4 个样点**：波形看得见，脉宽
> 与边沿时间都被采样分辨率卡住，读数只当相对量看。

### 5.2 桌面 GUI

```bash
# 自动连接模拟器并采集一帧
./host/run.sh run -p scope-gui -- --demo --scenario i2c_100k
```

`scope-gui` 为 egui/eframe 0.36 桌面应用，默认窗口 1180×760。面板组成：
设备、配置、波形（泳道显示、缩放平移、触发点、判决带）、I2C 解码（双泳道 +
色标 + 交易表 + 信号质量）、历史采集、测量、导出（CSV / 解码结果 / 结论 .md）、
AI 分析、故障注入、日志、帮助。

**表 4. 命令行启动参数**

| 参数 | 作用 |
|---|---|
| `--demo` | 免交互启动并采集一帧 |
| `--demo-cont` | `--demo` 的加强版：采完第一帧后打开「连续刷新」（脚本验证交互用） |
| `--scenario <名称>` | 指定初始场景 |
| `--font <路径>` | 指定中文字体 |
| `--drive "<需求>"` | 无窗口启动一次 AI 自采集会话，见 5.4.4 节 |
| `--port <串口>` | 仅与 `--drive` 联用：把会话的目标从模拟器换成真机 |

### 5.3 连接实机

```bash
# 列出可用串口
./host/run.sh run -p scope-cli -- ports

# 连接实机采集（--port / --baud 位于 serial 之后、子命令之前）
./host/run.sh run -p scope-cli -- serial --port COM3 --baud 921600 capture -n 2048 -o wave.csv
```

串口默认参数 921600 8N1。固件侧编译状态、栈预算与烧录接线见
[firmware/README.md](firmware/README.md)；真机采集尚未验证。
接线细节另见 [docs/02-hardware.md](docs/02-hardware.md) §3。

### 5.4 接入 AI Agent

#### 5.4.1 MCP 服务

`scope-mcp` 是运行在 stdio 上的标准 MCP Server，每行一条 JSON-RPC 消息。
注册 15 个工具：

`list_devices`、`connect`、`disconnect`、`status`、`configure`、`capture`、
`read_waveform`、`measure`、`decode`、`i2c_decode`、`list_captures`、`save_capture`、
`watch`、`sim_set_scenario`、`debug_raw`。

`scope_decode` 是统一解码入口（`protocol=i2c|uart`；UART 8N1 单通道，真机可用），
`scope_i2c_decode` 是它的兼容别名。

`scope_debug_raw` 仅在环境变量 `SCOPE_MCP_DEBUG=1` 时注册，故默认 `tools/list`
返回 14 个。工具的参数 schema 由 Rust 参数类型（schemars）现场生成，
并有测试钉住「schema 声明的字段集等于实现能解析的字段集」。

```bash
# 接入 Claude Code
claude mcp add scope -- <仓库绝对路径>/host/target/debug/scope-mcp.exe

# 或直接以 JSON-RPC 对话
printf '%s\n' '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | ./host/target/debug/scope-mcp.exe

# 查看工具清单与三层 token 防护说明
./host/run.sh run -p scope-mcp -- --list-tools
```

**三层 token 防护。** `scope_capture` 与 `scope_watch` 默认只返回统计量与
不超过 256 点的 minmax 预览；`scope_read_waveform` 分页返回，默认 512 点，
硬上限 4096 点，超出请求直接拒绝而不截断；全量数据只写入 capture store 与磁盘。

> ⚠ `capture_id` 的作用域是进程。一次管道输入即一个会话，进程退出后
> `capture_id` 作废，下次从 1 重新开始。因此「先采集后读取」必须把
> `scope_capture` 与后续的 `scope_read_waveform` / `scope_measure` /
> `scope_i2c_decode` 送入同一个进程。

#### 5.4.2 参考客户端

[`tools/agent_demo/`](tools/agent_demo/) 是一个零依赖的 MCP 客户端（仅用 Python
标准库）：启动 `scope-mcp` 子进程 → `tools/list` 取得 schema → 转换为语言模型的
工具表 → 运行 tool-use 循环直至模型给出结论。它同时是
[docs/08-agent-walkthrough.md](docs/08-agent-walkthrough.md) 那份实录的生成程序。

```bash
./host/run.sh build -p scope-mcp
export DEEPSEEK_API_KEY=...        # 或 ANTHROPIC_API_KEY
python tools/agent_demo/agent.py
```

#### 5.4.3 GUI「AI 分析」面板

将当前采集**已经算好的**证据（链路与设备信息、配置回显、采集统计、通道测量、
**信号分类**、32 桶 minmax 包络、总线解码帧表与信号质量）发送至语言模型，
结论直接显示在界面上。证据包由 `scope_core::report::build_evidence` 生成，
只转述 `scope_core::measure`、`scope_core::signal` 与解码器的计算结果，
不含原始样点数组；上限 8000 字符、64 帧。

信号分类见 §2 表 3，实现于 `core/src/signal.rs`。AI 按「场景」行与「波形形状」段
选分析分支，非 I2C 波形（PWM、PID 阶跃响应）有专门的诊断指引。

请求格式为 Anthropic Messages 格式，经 HTTPS 单发。配置存于用户配置目录
（Windows：`%APPDATA%\scope-gui\config.json`），**密钥为明文存储**。
默认端点为 `api.deepseek.com/anthropic/v1/messages`，默认模型 `deepseek-flash`。
分析在独立线程执行，连接超时 10 s、总超时 60 s、`max_tokens` 8192
（与 §5.4.4 的会话**共用同一个常量**）。
响应被服务端按 `max_tokens` 截断时，结论照常显示，但上方会标出「内容不完整」。

#### 5.4.4 GUI「交给 AI 采集」

工具栏的「交给 AI 采集」按钮执行设备交接：断开 GUI 与设备的连接以释放串口，
启动 `scope-mcp` 子进程并由 GUI 充当 MCP 客户端，运行语言模型的原生工具调用
循环，由 Agent 自行配置、采集与分析。见表 5。

**表 5. AI 自采集会话参数**

| 参数 | 取值 |
|---|---|
| 工具循环上限 | 24 轮 |
| 单次模型请求 | 8192 tokens |
| 单条工具结果上限 | 12000 字符（超出时带说明截断） |
| LLM 超时 | 180 s |
| JSON-RPC 超时 | 30 s |

Agent 采集到的新波形由 GUI 自行调用 `scope_list_captures` 与
`scope_read_waveform` 取回并显示，这些调用不进入模型消息。会话结束或失败时，
GUI 按交接快照自动重连设备；运行期间可「终止」；设备收回后可用
「恢复交接前的设置」还原交接前的配置。交接期间设备控件全部停用。

以下是已知限制：

- **AI 操作期间 GUI 无法访问设备。** 串口为独占资源，这是链路层的必然结果，
  非界面设计选择。界面在此期间显示设备已交出。
- **AI 修改过的 `range_idx` 与 `offset_lsb` 在重连后无法读回。** `GET_CONFIG`
  不回报这两个字段，因此只能在动作轨迹中如实记录。
- `connect` 工具被 GUI 钉死为交接快照中的目标，Agent 不能连接其他设备。

---

## 6 路线图

**表 6. 路线图**

| 阶段 | 目标 | 状态 | 验收标准 |
|---|---|---|---|
| P0 | 协议闭环（无硬件） | 完成 | 黄金向量在 C 与 Rust 两端全绿；`sim` 可完成 capture |
| P1 | 最简硬件闭环 | 固件侧可编译（252 项 PC 断言，0 Error / 0 Warning）；**真机采集未验证** | F103 采 2048 点 → 串口 → CLI 落 CSV → 绘图 |
| P2 | 数字通路 I2C 解码 | 进行中：主机侧完成；**固件侧未开始**（`App/i2c_decode.c` 与边沿时间戳缓冲均未创建） | 100 kHz / 400 kHz / 1 MHz 实总线解码结果与逻辑分析仪一致 |
| P3 | MCP + Agent | 完成，见 [docs/08](docs/08-agent-walkthrough.md) | Agent 一句话完成「抓一次 I2C 写时序并说明 NACK 原因」 |
| P4 | 增强 | 进行中：GUI、历史、导出、AI 分析面板已完成；板载屏幕已**冻结**（`display.c` 移出 Keil 工程，代码保留，见 [docs/09](docs/09-realtime-display.md) §4.4）；改板与自动量程未做 | 双通道改板、自动量程、等效时间采样 |
| P5 | 主板升级 STM32F407（后期计划） | 未开始 | 以太网 / FSMC / 3 个 ADC：7.2 MSPS、深存储、原始波形直传 |

详见 [docs/05-roadmap.md](docs/05-roadmap.md)。

---

## 7 硬件约束

完整内容见 [docs/02-hardware.md](docs/02-hardware.md)。
以下三条直接影响软件能力边界，接线前必须确认。

### 7.1 数据链路

串口经**外接 CH340 模块**接到 USART1，**需要接四根线**（模块 TX→PA10、模块 RX→PA9、GND↔GND、VCC）。

**表 7. 数据链路选型**

| 优先级 | 链路 | 引脚 | 说明 |
|---|---|---|---|
| 主力 | USART1 + 外接 CH340 模块 | PA9 (TX) / PA10 (RX) | 四根线（TX/RX **交叉**）；921600 下 92 KB/s |
| 备选 | USB CDC | PA11 / PA12 | 500–900 KB/s，需自行实现 USB device 固件 |
| 🚫 禁用 | USART2 | PA2 / PA3 | PA2 为底板 PWM 输出，PA3 为模拟输入，硬冲突 |

两条可用链路在操作系统层都表现为 COM 口，由同一个 `SerialDevice` 实现覆盖，
协议层不受链路差异影响（[ADR-012](docs/06-decisions.md)）。

> 核心板**到底有没有**板载桥、以及外接模块的最高可靠波特率，均标为 `【待核实】`（[ADR-013](docs/06-decisions.md)）。

### 7.2 模拟输入只有一路

底板只有一路模拟输入（BNC → TL072 → **PA3 / ADC_IN3**），而 I2C 解码需要
SCL 与 SDA 两路同时刻采样。P0–P3 采用「单路 ADC + 一路数字」：一路经 ADC 观察
模拟质量，另一路经 LM393 → PA6 输入捕获解码协议。其余改板方案与工作量见
[hardware/README.md](hardware/README.md) §3 与 [ADR-010](docs/06-decisions.md)。

### 7.3 耦合与量程为机械开关

SW2（AC/DC 耦合）与 SW3（X1/X50 衰减）为手拨开关，AI 与固件均无法程控。
自动量程需要改板（改用继电器或模拟开关，如 CD4053）。

触发电平不受此限制：它由固件按采样值与 `trigger_level_lsb` 比较得出，可程控。

数字通路所用 LM393 的阈值由电阻分压固定（`Uth = 2.214 V` / `Utl = 2.172 V`，
见 [docs/02-hardware.md](docs/02-hardware.md) §5），不受 `SET_TRIGGER.level_lsb`
影响；该参数只作用于模拟通路的触发搜索。

---

## 8 许可与致谢

本项目的硬件设计基于以下上游开源工程。

**表 8. 上游来源与许可**

| 上游 | 内容 | 许可 |
|---|---|---|
| [立创开源《简易数字示波器设计（入门版）》](https://oshwhub.com/course-examples/yi-qi-yi-biao-jian-yi-shu-zi-shi-bo-qi-she-ji-cha-jian-ban) | 硬件设计（原理图 / PCB） | **GPL-3.0** |

- 硬件部分为上游设计的修改版本，按 GPL-3.0 发布。
- 上游页面明文禁止商业性使用。
- 本仓库整体以 GPL-3.0 发布，见 [`LICENSE`](LICENSE)。
- 固件中外设驱动层的来源另见 [`NOTICE.md`](NOTICE.md) §2。

致谢：立创开源硬件平台、立创EDA-莫工、EDA课程案例团队。
