# 05 · 路线图

**原则：先让「无硬件也能开发」跑起来，再让硬件闭环，最后才做 AI。**

每个阶段都有**可验收的产出**，不写"完成 XX 模块"这种没法验收的话。

---

## P0 · 协议闭环（无硬件）

> **目标：不碰硬件，先把两端协议跑通。**

| 交付物 | 位置 | 验收标准 |
|---|---|---|
| 协议规格 | `docs/03-protocol.md` | 命令表齐全，字段布局无歧义 |
| C 端编解码 | `proto/protocol.{h,c}` | 纯 C，不依赖 HAL，`gcc` 可编译 |
| Rust 端编解码 | `host/crates/proto/` | `cargo test -p scope-proto` 全绿 |
| 黄金向量 | `proto/tests/vectors.json` | **C 与 Rust 两端产出逐字节相同的 `frame_hex`** |
| CI | `.github/workflows/ci.yml` | 两端向量测试 + `cargo fmt`/`clippy` 门禁 |

**验收命令**：

```bash
make -C proto test          # C 端向量测试
cd host && cargo test       # Rust 端全部测试
```

**这一阶段的隐藏收益**：把「帧格式 / CRC / 粘包拆包 / 状态机」这些最容易埋雷的地方，在没有硬件干扰的情况下测透。

---

## P1 · 最简硬件闭环

> **目标：F103 采到 2048 点 → 串口 → CLI 落 CSV → 能画出来。**

| 交付物 | 位置 | 验收标准 | 状态 |
|---|---|---|---|
| 时钟树 + ADC/DMA 采集 | `firmware/Hardware/src/adc_dma.c` | TIM4_CC4 → ADC1 → DMA1Ch1，857.143 kSPS 实测（用 `actual_hz` 校验） | ✅ 已实现，**真机未验** |
| 串口链路 + 帧解析 | `firmware/App/proto_task.c` | `PING` / `ECHO` / `GET_INFO` 在真机上通过 | ✅ 已实现，**真机未验** |
| 主机传输层 | `host/crates/transport-serial/` | 能枚举串口、连接、超时重试 | ✅ |
| CLI | `host/crates/cli/` | `scope-cli capture -o wave.csv` 能出图 | ✅ |
| 标定表 | `host/crates/core/src/calib.rs` | 用已知直流电压反推 LSB/V，写进配置文件 —— **主机侧已完成**（见下表）；**真值待上板实测** | ⏳ 待实板 |

**已经做掉的部分**（不需要板子，或已在本机完成）：

| 交付物 | 位置 | 状态 |
|---|---|---|
| `App/` 看到的硬件接口 | `firmware/App/hal.h` | ✅ 纯函数指针，无寄存器 —— ADR-008 那条纪律的落点 |
| 触发搜索 | `firmware/App/trigger.c` | ✅ 施密特迟滞状态机，**状态跨块保留**；`tests/test_trigger.c` 覆盖四个边界用例 |
| 采集状态机 | `firmware/App/acq.c` | ✅ ARM → 搜触发（可跨块）→ 收尾 → DONE；含 auto 超时与溢出检测 |
| 命令分发 | `firmware/App/proto_task.c` | ✅ 17 条命令；往返测试走完整的编解码（不是直接调处理函数） |
| 屏幕帧算术 | `firmware/App/waveform.c` | ✅ 采集窗 → 100 列上下沿 / Vpp / 频率（纯算术，PC 上可测） |
| 本机输入与测频 | `firmware/App/local_input.c` · `local_policy.c` · `freq_meter.c` | ✅ 编码器/按键/测频，均为纯逻辑 |
| 硬件层 | `firmware/Hardware/src/`（10 个源文件 + 11 个头） | ✅ ADC/DMA 采集环、USART 链路、hal 装配、TFT、本机 IO/测频/函数发生器 |
| Keil 工程 | `firmware/MDK-ARM/synthesize-project.uvprojx` | ✅ 可编译（ARMCC V5.06 update 5，**0 Error / 0 Warning**，Flash 19.02 KB / RAM 18.46 KB） |
| PC 端测试 | `firmware/run_tests.sh` | ✅ 7 个测试程序 / **252 项断言** |
| **标定表** | `host/crates/core/src/calib.rs` | ✅ 按设备 `uid` 索引；CLI 写入（`scope-cli cal set`），GUI / MCP 只读。**四个前端全部接上**，未标定时三端都如实标出。⚠ 真值仍需上板实测 —— 见 `docs/02-hardware.md` §9 |

跑法：`./firmware/run_tests.sh` —— **不需要板子、不需要 Keil、不需要交叉编译**，
CI 里也跑它。这正是那条分层的意义：三个人里两个人不拿板子也能改采集逻辑。

**验收**：接一个 1 kHz 方波信号发生器，CLI 抓回来的波形用 Python 画出来，周期测量误差 < 2%。

**注意**：这一阶段**先不要碰屏幕 UI**。屏幕是干扰项，先用串口把数据链路打通。

---

## P2 · 数字通路 I2C 解码 ★ 项目核心价值

> **目标：真总线上抓 I2C，解码结果与逻辑分析仪逐帧一致。**

| 交付物 | 位置 | 验收标准 |
|---|---|---|
| TIM 输入捕获 | `firmware/Hardware/src/tim_capture.c`（⚠ **尚未创建**；现有的 `local_freq.c` 只是测频，不做边沿时间戳缓冲） | 13.9 ns 边沿时间戳，捕获缓冲区 |
| I2C 解码器（C） | `firmware/App/i2c_decode.c`（⚠ **尚未创建**） | START/重复START/地址/ACK/NACK/数据/STOP |
| I2C 解码器（Rust） | `host/crates/core/src/i2c_decode.rs` | 与 C 端同一批测试向量（`i2c_vectors.json`）结果一致 |
| 泳道渲染 | `host/crates/cli` 或 GUI | SCL/SDA 双泳道 + 阈值判定 + 去抖，「共 N 帧」统计 |
| 双通路时间对齐 | | ADC 波形与数字边沿在同一时间轴对齐（误差 < 1 µs） |

**验收**：
1. 抓一次 100 kHz / 400 kHz / 1 MHz 的 I2C 写操作，解码出的地址+数据与逻辑分析仪**完全一致**
2. 制造一次 NACK，两边都能定位到同一个比特
3. 抓一次时钟拉伸，数字通路正确识别

**风险**：板子上只有一路 LM393。见 [`02-hardware.md`](02-hardware.md#7--待决策单通道-vs-双通道) 的待决策 —— 若走「纯数字双通路」需要加一颗比较器。

---

## P3 · MCP + Agent ★ 项目的"AI"部分

> **目标：Agent 一句话完成一次真实的总线调试。**

| 交付物 | 位置 | 验收标准 |
|---|---|---|
| 模拟器 | `host/crates/sim/` | 含 F103 真实档位表、触发语义、故障注入；故障注入必须**在主机侧可观测** |
| MCP Server | `host/crates/mcp/` | 工具 schema 由参数类型经 `schemars` 生成，**不是手写的**；见下方「schema 与实现的一致性」 |
| 14 个粗粒度工具 | `host/crates/mcp/src/main.rs` 的 `TOOLS` | 见下表 |
| Agent 工作流 | [`08-agent-walkthrough.md`](08-agent-walkthrough.md) 生成程序在 [`tools/agent_demo/`](../tools/agent_demo/) | 一段**可复现**的对话实录：一个 LLM 走 MCP 原生工具调用完成一次总线排查，并自己做对照实验。写它的过程暴露了四个问题（模拟器没有读事务、造不出 NACK、解码器把两个正常现象报成异常），都已修 —— 见该文第 5 节 |

### schema 与实现的一致性（P3 的硬性验收）

工具的参数 schema **不许手写**。每个工具的参数类型定义在
`host/crates/mcp/src/params.rs`，同一个类型既生成 `inputSchema`，又接收
`arguments` —— 结构上不可能分家。

这条不是风格问题。曾经的实现是「schema 手写一段 JSON 字符串 + 实现里
`p.get("字段名")` 逐个手抠」，两边没有任何机制保证一致，于是出现了 4 个
**声明了却没人读**的字段（`scope_capture.mode`、`scope_read_waveform.format`、
`scope_sim_set_scenario` 的 `seed` 与 `inject`）：传了不报错、静默走默认值，
Agent 拿到的是它没要求的配置下的数据。

守门的是 `main.rs` 里那条 `every_tool_accepts_exactly_the_parameters_its_schema_declares`：
对每个工具，按 schema 把参数填满发过去必须被接受，再塞一个 schema 里
没声明的字段必须被拒绝。两头夹住即 `properties(schema) == 字段集`。

**它管什么、不管什么**（别把它的能力说大）：

| 管 | 不管 |
|---|---|
| schema 声明的字段集 == 实现能解析的字段集 | 字段解析出来之后**有没有被用**（解析照常、行为删掉，它照样绿） |
| 字段拼错会被拒绝，而不是静默走默认值 | 字段的**语义**对不对（上界、默认值、跨字段约束） |
| 每个属性都有名字、类型、必填标记 | |

「声明了却没人读」那半要靠另外的手段：默认值进 schema（`#[serde(default)]`）、
范围进 schema 且**实现真的校验**（`#[schemars(range)]` 只是注解，serde 不看）、
以及每个工具都有一条「配置之后夹一次真实操作再读回」的测试
（如 `configured_trigger_survives_a_capture`）。

### MCP 工具清单（粗粒度，**14 个** Agent 意图）

> 这份清单必须与 `host/crates/mcp/src/main.rs` 的 `TOOLS` 常量**逐字一致** ——
> 那边是唯一真相源，本表是给人看的镜像。改一处必须改两处。

| 工具 | 作用 | 对应协议命令 |
|---|---|---|
| `scope_list_devices()` | 枚举串口并短超时探测 | `GET_INFO` |
| `scope_connect(port?, transport='sim')` | 连接 + 建上下文 | `GET_INFO` + `GET_CONFIG` |
| `scope_disconnect()` | 停流并断开 | `STOP` |
| `scope_status()` | 「现在什么情况」 | `GET_STATUS` |
| `scope_configure(...)` | **一个工具替代 N 个 `set_*`**，返回 `applied` + `warnings` | `SET_*` |
| `scope_capture(mode?, timeout_ms?)` | 阻塞到触发完成，默认回**统计量 + ≤256 点 minmax 预览** | `ARM` + `EVENT_TRIGGER` + `READ_BUFFER` |
| `scope_read_waveform(capture_id, ...)` | 唯一按需拉分片入口，硬顶 4096 点，超出拒绝 | `READ_BUFFER` |
| `scope_measure(capture_id, metrics)` | 纯数字 + 单位，无波形 | `MEASURE` 或主机侧算 |
| `scope_i2c_decode(capture_id, ...)` | **解码 I2C 帧序列 + 信号质量评估**（本项目的招牌能力） | 主机侧解码 |
| `scope_list_captures()` | 历史采集注册表（保留最近 16 次） | 无（本地） |
| `scope_save_capture(capture_id, path)` | **全量数据只进磁盘，不进 LLM token** | 无（本地） |
| `scope_watch(duration_ms?)` | 在一段时间里**重复单次采集**，返回滚动摘要（设备侧流模式尚未实现） | `ARM` + `EVENT_TRIGGER` + `STOP`，循环 |
| `scope_sim_set_scenario(...)` | 切换场景 / 换随机种子 / 注入故障。故障**整体替换**（`inject:{}` 即清除）。⚠ 它**总是**出现在 `tools/list` 里，连上串口后调用会得到一句明确错误 —— 见下方说明 | 无（模拟器内部） |
| `scope_debug_raw(...)` | 逃生门。仅 `SCOPE_MCP_DEBUG=1` 时注册 | 任意（含 `MEM_READ`） |

**未列入但已规划**：`scope_autoset()`（主机侧复合算法，自动找时基/量程/触发）—— 属 **P4**，
因为它依赖 `scope_configure` + `scope_capture` + `scope_measure` 三件套都稳定下来才有意义。

### 三层 token 防护（硬性）

1. `capture` / `watch` 默认只回**统计量 + ≤256 点 minmax 预览**
2. `read_waveform` 分页、默认 512 点、硬顶 4096 点、超出拒绝并提示 `save_capture`
3. **全量数据只进 capture store 和磁盘，不进上下文**

**验收对话实录**（这就是 P3 的验收标准）：

> **用户**：抓一下 I2C 总线上这次温湿度传感器的读操作，告诉我它有没有被正确应答。
>
> **Agent**：
> 1. `scope_configure(trigger={mode:normal, edge:falling, level_v:1.65})`
> 2. `scope_capture()` → 拿到 `capture_id`、触发位置、统计量
> 3. `scope_measure(capture_id, ['freq','duty'])` → SCL ≈ 100 kHz
> 4. `scope_i2c_decode(capture_id, scl_channel=0, sda_channel=1, vih_lsb=2866, vil_lsb=1228)`
> 5. 回答：**「共 3 帧：写 0x44 地址 → ACK；写 0x00 寄存器 → ACK；读 0x44 → ACK，返回 2 字节 0x1A 0x2B。SCL 上升时间约 480 ns，在 4.7 kΩ 上拉下偏慢，建议换 2.2 kΩ。第 2 字节的 SDA 低电平只有 0.42 V，裕量偏小。」**

---

## P4 · 增强

| 项 | 说明 | 优先级 |
|---|---|---|
| **双通道改板** | 加第二路模拟前端（见待决策） | 高（决定产品形态） |
| ~~控制台/上位机 GUI~~ | eframe/egui 0.36.2 + egui_plot 0.37.0 | ✅ **已完成**（提前到本轮做了） |
| ~~GUI 内嵌 AI 分析面板~~ | 证据包（`core/src/report.rs`）→ 语言模型 → 界面显示。**全项目第一个网络依赖**（`ureq` + rustls/ring） | ✅ **已完成** |
| 自动量程 | 需把 SW2/SW3 换成继电器/模拟开关 | 中（要改板） |
| 等效时间采样 | 数字通路触发 + 帧平均，用于看边沿形状 | 中（**必须标注失效场景**） |
| 其他协议解码 | UART / SPI 解码器 | 中 |
| ~~屏幕 UI~~ | 板载 1.8 寸 TFT：波形 + Vpp + 输入频率。**被动刷新**（采集完成后刷一帧），见 [ADR-014](06-decisions.md)。**屏幕代码已冻结**（2026-10-06 从 Keil 工程移除，文件保留），见 [09-realtime-display.md](09-realtime-display.md) §4.4 | ✅ **已完成**（2026-10-05） |
| 屏幕空闲自刷新 | 设备不发命令时也让画面滚动。需要先定义「设备自己的采集与 host 的 ARM 谁优先」。**屏幕代码已冻结**（见 [09-realtime-display.md](09-realtime-display.md) §4.4） | 低（当前够用） |

---

## P5 · 主板升级 STM32F407

> **目标：解除 F103C8T6 这颗芯片带来的三条硬约束**（`00-origin.md` §四条约束）。项目本身的定位与接口不变。

| 项 | 解锁什么 |
|---|---|
| 三 ADC 三重交替 | 采样率 → **7.2 MSPS**，模拟通路单独即可覆盖 400 kHz I2C |
| FSMC | 深存储（外挂 SRAM），不再受 20 KB SRAM 限制 |
| 以太网 MAC | 直接传原始波形，摆脱 92 KB/s ~ 900 KB/s 的链路预算 |

**不变量**：协议层与上位机**一行都不用改** —— `DevicePort` / `GetCapabilities` / 分片策略都与链路和器件无关。
**数字通路保留**：1 MHz I2C 下它仍是最可靠的解码路径（见 [ADR-005](06-decisions.md)）。

---

## 协作纪律（三条硬规矩）

1. **改协议 = 改四处 + 一次提交**
   `docs/03-protocol.md` + `proto/protocol.h` + `host/crates/proto/src/lib.rs` + `proto/tests/vectors.json`
   两端向量测试全绿才允许合并。

2. **`App/` 层不许 `#include` HAL**
   这是"无硬件也能开发"的根基。破坏了它，三个人就都得等板子。

3. **文档里的每一个数字都要有出处**
   要么来自数据手册（标注页码/表号），要么来自实测（标注日期与测试条件），要么标 `【推断】`。
   **不许写没有依据的性能数字。**

---

## 里程碑节奏建议

| 周 | 目标 |
|---|---|
| W1–W2 | P0 完成：两端向量测试全绿，模拟器能跑 capture |
| W3–W4 | 硬件到货 + P1：真人拿到 1 kHz 方波波形 |
| W5–W6 | P2：I2C 解码与逻辑分析仪对齐（**项目最关键的两周**） |
| W7–W8 | P3：Agent 完成端到端调试对话 |
| W9+ | P4：改板 / 自动量程 / 参赛材料（GUI 已提前做完） |

> **W5–W6 是项目成败点**。数字通路的 I2C 解码如果做不出来，这个项目就退化成一个普通的入门示波器 —— 而那个立创原工程已经有了。
> 建议 3 人在 W5 集中攻坚解码器，不要分散。
