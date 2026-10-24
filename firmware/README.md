# firmware/ —— STM32F103C8T6 固件

> **当前状态（2026-10-10 实测）**：Keil 工程可编译并链接通过（ARMCC `V5.06 update 5`，
> **0 Error / 0 Warning**，Flash 19.02 KB / 64 KB、RAM 18.46 KB / 20 KB）。
> `App/measure.c`、`App/i2c_decode.c`、`Hardware/src/tim_capture.c` 与 `App/ui.c`
> **尚未创建**；真机采集、2 Mbps 链路与 I2C 数字通路均未在实板上验证。
>
> ```bash
> ./firmware/run_tests.sh      # 不需要板子、不需要 Keil、不需要交叉编译
> ```

语言：**C11**。库：HAL 或标准外设库（见下方「库的选择」）。

---

## 分层结构

```
firmware/
├─ App/             应用层 —— 与硬件无关的逻辑，可在 PC 上单测
│   ├─ hal.h              App 看到的硬件（纯函数指针 —— 这条纪律的落点）
│   ├─ proto_task.c       帧解析状态机 → 命令分发 → 响应编码（17 条命令）
│   ├─ acq.c              采集状态机 IDLE/ARMED/DONE/STREAMING/FAULT
│   ├─ trigger.c          触发搜索（迟滞状态机）
│   ├─ waveform.c         采集窗 → 屏幕一帧（100 列上下沿 / Vpp / 频率）
│   ├─ freq_meter.c       测频纯算术（TIM3 捕获 → Hz）
│   ├─ local_input.c      编码器/按键输入（消抖 + 正交解码 + 事件队列）
│   ├─ local_policy.c     本机输入 → 动作的纯策略
│   ├─ measure.c          ⏳ 未实现（定点测量）
│   ├─ i2c_decode.c       ⏳ 未实现（数字通路 I2C 解码器）
│   └─ ui.c               ⏳ 未实现（屏幕 / 编码器 / 按键）
├─ tests/           PC 上的单元测试（与 App/ 一起编译，见 run_tests.sh）
├─ run_tests.sh     PC 上编译并运行测试 —— **CI 会跑它**
│
├─ Hardware/        硬件层 —— 唯一允许碰寄存器的地方
│   ├─ inc/ (11)  src/ (10)   ✅ 21 个文件已实现
│   ├─ src/adc_dma.c      ✅ TIM4_CC4 → ADC1 → DMA1Ch1 → 8 KB 环
│   ├─ src/link_uart.c    ✅ USART1 + 外接 CH340（1 KB RX / 2 KB TX 环）
│   ├─ src/hal_impl.c     ✅ 把 9 个函数指针填进 `hal_t`
│   ├─ src/display.c      ⏸ 被动帧渲染（**已冻结**，不在 Keil 工程里，见 docs/09 §4.4）
│   ├─ src/tft*.c         ✅ 上游 ST7735S 驱动
│   ├─ src/local_*.c      ✅ 本机 IO / 测频 / 函数发生器
│   └─ src/scope_ui.c     上游的本机显示/按键/测频逻辑，`#if SCOPE_LOCAL_UI`（默认 0，存档）
│
├─ Core/            CubeMX 生成的时钟树与外设初始化（Inc/ + Src/）
├─ Drivers/         ST HAL + CMSIS（厂商代码，许可见 `NOTICE.md` §6）
├─ MDK-ARM/         Keil 工程（`synthesize-project.uvprojx`）—— **已并入，可编译**
└─ synthesize-project.ioc   CubeMX 工程定义
```

---

## 屏幕（板载 1.8 寸 TFT，160×128，ST7735S）

> ⚠ **屏幕已冻结**（[docs/09-realtime-display.md](../docs/09-realtime-display.md) §4.4）：
> `display.c` 已从 Keil 工程移除、代码保留不编；**本节以下规则仅在恢复屏幕时适用**。

渲染在 `Hardware/src/display.c`，要画什么由 `App/waveform.c` 算 ——
后者是纯算术（采集窗 → 100 列的上下沿 + Vpp + 频率），**PC 上可测**，
所以把它放在 App 层（ADR-008 的用意）。

**布局照上游那台的样子做**：标题「简易示波器」、右侧 PWM 面板
（输出状态 / 输出频率 / 占空比）、底部输入峰值 / 输入频率、波形区
x 0..99 y 30..80。坐标逐项对着上游 `TFT_StaticUI` / `TFT_ShowUI` 抄的。

> ⚠ **有两处配色与上游源码不同，以真机为准**：上游源码里标题与两个底部数值
> 写的是「黑字绿底」，真机上是**绿字黑底**。也就是说板上跑的那版源码
> 与本仓这份在两处不一样。以照片为准 —— 那是用户认的"原来那个界面"。

**刷新是被动的**：采集完成（`ACQ_EV_TRIGGERED`）时刷一帧，空闲时画面静止。
设备不会自己去抢 ADC。决策与理由见 [ADR-014](../docs/06-decisions.md)。

三条要记住的：

1. **绘制绝不能拖慢主循环。** 采集期间半个 8 KB 环每 **2.39 ms** 发布一次，
   而发布位图只有 2 位 —— 单轮显著超过它就会丢样点。所以一列的 51 个像素
   攒成一个缓冲走 `TFT_Blit()` **一次**传出去（每列 2 次 SPI 调用），
   而不是逐像素 `TFT_WR_DATA`（每列约 138 次）。
   `Display_Poll()` 一轮最多画 `COLS_PER_TICK` 列、或写**一个**字符。
2. **`Display_Poll()` 必须排在 `proto_task_poll()` 之后** —— 串口收包环只有
   1 KB（约 11 ms 就满），屏幕没资格排在它前面。
3. **屏幕是可选件。** SPI 走有限超时（不是 `HAL_MAX_DELAY`），失败粘滞记账后
   停止绘制，但协议链路照跑。一块坏屏幕不该让设备连 PING 都不回。

> ⚠ **`TFT_Fill(x0, y0, x1, y1, c)` 的 `x1`/`y1` 是开区间端点** ——
> 它内部先 `TFT_Address_Set(x0, y0, x1-1, y1-1)` 设窗口，再按 `x0..x1-1` 循环写。
> 画一条"从 x=106 到 x=106"的竖线要传 **`106, 0, 107, 128`**，传 `106, 0, 106, 127`
> 会得到起点大于终点的退化窗口 —— **`for` 一次都不进，一个像素都不写，
> 返回 `void`，没有任何错误码**。
>
> `Hardware/src/display.c` 里用 `fill_rect()` 包了一层**闭区间**接口，
> **新代码一律用它，不要直接调 `TFT_Fill`**。

屏幕上的 Vpp 用**与上位机相同的占位换算**（3.3 V / 4096）。底板的模拟前端是
`Uadc = (5 − Vin) / 2`，输入摆幅是 ADC 摆幅的两倍，占位换算不含这个因子 ——
**真值要标定之后才有**（见 `docs/02-hardware.md` §9）。

---

## 本机 UI：`SCOPE_LOCAL_UI`（默认关）

上游那套「本机采集 + TFT 显示 + 按键/编码器」的完整应用搬到了
`Hardware/src/scope_ui.c`，**整份代码被 `#if SCOPE_LOCAL_UI` 包着，默认 0**。

为什么不直接删：路线图规定 P1 先不碰屏幕（先用串口建立数据链路），
但 P4 要接屏幕时那套逻辑还能用。**留着又不开，就有腐烂的风险** ——
所以 CI 会带 `-DSCOPE_LOCAL_UI=1` 把它也编一遍（见「固件分层纪律检查」那一 job）。

打开它的办法：把 `Hardware/inc/scope_ui.h` 里的默认值改成 1，
并把 `Hardware/src/scope_ui.c` 加进 Keil 工程的 `Application/Hardware` 组。

⚠ **它与已冻结的屏幕驱动（`Hardware/src/display.c`，docs/09 §4.4）互斥，不要同时开**，
理由三条，每一条单独都足以致命：

- 它与本项目的采集链**抢同一批外设**（ADC1 + DMA1Ch1 + TIM），二者不能同时生效
- 它定义了 `HAL_ADC_ConvCpltCallback`，与 `Hardware/src/adc_dma.c` 里的**同名，
  两个都编会重定义**
- 同一块屏会有两套驱动在写

> ⚠ 上游那三个文件（`main.c` / `main.h` / `gpio.c`）里的**中文注释已经被
> 不可逆地损坏了**（GBK 字节被当 UTF-8 读坏，文件里只剩 U+FFFD）。不是本仓库
> 造成的 —— 上游原件里就是坏的。所以 `struct Oscilloscope` 的字段注释是
> **按代码用法重建**的，不是原文。字段名与行为一个字没改。

---

## ⭐ 硬纪律：`App/` 不许 `#include` HAL

这是**整个项目最重要的一条工程约束**，理由是现实的：

> 三个人里只有一个人能同时拿到板子。
> 如果 `App/` 依赖 HAL，另外两个人就只能干等 —— 这是项目最常见的死法。

`App/` 层必须能这样测试：

```bash
# 在 PC 上编译 App 层的纯逻辑，不需要板子、不需要 Keil、不需要交叉编译
gcc -std=c11 -I../proto -I App App/proto_task.c App/measure.c App/trigger.c \
    ../proto/protocol.c -o build/app_test
```

**CI 会强制检查这条规矩**：

```bash
grep -rn '#include.*\(stm32\|hal_\|gd32\|HAL\)' firmware/App/ && exit 1 || exit 0
```

违反它 = CI 红。

---

## 库的选择：HAL vs 标准外设库

| | ST HAL | 标准外设库 (SPL) |
|---|---|---|
| CubeMX 支持 | ✅ 可视化配置 | ❌ |
| 代码体积 | 大（20–35 KB） | 小（~10 KB） |
| USB CDC | ✅ 官方例程 | 需移植 |

> **实测结论（2026-10-10 核对 `MDK-ARM/synthesize-project/*.build_log.htm` 与 `.map`）**：
> 本工程用的就是 **HAL**，而上面"20–35 KB"的担心**没有兑现** ——
> Flash 实际占用 `Code 19028 + RO-data 396 + RW-data 56 = 19480 B`（**19.02 KB / 64 KB，30%**）。
> **真正紧的是 RAM**：`RW-data 56 + ZI-data 18848 = 18904 B`（**18.46 KB / 20 KB**），
> 只剩约 **1.5 KB** 余量（栈 1 KB + 堆 0.5 KB 都已算在 ZI 里）。
>
> 所以：**不必为 Flash 换 F103CB**；加功能前先看 `.map` 的 `Total RW Size`。
>
> 当前链路是 USART1 + 外接 CH340，**没有 USB CDC**，所以 HAL 的体积里还没算上它；
> 将来真上 USB CDC 要重新核一遍 Flash/RAM。
> 见 [`docs/04-performance.md`](../docs/04-performance.md) §3。

---

## 数据链路选型

P1 期主力为 USART1 + 外接 CH340 模块（PA9 / PA10，四根线，TX/RX **交叉**）；
P2 起 USB CDC（PA11 / PA12）为备选；USART2（PA2 / PA3）与底板硬冲突，禁用。
完整表见根 [`README.md`](../README.md) §7.1 与 [`docs/02-hardware.md`](../docs/02-hardware.md) §2。

> 核心板**到底有没有**板载桥、以及外接模块的最高可靠波特率，均 `【待核实】`（见 [ADR-013](../docs/06-decisions.md)）。
> ⚠ **不要同时给核心板 Type-C 与底板 Type-C 供电** —— 双路供电倒灌，见 `docs/02-hardware.md` §9。
>
> **链路波特率**由固件宏 `LINK_BAUD`（`Core/Src/usart.c`）决定，默认 **921600**，
> 主机侧 `scope-cli serial --baud` 必须与它一致；实测稳定后可切 **2000000**
> （USART1 挂 APB2 = 72 MHz，BRR = 36 为整数、零误差；CH340 系列按手册支持 2 Mbps）——
> ⚠ **切换必须先实测**：过 `docs/09-realtime-display.md` §4.2 的压测判据（连续 50 次
> capture 无 CRC 错误）。默认值暂不动。

**无论是哪条，上层都用同一个 `App/proto_task.c`** —— 链路差异只在 `Hardware/link_*.c` 里。

---

## 外设分配 ⭐（2026-10-05 拿到真实工程后校正）

**这张表是唯一的依据。** 早期记载的「TIM3_TRGO → ADC」见下。

| 用途 | 定时器 | 引脚 | 为什么是它 |
|---|---|---|---|
| **ADC 采样触发** | **TIM4**（PSC=0、ARR=83、**CCR4=83**） | 无 | ADC1 的触发源表里**没有 `T4_TRGO`**，只有 `T4_CC4`（见 `stm32f1xx_hal_adc_ex.h`）。CC 事件**不需要启用输出脚**也能触发 ADC |
| **比较器输入捕获** | **TIM3_CH1** | PA6 | PA6 在 F103 上**只有 TIM3_CH1 一个定时器功能**；TIM3 部分重映射会把 CH1 挪到 PB4，那样 PA6 就没有定时器功能了 —— 所以 TIM3 被它占死，没有别的选择 |
| 微秒时基（`tick_us`） | TIM1（PSC=71 → 1 MHz） | 无 | 需要一个**独立**定时器做 32 位 µs 计数（协议要求约 71.6 分钟回绕）。不启用其输出脚，与 USART1 的 PA9/PA10 不冲突 |
| 简易函数发生器 | TIM2_CH3 | PA2 | 底板自带，上游保留；P1 不用 |

### ⚠ 早期记载的「TIM3_TRGO → ADC」

该路径不成立：ADC1 的外部触发源只有
`T1_CC1 / T1_CC2 / T1_CC3 / T2_CC2 / T3_TRGO / T4_CC4 / EXT_IT11`，
**没有 `T2_TRGO`，也没有 `T4_TRGO`**；且 PA6 的捕获已占用 TIM3（见上表）。
2026-10-05 取得真实 CubeMX 工程后按上表校正。

> **教训：引脚与定时器通道的分配，要以数据手册/厂商头文件为准，不能凭印象。**

---

## 采样架构（定时器触发 + DMA + 环缓冲）

```
TIM4 (PSC=0, ARR=83, CCR4=83) ──CC4───► ADC1（P1 单 ADC）
                                          │
                                          ▼
                              DMA1 Channel 1（循环模式）
                                          │
                                          ▼
                            uint16_t ring[4096]  (8 KB)
                                          │
                            半传输/全传输中断（每 2048 点发布一个半区）
                                          │
                                          ▼
                    主循环在**已发布**的半区上搜索触发点（迟滞状态机）
                                          │
                          找到 T → 继续采 T+post+512 余量 → 停 DMA
                                          │
                                          ▼
                              EVENT_TRIGGER 上报 → 状态 DONE
```

### 关键数字

| 项 | 值 | 来源 |
|---|---|---|
| 采样率 | **857.143 kSPS** | 12 MHz ADCCLK ÷ 14 周期 |
| 缓冲深度 | 4096 点 / 8 KB | 20 KB SRAM 的现实预算 |
| 块中断频率 | 419 Hz（每 2048 点） | 857143 / 2048 |
| 每样点可用周期 | **84** | 72 MHz / 857 kHz |
| 停 DMA 余量 | +512 样点 | 吸收停驻延迟 |

完整推导见 [`docs/04-performance.md`](../docs/04-performance.md)。

### ⚠️ 三条最容易踩的坑

1. **ADC2 在中容量型号上没有 DMA 请求通道**
   → 双 ADC 模式下必须借 `ADC1_DR` 组合寄存器经 ADC1 的 DMA 搬出，
   且 DMA 必须配 **32-bit 宽度**（低半字 = ADC1，高半字 = ADC2）。
   配成 16-bit 会**静默丢掉一半**，速率直接腰斩。

2. **禁止逐样本中断**
   857 kSPS 下每样本只有 84 个周期，逐样本中断会吃掉全部 CPU。
   必须 DMA + 块中断。

3. **绝不读正在被 DMA 写的半区**
   所有权规则：只有 HT/TC 中断发布过的半区才可以读、算 CRC、打包发送。
   宁慢勿乱。

---

## 触发搜索：必须是状态机

**不能**写成"相邻两点跨越整条迟滞带"——那样缓变信号永远触发不了：

```
1 kHz 正弦 @ 857 kSPS → 每样点变化约 14 LSB
迟滞带宽 ±16 LSB → 32 LSB
一次跳变跨不过去 → 永远不触发
```

正确语义（施密特触发）：

```
1. 信号必须先跌到 lo 以下置位（armed）
2. 之后升到 hi 以上才算一次上升沿
3. 落在带内的值既不置位也不触发 —— 这就是迟滞抑制抖动的原理
```

参考实现见 `host/crates/sim/src/device.rs` 的 `find_trigger()`，
那里有完整的边界测试用例（缓变斜坡 / 带内抖动 / 对称下降沿 / 从不跨越）。

---

## 编译与烧录

> **工程已经建好。** `firmware/MDK-ARM/synthesize-project.uvprojx`
> 已包含 `App/`（7 个 .c）、`Hardware/src/`（8 个 .c，不含已冻结的 `display.c` 与存档的
> `scope_ui.c`）、`Core/Src/`、`Drivers/` 与 `proto/protocol.c`。
> 最后一次完整重建：ARMCC `V5.06 update 5`，**0 Error / 0 Warning**，
> 产物在 `firmware/MDK-ARM/synthesize-project/`（`.axf` / `.hex` / `.map`）。
>
> ⚠ 那份产物是**屏幕冻结之前**的构建 —— `synthesize-project.lnp` 里仍有 `display.o`。
> 重新编译一次即与当前工程一致（并省下约 448 B RAM）。

```bash
# Keil MDK：打开 firmware/MDK-ARM/synthesize-project.uvprojx，F7 编译，DAP-Link 下载

# 命令行：本仓**没有** firmware 的 Makefile（`make -C firmware` 不成立），
# 要走命令行得自建 arm-none-eabi-gcc 构建脚本。
```

> ⚠ **不要拿 `firmware/synthesize-project.ioc` 让 CubeMX 重新生成代码** ——
> 它的外设清单里**没有 TIM4、也没有 USART1**（只有 ADC1/DMA/NVIC/RCC/SPI1/SYS/TIM2/TIM3），
> 而 `Core/Src/tim.c` 的 `MX_TIM4_Init()`（ADC 外部触发）与 `Core/Src/usart.c` 的
> `MX_USART1_UART_Init()`（协议链路）才是当前工程的真实配置。
> 重新生成会把这两条链路按 `.ioc` 的清单一起删掉。

**DAP-Link 接线**（核心板上的 SWD 四脚）：

```
DAP-Link 3V3  ──  核心板 3V3
DAP-Link DIO  ──  核心板 DIO
DAP-Link CLK  ──  核心板 CLK
DAP-Link GND  ──  核心板 GND
```

> Win11 下装了 ST-Link 驱动可能导致 DAPLink 识别不到（官方文档提到过）。`【原文】`

---

## 上游参考

**固件侧**：外部提供的 `STM32-Oscilloscope` 工作副本（本地目录
`STM32-Oscilloscope/`，**不含 `.git`**）。来源、许可状态与待办见
[`NOTICE.md`](../NOTICE.md) §2。

**直接复用是允许且推荐的**：屏幕驱动、编码器、按键、ADC 初始化都可以从上位工程搬，
**但采样架构、触发、协议、数字通路解码必须重写** —— 上游只做本机采集显示，
没有任何对外命令接口。
