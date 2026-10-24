# NOTICE — 上游来源与许可归属

本项目（F103 Agent Scope）是**派生作品**，包含并衍生自以下上游作品。
分发、修改、再发布时必须保留本文件。

---

## 1. 硬件设计

**《简易数字示波器设计（入门版）》**

- 来源：立创开源硬件平台（OSHWHub）
- 项目页：<https://oshwhub.com/course-examples/yi-qi-yi-biao-jian-yi-shu-zi-shi-bo-qi-she-ji-cha-jian-ban>
- 工程 UUID：`5e03b6545745463bb8b64813209ffb8c`
- 作者：嘉立创EDA-莫工（`lceda_01`）
- 版权归属：EDA课程案例团队（`course-examples`）
- 团队成员：嘉立创EDA-莫工、牧尘（`chenlong11`）、`eda_kngxlawxc`、OSHWHub、立创开发板（`jlckfb`）
- 参与者（页面原文感谢名单）：
  - 福州大学旗山校区 李健东 — CW32 核心板移植与适配
  - 长春电子科技学院 郭富城 — STM32 核心板移植与适配
- **许可证：GPL-3.0**

**页面原文的知识产权声明：**

> 本项目为开源硬件项目，其相关的知识产权归创作者所有……仅供平台用户用于学习交流及研究，**不包括任何商业性使用**。

### 对本项目的约束

- 本项目对硬件的任何修改（改板、加装第二路前端等）**必须同样以 GPL-3.0 开源**。
- **禁止商业性使用**。
- 必须保留原作者署名与本声明。

---

## 2. 参考固件源码

**上游：STM32-Oscilloscope（简易数字示波器，STM32 版）**

- 来源：外部提供的工作副本（本地目录 `STM32-Oscilloscope/`，**不含 `.git`**）
- **许可证：文件头标注为 MulanPSL-2.0，但本仓库无法核实。** 该工作副本里
  **没有 LICENSE 文件**，只有 STMicroelectronics 的 CubeMX 生成代码声明
  （BSD-3-Clause，见 §6），没有上游作者自己的许可声明；仓库地址与作者
  信息同样无从考证。
- 取舍：**保留文件头里的 MulanPSL-2.0 声明不动。** 多归属比少归属安全 ——
  若上游确是 MulanPSL-2.0，删掉许可名就构成违约；若上游不是，这句话只是
  多署了一个名字，不会伤害任何一方。

> ⚠ **2026-10-07 更正**：本节此前把上游记为 Gitee 上的
> `chen11232/GD32E230-Oscilloscope`（MulanPSL-2.0），并自注「最可能的
> 来源，待确认」。用户 2026-10-07 明确：**未使用该工程**。该归属已删除。
> 本条只保留能核实的事实。
>
> **待办**：向上游提供者确认许可，或确认这些文件是否仍有上游代码；查清后同步
> 文件头与本节的「无法核实」措辞。在那之前，下列文件的许可状态是**未定**的。

### ⚠️ 本项目引入了这份上游的代码（2026-10-05）

用户 2026-10-05 确认：`firmware/` 里的 TFT 驱动与 CubeMX 文件的 USER CODE 段
来自上游仓库的 STM32 移植分支。

引入清单（文件头目前标着 MulanPSL-2.0）：

| 文件 | 内容 |
|---|---|
| `firmware/Hardware/src/tft.c` · `tft_init.c` | ST7735S 显示驱动 |
| `firmware/Hardware/inc/tft.h` · `tft_init.h` · `font.h` | 显示驱动头 + 点阵字库 |
| `firmware/Core/Src/main.c` · `gpio.c` · `tim.c` · `adc.c` | CubeMX 文件，**USER CODE 段**是上游的应用逻辑（按键、测频、本机显示） |

> 注：`Core/` 与 `Drivers/` 里 ST 生成的代码另有 ST 的许可证，见 §6。

### 对本项目的约束

- 在许可查清之前，**上述文件头的来源声明不得删除**（删掉会把 provenance
  一起丢掉，而 provenance 是查清许可的唯一线索）。
- MulanPSL-2.0 与 GPL-3.0 兼容，本项目整体以 GPL-3.0 发布不构成冲突。

---

## 3. 开发文档

- 立创课程 wiki：<https://wiki.lceda.cn/zh-hans/course-projects/microcontroller/32-simple-oscilloscope/introduce.html>
- B 站视频教程 16 集（软件 11 集 + 硬件 5 集），见 `hardware/README.md`

文档内容版权归立创所有，本项目仅在 `docs/` 中**引用并标注来源**，不复制原文。

---

## 4. 早期设计输入的来源

本项目立项阶段收到一份外部设计方案与配套演示程序（提供者：**马忠学**，2026-09-24）。
下列设计要素由其启发（**思路借鉴，非代码复制**）：

- 「命令只定义一次，三端复用」的单一真相源原则
- `AA 55` 帧头 + `seq` + `crc16` 的帧格式
- `GetCapabilities` 能力协商
- `DevicePort` 传输层解耦 + 模拟器一等公民
- I2C 解码器的行为规范：默认 SCL/SDA 通道、阈值 + 去抖、泳道显示、帧数统计、导出 `i2c_decode.txt`

> 该方案与演示程序**不构成本项目的技术基线** —— 平台、架构与性能目标均由本项目独立确定，
> 见 `docs/00-origin.md`。本条仅用于记录上述要素的出处。

上述两份材料**未收录进本仓库**：`.exe` 是第三方二进制（5.3 MB，不适合入库），方案文档是私人交流材料。
需要时由团队成员自行留存于 `reference/`（该目录已被 `.gitignore` 排除）。

---

## 5. 第三方依赖

各语言生态的第三方依赖清单见对应目录：

- Rust：`host/Cargo.toml`
- C 固件：`firmware/README.md`
- 工具：`tools/README.md`

---

## 6. 固件里的厂商代码（2026-10-05 起）

`firmware/Core/` 与 `firmware/Drivers/` 现在是 ST 生成的代码。
**它们的许可与本仓库的 GPL-3.0 不同，但兼容**，且**各文件自带的声明不得删除**：

| 目录 | 内容 | 许可证 | 许可文件 |
|---|---|---|---|
| `firmware/Drivers/STM32F1xx_HAL_Driver/` | STM32F1 HAL 驱动 | **BSD-3-Clause** | 同目录 `LICENSE.txt` |
| `firmware/Drivers/CMSIS/Include/`、`Device/ST/STM32F1xx/` | ARM CMSIS | **Apache-2.0** | 各目录下 `LICENSE.txt` |
| `firmware/Core/` | CubeMX 生成的初始化与时钟树 | **BSD-3-Clause**（ST） | 各文件头 |

**兼容性**：BSD-3-Clause 与 Apache-2.0 都与 GPL-3.0 兼容，
所以本项目整体以 GPL-3.0 发布不构成冲突 —— 前提是**保留上述声明**。

> ⚠ **裁剪记录**：入库时丢掉了上游包里的 `Drivers/CMSIS/DSP/`（14 MB）
> 与 `Drivers/CMSIS/Lib/`（35 MB，数学库的 `.lib`/`.a`），本工程用不到。
> 设备头只保留了 F103xB 相关的三个。**将来若要换型号或用到 DSP，要重新拉原始包。**

> ⚠ `firmware/MDK-ARM/synthesize-project/`（构建产物，34 MB）**不入库** ——
> `.gitignore` 已经挡掉，Keil 打开工程时会自己重建。
