# hardware/ —— 硬件资料与改板

> **本目录不放原理图源文件**，因为上游根本没提供。
> 下面写清楚怎么拿到，以及本项目的改板放在哪。

---

## 1. 上游工程（立创开源）

**《简易数字示波器设计（入门版）》**

- 项目页：<https://oshwhub.com/course-examples/yi-qi-yi-biao-jian-yi-shu-zi-shi-bo-qi-she-ji-cha-jian-ban>
- 工程 UUID：`5e03b6545745463bb8b64813209ffb8c`
- 许可：**GPL-3.0**（硬件）；参考代码的许可与来源见 [NOTICE.md](../NOTICE.md) §2
- 页面明文**禁止商业性使用**

### ⚠️ 没有可直接下载的原理图/Gerber

页面「设计图」页签原文「未生成预览图」，「BOM」页签「暂无BOM」，
「3D模型」页签「暂无数据」。`【原文】`

**要改板必须先在立创EDA专业版里克隆工程**：

```
1. 登录 https://pro.lceda.cn
2. 打开工程（项目页点「打开设计图」也能进）
3. 文件 → 另存为 / 克隆到自己账号
4. 导出 → Gerber / BOM / 坐标文件
5. 把导出的文件放到本目录的 bom/ 或对应子目录
```

### 能直接下载的 4 个附件

| 附件 | 大小 | 用途 |
|---|---|---|
| 简易数字示波器焊接文档.pdf | 732 KB | 装配 |
| 物料清单-简易数字示波器.xlsx | 11.6 KB | 采购（含立创编号） |
| PCB焊接辅助工具-简易数字示波器V1.2.html | 7.1 MB | 交互式点选定位器件 |
| 简易数字示波器-装配图.pdf | 2.6 MB | 装配 |

**这些文件体积大且非本项目所有，不入库。** 用脚本重新拉取：

```bash
./hardware/fetch-upstream.sh
# 下载到 hardware/upstream/downloads/（已被 .gitignore 排除）
```

---

## 2. 上游技术指标（从 Wiki 与 BOM 整理）

### 模拟前端

| 环节 | 器件 | 参数 |
|---|---|---|
| 耦合 | SW2 + 100 nF | AC 耦合隔直 |
| 衰减 | SW3 + 分压网络 | X1 直连，或 `20k/(510k+470k+20k) = 1/50` |
| 缓冲/放大 | **TL072IP** | 一路跟随器，一路 `Vo=(5−Vin)/2` |
| 比较器 | **LM393P** | 滞回 `Uth=2.214V / Utl=2.172V` → PA6 输入捕获 |
| 负压 | **XD7660** + 2 电容 + 1N4148 | 理论 −5 V，实测约 −4.3 V |

**量程**：低压 **−1.6 V ~ 5 V**；高压 **−80 V ~ 250 V**

### 关键 BOM（含立创编号）

| 器件 | 编号 | 数量 |
|---|---|---|
| STM32F103C8T6 最小系统板 | 以实物为准 | 1 |
| TL072IP | C110329 | 1 |
| LM393P | C725322 | 1 |
| XD7660 | C521200 | 1 |
| 1.8" TFT（ST7735）8PIN | C2897371 | 1 |
| EC11 旋转编码器 | C2991196 | 1 |
| BNC 接口 KH-BNC75-3511 | C2837587 | 1 |
| 拨动开关 12E12-G15 | C136720 | 3 |
| TYPE-C-2P | C2919656 | 1 |

完整清单见附件 `物料清单-简易数字示波器.xlsx`，或上游 wiki 的 BOM 表。

### 引脚占用（接线唯一依据）

见 [`docs/02-hardware.md`](../docs/02-hardware.md) §2 的完整表格。

**最关键的一条**：🚫 **不要用 USART2（PA2/PA3）** —— PA2 是 PWM 输出、PA3 是模拟输入。

---

## 3. 本项目的改板（mods/）

### 当前需求：第二路模拟前端

**问题**：底板只有一路模拟输入（BNC → TL072 → PA3），
而 I2C 解码需要 SCL/SDA 两路同时刻采样。详见 [ADR-010](../docs/06-decisions.md)。

| 方案 | 工作量 | 何时做 |
|---|---|---|
| ② 单通道 ADC + 一路数字（LM393 → PA6） | 零（不改板） | **P0–P3** |
| ① 复制一路 TL072 前端 + 第二路 ADC | 改板 + 打样 | **P4** |
| ③ 加一颗比较器做双数字通道 | 飞线 | P4 可选 |

### 改板注意事项

1. **ADC 源阻抗**：若用快速交织（仅显示用），采样时间锁死 1.5 周期，
   要求源阻抗 ≤ 0.4 kΩ。TL072 跟随器输出阻抗足够低，但**串联的
   保护电阻/RC 滤波会把它抬高**，必须算过再用。
   （常规同步模式可用 7.5 周期，放宽到 ~19 kΩ）
2. **模数地分离**：上游已用 0 Ω 电阻单点连接 GND/AGND、分开铺铜，**保持这个做法**
3. **探头电容**：4.7 kΩ 上拉下，探头电容必须 ≤15 pF，
   否则你自己的探头会把被测信号的上升沿拉过 I2C 的 300 ns 规格 ——
   **测出来的"信号完整性问题"是测量系统自造的**
4. **禁止双路供电倒灌**：底板自带 Type-C（仅供电），核心板自带 Type-C
   （供电 + 串口）。两者同时接入即为双路供电，是否有倒灌风险取决于当前
   核心板的电源拓扑 `【待测】` —— 换板前那条社区帖的结论针对上一块板，不适用

---

## 4. 实板必测清单

见 [`docs/02-hardware.md`](../docs/02-hardware.md) §9 —— **十二项，拿到板子第一件事**。

测完请更新那份文档并去掉对应的 `【待测】` 标记。

---

## 5. 参考资料

| 类型 | 链接 |
|---|---|
| 上游硬件工程 | <https://oshwhub.com/course-examples/yi-qi-yi-biao-jian-yi-shu-zi-shi-bo-qi-she-ji-cha-jian-ban> |
| 上游开发 wiki（16 节教程） | <https://wiki.lceda.cn/zh-hans/course-projects/microcontroller/32-simple-oscilloscope/introduce.html> |
| 上游参考代码 | 外部提供的 `STM32-Oscilloscope` 工作副本 —— 见 [NOTICE.md](../NOTICE.md) §2 |
| 上一块核心板资料（留档） | <https://wiki.lckfb.com/zh-hans/dkx-stm32f103c8t6/> |

### B 站视频教程（上游，16 集）

**软件 11 集**：工程模块创建 BV1awjF6fE6r、LED BV1awjF6fE1M、按键 BV1QwjF6fE62、
串口 BV1QwjF6fESq、外部中断 BV1GAjF6FEVX、ADC BV1awjF6fE35、定时器 BV1awjF6fEZB、
PWM BV1awjF6fECw、输入捕获 BV1awjF6fEHb、屏幕显示 BV1BdjA6JEUn、波形显示 BV1zdjA6JENt

**硬件 5 集**：电路原理解析 BV1BdjA6JE2P、原理图设计 BV1kmjA6REBQ、
PCB布局 BV1pmjA6dEqL、PCB走线 BV1wmjA6dEqU、焊接教学 BV1QwjF6fEH1
