# tools/ —— 独立工具

> 这里放**不依赖整个 workspace 也能跑**的小工具。
> 它们的存在是为了让「看一眼数据」这件事不需要开 IDE、不需要接线。

---

## 已实现的工具

### `agent_demo/` —— 用 LLM 的原生工具调用驱动这台示波器

P3 验收所用的演示程序。运行方式与已知局限以
[`agent_demo/README.md`](agent_demo/README.md) 为准；运行实录见
[`docs/08-agent-walkthrough.md`](../docs/08-agent-walkthrough.md)。

---

## 规划中的工具

> ⚠️ **下面两个只有设计，没有代码**（目录里只有 `.gitkeep`）。
> 保留在这里是为了说明「打算往哪放」，别当成已有实现。

### `i2c_decode/` —— 独立 I2C 解码器

**为什么独立于 `scope-core`**：这个工具要能处理**别人的**数据 ——
从任意示波器（Rigol / Keysight 等）导出的 CSV，
不一定是本项目设备抓的。

**行为规范**（出处见 [`NOTICE.md`](../NOTICE.md) §4）：

| 特性 | 说明 |
|---|---|
| 通道选择 | 默认 `SCL = CH1`、`SDA = CH2`，其余通道组合自动检测（兼容 CH1/CH4 等） |
| 阈值 | 可调，单位 V。**保留中间带为「未知」并报警**，不硬判 0/1 |
| 去抖 | debounce 时间可配 |
| 解码输出 | START / 重复START / 地址(7位+10位) / ACK / NACK / 数据 / STOP |
| 统计 | 「共 N 帧」、每帧的字节数、错误标记 |
| 告警 | 无信号、阈值落在判决带内、上拉过弱、SCL 卡死 |
| 导出 | `i2c_decode.txt`（逐帧文本）+ `i2c_decode.json`（结构化） |

**输入格式**：兼容两种 CSV

```csv
# 格式 A：通用两通道格式
Time(s),CH1V,CH2V
0.000000000,0.000000,3.300000
...

# 格式 B：Rigol 导出
CH1V,CH4V,t0=...,tInc=...
```

**计划接口**：

```bash
python tools/i2c_decode/decode.py wave.csv --scl CH1 --sda CH2 \
       --vih 2.31 --vil 0.99 --debounce-ns 50 -o out.txt
```

**语言**：Python（快速迭代 + 无需编译）。等逻辑稳定了可以考虑用 Rust 重写，
或者直接把逻辑并进 `scope-core` 供 MCP 工具调用。

> **重要**：`scope-core` 里也有一份 Rust 版解码器（P2 实现）。
> 两份实现必须对**同一批测试向量**产出相同结果 ——
> 目的是交叉验证，不是重复劳动。向量放 `tools/i2c_decode/testdata/`。

---

### `csv_export/` —— CSV → 其他格式

给用户导出数据用的转换脚本：

| 输出 | 用途 |
|---|---|
| `.npy` | NumPy 直接加载，做离线分析最方便 |
| `.vcd` | 逻辑分析仪格式（二值化后） |
| `.png` | 快速画个波形图，不依赖 GUI |
| `.md` | 直接贴进报告/文档的表格 |

**计划接口**：

```bash
python tools/csv_export/to_npy.py wave.csv
python tools/csv_export/plot.py wave.csv --out wave.png --title "I2C 写时序"
```

> `plot.py` 要处理中文字体（Windows 上 matplotlib 默认不显示中文）——
> 从 `C:\Windows\Fonts\msyh.ttc` 加载。

---

## 约定

1. **工具只读你的数据，不写回、不联网、不发遥测。**
2. 每个工具至少配一个 `testdata/` 样本 + 一条自检命令，写进本文件。
3. 依赖写在各自的 `requirements.txt`，**不用全局 pip**。
4. 工具产生的中间文件进 `.gitignore`，**样本数据除外**
   （样本要小的、脱敏的，`testdata/` 目录下的 CSV 已被 `.gitignore` 白名单放行）。
