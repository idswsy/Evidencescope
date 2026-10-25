# 12 · 软件能力规划（分析结果对象 / 协议解码插件 / 波形导入）

> **文档状态**：D1–D3 已实施（G1/G2a/G2b）。本文件写设计与验收，
> 实施状态只在各能力开头标注，不贴实现代码。
> **与其它文档的关系**：
> - `docs/10-waveform-analysis-direction.md` —— 创新点依据、证据、红线（本文件不重复）。
> - `docs/11-waveform-analysis-plan.md` —— **算法层**方案（迟滞、σ̂、TIE、瞬态、混叠判决）。
> - **本文件 —— 数据层与接口层方案**。三条能力不改变任何算法结论，只决定「结论存在哪里、从哪个出口出去、数据从哪里来」。
> - `docs/05-roadmap.md` —— 阶段定义（P0–P5）。本文件三条能力属 host 侧，不与 P 阶段争夺优先级，登记在 P2/P3 之后。
>
> 生成时间：2026-10-24。

---

## 0 摘要

| 序 | 能力 | 解决的结构缺陷 | 成本 | 依赖 |
|---|---|---|---|---|
| **D1** | 统一分析结果对象 `AnalysisReport` + 质量载体 | 无机器可读单一出口；`Measurements` 无质量字段；四端各自解释同一批数字 | 3–4 人日 | 无（前置） |
| **D2** | 协议解码器插件化 + UART 解码器 | 解码无插件点；招牌能力在真机不可达（真机 1 通道，I2C 需 2 通道） | 5–7 人日 | D1 |
| **D3** | 第三方波形导入（CSV） | 数据只能来自自家板子或模拟器；`tools/csv_export` 为空 | 2 人日 | 无（可与 D2 并行） |

合计 **10–13 人日**，全部为 host 侧改动，不改固件、不改硬件、不改协议帧格式。

**顺序**：D1 → D2，D3 并行。

**为什么是这个顺序**：`docs/11` 的 `σ̂`、迟滞判决、混叠判决、`Refused` 需要一个类型化的承载结构；没有 D1，这些结论只能停在日志字符串里，无法进 MCP、报告、GUI。D2 的解码结果同样要写入 `AnalysisReport.decoded`。D3 只产出 `Capture`，是 D1 的输入源之一，因此可以并行。

---

## 1 筛选判据

三条，按优先级：

1. **能否在真机上运行**。当前 `scope_i2c_decode` 要求 ≥2 通道（`host/crates/mcp/src/session.rs:932-940`），真机 `firmware/App/proto_task.c:39` 的 `DEV_CH_COUNT = 1`，解码能力只在模拟器成立。新能力若同样只能在模拟器验证，价值打折。
2. **是否解决已确认的结构缺陷**，而不是增加新名词。
3. **能否独立验收**。要有可复现输入与判定标准，不依赖「看起来更专业」。

按这三条淘汰的方向见 §6。

---

## 2 D1 · 统一分析结果对象与质量载体

### 2.1 现状

| 事实 | 位置 |
|---|---|
| `Measurements` 14 个字段，全部是 `f64` 或 `Option<f64>`，**无质量字段** | `host/crates/core/src/measure.rs:25-80` |
| `Measurements`、`Classification` 均**未派生 serde** | 同上；`signal.rs` |
| MCP 出口手抠 13 个 JSON 键 | `host/crates/mcp/src/session.rs:866-906` |
| 文本出口另一套渲染，8 段硬编码 | `host/crates/core/src/report.rs:91-104` |
| 唯一的「判不出」机制是分类枚举，不携带数值 | `host/crates/core/src/signal.rs:440-455`（7 变体） |
| 直流/交流换算来源与「标没标定」由 `ScaleSet` 携带 | `report.rs:69-73` |

⇒ 同一批数字有 4 套表示（`Measurements` / MCP JSON / 证据包文本 / GUI 视图模型），加一个字段要改四处，且没有任何一处能表达「这个数的可信度」。

### 2.2 设计

新增 `host/crates/core/src/analysis.rs`。

```rust
/// 对外唯一的分析结果类型。所有出口（MCP / 证据包 / CLI / GUI）消费它。
pub struct AnalysisReport {
    pub schema: String,               // "analysis/v1"（String 而非 &'static str，serde 可反序列化）
    pub capture_id: u16,
    pub rate_hz: u32,
    pub provenance: Provenance,
    pub channels: Vec<ChannelAnalysis>,
    pub decoded: Option<DecodedFrames>, // D2 的产出，见 §3
    pub notes: Vec<String>,             // 不为空时必须原样透出
}

pub struct Provenance {
    pub source: Source,          // Device{link, simulated} | File{path}
    pub scale: ScaleSet,         // 复用现有类型
    pub calibration: CalibrationState, // Uncalibrated | Calibrated{date} | Unknown
    pub captured_at_unix: u64,
}

pub struct ChannelAnalysis {
    pub channel: usize,
    pub measurements: Measurements,   // 内部类型，字段不动
    pub quality: Quality,
}

pub struct Quality {
    pub noise_lsb: Option<f64>,       // σ̂，来自 docs/11 §4.2，未自标定时为 None
    pub hysteresis_lsb: Option<f64>,  // h = min(3σ̂, 0.25·sep)；σ̂ 为 None 时也为 None
    pub verdicts: BTreeMap<String, Verdict>, // 键与 Measurements 字段同名
}

pub enum Verdict {
    Ok { u: f64 },          // 有数值不确定度
    Bounded { lower: f64 }, // 只能给下界（如亚样点边沿）
    Refused(RefuseReason),  // 不出数，且必须说明原因
}

pub enum RefuseReason {
    TooFewSamples, TooSmallAmplitude, NoPeriodicity, IntervalUnstable,
    FrequencyDrift, Aliased, ShapeUnrecognized,  // 与 signal::UncertainReason 一一对应
    CalibrationMissing,      // 新：需要标定项而无标定数据（只用于电压派生量）
    NotApplicable,           // 新：该量对本波形无定义（如直流信号的占空比）
    NoUncertaintyModel,      // 新：该量没有不确定度模型（不是没标定）
    ModelUnverified,         // 新：模型存在但未通过自校验（如占空比，见 docs/10 §5.2）
}
```

> `AnalysisReport.decoded` **不在 D1 落地**：D1 先交付不含解码字段的版本，D2 以加性字段补入。
> 这样 D1 不依赖尚未存在的 `DecodedFrames`，两条能力可以真正并行。

**四条硬约束**：

1. **数值与判决成对**。出口层不允许出现裸 `f64` 测量值 —— 每个量要么带 `u`，要么带 `Bounded`，要么带 `Refused` 与原因。这是「同一批数字不再有 4 套解释」的唯一手段。
2. **`Measurements` 不改**。它是内部计算类型，字段与 `None` 语义（测不出，不是 0）全部保留；`AnalysisReport` 是对外类型。改动隔离在一层转换函数里。
3. **不新增「结论文本」层**。文本渲染器只有一处：`analysis::render_text(&AnalysisReport, &EvidenceContext) -> String`。`build_evidence(&EvidenceInput)` **签名不变**，内部改为先组装 `AnalysisReport` 与 `EvidenceContext`（链路、模拟器标记、设备配置、用户问题），再调用 `render_text`。这样 `report.rs` 既有测试、`gui/app.rs` 与 `gui/ai.rs` 的调用点全部零改动。
4. **`RefuseReason` 不改 `UncertainReason`**。前者是新枚举（11 变体），后者（7 变体）继续服务形状判定；写一个 `From<UncertainReason>` 转换。这样既有 7 处用法与测试不受影响。
5. **文本出口必须带判决**。每个通道的测量行后追加一行「可信度」，按 `Ok / Bounded / Refused` 分组；`RefuseReason::text()` 是中文措辞的唯一来源。否则模型看到 `频率=207046 Hz` 时无从知道它其实是 `Refused` —— 这正是「同一份结论给人和给模型」不能只在 JSON 出口成立的原因。6 个黄金文本 fixture 钉住这一行。

### 2.3 serde 形状（对外契约）

```json
{
  "schema": "analysis/v1",
  "capture_id": 7,
  "rate_hz": 857142,
  "provenance": {"source": {"kind": "device", "link": "sim(i2c_100k)", "simulated": true},
                 "calibration": {"state": "uncalibrated"},
                 "captured_at_unix": 1792811005},
  "channels": [{
    "channel": 0,
    "measurements": {"vpp": 3.134, "mean": 1.652, "freq_hz": 1000.2, "...": "同现有 14 字段"},
    "quality": {
      "noise_lsb": 18.7,
      "hysteresis_lsb": 56.1,
      "verdicts": {"freq_hz": {"kind": "ok", "u": 0.4},
                   "rise_ns": {"kind": "bounded", "lower": 933.0},
                   "duty_pct": {"kind": "refused", "reason": "ModelUnverified"}}
    }
  }],
  "notes": ["本采集来自模拟器，标量未经过真机标定"]
}
```

**兼容性规则**：

- `session.rs` 现有 13 个 JSON 键**保持不变**（外部 MCP 客户端不破），在其上新增 `quality`、`provenance`、`schema` 三键。
- 未提供不确定度模型时，`Verdict` 一律是 `Refused(NoUncertaintyModel)`；电压量在未标定时为 `Refused(CalibrationMissing)`；模型存在但未通过自校验的量（如占空比）为 `Refused(ModelUnverified)`。**不是** `Ok{u: 0.0}` —— 零不确定度是错误声明，不是缺省值。
- 删除键属于破坏性变更，需 `schema` 升版。

### 2.4 验收

| # | 判据 | 命令 / 检查 |
|---|---|---|
| A1 | serde round-trip：任一样本 `AnalysisReport` 序列化再反序列化后与原值相等 | `cargo test -p scope-core analysis::` |
| A2 | `report.rs` 不再直接取 `Measurements` 字段（唯一取字段处是 `analysis::render_text`） | `grep -nE "\bm\.(vpp|min|max|mean|rms|ac_rms|freq_hz|duty_pct|rise_ns|fall_ns|overshoot_pct|undershoot_pct|high_ns|edges)\b" host/crates/core/src/report.rs` 应为 0 行；另加 6 个代表场景的证据包黄金文本逐字节比对 —— `host/crates/sim/tests/evidence_golden.rs` + `tests/fixtures/evidence/`（sine / step_ring / noise / i2c / uart / clock_62k） |
| A3 | MCP 与证据包对同一采集给出一致的标量 | 新增测试：同一 `Capture` 走两个出口，逐字段比较 —— `host/crates/mcp/src/session.rs` 的 `mcp_measure_and_evidence_text_agree_on_every_scalar`（6 个场景，覆盖 `Some` / `None` 两种数值） |
| A4 | 无不确定度时**不得**输出 `ok` 判决 | `Quality::empty()` 下：无模型的量为 `Refused(NoUncertaintyModel)`；未标定的电压量为 `Refused(CalibrationMissing)`；占空比为 `Refused(ModelUnverified)`；**已标定只换电压口径，不改变「无模型就拒答」**（测试 `calibrated_voltages_still_refuse_without_a_model`） |
| A5 | 主机侧测试数不下降 | `host/run.sh test`，起点 386 passed / 5 ignored；G1 落地后 399 passed / 5 ignored；G3 落地后 463 passed / 5 ignored；F2–F4 后 466 passed / 5 ignored；**2026-10-25 GUI 轮（UART 解码 + CSV 导出 + 定位对齐）后实测 470 passed / 5 ignored**（含 GUI 88 + 5 ignored） |

### 2.5 风险

| 风险 | 处置 |
|---|---|
| `render_text` 搬运导致证据包文本漂移 | 现有测试只有 `contains` 断言，抓不住措辞漂移。先把现有 8 段渲染逻辑整体搬进 `render_text`，**只搬不改**；新增 6 个代表场景的黄金文本 fixture 逐字节比对；`build_evidence` 签名不变，调用点零改动 |
| `Verdict` 全部 `Refused` 会让现有 GUI/验收看起来「退化」 | GUI 侧只增加一列「可信度」，数值列不变；退化的是**声明**，不是数值 |
| 13 个 JSON 键 + 新键让 MCP 响应变长 | 证据包 `MAX_CHARS = 8000`（`report.rs:57-61`）不受影响；MCP JSON 不设上限但 `quality.verdicts` 只列有判决的量 |

---

## 3 D2 · 协议解码器插件化与 UART 解码器

### 3.1 现状

| 事实 | 位置 |
|---|---|
| I2C 解码器是具体类型，配置与结果都是 `I2c*` | `host/crates/core/src/i2c_decode.rs:139`（`I2cDecodeConfig`）、`:421`（`I2cDecode`）、`:678`（`decode`）、`:1040`（`decode_capture`）、`:1064`（`detect_channels`） |
| 证据包只认 I2C | `host/crates/core/src/report.rs:81` `decode: Option<&'a I2cDecode>` |
| I2C 需要 ≥2 通道 | `host/crates/mcp/src/session.rs:932-940` |
| 真机只有 1 通道 | `firmware/App/proto_task.c:39` `DEV_CH_COUNT = 1` |
| 模拟器已有 `uart_115k` 场景 | `host/crates/sim/src/waveform.rs:143`（名称）、`:366`（生成）、`:768`（`uart_115k_bit`） |
| 模拟器自带解码回读测试，但那是**测试代码**，不是产品能力 | `host/crates/sim/src/waveform.rs:1386` `the_uart_scenario_decodes_back_to_the_byte_sequence` |

⇒ 结论：**UART 是唯一能在真机 1 通道上成立、且模拟器已有真值可对拍的第二种协议**。加它不是「多做一种协议」，而是让「协议分析」这件事在真机上第一次成立。

### 3.2 设计

新增 `host/crates/core/src/decoder.rs`。用**枚举分发**而不是 trait 对象：核心 crate 无 `dyn`、可 `Clone`、可 serde、可穷举匹配。

```rust
pub enum Protocol { I2c, Uart }

pub struct DecodeRequest {
    pub protocol: Protocol,
    pub channels: Vec<usize>,   // 显式通道；空表示自动探测
    pub params: DecodeParams,
}

pub enum DecodeParams {
    I2c(I2cDecodeConfig),       // 现有类型，不改
    Uart(UartConfig),
}

pub struct UartConfig {
    pub baud: u32,              // 0 表示自动估计
    pub data_bits: u8,          // 8
    pub parity: Parity,         // None
    pub stop_bits: u8,          // 1
    pub start_bit_low: bool,    // true = 起始位为低（标准 UART；原字段名 idle_lsb 易误解为空闲低）
    pub msb_first: bool,        // false
}

// 通道不由 UartConfig 再写一遍：DecodeRequest.channels 是唯一来源。
// 空 = 自动探测（单通道采集直接用通道 0）；非空 = 必须恰好 1 个通道。

pub struct DecodedFrames {
    pub protocol: Protocol,
    pub rate_hz: u32,
    pub frames: Vec<Frame>,
    pub warnings: Vec<DecodeWarning>,
    pub quality: DecodeQuality, // bit_period_err_pct / ambiguous_bits / sample_margin
}

pub fn decode(cap: &Capture, req: &DecodeRequest) -> Result<DecodedFrames, DecodeError>;
pub fn autodetect(cap: &Capture) -> Vec<ProtocolCandidate>; // 只给建议，不自动应用
```

**关键设计决定**：

1. **I2C 只做适配，不重写**。`DecodeParams::I2c` 直接转调现有 `i2c_decode::decode_capture`，`DecodedFrames` 由 `I2cDecode` 转换得到。这样 I2C 的既有测试（含 `nack` 场景）全部继续有效，插件化不引入行为变化。
2. **插件点是 `report.rs` 的 `decode_section`**。`EvidenceInput.decode` 从 `Option<&I2cDecode>` 改为 `Option<&DecodedFrames>`，并新增 `decode_req: Option<&DecodeRequest>` 回显通道/门限/波特率；生产调用点 3 处（`gui/app.rs` 与 `gui/ai.rs` 两处），改动半径可控。
3. **UART 自动波特率只是建议**。先由最短脉宽估计位宽，再在候选波特率上校验「起始位低、停止位高」一致性，得分最高者作为建议值填进 `UartConfig`；**不自动应用**，因为自动应用一旦估错，解码结果会「看起来合理但全错」，属于红线（`docs/10` §7.4）。
4. **每次解码必须给 `sample_margin`**：**每比特样点数** `rate_hz / baud`（不是 `rate_hz / (baud × bits_per_frame)`，后者是每帧样点数）。低于 4 时返回 `DecodeError::UnderSampled`，不出帧。`uart_115k` 在 857142 采样率下为 **7.44**，通过。

### 3.3 UART 解码算法（8N1）

1. 取通道样点，二值化：`seq = s > (max+min)/2`。不用 Otsu —— UART 空闲态占绝大多数，直方图双峰不保证成立，`signal::detect_levels` 也不适用（它是为周期信号写的）。这里只做「中点比较 + 边沿位置」。
2. 找下降沿（`1→0`）作为候选起始位。连续两个下降沿间隔 < 0.5 位时丢弃前者（判为毛刺）。
3. 起始位校验：位宽 `T = rate_hz / baud`；在 `0.5T` 处采样应为低。
4. 数据位：在 `(1.5 + k)T` 处采样，`k = 0..data_bits`。
5. 停止位：在 `(1.5 + data_bits)T` 处采样应为高；不为高则记 `FrameError`，**该帧标 `ok: false` 并保留**（丢弃会让错误率不可见）。
6. **同步锁定态（`UartSync`）**。采集窗起点可能正好落在起始位（`uart_115k` 场景的 sample 0 就是低电平），此时没有 1→0 下降沿可找。规则：
   - 若 sample 0 为低，允许它作为首个起始位候选；
   - 解完一帧后锁定到 `start + 12T`（该场景 10 位帧 + 2 位空闲），在 ±0.5T 窗口内找下一帧起始边沿；
   - 窗口内找不到则失锁，回到全域搜索；
   - 帧间隔 > 2 字符时间时同样重启搜索。
   **不加锁定态会在采集窗起点解出 `0xD5` 一类假帧**（已用探针复现）。

### 3.4 验收

| # | 判据 |
|---|---|
| B1 | `uart_115k` 场景 4096 样点（≈4.78 ms，**约 46 个字节时隙**：12 位/帧 × 7.44 样点/位）解出以 `[0x55, 0x0F, 0xA5]` 周期重复的字节序列，与 `waveform.rs:1386` 的真值一致 |
| B2 | 波特率偏离 ±10% 时，**解出的字节序列不得等于真值序列**，且字节错误率 > 50%、有 `warnings`。**不以 `FrameError` 帧数过半为判据** —— 探针在 +10% 时只有 28% 的帧错，但字节全错；帧错数会低估错误 |
| B3 | I2C 既有测试全绿（`i2c_100k` / `i2c_400k` / `i2c_nack`），且 `DecodedFrames` 与旧 `I2cDecode` 的帧数、字节序列一致 |
| B4 | 通道数不足时返回明确错误：UART 请求 0 通道、I2C 请求 1 通道均须报「通道数不满足」，不得 panic |
| B5 | 欠采样保护：构造 `rate_hz=10000, baud=115200` 的采集，必须返回 `UnderSampled` |
| B6 | 证据包中解码段对 I2C 与 UART 都成立，且协议名出现在段标题 |

### 3.5 MCP / CLI 接口

- 新增 `scope_decode(protocol, channels, params)`；`scope_i2c_decode` 保留为别名，一个版本周期后废弃。
- 工具描述里必须写明「UART 需 1 通道，I2C 需 2 通道」——这是调用方决定能不能用真机的依据。
- CLI：`scope decode --protocol uart --channel 0 --baud 115200`。

### 3.6 风险

| 风险 | 处置 |
|---|---|
| 11 处 I2C 相关调用点（`session.rs` / `main.rs` / GUI）一次性改签名会引入回归 | 保留 `i2c_decode::decode_capture` 不动，新增 `decoder::decode` 包装；先迁移 MCP 与证据包两处，GUI 最后 |
| UART 自动波特率在噪声大时给出错误建议 | 建议值附 `confidence`；低于阈值时不给建议，要求显式 `--baud` |
| `uart_115k` 场景只有单通道，无法验证通道选择逻辑 | 新增用例：把该场景复制到 CH2，验证 `channel: 1` 可解、`channel: 0` 解出无效帧 |
| 真机 1 通道上 UART 仍需固件能采到足够长窗口 | 不属本文件范围；本文件只保证 host 侧能力对 1 通道成立。真机验证另立任务 |

---

## 4 D3 · 第三方波形导入

> **状态**：已实现（G2b）。`core/src/import.rs` 提供 `from_csv`，CLI 入口
> `scope-cli import`、MCP 入口 `scope_import_capture`；C1–C6 由
> `import.rs` 的 14 项单元测试与 `mcp/src/session.rs` 的导入回环测试钉住。

### 4.1 现状

| 事实 | 位置 |
|---|---|
| 只有导出，无导入：`Time(s),CH1V[,CH2V...]`，时间 9 位小数、电压 6 位小数 | `host/crates/core/src/capture.rs:204` `to_csv` |
| 电压换算 `volts_per_lsb` / `zero_lsb`，默认 3.3 V / 4096 与 2048 | `capture.rs:284-309` `ChannelScale` |
| 采集存储按 `id` 覆盖同号采集 | `capture.rs:315-364` `CaptureStore` |
| `tools/csv_export/` 只有 `.gitkeep` | `tools/csv_export/.gitkeep` |

### 4.2 CSV 契约

与 `to_csv` 输出**逐列一致**，读取端额外容忍 `#` 注释行：

```
# source=wavesurfer rate_hz=1000000 full_scale_v=5.0 offset_v=0.0
Time(s),CH1V,CH2V
0.000000000,1.650000,0.000000
...
```

| 规则 | 处置 |
|---|---|
| 首列必须是 `Time(s)` 或 `Time` | 否则 `NoDataColumns` |
| **速率来源优先级** | `# rate_hz=<n>` 注释 > `--rate-hz` > 由时间列中位差分反推。反推只在 9 位小数能表达该速率时成立；否则报 `RateMissing`，**不隐式推断** |
| 时间列等间隔性 | 相邻差分的相对偏差 > 1% 报 `NonUniformTime`；速率由中位差分倒数得到 |
| **时间原点** | `Time(s)` 首值偏离 0 超过 1 个采样间隔时：默认拒绝（`NonZeroTimeOrigin`），`--normalize-t0` 时归一化并在 `Provenance` 记录原始偏移 |
| 通道列 | `CH<n>V` 或 `CH<n>`；`CH<n>` 视为原始 LSB（等价 `--raw`）。超过 4 通道报 `TooManyChannels` |
| 采样率上限 | `Capture::rate_hz` 是 `u32`。非整数或超过 `u32::MAX` 报 `RateUnsupported`，**不取整**（取整会让时间轴静默错误） |
| 电压反推 | `lsb = round((v − offset) / full_scale × 4096)`，`offset` 是**码值 0 对应的电压**；越界 [0, 4095] 报 `OutOfRange`；缺省 3.3 V / 12 bit / offset = 0（单极性）。导入本项目导出的双极性文件时用 `ImportOptions::from_scale` 折算（`offset = −zero_lsb × volts_per_lsb`，默认 −1.65 V），C1 的 round-trip 靠它 |
| 采集编号 | 导入采集使用 `id ∈ [40000, 60000)`，与设备分配的编号段隔离，避免 `CaptureStore` 同号覆盖 |
| 数据来源标记 | `Capture` 增加 `source: CaptureSource`（`Device | Simulator | File{path}`），`Provenance` 直接取它 |

> **`to_csv` 必须写出 `# rate_hz=<n>` 头**。原因：9 位小数把 857142 Hz 的 `dt=1.166666667 µs` 量化成 `1.167 µs`，反推得 856898 Hz，**C1 的 `rate_hz` 相等不可能成立**。这是导出端必须补的自描述信息，不是导入端的启发式。

### 4.3 接口

- CLI：`./host/run.sh run -p scope-cli -- import <file.csv> [--rate-hz N] [--full-scale V] [--offset V] [--raw] [--normalize-t0] [--channel N]`
- MCP：`scope_import_capture(path)`，路径限定为工作目录内且扩展名为 `.csv`；越界路径报错，不做隐式展开。
- 无 `Time` 列时必须给 `--rate-hz`；既无时间列又无速率 → `RateMissing`，不隐式推断。

### 4.4 验收

| # | 判据 |
|---|---|
| C1 | round-trip：`to_csv` → `from_csv` → 样点逐点相等、`rate_hz` 相等（靠 `# rate_hz=` 头）、通道数相等（使用导出时的同一 `ChannelScale`） |
| C2 | 1 通道与 2 通道文件都能导入；2 通道文件的 `CH2V` 落在第二通道 |
| C3 | 外部数据：手工构造 1 MS/s（`rate_hz = 1_000_000`）文件可导入；构造 **5 GS/s** 文件必须报 `RateUnsupported`（5e9 > `u32::MAX`）。**1.5 GS/s 在 `u32` 内，按本表规则应接受** —— 原验收写 1.5 GS/s 与 §4.2 自相矛盾 |
| C4 | 非等间隔文件报 `NonUniformTime`，不得用平均值静默补齐 |
| C5 | 导入后的采集能直接跑 `process` / `report` / `decode` 三个下游命令（证明「分析链不假设数据来源」） |
| C6 | `#` 注释行被忽略，且未知键不报错（向前兼容） |

### 4.5 风险

| 风险 | 处置 |
|---|---|
| 不同示波器导出的列名五花八门 | 第一版只支持本项目契约 + 显式 `--full-scale/--offset` 参数；不做启发式列名猜测（猜错的代价是静默错误数据） |
| 量程/偏置信息不在文件里 | 必须由命令行给出；否则电压值标 `Uncalibrated`，`Verdict` 为 `Refused(CalibrationMissing)` |
| 大文件（百万点）导入性能 | `Capture::new` 已按 `expected_samples` 预留容量；先按行 `split_once(',')`，不引入 CSV crate |

---

## 5 顺序、依赖与总账

```
D1 AnalysisReport ──→ D2 decoder 插件（DecodedFrames 存在 AnalysisReport.decoded）
   (3–4 人日)                (5–7 人日)

D3 import ──→ 产出 Capture ──→ 复用 D1/D2 的出口（可并行，2 人日）
```

| 阶段 | 内容 | 累计人日 | 完成标志 |
|---|---|---|---|
| S1 | D1：`analysis.rs` + `render_text` + MCP 新增三键 | 3–4 | A1–A5 全过 |
| S2 | D2：`decoder.rs` + I2C 适配 + UART | 8–11 | B1–B6 全过 |
| S3 | D3：`from_csv` + CLI/MCP 入口 | 10–13 | C1–C6 全过 |

**三条做完之后才成立的事**（现在都不能写进材料）：

- 「同一份结论给人和给模型」—— 需要 A2/A3 有测试锁住，否则只是口号。
- 「协议分析在真机上成立」—— 需要 UART 在真机 1 通道上验证可用；本文件只保证 host 侧成立，真机验证另立任务。
- 「硬件可替换」—— 需要 D3 的 C5 证明分析链不假设数据来源；目前 `signal.rs:26/65/585` 仍引用 `crate::f103`。

---

## 6 不做（附理由）

| 方向 | 理由 |
|---|---|
| FFT / 频谱 | 无场景使用；857 kSPS、1024–65536 点的频率分辨率与窗函数误差使结果不可解释 |
| 眼图 / 数字荧光 | 需要亚微秒与高捕获率，物理上不可得；模拟器 `waveform.rs:300-310` 的 `next_sample` 是索引纯函数，做出来的是无依据数据 |
| 实时流模式 | 设备侧未实现；单包上限 2060 字节（`host/crates/proto/src/lib.rs:56-58`），持续传输吞吐与丢包无解 |
| 多板聚合 / 远程测控 | 无需求证据 |
| 「AI 自动诊断故障」 | 无可验证判定标准，无法独立验收 |
| 触发增强（脉宽 / runt 触发） | 成本 5–6 人日且需真机验证；是 `docs/11` runt 计数的前置，但排在上述三条之后 |
| 自动量测序列（autoset） | 真人可用性收益最高，但依赖 D1 的出口才能报告「每一项的可测性」；列为 D1 之后的可选第四条 |

---

## 7 已裁决（G0 冻结）

1. `schema` = **`analysis/v1`**（单字符串，升版直观）。
2. `scope_i2c_decode` 保留为 `TOOLS` 里的**独立条目**，描述标「`scope_decode` 的别名」；不设删除期限，避免外部客户端破。
3. 导入采集编号 = **`[40000, 60000)`**。
4. `Quality.verdicts` = **`BTreeMap<String, Verdict>`**。分析出口是冷路径，serde 直接成 JSON object、键序稳定，比自定义 `Vec` 序列化省事。
5. `build_evidence(&EvidenceInput)` **签名不变**；新增进程内 `EvidenceContext`，`analysis::render_text(&AnalysisReport, &EvidenceContext)` 是唯一文本出口。
6. `AnalysisReport.decoded` **D1 不落地**，D2 以加性字段补入。
7. Verdict 初版矩阵（G1 冻结）：`freq_hz → Ok{u}`；`rise/fall_ns → Bounded{lower}`；`duty_pct → Refused(ModelUnverified)`；电压量未标定 → `Refused(CalibrationMissing)`；`edges → Ok{0}`；其余 → `Refused(NoUncertaintyModel)`。
8. `to_csv` 写出 **`# rate_hz=<n>`** 头；C1 的速率相等靠它，不靠时间列反推。
9. 模拟器 `caps` 对齐列为 G0 代码任务：默认对齐真机；若删除 `CAP_TRIG_SOFT`/`CAP_FMT_MINMAX` 牵连的行为改动超过预算，降级为「默认真机能力 + `--ideal-caps` 开关」。

---

## 附录 代码事实索引

| file:line | 内容 |
|---|---|
| `host/crates/core/src/measure.rs:25-80` | `Measurements` 14 字段，无质量字段 |
| `host/crates/core/src/measure.rs:22-23` | 「无可信结论必须显式拒答」纪律注释 |
| `host/crates/core/src/signal.rs:440-455` | `UncertainReason` 7 变体 |
| `host/crates/core/src/report.rs:57-61` | `MAX_CHARS = 8_000` |
| `host/crates/core/src/report.rs:65-86` | `EvidenceInput`（`decode: Option<&I2cDecode>`） |
| `host/crates/core/src/report.rs:91-104` | `build_evidence` 八段硬编码 |
| `host/crates/core/src/capture.rs:28-45` | `Capture` 8 字段 |
| `host/crates/core/src/capture.rs:204` | `to_csv` |
| `host/crates/core/src/capture.rs:284-309` | `ChannelScale` |
| `host/crates/core/src/capture.rs:315-364` | `CaptureStore`，同 id 覆盖 |
| `host/crates/core/src/i2c_decode.rs:139` | `I2cDecodeConfig` |
| `host/crates/core/src/i2c_decode.rs:421` | `I2cDecode` |
| `host/crates/core/src/i2c_decode.rs:678` | `decode` |
| `host/crates/core/src/i2c_decode.rs:1040` | `decode_capture` |
| `host/crates/mcp/src/session.rs:866-906` | 手抠 13 个 JSON 键 |
| `host/crates/mcp/src/session.rs:932-940` | I2C 解码要求 ≥2 通道 |
| `host/crates/mcp/src/main.rs:321-438` | 16 个 MCP 工具 |
| `host/crates/sim/src/waveform.rs:143/366/768/1386` | `uart_115k` 场景与真值测试 |
| `host/crates/sim/src/waveform.rs:300-310` | `next_sample` 索引纯函数 |
| `firmware/App/proto_task.c:39` | `DEV_CH_COUNT = 1` |
| `tools/csv_export/` | 只有 `.gitkeep` |
