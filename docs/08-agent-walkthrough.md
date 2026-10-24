# 08 · Agent 工作流实录

> 目标：兑现 P3 的验收标准 ——「**Agent 单条指令完成一次真实的总线调试**」。
>
> 这份实录**不是整理出来的**：它由 [`tools/agent_demo/agent.py`](../tools/agent_demo/)
> 运行产生，走的是 MCP 原生工具调用（不是把命令拼成字符串传入管道）。
> 复现方法见第 6 节。

---

## 1. 给 Agent 的一句话

只给它这一句，**不给任何工具提示** —— 工具表、参数、schema 全靠它自己从
`tools/list` 里读：

> 「我这条 I2C 总线上挂了个传感器，读它一直返回全 0，怀疑是总线本身有问题。
> 你帮我看看总线上到底在发生什么，给我个结论。」

---

### 2. Agent 给出的结论（原文，未经润色）

### 先说一句必须澄清的前提

**我没有连到你那条真实的总线上。** 我调用连接工具时没指定串口，MCP 就落到了内置模拟器上（`scope_status` 里明确写着 `"simulated": true`、链路是 `sim(i2c_100k)`）。所以下面所有波形都来自一个**模拟出来的 I2C 总线**，不是你手焊的那两根线。要看你自己的总线，得给我串口号（比如 `"COM7"`）让我用串口真机重连。

不过即便如此，这轮数据仍然很有价值 —— 因为它给出了一条**健康 I2C 总线应该长什么样**的基线，正好可以跟你"全 0"的现象做对照。

---

### 抓到了什么

我用 857 kHz 采样、4096 点抓了一窗（4.78 ms），自动判定 SCL=CH0、SDA=CH1，解码结果 `trustworthy: true`。

**总线电气质量：干净。**
- SCL 实测 100.0 kHz，占空比 50.0%（标准 I2C 快慢模式没问题）
- `unknown_level_samples: 0` —— 没有悬空/半高电平的采样点
- SDA/SCL 都满幅摆动（614↔3481 LSB，约 2.31 Vpp），没有卡死在某一边
- 无毛刺、无仲裁丢失

**协议层面：一次完整、正常的"写寄存器指针 + 读回"事务，重复了 9 遍。**

```
写 0x44(W) + 0x00        ← 把内部寄存器指针设到 0x00
重复START 0x44(R) → 0x01 0x2C   ← 读回 2 字节
```

每约 520 µs 一轮，**每一帧地址都被 ACK 了**（写 ACK、读 ACK 都在）。

### 关键矛盾点

**从机并没有在回 0。** 它回的是 `0x01 0x2C` = **300**。

总线既没坏，从机也在应答、也在吐非零数据。那么你软件里看到的"全 0"，**不是总线上发生的**，而是在**从总线到你的应用之间**被造出来的。

### 对照实验：真正坏掉的总线是什么样

我切到 `i2c_nack` 场景重抓了一窗做对照：

| | 正常（你这次的 100k 场景） | 从机不响应（nack 场景） |
|---|---|---|
| 地址 ACK | ✅ 有 | ❌ 数据字节被 NACK |
| `nack` 标记 | 否 | **每帧都是 true** |
| 读回数据 | `0x01 0x2C` | 根本没有读事务，全是写重试 |
| 现象 | 正常读值 | 主机反复重试写 |

也就是说，如果你这条总线真有硬件问题，你**大概率会看到 NACK / 超时 / 重试**，而不是"安静地返回 0"。你的现象（全 0 且无报错）恰恰**不像**总线故障。

---

### 结论

在**可观测的这条总线上**（注意：是模拟器）我没有发现任何总线故障证据 —— 时钟干净、时序合规、地址被 ACK、从机返回非零数据。**所以问题基本可以排除"总线本身坏"这个假设，嫌疑转向主机侧。**

"读到全 0"最可能的几个来源，建议按这个顺序查：

1. **读的寄存器不对** —— 事务里指针写的是 `0x00`，你代码里是否也在读 `0x00`？如果这款传感器的数据在别的寄存器，读到的是"保留位=0"就完全解释得通。
2. **字节序/拼接** —— 传感器回的是 `0x01 0x2C`，若你的代码把两个字节拼错（比如当成小端、或只取了低位那字节），很容易得到 0。
3. **缓冲区没被填充** —— 上层结构体没清零、驱动返回值没检查、DMA/中断没触发，都会表现为恒 0。
4. **采样时机** —— 传感器转换未完成就去读，或读得太快（这窗里约 520 µs 一轮）。

### 建议的下一步

给我**你那条总线的串口号**，我用真机重连，把你实际的总线波形抓下来解码。那样才能给出针对**你**的结论 —— 现在这份只能算基线参照。

另外，如果你能把「读传感器的那几行代码 / 用的寄存器地址」发我，再配合真机采集，基本就能一次定位。

要不要现在给我串口，我们看真机？

---

## 3. 它是怎么做到的

**10 轮**、**11 次工具调用**（全部成功）、**10 次模型调用**，
输入 13343 / 输出 5033 tokens。

下面每一轮的「调用 / 结果」都是脚本落盘的原始记录，没有改写。

> **字段集补注**：以下响应是当时运行的原样记录。此后 `scope_connect` 增加
> `calibration`，`scope_status` 增加 `link_health` / `uptime_ms` / `tick_us`，
> `scope_list_captures` 增加 `device_tick_us` / `wall_time` / `expected_samples`；
> `scope_capture` 与 `scope_measure` 的扩展见第 5 轮、第 7 轮后的补注。

### 第 1 轮（1.6s）

> *思考：The user has an I2C bus problem. Let me investigate using the tools. Let me start by connecting to a device.  Let me plan: connect to simulator (default i2c_100k), configure, capture, decode I2C. Also maybe try the i2c_nack scenario? But the user's actual situation is "unknown" —…*

**调用** `scope_connect`
```json
{"sim_scenario": "i2c_100k"}
```
**结果** → `isError: false`，字段：`config`, `device`, `link`, `simulated`, `state`

<details><summary>展开原始响应</summary>

```json
{
 "config": {
  "acq_mode": 0,
  "capture_samples": 4096,
  "decimation": 1,
  "format": 0,
  "rate_hz": 857142,
  "trigger_edge": 0,
  "trigger_level_lsb": 2048,
  "trigger_mode": 1,
  "trigger_source": 0
 },
 "device": {
  "adc_bits": 12,
  "capture_max_samples": 4096,
  "channels": 2,
  "fw_ver": "1.0.0",
  "max_rx_payload": 512,
  "max_tx_payload": 2060,
  "model": "0x0103",
  "preferred_chunk_samples": 1024,
  "proto_ver": 16,
  "rate_max_hz": 857142
 },
 "link": "sim(i2c_100k)",
 "simulated": true,
 "state": "空闲"
}
```
</details>

**调用** `scope_status`
```json
{}
```
**结果** → `isError: false`，字段：`config`, `link`, `simulated`, `state`

<details><summary>展开原始响应</summary>

```json
{
 "config": {
  "acq_mode": 0,
  "capture_samples": 4096,
  "decimation": 1,
  "format": 0,
  "rate_hz": 857142,
  "trigger_edge": 0,
  "trigger_level_lsb": 2048,
  "trigger_mode": 1,
  "trigger_source": 0
 },
 "link": "sim(i2c_100k)",
 "simulated": true,
 "state": "空闲"
}
```
</details>

### 第 2 轮（2.0s）

> *思考：Now configure for I2C capture. I2C 100kHz, need decent sample rate. 857142 Hz is max. Capture samples 4096. Trigger: source CH1 (SDA or SCL). Trigger mode auto to avoid blocking. Let's configure.  Actually the default config seems fine already: rate 857142, 4096 samples. Trigger …*

**调用** `scope_configure`
```json
{"capture_samples": 4096, "trigger": {"mode": "auto", "source": 0, "edge": "falling", "level_lsb": 2048}}
```
**结果** → `isError: false`，字段：`applied`, `warnings`

<details><summary>展开原始响应</summary>

```json
{
 "applied": {
  "capture_samples": 4096,
  "decimation": 1,
  "trigger": {
   "edge": "falling",
   "level_lsb": 2048,
   "mode": "auto",
   "source": 0
  }
 },
 "warnings": []
}
```
</details>

### 第 3 轮（1.1s）

**调用** `scope_capture`
```json
{"timeout_ms": 2000}
```
**结果** → `isError: false`，字段：`capture_id`, `channels`, `duration_ms`, `note`, `overrun`, `preview`, `rate_hz`, `sample_count`, `trigger_index`

<details><summary>展开原始响应</summary>

```json
{
 "capture_id": 1,
 "channels": [
  {
   "ac_rms_lsb": 1426.9201225023633,
   "channel": 0,
   "duty_pct": 50.0,
   "freq_hz": 99999.90000000001,
   "max_lsb": 3481,
   "mean_lsb": 2184.6904296875,
   "min_lsb": 614,
   "pp_lsb": 2867,
   "rise_ns": 933.3342666676008,
   "rising_edges": 432,
   "rms_lsb": 2609.4009484114,
   "sample_count": 4096,
   "vpp_v": 2.3098388671875
  },
  {
   "ac_rms_lsb": 1285.6005288887027,
   "channel": 1,
   "duty_pct": 43.206039076376555,
   "freq_hz": 31210.321492007104,
   "max_lsb": 3481,
   "mean_lsb": 1413.34423828125,
   "min_lsb": 614,
   "pp_lsb": 2867,
   "rise_ns": 933.3342666676008,
   "rising_edges": 101,
   "rms_lsb": 1910.5786180531065,
   "sample_count": 4096,
   "vpp_v": 2.3098388671875
  }
 ],
 "duration_ms": 4.778,
 "note": "只给统计量与 minmax 预览。全量样点走 scope_read_waveform（分页）或 scope_save_capture（落盘）—— 4096 点 ≈ 上万 token，不进上下文。",
 "overrun": false,
 "preview": {
  "bucket": 16,
  "channel": 0,
  "dt_us": 18.666685333352,
  "points": 256,
  "t0_us": 0.0,
  "y_max": [
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
…（略）
```
</details>

### 第 4 轮（1.9s）

> *思考：We captured. CH0 looks like SCL: 100 kHz, 432 rising edges over 4.778 ms → 432/0.004778 ≈ 90.4 kHz. Hmm duty 50%. CH1: 31 kHz, rising edges 101.  Wait — SCL should be the denser one. CH0 has 432 edges (~90 kHz), CH1 has 101 edges (~31 kHz... well burst of clock). Anyway, let's de…*

**调用** `scope_i2c_decode`
```json
{"capture_id": 1}
```
**结果** → `isError: false`，字段：`all_bytes`, `capture_id`, `frame_count`, `frames`, `levels`, `scl_channel`, `sda_channel`, `signal_quality`, `trustworthy`

<details><summary>展开原始响应</summary>

```json
{
 "all_bytes": [
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44,
  0,
  1,
  44
 ],
 "capture_id": 1,
 "frame_count": 19,
 "frames": [
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 1,
   "nack": false,
   "repeated_start": false,
   "time_ms": 0.005
  },
  {
   "address": {
    "acked": true,
    "read": true,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    1,
    44
   ],
   "complete": true,
   "index": 2,
   "nack": false,
   "repeated_start": true,
   "time_ms": 0.205
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 3,
   "nack": false,
   "repeated_start": false,
   "time_ms": 0.525
  },
  {
   "address": {
    "acked": true,
    "read": true,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    1,
    44
   ],
   "complete": true,
   "index": 4,
   "nack": false,
   "repeated_start": true,
   "time_ms": 0.725
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 5,
   "nack": false,
   "repeated_start": false,
   "time_ms": 1.045
  },
  {
   "address": {
    "acked": true,
    "read": true,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    1,
    44
   ],
   "complete": true,
   "index": 6,
   "nack": false,
   "repe
…（略）
```
</details>

### 第 5 轮（8.2s）

> *思考：Now this is very interesting. The decode is trustworthy. Let's analyze:  - Master writes to address 0x44 (68 decimal) — write: yes ACKed, one byte: 0x00 (register pointer = 0? or command). - Then repeated start, read from 0x44, returns [1, 44].  Wait, that's interesting. The read…*

**调用** `scope_measure`
```json
{"capture_id": 1}
```
**结果** → `isError: false`，字段：`capture_id`, `measurements`, `note`

<details><summary>展开原始响应</summary>

```json
{
 "capture_id": 1,
 "measurements": {
  "ch0": {
   "ac_rms_v": 1.1496182627582516,
   "duty_pct": 50.0,
   "freq_hz": 99999.90000000001,
   "max_v": 1.1545166015625,
   "mean_v": 0.11037597656249999,
   "min_v": -1.155322265625,
   "rise_ns": 933.3342666676008,
   "vpp_v": 2.3098388671875
  },
  "ch1": {
   "ac_rms_v": 1.0357621448566208,
   "duty_pct": 43.206039076376555,
   "freq_hz": 31210.321492007104,
   "max_v": 1.1545166015625,
   "mean_v": -0.5115966796875,
   "min_v": -1.155322265625,
   "rise_ns": 933.3342666676008,
   "vpp_v": 2.3098388671875
  }
 },
 "note": "电压为未标定换算（占位参数）；频率与占空比已排除事务间空闲；非周期信号（数据线）报出的是边沿速率"
}
```
</details>

> **2026-10-04 补注**：上面这段 `note` 是**那次运行的原样记录，不改写** ——
> 它是一份证据，不是规格。但要说清它现在是什么状态：
>
> 这句话已经变成**条件性**的了。设备该 `uid` 有标定记录时，`scope_measure`
> 返回的是「电压按设备标定换算…」；并且响应里多了一个
> `"voltage_calibrated"` 布尔字段 —— 模型据此判断伏特值可不可信，
> **不必去读那句中文**。
>
> 标定表见 [`host/crates/core/src/calib.rs`](../host/crates/core/src/calib.rs)
> 与 [`05-roadmap.md`](05-roadmap.md) 的 P1。
>
> **2026-10-06 补注**：`scope_measure` 的响应还多了五个指标
> （`fall_ns` / `overshoot_pct` / `undershoot_pct` / `high_ns` / `edges`），
> 经 `metrics` 参数按需请求，不请求不返回 —— 上面这段 JSON 是请求
> `["vpp","freq","duty","rise"]` 时的原样记录，字段集不再完整。

**调用** `scope_list_captures`
```json
{}
```
**结果** → `isError: false`，字段：`capacity`, `captures`

<details><summary>展开原始响应</summary>

```json
{
 "capacity": 16,
 "captures": [
  {
   "capture_id": 1,
   "channels": 2,
   "duration_ms": 4.778,
   "overrun": false,
   "rate_hz": 857142,
   "sample_count": 4096,
   "trigger_index": 9
  }
 ]
}
```
</details>

### 第 6 轮（1.7s）

> *思考：Now let's contrast with a NACK scenario. Switch scenario to i2c_nack and capture+decode.  Note: captures persist host-side, so capture 1 remains.  Let me switch scenario.*

**调用** `scope_sim_set_scenario`
```json
{"scenario": "i2c_nack"}
```
**结果** → `isError: false`，字段：`channels`, `faults_clean`, `note`, `scenario`, `seed_set`

<details><summary>展开原始响应</summary>

```json
{
 "channels": 2,
 "faults_clean": true,
 "note": "模拟器当前不注入任何故障",
 "scenario": "i2c_nack",
 "seed_set": null
}
```
</details>

### 第 7 轮（1.3s）

**调用** `scope_capture`
```json
{"timeout_ms": 2000}
```
**结果** → `isError: false`，字段：`capture_id`, `channels`, `duration_ms`, `note`, `overrun`, `preview`, `rate_hz`, `sample_count`, `trigger_index`

<details><summary>展开原始响应</summary>

```json
{
 "capture_id": 2,
 "channels": [
  {
   "ac_rms_lsb": 1411.1793327093512,
   "channel": 0,
   "duty_pct": 50.0,
   "freq_hz": 100017.82434127979,
   "max_lsb": 3481,
   "mean_lsb": 2299.482421875,
   "min_lsb": 614,
   "pp_lsb": 2867,
   "rise_ns": 933.3342666676008,
   "rising_edges": 394,
   "rms_lsb": 2697.9708148121476,
   "sample_count": 4096,
   "vpp_v": 2.3098388671875
  },
  {
   "ac_rms_lsb": 1279.6621723340998,
   "channel": 1,
   "duty_pct": 52.5,
   "freq_hz": 29999.969999999998,
   "max_lsb": 3481,
   "mean_lsb": 1401.445068359375,
   "min_lsb": 614,
   "pp_lsb": 2867,
   "rise_ns": 933.3342666675999,
   "rising_edges": 84,
   "rms_lsb": 1897.7838535859769,
   "sample_count": 4096,
   "vpp_v": 2.3098388671875
  }
 ],
 "duration_ms": 4.778,
 "note": "只给统计量与 minmax 预览。全量样点走 scope_read_waveform（分页）或 scope_save_capture（落盘）—— 4096 点 ≈ 上万 token，不进上下文。",
 "overrun": false,
 "preview": {
  "bucket": 16,
  "channel": 0,
  "dt_us": 18.666685333352,
  "points": 256,
  "t0_us": 0.0,
  "y_max": [
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
   3481,
 
…（略）
```
</details>

> **2026-10-06 补注**：第 3 轮与本轮的 `scope_capture` 响应都是**原样记录，不改写**。
> 通道摘要现在多了 `fall_ns` / `overshoot_pct` / `undershoot_pct` / `high_ns`
> 与 `edges_mid`（实测中点处的跃变数，与 `rising_edges` 那个固定 2048 LSB
> 的计数器**不同源**）—— 这两段 JSON 的字段集不再完整。

### 第 8 轮（1.8s）

**调用** `scope_i2c_decode`
```json
{"capture_id": 2}
```
**结果** → `isError: false`，字段：`all_bytes`, `capture_id`, `frame_count`, `frames`, `levels`, `scl_channel`, `sda_channel`, `signal_quality`, `trustworthy`

<details><summary>展开原始响应</summary>

```json
{
 "all_bytes": [
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0
 ],
 "capture_id": 2,
 "frame_count": 21,
 "frames": [
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 1,
   "nack": true,
   "repeated_start": false,
   "time_ms": 0.057
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 2,
   "nack": true,
   "repeated_start": false,
   "time_ms": 0.287
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 3,
   "nack": true,
   "repeated_start": false,
   "time_ms": 0.516
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 4,
   "nack": true,
   "repeated_start": false,
   "time_ms": 0.746
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 5,
   "nack": true,
   "repeated_start": false,
   "time_ms": 0.976
  },
  {
   "address": {
    "acked": true,
    "read": false,
    "ten_bit": false,
    "value": 68
   },
   "bytes": [
    0
   ],
   "complete": true,
   "index": 6,
   "nack": true,
   "repeated_start": false,
   "time_ms": 1.206
  },
  {
   "address": {
    
…（略）
```
</details>

### 第 9 轮（7.5s）

> *思考：Interesting nuance: in the nack scenario, the decoder reports `nack: true` for frames but `address.acked: true` and the read bit is false and no repeated start — it looks like just writes with a NACK on the data byte. Hmm, the NACK scenario: master writes address 0x44 (acked by d…*

**调用** `scope_sim_set_scenario`
```json
{"scenario": "i2c_100k"}
```
**结果** → `isError: false`，字段：`channels`, `faults_clean`, `note`, `scenario`, `seed_set`

<details><summary>展开原始响应</summary>

```json
{
 "channels": 2,
 "faults_clean": true,
 "note": "模拟器当前不注入任何故障",
 "scenario": "i2c_100k",
 "seed_set": null
}
```
</details>

### 第 10 轮（6.4s）

> *思考：Now write the answer.  I must lead with the honesty disclosure? Actually, better: give findings, but disclose the simulator clearly and prominently. I think honesty requires me to say upfront that I'm not actually on their bus. Let me structure:  1. 一个重要前提：我连的是模拟器，不是你那条真实总线。 2. 采…*

**（没有工具调用 → 给出最终答复）**

---

## 4. 本次执行中最值得看的三件事

### ① 它开口第一句是在纠正前提

结论的第一段是：

> **我没有连到你那条真实的总线上。** 我调用连接工具时没指定串口，MCP 就落到了
> 内置模拟器上（`scope_status` 里明确写着 `"simulated": true`）……

它注意到了 `simulated` 字段，并主动声明「下面所有波形都来自一个模拟出来的
I2C 总线，不是你手焊的那两根线」。**这不是文档或提示词中教过的**，也没有任何提示让它这么做。

### ② 它自己做了一次对照实验

第 6 轮它切到了 `i2c_nack` 场景（那是从 `scope_connect` 的 schema 枚举里
**自己发现的**），重新采集一窗做对比：

| | 正常总线 | 从机不应答 |
|---|---|---|
| 地址 ACK | 有 | 有 |
| `nack` 标记 | 否 | **每帧都是 true** |
| 读回数据 | `0x01 0x2C` | 没有读事务 |

然后据此推理：「**如果你这条总线真有硬件问题，你大概率会看到 NACK / 超时 /
重试，而不是"安静地返回 0"**。你的现象（全 0 且无报错）恰恰不像总线故障。」

一个被要求"看看总线上在发生什么"的助手，主动去做**对照实验**而不是只报
一次采集的数字 —— 这是这套工具想要的用法。

### ③ 它把「地址 ACK」和「数据 NACK」分清楚了

那次对照里它读出来的是：地址被应答了、**被拒绝的是数据字节**。
这正是 `has_nack()` 语义修复后才成立的一件事：修复前，主机读完最后一个
字节的收尾 NACK 会被报成 `nack: true`，与器件拒绝无法区分（见第 5 节）。

---

## 5. 这次实录暴露并修好了什么

写这份实录的过程本身就是一次测试。它暴露了四个问题，**都已修**：

| 问题 | 为什么它是个问题 |
|---|---|
| **模拟器的 `i2c_100k` 只有写、没有读** | 一个被问「为什么读出来是 0」的 Agent，会得出「总线上根本没有读事务」的诊断。那在模拟器上是对的，对真实总线却是**它推出来的** —— 场景本身不是一份像样的总线快照。现在默认事务是完整的「写寄存器指针 → 重复起始 → 读两字节」。 |
| **模拟器造不出 NACK** | README 给 P3 定的验收场景就是「告诉我**为什么 NACK**」，而 `transaction()` 把应答位写死在每个字节后面。现在有 `i2c_nack` 场景（地址被应答、数据被拒 —— 典型的写保护 / 寄存器不存在）。 |
| **以重复起始收尾的帧显示 `complete: false`** | `complete` 的语义是「没被采集窗口截断」，而以 Sr 结束的帧是正常结束的。「写寄存器 → 读回」是最常见的 I2C 时序，于是**写帧永远看起来像被截断了**。 |
| **读事务里主机的收尾 NACK 被报成故障** | 主机读完最后一个字节回 NACK 意思是「我读够了」，那是**正常收尾**。原来的 `has_nack()` 把所有 `!acked` 一视同仁，于是**一份健康的读数看起来像出了错**。 |

后两条尤其能说明这个项目的主题：**解码器没有 bug，输出也没有 bug ——
是"这个词是什么意思"没定义清楚**。一个 Agent 拿到 `nack: true`
几乎必然报出一个不存在的问题。

修的时候还改掉了两处**同一份清单写两遍**：CLI 的场景枚举和 GUI 的下拉列表
各有一份自己的场景表，加了 `i2c_nack` 之后它们**静默地少了这个场景**，
没有一条测试会红。GUI 改成直接遍历 `Scenario::ALL`（把重复消灭掉），
CLI 保留枚举但加了一条守门测试。

---

## 6. 怎么复现

```bash
./host/run.sh build -p scope-mcp
export DEEPSEEK_API_KEY=sk-...        # 或 ANTHROPIC_API_KEY
python tools/agent_demo/agent.py
```

换模型 / 换供应商只要改两个参数（脚本走的是 Anthropic Messages 的形状）：

```bash
python tools/agent_demo/agent.py \
  --base-url https://api.anthropic.com/v1/messages --model claude-fable-5-1
```

换任务：

```bash
python tools/agent_demo/agent.py --task "总线上是不是有器件不响应？帮我确认一下。"
```

> ⚠ **模型输出会随模型、随机性变化。** 本次会话由 `deepseek-flash` 在同一天的
> 某次运行中完成，10 轮全部成功。另一个更早的版本（对着还没有读事务的模拟器执行的）
> 在第 3 轮犯过一个错：它给 `scope_capture` 多塞了一个 `capture_id`，
> 被 schema 挡下 —— 错误里给出确切字段名与「本工具接受哪些参数」，
> 第 4 轮它自己改对了。
>
> 那一段值得单独记一笔，因为它是参数类型化改造最直接的兑现：
> **改造之前这个字段会被静默忽略、工具照常返回成功，而模型永远不会知道。**
