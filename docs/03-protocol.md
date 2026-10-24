# 03 · 协议规格（唯一真相源）

> **本文件是契约。** 修改协议必须**同一次提交**更新四处：
> `docs/03-protocol.md` + `proto/protocol.h` + `host/crates/proto/src/lib.rs` + `proto/tests/vectors.json`。
> CI 会跑两端黄金向量，不同步就红。

版本：`0x10`（主 1 / 次 0） · 状态：v1 生效（C 与 Rust 两端逐字节一致，50 项黄金向量全绿）
最后更新：2026-10-10 · ⚠ **设备侧尚未实现的部分逐条列在文末 [§11](#11-实现状态2026-10-10-实测)** ——
那份清单是实测结果，不是待办愿望；把它当成本契约的"当前实现覆盖率"来读。

---

## 1. 总则

1. **线上只用定长小端二进制**。JSON 只存在于 Agent ↔ MCP 层。
2. **控制链路永不出现 `f32`**。电平用 ADC LSB 整数，时间用整数 µs，测量结果用定点（字段名自带缩放）。
3. **所有 `SET_*` 必须回显实际生效值**。F103 的采样率被定时器分频量化，主机必须以回显建时间轴。
4. **未知 `CMD` 必须按 `LEN` 吞掉整帧**再回 `UNKNOWN_CMD` —— 否则帧会错位。
5. **不可信的帧不回应答**。CRC 错、超长的帧静默丢弃并计数（`rx_crc_err` / `rx_dropped`），通过 `GET_STATUS` 可见。
6. **设备端绝不能阻塞**。TX 队列满就丢分片并计数（`tx_dropped`），任何发送点都不得等待。

---

## 2. 帧格式

```
偏移  长度  字段      说明
 0     2   SYNC      0xAA 0x55 恒定
 2     1   VER       高 4 位主版本 / 低 4 位次版本，当前 0x10
 3     1   FLAGS     bit0 RESP    1=响应
                     bit1 ERROR   1=错误
                     bit2 MORE    分片未结束
                     bit3 LAST    分片结束
                     bit4 EVENT   设备主动上报
                     bit5 NO_REPLY 免回复
                     bit6-7 保留，写 0 读忽略
 4     2   SEQ       u16 LE，方向独立计数
 6     2   CMD       u16 LE，高字节=命令类，低字节=类内编号
 8     2   LEN       u16 LE，payload 字节数 0..MAX_PAYLOAD
10     N   PAYLOAD   按 CMD 定长的字段，或裸样本数组
10+N   2   CRC16     u16 LE
```

**固定开销 12 字节**，帧长 = `12 + LEN`。

### 字段为何存在

| 字段 | 理由 |
|---|---|
| `SYNC` | UART / USB CDC 都是字节流，必须有重同步锚点。双字节降低 payload 内假同步率。出错时**以 +1 字节步进扫描** `AA 55` 重同步（不是按帧长跳过），CRC 兜底，因此**不需要 COBS / 转义**。 |
| `VER` | 联合开发期两端版本必然漂移。主版本不符时设备仍应答 `GET_INFO` / `PING`，其余回 `VERSION_MISMATCH`，便于诊断。 |
| `FLAGS` | 不解 payload 即可分派（响应/错误/分片/事件/免回复）。通用解析器只依赖它。 |
| `SEQ` | 请求-响应配对 + 重试去重。下行用于流分片丢帧检测。 |
| `CMD` | 16 bit 操作码，高字节分类，为新增命令留足空间。 |
| `LEN` | 定长二进制不自描述。**解析顺序固定**：读头 → 校验 `LEN ≤ 上限` → 读 payload → 算 CRC。先验长度再处理，防越界。 |
| `CRC16` | 位错误检测。见下节。 |

### CRC16 参数

```
算法      CRC-16/CCITT-FALSE
poly      0x1021
init      0xFFFF
refin     false
refout    false
xorout    0x0000
覆盖范围  VER .. PAYLOAD 末尾（不含 SYNC）
标准校验  "123456789" → 0x29B1
```

F103 的**硬件 CRC 单元只支持固定 CRC-32 多项式**，做不了 CCITT —— 用 256 项查表（512 B Flash）软件实现，2 KB 帧约 0.2 ms @72MHz，可接受。CRC 增量计算，早失败早丢弃。

### 最大帧长（双向上限不对称）

| 方向 | MAX_PAYLOAD | MAX_FRAME | 说明 |
|---|---|---|---|
| 设备 → 主机 | **2060 B** | **2072 B** | 12 B 分片头 + 1024 样点 × u16 |
| 主机 → 设备 | **512 B** | 524 B | 实际命令都 ≤ 64 B，超限回 `BAD_LEN` |

> ⚠ **实现状态（2026-10-10）**：设备侧**没有强制上行 512 B 这条上限** ——
> `PROTO_MAX_PAYLOAD_RX` 只在 `GET_INFO` 里被如实上报（`proto_task.c` 的 `max_rx_payload`），
> 解析器判越界用的是 `PROTO_MAX_PAYLOAD_TX`(2060)（`protocol.c:229`），
> 而设备接收缓冲只有 `PROTO_MAX_FRAME_RX`(524)。所以 >512 B 的上行帧不会被回
> `BAD_LEN`，而是先撞上缓冲上限、按"丢最旧字节"处理。
> 主机侧在本地就拦住了（`scope-core` 的 `transaction` 拒绝 >512 B 的载荷），
> 正常路径碰不到它。见文末 §11。

> ⚠️ **`MAX_PAYLOAD_TX` 是 2060，不是 2048。**
> 波形分片的 **12 字节分片头是算在 payload 之内的** ——
> 1024 × 2 + 12 = 2060。写成 2048 会让满载分片超限，这是很容易算漏的一处。
> 代码里的常量由 `PROTO_CHUNK_HEADER_LEN + PROTO_CHUNK_SAMPLES_MAX * 2` 推出，不要手写数字。

设备 RX 环形缓冲 1 KB（容纳 2 个最大上行帧）。
设备 TX **不做大帧暂存** —— 从采集缓冲零拷贝分段发送（一边算 CRC 一边进 TX），只暂存 10 B 帧头。

---

## 3. 字节序与结构布局纪律

- 所有多字节整数**一律小端**（STM32 与 x86/ARM64 主机均 LE），显式写进规格。
- **禁止线上格式依赖编译器结构体布局**：
  - C 侧：`#pragma pack(1)` + `_Static_assert(offsetof(...))` 锁死每个字段偏移与总长
  - Rust 侧：`#[repr(C)]` + 偏移断言
- 保留位**写 0 读忽略**。

---

## 4. 状态机

```
                    ┌──────────────────────────────┐
                    │                              │
   ┌────────┐ ARM   ┌────────┐  触发   ┌────────┐  │
   │  IDLE  ├──────►│ ARMED  ├────────►│  DONE  │  │
   └───┬────┘       └───┬────┘         └───┬────┘  │
       │                │                  │       │
       │           STOP │                  │ 重新  │
       │                ▼                  │ ARM   │
       │           ┌──────────┐            │       │
       └──────────►│STREAMING │────────────┘       │
             STOP  └────┬─────┘                    │
                        │                          │
                   FAULT└──────────────────────────┘
```

| 状态 | 值 | 允许的命令 |
|---|---|---|
| `IDLE` | 0 | **全部** |
| `ARMED` | 1 | 通用命令 + `FORCE_TRIGGER` |
| `STREAMING` | 2 | 同 `ARMED` |
| `DONE` | 3 | **全部**（再次 `ARM` 会丢弃当前采集的数据） |
| `FAULT` | 4 | 通用命令 + `RESET` |

**通用命令**（任何状态都允许）：`GET_INFO` / `PING` / `GET_STATUS` / `STOP` /
`GET_LAST_ERROR`，以及三个设备→主机的事件帧
（`EVENT_TRIGGER` / `EVENT_OVERRUN` / `EVENT_LOG`）。

`ARMED` / `STREAMING` 下 `RESET` / `GET_CONFIG` / `SET_*` / `ARM` / `READ_BUFFER` /
`MEASURE` **全部回 `BUSY`**（提示先 `STOP`）。

> ⚠ 这张表的真相源是 `proto/tests/vectors.json` 里的 `state_matrix` ——
> C 与 Rust 两侧都按它断言，两边实测一致。
>
> **曾经的错误**：这张表写过「`IDLE` 除 `STOP` 外全部」「`DONE` 除 `ARM` 外全部」，
> 而实现（以及 `protocol.c` 里紧挨着 `return true` 的那句注释）说的是另一回事 ——
> 实际两者都是**全允许**。按错误的表去推断，会以为「`DONE` 下 `ARM` 会被拒绝、
> 从而保护住当前采集」，实际上它会被接受并**丢弃**那份数据。
> 契约页的错比别处的错更贵：它会被人当成行为去依赖。

**重试安全**：设备缓存**最近一次的 `(seq, CMD, 完整响应)`**，同一 seq 在 1 s 内重发**直接回放**，保证 `ARM` 等非幂等命令的重试安全。

> ⚠ **实现状态（2026-10-10 实测）**：**设备端尚未实现这个缓存** —— `proto_task_t`
> 里没有它（见 `firmware/App/proto_task.h`），`App/proto_task.c` 也没有任何去重逻辑；
> 目前只有模拟器 `host/crates/sim` 实现了单槽回放。
> 主机侧的重试策略按本条设计（沿用同一 seq 重发），因此**真机上重试 `ARM` 会被重新执行**。
> 见文末 §11。
>
> 缓存只有 1 槽 → **主机不得并发流水多个非幂等请求**。

---

## 5. 命令表

### 5.1 系统类 `0x01xx`

| CMD | 名称 | 请求 | 响应 |
|---|---|---|---|
| `0x0101` | `GET_INFO` | 空 | `{proto_ver:u8, fw_ver:u32, model:u16, uid:[u8;12], tick_hz:u32(=1000000), adc_bits:u8(=12), ch_count:u8, rate_min_hz:u32, rate_max_hz:u32, capture_max_samples:u16, max_rx_payload:u16, max_tx_payload:u16, preferred_chunk_samples:u16, caps:u32}` |
| `0x0102` | `PING` | `{len:u8≤32, data[]}` | payload 原样回显 + `{tick_us:u32}`（应答发出时刻） |
| `0x0103` | `GET_STATUS` | 空 | `{state:u8, err_flags:u16, ring_fill_samples:u16, last_capture_id:u16, last_trigger_index:u32(无=0xFFFFFFFF), overrun_samples:u32, rx_crc_err:u16, rx_dropped:u16, tx_dropped:u32, uptime_ms:u32, tick_us:u32, last_error_code:u16}` |
| `0x0104` | `RESET` | `{magic:u8=0xA5}` | `Ack`，随后软复位 ~50 ms |

**`GET_INFO` 是连接后的第一条命令**（主版本不匹配时也必须应答）。`caps` 位掩码声明通道数、触发模式集、AC 耦合支持、支持的波形格式（PACK12 / MINMAX / DELTA）、设备侧测量项。

`GET_STATUS` 是 Agent 问「现在什么情况」的首选命令，**任何状态可调**。

> **这几个计数器现在是真统计的**：模拟器按实际发生的溢出/丢帧/解析错误累加，
> `scope_core::DeviceStatus` 解析全部 33 字节，MCP 的 `scope_status` 通过
> `link_health` 透出（含一句 `clean` 判据）。
>
> 回归：这三个计数器原先是假的 —— `overrun_samples` / `rx_crc_err` / `rx_dropped`
> 写死 `0`，core 的 `get_status` 只读 `state` 字节、丢掉其余 32 字节。那时在 `sim`
> 上读到"零溢出"只说明没人统计过、不是证据，Agent 会把它当成"链路很干净"。

`RESET` 用 `magic` 防误触发。主机需重新 `GET_INFO` 并重置 seq。

> ⚠ **实现状态（2026-10-10 实测）** —— 本节有三处**契约与固件不一致**：
>
> | 命令 | 契约（本表） | 固件实测 |
> |---|---|---|
> | `PING` | 回显 payload **+ `tick_us`** | **只回显 payload，没有 `tick_us`**（`case CMD_PING:`，`firmware/App/proto_task.c:443-445` 只 `send_frame(... f->payload ...)`）；**主机也不解析它**（`host/crates/core/src/command.rs:463-469` 只取 `len` + 回显数据）→ 于是 §6.6 承诺的"设备时钟偏移"目前**两端都没实现** |
> | `RESET` | `Ack` 之后**软复位 ~50 ms** | **不软复位**：`acq_stop` + `acq_init`（配置回出厂值）+ `Ack`（`cmd_reset`，`proto_task.c:388-399`） |
> | `GET_INFO.caps` | 声明支持的触发模式与波形格式 | ✅ **已修（`4e165e6`）**：原先声明 `CAP_TRIG_SOFT` 与 `CAP_FMT_MINMAX`，但 `FORCE_TRIGGER` 恒回 `ERR_UNSUPPORTED`、`READ_BUFFER` 拒绝 MINMAX —— 两位已从 `DEV_CAPS` 摘掉（`proto_task.c`），详见 §11.2 |
>
> `RESET` 不软复位对主机其实是好事（会话中途不断链），但契约页必须写清。
> 其余两处见上表与文末 §11。

### 5.2 配置类 `0x02xx`

| CMD | 名称 | 请求 | 响应 |
|---|---|---|---|
| `0x0200` | `GET_CONFIG` | 空 | 全部可设置项镜像（**实际是手拼的 17 字节，见下方布局**） |
| `0x0201` | `SET_SAMPLE_RATE` | `{requested_hz:u32}` | **`{actual_hz:u32}`** |
| `0x0202` | `SET_CHANNEL` | `{ch:u8, enable:u8, range_idx:u8, coupling:u8, offset_lsb:i16}` | 回显生效值 |
| `0x0203` | `SET_TRIGGER` | `{mode:u8, source:u8, edge:u8, level_lsb:u16, pre_samples:u16, holdoff_us:u32}` | 回显生效值 |
| `0x0204` | `SET_ACQ` | `{mode:u8, capture_samples:u16, format:u8, decimation:u16}` | `Ack` + 回显 |

**`GET_CONFIG` 的实际线上布局（17 字节，小端）** —— 协议里**没有** `config_resp_t` 这个
结构体，设备是手拼的，主机按同样的偏移读（`host/crates/core/src/command.rs` 的
`DeviceConfig::decode`）。**两边都不许改**：

```
偏移  长度  字段                说明
 0     4   rate_hz            u32，实际生效的采样率（量化后）
 4     1   acq_mode           u8，0=single / 1=stream
 5     2   capture_samples    u16
 7     1   format             u8，0=RAW16 / 1=PACK12 / 2=MINMAX
 8     2   decimation         u16
10     1   trigger_mode       u8，0=auto / 1=normal / 2=single
11     1   trigger_source     u8，0=CH1 / 1=CH2 / 2=SOFT
12     1   trigger_edge       u8，0=rising / 1=falling
13     2   trigger_level_lsb  u16（ADC LSB）
15     1   ch0_enable         u8，固件恒为 1
16     1   ch0_coupling       u8，固件恒为 0（DC）
```

> ⚠ 因此 **`GET_CONFIG` 不回 `range_idx` 与 `offset_lsb`** —— 这两个字段在设备侧
> 只有 `SET_CHANNEL` 能写、读不回来（README §5.4.4 已把这条列为"交给 AI 采集"的已知限制）。

**`SET_SAMPLE_RATE` 必须回显**：F103 只有 72 MHz / 定时器整数分频的量化档位，主机时间轴一律用 `actual_hz`。

**`SET_CHANNEL` 的电平/偏移一律用 ADC LSB 整数**。伏特换算、量程衰减、AC/DC 校正**全部留在上位机**（按 `uid` 存标定表）：MCU 不使用浮点，协议规定电平/偏移字段的单位为 ADC LSB，伏特量纲仅在主机侧存在。

> 标定表实现在 [`host/crates/core/src/calib.rs`](../host/crates/core/src/calib.rs)：
> 一个按 `uid` 索引的 JSON 文件（与 AI 配置同目录），CLI 写入
> （`scope-cli cal set`），GUI 与 MCP **只读**。没有记录时用占位换算
> （3.3 V / 4096、零点 2048），四端都会如实标出「未标定」——
> MCP 侧是一个 `voltage_calibrated` 布尔字段，不是一个要读懂的中文句子。

> ⚠ **这块板上 `SET_CHANNEL` 的五个字段并不是都做得到**（见 [`02-hardware.md`](02-hardware.md) §5）：
>
> | 字段 | 本板能不能真的生效 |
> |---|---|
> | `enable` | ✅ 软件控制 |
> | `offset_lsb` | ✅ 软件控制（垂直偏移是数字的） |
> | `coupling` | ❌ **手拨开关 SW2** —— 下发值只被记录并回显，实际以开关位置为准 |
> | `range_idx` | ❌ **手拨开关 SW3** —— 同上 |
> | `ch` | ✅（决定上面几个作用在哪个通道） |
>
> MCP 层在收到被程控不了的字段时会**明确警告**，而不是让 Agent 以为耦合真的切了。

`SET_TRIGGER.mode`：`0=auto` / `1=normal` / `2=single`。
触发比较带 **±16 LSB 迟滞**防抖动重触发。
**`auto` 模式必须在约 200 ms 无触发后强制完成一次采集** —— 否则 Agent 的 capture 会永久阻塞。

`SET_ACQ.format`：`0=RAW16` / `1=PACK12` / `2=MINMAX`。
`SET_ACQ.decimation`：`1..256`，MINMAX 的桶大小或流的抽点倍数。

### 5.3 采集类 `0x03xx`

| CMD | 名称 | 请求 | 响应 | 幂等 |
|---|---|---|---|---|
| `0x0301` | `ARM` | 空 | `Ack`（已武装） | ❌ 靠 seq 去重 |
| `0x0302` | `STOP` | 空 | `Ack`（已停也回 Ack，不报错） | ✅ |
| `0x0303` | `FORCE_TRIGGER` | 空 | `Ack`，仅 `ARMED` 有效 **（⚠ 固件回 `ERR_UNSUPPORTED`，见下）** | ✅ |

单次模式：触发后发 `EVENT_TRIGGER`，数据冻结，主机用 `READ_BUFFER` 随机拉取。
流模式：按 `SET_ACQ` 的 format/decimation 持续推分片直到 `STOP`。

**流模式的设备发送循环必须在分片之间轮询 RX**，以保证 `STOP` 及时生效（目标延迟 < 100 ms）。

> ⚠ **实现状态（2026-10-10 实测）**：
> - **`FORCE_TRIGGER` 未实现** —— 固件恒回 `ERR_UNSUPPORTED`（"soft trigger not wired yet"，
>   `cmd_force_trigger`，`proto_task.c:282-287`）。原先 `caps` 还宣称 `CAP_TRIG_SOFT`，属自相矛盾；
>   **该位已于 `4e165e6` 从 `DEV_CAPS` 摘掉**（见 §11.2），
>   于是 `caps` 与行为重新一致 —— 但命令本身仍未实现。
> - **流模式（`STREAMING`）未实现** —— `ACQ_MODE_STREAM` 只是被存进配置、从不进入该状态
>   （`App/acq.c`），所以"分片之间轮询 RX"这条目前无从谈起。主机侧要连续观察走的是
>   `scope_watch`（重复单次采集），不是流模式。
> 见文末 §11。

### 5.4 读取类 `0x04xx`

**`0x0401 READ_BUFFER`**

请求：`{capture_id:u16, start_sample:u32, count:u16, format:u8, ch:u8}`

响应 payload = **12 B 分片头** + 样本：

```
{capture_id:u16, start_sample:u32, count:u16, decimation:u16, format:u8, flags:u8}
   flags: bit0 LAST (本采集最后一片)
          bit1 INVALID (数据无效 / 期间发生 overrun)
```

- `count` 上限：`RAW16` / `PACK12` ≤ 1024 样点；`MINMAX` ≤ 512 对
  （两者都是 payload ≤ 2060 B；`PACK12` 因为 2 样点挤进 3 字节，只占 1548 B）
- **`start_sample` 是绝对样点号** → 任何分片可独立识别、**按同偏移重拉即可，天然幂等**
- `capture_id` 不符、或当前没有已冻结的采集 → `NO_DATA`
  （⚠ **不是 `BUSY`**：`BUSY` 只由状态机门禁产生，见 §11.3）
- `count == 0` 或 `count > 1024` → `BAD_PARAM`（`proto_task.c:331`）
- `start_sample >= window_len`（越界）→ **不是错误**：回一个 `count=0` + `LAST` 的空分片，
  主机的分页循环靠它自然收尾（`proto_task.c:314-320`）
- 分片末端超出窗口 → **截断到窗口末尾**（`end = window_len`），不是报错

### 5.5 测量类 `0x05xx`

**`0x0501 MEASURE`**

请求：`{capture_id:u16, start_sample:u32, window_samples:u32, md_mask:u32}`

响应（**全部整数 / 定点，线上永不出现 f32**）：

```
{min_lsb:u16, max_lsb:u16, pp_lsb:u16,
 mean_x256:i32, rms_x256:u32,
 rising:u16, falling:u16,
 period_samples_x256:u32, duty_x10000:u16,
 rise_time_samples:u16, fall_time_samples:u16,
 quality:u16}   quality: bit0 削顶 / bit1 信噪比低 / bit2 有效沿太少
```

- RMS / 方差**必须用 `u64` 累加**（`4095² × 4096 ≈ 2³⁶`，u32 必溢出）
- 低信噪比时应返回 `UNSUPPORTED` 或置 `quality` 标志，**不得返回看似精确的无依据数值**
- v1 允许回 `UNSUPPORTED`（主机用全速原始数据自己算，精度更高）
- **实现状态（2026-10-10 实测）**：设备端**恒回 `ERR_UNSUPPORTED`**（`App/measure.c` 不存在，
  `cmd_measure`，`proto_task.c:401-407`）；主机侧也**从不调用它** —— `scope_measure` 走本地
  `scope_core::measure()`。也就是说这条命令目前是"契约上有、两端都没走"。
  它不会骗人（不会回全 0 的"测量结果"），但也确实还没有设备侧实现。

### 5.6 调试与事件 `0x06xx`

| CMD | 名称 | 说明 |
|---|---|---|
| `0x0601` | `ECHO` | 任意字节（≤512）原样回显。链路吞吐与无差错验证、最大帧长测试 |
| `0x0602` | `SET_LOG_LEVEL` | `{level:u8}` 0 关 .. 3 trace |
| `0x0603` | `GET_LAST_ERROR` | `{code:u16, severity:u8, tick_us:u32, msg_len:u8, msg[≤32]}` |
| `0x0610` | `MEM_READ` | **仅 DEBUG 构建**，绝不注册给 MCP/LLM |
| `0x0611` | `MEM_WRITE` | **仅 DEBUG 构建**，绝不注册给 MCP/LLM |
| `0x0682` | `EVENT_TRIGGER` | 设备→主机 `{capture_id:u16, trigger_index:u32, tick_us:u32, rate_hz:u32, n_samples:u32}` |
| `0x0683` | `EVENT_OVERRUN` | 设备→主机 `{dropped_samples:u32, capture_id:u16}` |
| `0x0684` | `EVENT_LOG` | 设备→主机 `{level:u8, tick_us:u32, len:u8, utf8[]}` |

> ⚠ **实现状态（2026-10-10 实测）** —— 本节**已实现的只有 `ECHO`、`EVENT_TRIGGER`、`EVENT_OVERRUN`**：
>
> | CMD | 固件实际行为 |
> |---|---|
> | `0x0602` `SET_LOG_LEVEL` | 只回一个**空 `Ack`**，等级不存（`case CMD_SET_LOG_LEVEL:`，`proto_task.c:486`） |
> | `0x0603` `GET_LAST_ERROR` | 同上：**没有 `{code, severity, tick_us, msg}` 载荷**（`proto_task.c:487`） |
> | `0x0610` / `0x0611` `MEM_*` | `protocol.h` 有定义，但 `handle_frame` 里**没有分支** → 落到 default 回 `UNKNOWN_CMD` |
> | `0x0684` `EVENT_LOG` | **从不发出**：`proto_cmd_allowed` 的白名单里有它，但没有生产者 |
>
> `ECHO` 的"≤512"上限由**设备接收缓冲**隐式限制（见 §2 的状态块），不是显式校验。
> 见文末 §11。

**固件日志必须走 `0x0684 EVENT_LOG` 帧（或独立调试 UART），严禁裸 `printf` 混入二进制流。**

### 5.7 错误 `0x0701`

带 `ERROR` flag 的响应，payload：
`{code:u16, severity:u8, msg_len:u8, msg[≤32]}`，`severity`：0 warn / 1 err / 2 fatal。

### 5.8 编号演进规则

| 区段 | 用途 |
|---|---|
| `0x00xx` | 保留 |
| `0x71xx-0x7Fxx` | 实验 / 厂商区 |

- 新增命令只动**低字节**并升 `VER` 次版本
- 字段变更走**新 `CMD` 号** + 升主版本
- 编号**一经分配永不复用**

---

## 6. 波形传输

### 6.1 设备侧缓冲组织（20 KB SRAM 的现实）

```c
uint16_t ring[4096];   // 8 KB，TIM 触发 ADC 以 857 kSPS 循环写，DMA1 Ch1
```

- HT/TC 中断每 **2048 样点**（≈2.39 ms）发布一个「完成半区」
- `prod`/`cons` 以样点计数、**u16 模 4096 运算**：`available = (uint16_t)(prod - cons)`
  **禁止直接用 `<` 比较原始索引**
- 绝对样点计数 `total_samples` 用 `u32`（857 kSPS 下约 83 分钟回绕，主机按模差处理）
- **所有权规则**：正在被 DMA 写的半区不可读、不可算 CRC。宁慢勿乱。

**单次采集 = 4096 点**（默认触发前 2048 / 后 2048）：
触发搜索在每个完成的 2048 点半区上做（迟滞电平比较）→ 找到触发绝对点 `T` → 继续采到 `T + post + 512` 样点余量（覆盖停 DMA 的延迟）→ 停 DMA → 切片 → 上报 `EVENT_TRIGGER`。

**8 KB = 4096 点就是 F103 的物理上限**，这一点必须写进 UI 与文档预期。

### 6.2 链路预算

| 链路 | 实测吞吐 | RAW u16 持续上限 | 设计值 |
|---|---|---|---|
| USB CDC | 500–900 KB/s | 250–450 kSa/s | **150–250 kSa/s**（留 40% 余量） |
| UART 921600 | 92 KB/s | ~46 kSa/s | **27 kSa/s** |
| UART 115200 | 11.5 KB/s | ~5.8 kSa/s | 5 kSa/s |

**任何链路都塞不下 857 kSPS 原始数据** —— 所以：

- **单次模式** = 采完缓存在 8 KB 环里慢慢传（4096 点 = 8 KB，USB 约 12 ms，921600 串口约 91 ms）✅ 可行
- **连续实时显示** 必须依赖设备侧抽点 / 降采样

### 6.3 分片规则

- 默认分片 = **1024 样点**（RAW16 时 payload 2060 B、帧 2072 B）
- UART 场景主机可改为 512 样点（1024 B，约 11 ms/片）降延迟
- 分片头 12 B，`start_sample` 是绝对样点号 → 任何分片可独立识别、独立按偏移重拉

### 6.4 压缩格式（按收益排序）

| 格式 | 编码 | 收益 | 何时用 |
|---|---|---|---|
| **v1 `RAW16`** | 2 B/样点 | — | 默认。便于 hexdump 目视调试与黄金向量 |
| **v2 `PACK12`** | 2 样点 → 3 B | 省 25% | `b0=s0低8位; b1=(s0>>8)&0x0F \| (s1&0x0F)<<4; b2=s1>>4`。**已实现（2026-10-06）**——设备编码、主机按分片头回显的 `format` 解码。UART 场景先上 |
| **v2 `MINMAX`** | 每桶 D 个原样点 → `(min,max)` 对 u16，共 4 B | **最大** | 显示用。4096 点 → 1600 B（5×）。示波器降采样的标准答案 |
| **v3 `DELTA`** | 64 样点块 = `{s0:u16, w:u4, 63 个 zigzag 增量按 w 位打包}` | 再省 1.5–2.5× | 仅当串口带宽仍不够。平滑信号 w≈6–9 位；噪声大时收益退化 |

`PACK12` 的 `count` 为奇数时的约定：**末样点与 `0` 凑成一对**，
payload = `12 + ceil(count/2) × 3` 字节；主机按 `count` 截断，解出的第 `count+1` 个值丢弃。

> ⚠ **实现状态（2026-10-10 实测）**：四种格式里**只有 `RAW16` 与 `PACK12` 实现了**。
> `MINMAX` 与 `DELTA` 在 `READ_BUFFER` 会被明确拒掉（`ERR_UNSUPPORTED`，`proto_task.c:309`），
> `SET_ACQ` 也**不校验** `format` 字段（存下而已）。设备侧降采样因此还不存在 ——
> 主机侧的 minmax 预览是**主机自己**在 `Capture::preview()` 里算的。
> `caps` 原先还声明 `CAP_FMT_MINMAX`（与上面的拒绝自相矛盾），
> **该位已于 `4e165e6` 摘掉**，见 §11.2 —— 但 MINMAX 格式本身仍未实现。

`MINMAX` 桶大小建议：USB `D≥16`（250 KB/s），921600 串口 `D≥64`（62.5 KB/s）。D 由主机按时基选择，用分片头 `decimation` 字段声明。

### 6.5 丢帧与重传（明确取舍）

**传输层不做 ACK/重传**（不在 MCU 上跑可靠协议状态机）。可靠性分层实现：

| 场景 | 策略 |
|---|---|
| **单次采集（拉模式）** | 整段冻结在 8 KB 环内 → 丢片 = 同偏移重拉，**数学上无损** |
| **流模式（推模式）** | 环只容 4096 点（≈4.8 ms），一片在 921600 下要约 22 ms → 缺口超出环容量**无法回补**。策略 = **接受缺口并用绝对样点号标注**（波形绘制断口，不做内插补齐） |
| 主机发现 SEQ / start_sample 跳变 | 缺口在环内（≤4096 点）→ `READ_BUFFER` 按偏移回补；超出范围 → 标缺口继续；连续缺口超阈值 → 自动 `STOP` + `ARM` 复位 |

**设备 TX 背压**：USB 主机不读 / 串口流控关闭导致 TX 队列满时，**丢片并 `tx_dropped++`，绝不阻塞采样循环**。

> ⚠ **实现状态（2026-10-10 实测）**：固件的 `GET_STATUS.tx_dropped` **恒为 0**
> （`proto_task.c:170-172`，注释也写明了原因：`link_write` 是**直写阻塞**的，
> 没有发送队列，所以"丢片"这个动作根本不存在）。也就是说：
> **它是"没被统计过"，不是"没有丢过"** —— 别拿它当链路健康的证据。
> 规则 5「设备端绝不能阻塞」在直写实现下也没有被真正满足（主循环会等 TX 完成）。

### 6.6 时序对齐

- 每片携带 `capture_id` + `start_sample` + 采集起点 `tick_us`
- 主机时间轴 = `start_sample / actual_rate_hz`
- 屏幕对齐用 `trigger_index`（触发点画在预置位置）
- 主机用 `PING` 维护设备时钟偏移：`offset = host_now − (t0 + RTT/2)`，连接时与周期性更新
  > ⚠ **实现状态（2026-10-10）**：**这条两端都没实现** —— 设备不回 `tick_us`（§5.1 状态块），
  > 主机的 `CommandBus::ping` 也只返回 `(回显内容, RTT)`、不计算偏移。
  > 当前所有时间信息都来自 `device_tick_us`（`EVENT_TRIGGER` 携带）与 `start_sample`，
  > 不是靠这个偏移。见文末 §11。
- **分片到达抖动不影响时间轴**（时间来自样点号，不是到达时刻）
- `tick_us` 是 1 MHz 32 位（约 71.6 分钟回绕，规格写明）

---

## 7. 错误码

| 码 | 名称 | 含义 |
|---|---|---|
| `0x0001` | `UNKNOWN_CMD` | 未定义的 CMD |
| `0x0002` | `BAD_LEN` | LEN 越界或不匹配 |
| `0x0003` | `BAD_PARAM` | 参数越界 |
| `0x0004` | `BUSY` | 当前状态不允许（先 STOP） |
| `0x0005` | `BAD_STATE` | 状态机不允许 |
| `0x0006` | `NO_DATA` | capture_id 不符 / 无数据 |
| `0x0007` | `OVERRUN` | 数据溢出 |
| `0x0008` | `TIMEOUT` | 未触发 / 设备侧超时 |
| `0x0009` | `VERSION_MISMATCH` | 主版本不符 |
| `0x000A` | `UNSUPPORTED` | 该功能此硬件不支持 |
| `0x000B` | `FLASH_ERR` | Flash 操作失败 |
| `0x000C` | `INTERNAL` | 内部错误 |

**编号一经发布永不复用。** 设备保存 `last_error` 供 `GET_LAST_ERROR` 复盘；主机侧集中映射为 Rust 枚举，UI/MCP 只用枚举不用裸码。

---

## 8. 超时与重试

| 命令类型 | 超时预算 |
|---|---|
| 控制命令 | 200 ms |
| 数据读取 | `50 ms + payload_bytes × 8 / baud × 1.5`（动态） |
| `PING` | 500 ms |

**重试策略**：

1. 幂等命令 → 直接重发
2. 非幂等（`ARM`）→ 靠设备端 `(seq, CMD, 响应)` 单槽缓存回放（1 s 窗口）；
   ⚠ **设备端尚未实现**，真机上重试会被重新执行（§11.1）
3. 超时后**不要立刻重发** → 先清空接收缓冲、重同步链路，再按原 seq 重发
4. 连续 N 次失败 → 上报链路故障并建议重连

**seq 回绕**：u16 自然回绕，**永远不要用 `<` / `>` 直接比较序号**。
用模差 `(uint16_t)(got - expected)` 并限制在途请求窗口（≤8）使回绕无歧义。
重连后双方序号重置。

---

## 9. 黄金测试向量

`proto/tests/vectors.json` 是**跨语言契约**。每个向量包含：

```json
{
  "name": "set_sample_rate_857143",
  "flags": 0,
  "seq": 5,
  "cmd": 513,
  "payload_hex": "37140d00",
  "frame_hex": "aa55100005000102040037140d0073c4",
  "frame_len": 16,
  "note": "F103 默认档；设备须回显 actual_hz"
}
```

逐字段对一遍（这是自查帧格式最快的方法）：

```
aa 55           SYNC
10              VER     主 1 次 0
00              FLAGS   请求帧
05 00           SEQ     5（小端）
01 02           CMD     0x0201 = SET_SAMPLE_RATE（小端）
04 00           LEN     4 字节 payload
37 14 0d 00     PAYLOAD 0x000d1437 = 857143（小端）
73 c4           CRC16   CCITT-FALSE，覆盖 VER..PAYLOAD 末尾
```

> 上面这个例子是 `proto/tests/vectors.json` 里的**真实向量**，可以直接复制去跑。
> 自己编的例子很容易写错（比如把 `857143` 的小端写成 `07 14 0d 00`），
> **要举例就从 vectors.json 里抄**。

**两端必须产出逐字节相同的 `frame_hex`。**

必覆盖的用例：

| 类别 | 用例 |
|---|---|
| 帧编解码 | 最短帧（LEN=0）、最长帧（LEN=2060，真实满载分片）、各 FLAGS 组合 |
| 粘包/拆包 | 一次喂 3.5 帧、半个帧、`AA 55` 出现在 payload 内部 |
| 重同步 | 垃圾字节前缀、CRC 错帧后紧跟正确帧 |
| 边界值 | `LEN = 0 / 1 / 512 / 2060`（合法边界，见上方 `frames` 表）；**越界**用解析器的重同步用例 `oversized_len_resync`（`LEN = 4096`）—— 向量表里**没有 2061 这一条** |
| 数值编码 | i16 负数、u32 最大值、定点缩放字段 |
| 帧解析 | 未知 CMD 按 LEN 吞帧 |
| 触发语义 | 状态机各状态下的命令合法性矩阵 |

---

## 10. 版本演进

- 编号与升降级规则见 §5.8：新增命令升**次版本**，字段变更走**新 CMD 号** + 升**主版本**
- 设备对主版本不符**拒绝非基础命令但仍答 `GET_INFO` / `PING`**
- 每次协议变更必须同时更新四处（见文首），并由两端跑同一份向量做门禁
- **CI 门禁**：C 原生向量测试 + `cargo test` + 模拟器回归，三件套全绿才允许合并

---

## 11. 实现状态（2026-10-10 实测）

> 这一节记录**契约与实现的差距**。CI 的黄金向量保证的是「两端编解码一致」，
> **不保证「设备实现了这条命令」** —— 本清单补的就是那个缺口。
> 每条都注明出处行号，改代码或改契约时请同步改这里。
>
> ⚠ 行号是**写作时**的快照，已发生过漂移（例如 `4e165e6` 在 `proto_task.c` 顶部
> 加了 8 行注释，此后该文件的行号整体 +8）。**引用时请按函数名检索，不要照抄行号。**

### 11.1 契约里写着、设备侧没做的

| 条目 | 契约位置 | 设备侧实测 |
|---|---|---|
| 非幂等命令的单槽响应缓存（同 seq 1 s 内回放） | §4 | ❌ **未实现**（`proto_task_t` 里没有；目前只有模拟器 `host/crates/sim` 有） |
| `PING` 附带 `tick_us` | §5.1 | ❌ 设备不回；**主机也不解析**（`command.rs:463-469`） |
| `RESET` 后软复位 ~50 ms | §5.1 | ❌ 只做 `acq_stop` + `acq_init`（配置回出厂），**不重启 MCU** |
| 上行 512 B 上限的强制 | §2 | ❌ 未强制：解析器按 2060 判越界，而接收缓冲只有 524 |
| `FORCE_TRIGGER` | §5.3 | ❌ 恒回 `ERR_UNSUPPORTED` |
| 流模式 `STREAMING` | §5.3 | ❌ 未实现（`ACQ_MODE_STREAM` 只被存进配置） |
| `MEASURE` | §5.5 | ❌ 恒回 `ERR_UNSUPPORTED`；主机侧改走本地 `scope_core::measure()` |
| `MINMAX` / `DELTA` 波形格式 | §6.4 | ❌ 未实现（`READ_BUFFER` 明确拒绝） |
| `SET_LOG_LEVEL` · `GET_LAST_ERROR` | §5.6 | ❌ 只回空 `Ack`（没有等级、没有错误载荷） |
| `EVENT_LOG` | §5.6 | ❌ 从不发出 |
| `MEM_READ` / `MEM_WRITE` | §5.6 | ❌ 未进 `handle_frame`，落 default 回 `UNKNOWN_CMD` |
| TX 背压丢片计数 `tx_dropped` | §6.5 | ❌ 恒 0（`link_write` 直写阻塞，没有发送队列） |
| 用 `PING` 维护设备时钟偏移 | §6.6 | ❌ **两端都没实现** |

### 11.2 能力位过度声明（`GET_INFO.caps`）—— ✅ 已修（`4e165e6`）

| 位 | 修复前声明 | 实际 | 状态 |
|---|---|---|---|
| `CAP_TRIG_SOFT` | 支持软触发 | `FORCE_TRIGGER` 恒 `ERR_UNSUPPORTED`（`cmd_force_trigger`，`proto_task.c:282-287`） | ✅ 已从 `DEV_CAPS` 摘掉 |
| `CAP_FMT_MINMAX` | 支持 MINMAX 格式 | `READ_BUFFER` 对 MINMAX 回 `ERR_UNSUPPORTED`（`proto_task.c:309`） | ✅ 已从 `DEV_CAPS` 摘掉 |

> 两处都违反了 `proto_task.c` 自己立下的规矩（「不要在这里声明做不到的能力」，
> 那里针对的是 `CAP_CH_DUAL` / `CAP_COUPLING_AC`）。
> `caps` 是**给 Agent 看的**：主机侧没有任何代码按它分支决策
> （`command.rs:78` 只解析、`cli/main.rs:512` 只打印），
> 所以声明了做不到的能力 = 让 Agent 规划出必然失败的动作。
>
> **2026-10-10 已按「摘掉」修复**：`DEV_CAPS` 现为
> `CAP_TRIG_AUTO | CAP_TRIG_NORMAL | CAP_TRIG_SINGLE | CAP_FMT_PACK12`。
> `test_proto_task.c` 的 GET_INFO 组补了三条守卫（两条负向 + `CAP_FMT_PACK12` 正向），
> 已验证：把 `CAP_TRIG_SOFT` 加回去会让该组以
> `FAIL [GET_INFO 往返] 软触发未实现…不该声明` 失败（固件断言 249 → 252）。
> 若将来真的实现软触发 / MINMAX 打包，请同步改回本节与那三条断言。
>
> ✅ **已对齐（G0，2026-10-24）**：主机侧模拟器默认 `CapsMode::RealDevice`，
> `GET_INFO.caps` 与真机 `DEV_CAPS` 逐位一致；`CapsMode::IdealSim` 保留完整模拟能力，
> 仅供模拟器专项测试。见 §11.4。

### 11.3 契约没写清、而实现已经依赖的

- **`GET_CONFIG` 的 17 字节布局**：协议里没有 `config_resp_t` 结构体，设备是手拼的，
  主机按同样偏移读。布局已补进 §5.2；**它不进 `protocol.h`、也没有 `static_assert` 保护**，
  改字段时两边都要手改。
- **`READ_BUFFER` 的越界语义**：`start_sample` 越界回**空分片 + `LAST`**（不是错误），
  `end` 超出窗口则截断；"对未冻结的采集区"回的是 `NO_DATA` 而**不是 `BUSY`**。
- **`GET_STATUS.err_flags` 的位定义**：契约页没定义，实现只用了 bit0（本次采集发生过溢出，
  `proto_task.c:157`）。**建议在 §5.1 补上位定义**。
- **`ECHO` 的 512 B 上限**：靠接收缓冲隐式限制，不是显式校验。

### 11.4 模拟器与真机的 `caps` 不一致 —— ✅ 已修（G0，2026-10-24）

原问题：`host/crates/sim/src/device.rs` 的 `GET_INFO.caps` 比真机**多声明 5 项**
（`CAP_TRIG_SOFT` / `CAP_FMT_MINMAX` / `CAP_COUPLING_AC` / `CAP_DIGITAL_I2C` /
I2C 场景下的 `CAP_CH_DUAL`）。模拟器自述是「复刻真实行为」，但它的 `caps`
描述的是**它自己会做什么**，不是**真机做得到什么** —— 对着模拟器开发 Agent 时，
会照着这套 `caps` 规划出真机必然拒绝的动作。

修法：`SimDevice` 增加 `CapsMode`，**默认 `RealDevice`**：

| 模式 | `caps` | 用途 |
|---|---|---|
| `RealDevice`（默认） | `CAP_TRIG_AUTO \| CAP_TRIG_NORMAL \| CAP_TRIG_SINGLE \| CAP_FMT_PACK12`，与真机逐位一致 | 对着模拟器开发真机 Agent |
| `IdealSim` | 上述四位 + 软触发 / MINMAX / AC 耦合 / 数字 I2C；双通道场景另加 `CAP_CH_DUAL` | 模拟器专项测试与演示 |

- **行为不变**：模拟器仍实现软触发与 MINMAX，只是默认不再**声明**它们。
  `sim/src/lib.rs:244` 的 `force_trigger()` 与 `:368` 的 `ch_count == 2` 两个测试
  继续有效（它们验证行为，不验证 caps）。
- 新增两条守卫：`default_caps_match_the_real_device` 与
  `ideal_caps_keep_the_simulator_extra_bits`。
- **I2C 场景仍是双通道**：`CAP_CH_DUAL` 描述的是真板能力，不是模拟场景的通道数；
  I2C 解码靠实际通道数，不靠 caps 分支。
- 注：主机侧没有任何代码按 `caps` 分支决策（见 §11.2），
  所以这是**面向 Agent 的信息真实性**问题，不是主机功能问题。

### 11.5 主机侧（host）的对应缺口

- `PING` 的 `tick_us` 不解析（见 §5.1 / §6.6）。
- ~~MCP 的 `scope_measure` 只出口 **AC-RMS**（`ac_rms_v`，`session.rs:879`）；
  `Measurements` 里的真 RMS **没有任何出口**（不存在 `rms_v` 字段）。~~
  **✅ 已修（G1，2026-10-24）**：`scope_measure` 同时返回 `ac_rms_v` 与 `rms_v`。
  文案若承诺"RMS"，必须指明是哪一个 —— 两者对带直流偏置的信号差别很大。

> **契约与实现的关系**：以上都是"实现落后于契约"，**没有一条是契约写错了**。
> 所以这一轮修的是文档（把差距写明），不是把契约改软。

---

## 附：为什么不用 postcard / protobuf / CBOR

| 方案 | 否决理由 |
|---|---|
| **postcard** | 核心价值是「同一份 Rust 类型编译到 PC 与 MCU，编译期防 drift」。但固件用 C → C 侧必须手写 LEB128/zigzag/长度前缀，postcard 无官方 C 实现也无代码生成，**收益归零**；且变长编码使字段偏移不可预测，C 侧边界检查、黄金向量编写、hexdump 目视调试都更难 —— 净亏。**仅当固件也改 Rust 时才成立。** |
| **nanopb + prost** | 唯一真正适配 20 KB 的 protobuf 方案，有 C 生成器与稳定 schema 演进。代价是 protoc + nanopb 工具链、回调式解码、约 2–4 KB Flash/RAM。约 30 条消息内不值得；消息数 > 50 或出现第二台设备时再迁移。 |
| **CBOR** | 自描述的标签特性两端都用不上，体积 +10~30%，MCU 解码器成本高。 |

上位的「命令只定义一次」原则**在上位机 + MCP 侧完整保留**（Rust 类型 → serde/schemars → MCP JSON Schema），只是线上 codec 换成手写定长。将来固件若迁移 Rust，只需把 codec 换回 postcard，其余不动。

这条在 MCP 侧是**字面落实**的：工具参数类型定义在 `host/crates/mcp/src/params.rs`，
同一个类型既是 `inputSchema` 的来源、又是 `arguments` 的解析目标。手写 schema
字符串与实现之间没有任何机制保证一致，而它的失效方式很隐蔽 —— 字段声明了
却没人读，传了不报错、静默走默认值。`main.rs` 有一条测试两头夹住这件事。
