/* App/proto_task.c —— 命令分发的实现。设计说明见 proto_task.h。 */

#include "proto_task.h"

#include <string.h>

/* ── 设备标识 ─────────────────────────────────────────────────────
 *
 * `uid` 是标定表的键（见 docs/03-protocol.md：量程衰减、AC/DC 校正
 * 全部留在上位机，按 uid 存标定表）。真机上应当从 MCU 的唯一 ID
 * 寄存器读（`0x1FFFF7E8`，96 位）；这里填的是占位值 ——
 * 硬件访问属于 `Hardware/`，而这半边还没有。 */
static const uint8_t DEV_UID[12] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

#define DEV_MODEL 0x0103u      /* F103C8T6 立创底板 */
#define FW_VER    0x00010000u /* 1.0.0 */

/* 本板能力位。
 *
 * ⚠ **不要在这里声明 `CAP_CH_DUAL`。** 底板只有一路模拟输入
 * （见 docs/02-hardware.md §5），真双通道要等 P4 改板（ADR-010）。
 * 声明了它，主机就会去拉一个不存在的通道。
 *
 * `CAP_COUPLING_AC` 同理：耦合是手拨开关 SW2，AI 无法程控 ——
 * 说支持 AC 耦合会让 Agent 以为能切。
 *
 * `CAP_TRIG_SOFT` 与 `CAP_FMT_MINMAX` 同样不许声明，理由和上面一样：
 *   - 软触发没有实现 —— `cmd_force_trigger()` 恒回 `ERR_UNSUPPORTED`；
 *   - MINMAX 打包没有实现 —— `cmd_read_buffer()` 对 MINMAX 回
 *     `ERR_UNSUPPORTED`（只有 RAW16 / PACK12 有编码分支）。
 * 主机与 Agent 会照 `caps` 规划动作，声明了做不到的能力 = 让 Agent
 * 去调一个必然失败的命令。真要声明，请先把上面两处实现补上，
 * 并同步 `firmware/tests/test_proto_task.c` 的负向断言。 */
#define DEV_CAPS (CAP_TRIG_AUTO | CAP_TRIG_NORMAL | CAP_TRIG_SINGLE | \
                  CAP_FMT_PACK12)

/* 本板采集窗里只有 1 个通道（模拟通路）。 */
#define DEV_CH_COUNT 1u

/* ── 内部：发送 ─────────────────────────────────────────────────── */

static void send_frame(proto_task_t *pt, uint8_t flags, uint16_t seq, uint16_t cmd,
                       const uint8_t *payload, uint16_t len)
{
    size_t out_len = 0;
    if (proto_encode(pt->tx, sizeof(pt->tx), flags, seq, cmd, payload, len, &out_len)
        != PROTO_OK) {
        /* 编不出来说明载荷超了 tx 缓冲 —— 那是代码 bug，不是线路问题。
         * 回一个 INTERNAL 让主机知道这次没成，而不是静默丢帧。 */
        size_t elen = 0;
        if (proto_encode_error(pt->tx, sizeof(pt->tx), seq, cmd, ERR_INTERNAL,
                               ERR_SEV_ERR, "encode failed", 14, &elen) == PROTO_OK) {
            pt->hal->link_write(pt->tx, (uint32_t)elen);
            pt->frames_tx++;
        }
        return;
    }
    pt->hal->link_write(pt->tx, (uint32_t)out_len);
    pt->frames_tx++;
}

static void send_error(proto_task_t *pt, uint16_t seq, uint16_t cmd, uint16_t code,
                       const char *msg)
{
    size_t len = 0;
    size_t n = msg ? strlen(msg) : 0u;
    if (n > 32u) {
        n = 32u;
    }
    if (proto_encode_error(pt->tx, sizeof(pt->tx), seq, cmd, code, ERR_SEV_ERR, msg,
                           (uint8_t)n, &len) == PROTO_OK) {
        pt->hal->link_write(pt->tx, (uint32_t)len);
        pt->frames_tx++;
    }
    pt->last_error_code = code;
}

/* 一个空载荷的确认（`Ack`）。 */
static void send_ack(proto_task_t *pt, uint16_t seq, uint16_t cmd)
{
    send_frame(pt, PROTO_FLAG_RESP, seq, cmd, NULL, 0u);
}

/* ── 内部：配置快照（GET_CONFIG 的 17 字节）──────────────────────
 *
 * ⚠ 协议里**没有** `config_resp_t` 这个结构体 —— GET_CONFIG 的 payload
 * 是手拼的定长 17 字节。主机侧（host/crates/core/src/command.rs 的
 * `DeviceConfig` 解析）按同样的偏移读，两边都不许改。 */
static void send_config(proto_task_t *pt, uint16_t seq)
{
    const acq_t *a = pt->acq;
    uint8_t p[17];
    memset(p, 0, sizeof(p));
    /*  0  rate_hz            u32 */
    p[0] = (uint8_t)(a->rate_hz & 0xFFu);
    p[1] = (uint8_t)((a->rate_hz >> 8) & 0xFFu);
    p[2] = (uint8_t)((a->rate_hz >> 16) & 0xFFu);
    p[3] = (uint8_t)((a->rate_hz >> 24) & 0xFFu);
    /*  4  acq_mode           u8  */
    p[4] = a->acq_mode;
    /*  5  capture_samples    u16 */
    p[5] = (uint8_t)(a->capture_samples & 0xFFu);
    p[6] = (uint8_t)((a->capture_samples >> 8) & 0xFFu);
    /*  7  format             u8  */
    p[7] = a->format;
    /*  8  decimation         u16 */
    p[8] = (uint8_t)(a->decimation & 0xFFu);
    p[9] = (uint8_t)((a->decimation >> 8) & 0xFFu);
    /* 10  trigger_mode       u8  */
    p[10] = a->trig_cfg.mode;
    /* 11  trigger_source     u8  */
    p[11] = a->trig_cfg.source;
    /* 12  trigger_edge       u8  */
    p[12] = a->trig_cfg.edge;
    /* 13  trigger_level_lsb  u16 */
    p[13] = (uint8_t)(a->trig_cfg.level_lsb & 0xFFu);
    p[14] = (uint8_t)((a->trig_cfg.level_lsb >> 8) & 0xFFu);
    /* 15  ch0_enable         u8  */
    p[15] = 1u;
    /* 16  ch0_coupling       u8  */
    p[16] = COUPLING_DC;
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_GET_CONFIG, p, sizeof(p));
}

/* ── 各命令的处理 ───────────────────────────────────────────────── */

static void cmd_get_info(proto_task_t *pt, uint16_t seq)
{
    info_resp_t r;
    memset(&r, 0, sizeof(r));
    r.proto_ver = PROTO_VER;
    r.fw_ver = FW_VER;
    r.model = DEV_MODEL;
    memcpy(r.uid, DEV_UID, sizeof(r.uid));
    r.tick_hz = PROTO_TICK_HZ;
    r.adc_bits = 12u;
    r.ch_count = (uint8_t)DEV_CH_COUNT;
    r.rate_min_hz = ACQ_RATE_MIN_HZ;
    r.rate_max_hz = ACQ_RATE_MAX_HZ;
    r.capture_max_samples = PROTO_CAPTURE_MAX_SAMPLES;
    r.max_rx_payload = PROTO_MAX_PAYLOAD_RX;
    r.max_tx_payload = PROTO_MAX_PAYLOAD_TX;
    r.preferred_chunk_samples = PROTO_PREFERRED_CHUNK;
    r.caps = DEV_CAPS;
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_GET_INFO, (const uint8_t *)&r,
               (uint16_t)sizeof(r));
}

static void cmd_get_status(proto_task_t *pt, uint16_t seq)
{
    const acq_t *a = pt->acq;
    status_resp_t r;
    memset(&r, 0, sizeof(r));
    r.state = (uint8_t)a->state;
    /* bit0 = 这次采集发生过溢出 */
    r.err_flags = a->overrun ? 1u : 0u;
    r.ring_fill_samples = (uint16_t)(a->capture_ready ? a->window_len : 0u);
    r.last_capture_id = a->capture_ready ? a->capture_id : 0u;
    r.last_trigger_index = (a->capture_ready && a->trigger_at != ACQ_NO_TRIGGER)
                               ? a->trigger_index_in_window
                               : ACQ_NO_TRIGGER;
    r.overrun_samples = a->overrun_samples;
    r.rx_crc_err = (uint16_t)(pt->parser.crc_err_count > 0xFFFFu
                                  ? 0xFFFFu
                                  : pt->parser.crc_err_count);
    r.rx_dropped = (uint16_t)(pt->parser.dropped_count > 0xFFFFu
                                  ? 0xFFFFu
                                  : pt->parser.dropped_count);
    /* tx_dropped：本实现直写链路，没有发送队列，所以恒为 0。
     * **不要**拿它当「链路一定健康」的证据 —— 它只是没被统计过。 */
    r.tx_dropped = 0u;
    r.uptime_ms = pt->hal->now_ms();
    r.tick_us = pt->hal->tick_us();
    r.last_error_code = pt->last_error_code;
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_GET_STATUS, (const uint8_t *)&r,
               (uint16_t)sizeof(r));
}

static void cmd_set_sample_rate(proto_task_t *pt, uint16_t seq, const proto_frame_t *f)
{
    if (f->hdr.len < sizeof(set_rate_req_t)) {
        send_error(pt, seq, CMD_SET_SAMPLE_RATE, ERR_BAD_LEN, "need 4 bytes");
        return;
    }
    set_rate_req_t req;
    memcpy(&req, f->payload, sizeof(req));

    uint32_t actual = 0;
    acq_status_t st = acq_set_rate(pt->acq, req.requested_hz, &actual);
    if (st != ACQ_OK) {
        send_error(pt, seq, CMD_SET_SAMPLE_RATE,
                   st == ACQ_ERR_BUSY ? ERR_BUSY : ERR_BAD_PARAM,
                   st == ACQ_ERR_BUSY ? "armed" : "unreachable rate");
        return;
    }
    set_rate_resp_t r = {.actual_hz = actual};
    /* **必须回显实际值** —— 定时器只能出整数分频的档位，
     * 主机的时间轴一律以这个值为准（协议纪律）。 */
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_SET_SAMPLE_RATE, (const uint8_t *)&r,
               (uint16_t)sizeof(r));
}

static void cmd_set_trigger(proto_task_t *pt, uint16_t seq, const proto_frame_t *f)
{
    if (f->hdr.len < sizeof(set_trigger_req_t)) {
        send_error(pt, seq, CMD_SET_TRIGGER, ERR_BAD_LEN, "need 11 bytes");
        return;
    }
    set_trigger_req_t req;
    memcpy(&req, f->payload, sizeof(req));
    acq_status_t st = acq_set_trigger(pt->acq, &req);
    if (st != ACQ_OK) {
        send_error(pt, seq, CMD_SET_TRIGGER,
                   st == ACQ_ERR_BUSY ? ERR_BUSY : ERR_BAD_PARAM,
                   st == ACQ_ERR_BUSY ? "armed" : "bad trigger");
        return;
    }
    /* 协议规定 SET_* 回显生效值 —— 原样回 payload 即可，因为
     * 上面已经把非法值挡在外面了（回显的就是真用的那套）。 */
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_SET_TRIGGER, f->payload, f->hdr.len);
}

static void cmd_set_acq(proto_task_t *pt, uint16_t seq, const proto_frame_t *f)
{
    if (f->hdr.len < sizeof(set_acq_req_t)) {
        send_error(pt, seq, CMD_SET_ACQ, ERR_BAD_LEN, "need 6 bytes");
        return;
    }
    set_acq_req_t req;
    memcpy(&req, f->payload, sizeof(req));
    acq_status_t st = acq_set_acq(pt->acq, &req);
    if (st != ACQ_OK) {
        send_error(pt, seq, CMD_SET_ACQ,
                   st == ACQ_ERR_BUSY ? ERR_BUSY : ERR_BAD_PARAM,
                   st == ACQ_ERR_BUSY ? "armed" : "bad acq");
        return;
    }
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_SET_ACQ, f->payload, f->hdr.len);
}

static void cmd_set_channel(proto_task_t *pt, uint16_t seq, const proto_frame_t *f)
{
    if (f->hdr.len < sizeof(set_channel_req_t)) {
        send_error(pt, seq, CMD_SET_CHANNEL, ERR_BAD_LEN, "need 6 bytes");
        return;
    }
    set_channel_req_t req;
    memcpy(&req, f->payload, sizeof(req));
    if (req.ch >= DEV_CH_COUNT) {
        /* **本板只有一路模拟通道。** 主机若按「双通道」去配 CH2，
         * 这里必须明确拒绝 —— 收下并回显会让它以为配成功了。 */
        send_error(pt, seq, CMD_SET_CHANNEL, ERR_BAD_PARAM, "ch not present");
        return;
    }
    acq_status_t st = acq_set_channel(pt->acq, &req);
    if (st != ACQ_OK) {
        send_error(pt, seq, CMD_SET_CHANNEL,
                   st == ACQ_ERR_BUSY ? ERR_BUSY : ERR_BAD_PARAM, "channel");
        return;
    }
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_SET_CHANNEL, f->payload, f->hdr.len);
}

static void cmd_arm(proto_task_t *pt, uint16_t seq)
{
    acq_status_t st = acq_arm(pt->acq);
    if (st != ACQ_OK) {
        send_error(pt, seq, CMD_ARM, ERR_BAD_STATE, "cannot arm now");
        return;
    }
    send_ack(pt, seq, CMD_ARM);
}

static void cmd_stop(proto_task_t *pt, uint16_t seq)
{
    /* 协议规定：**已停也回 Ack，不报错**。 */
    acq_stop(pt->acq);
    send_ack(pt, seq, CMD_STOP);
}

static void cmd_force_trigger(proto_task_t *pt, uint16_t seq)
{
    /* 只应在 ARMED / STREAMING 下来 —— `proto_cmd_allowed` 已经挡了其余状态。 */
    send_error(pt, seq, CMD_FORCE_TRIGGER, ERR_UNSUPPORTED,
               "soft trigger not wired yet");
}

static void cmd_read_buffer(proto_task_t *pt, uint16_t seq, const proto_frame_t *f)
{
    if (f->hdr.len < sizeof(read_buffer_req_t)) {
        send_error(pt, seq, CMD_READ_BUFFER, ERR_BAD_LEN, "need 10 bytes");
        return;
    }
    read_buffer_req_t req;
    memcpy(&req, f->payload, sizeof(req));

    const acq_t *a = pt->acq;
    if (!a->capture_ready || req.capture_id != a->capture_id) {
        /* id 对不上 = 那份数据已经不在了（被新的采集覆盖）。
         * **不能**拿当前的采集去顶 —— 主机会把两次采集拼在一起。 */
        send_error(pt, seq, CMD_READ_BUFFER, ERR_NO_DATA, "capture id mismatch");
        return;
    }
    if (req.format != FMT_RAW16 && req.format != FMT_PACK12) {
        /* MINMAX / DELTA 的打包还没做。明确说「不支持」，
         * 而不是按 RAW16 回 —— 主机把 3 字节当 2 字节解，整段数据全错。
         * （PACK12 已实现，见下面的编码分支。） */
        send_error(pt, seq, CMD_READ_BUFFER, ERR_UNSUPPORTED, "format not supported");
        return;
    }

    uint32_t start = req.start_sample;
    if (start >= a->window_len) {
        /* 越界 start 回一个空分片（带 LAST），而不是错误 ——
         * 主机的分页循环靠这个自然收尾（与 host 侧的行为一致）。 */
        chunk_header_t h = {.capture_id = req.capture_id,
                            .start_sample = start,
                            .count = 0,
                            .decimation = a->decimation,
                            /* 如实回显请求的格式 —— 主机按它解码 */
                            .format = req.format,
                            .flags = CHUNK_FLAG_LAST};
        send_frame(pt, PROTO_FLAG_RESP, seq, CMD_READ_BUFFER, (const uint8_t *)&h,
                   (uint16_t)sizeof(h));
        return;
    }

    uint32_t want = req.count;
    if (want == 0 || want > PROTO_CHUNK_SAMPLES_MAX) {
        send_error(pt, seq, CMD_READ_BUFFER, ERR_BAD_PARAM, "count out of range");
        return;
    }
    uint32_t end = start + want;
    if (end > a->window_len) {
        end = a->window_len;
    }
    uint32_t n = end - start;
    if (n > PROTO_MAX_PAYLOAD_TX / 2u - PROTO_CHUNK_HEADER_LEN / 2u) {
        n = PROTO_CHUNK_SAMPLES_MAX;
    }

    /* 分片头 + 样点，一次编码。**分片头算在 payload 之内** ——
     * 这一点很容易算漏（见 protocol.h 的 PROTO_CHUNK_SAMPLES_MAX 注释）。
     *
     * ⚠ **必须保持 static**。这块 2060 B 若放在栈上，就是 `Stack_Size`
     *   （`startup_stm32f103xb.s`，0x400 = 1024 B）的两倍。栈向下生长，
     *   越界的那 1036 B 会吃掉整个堆（512 B）与 C 库的 libspace，
     *   再改写 `protocol.o` 的静态缓冲 —— 全程**没有任何征兆**。
     *   改成 `static` 之后，`App/` 里最大的栈帧是 `proto_task_poll` 的 240 B。
     * ⚠ `firmware/run_tests.sh` 看不见这一类：宿主的栈有好几 MB。 */
    static uint8_t buf[PROTO_CHUNK_HEADER_LEN + PROTO_CHUNK_SAMPLES_MAX * 2u];
    chunk_header_t h = {.capture_id = req.capture_id,
                        .start_sample = start,
                        .count = (uint16_t)n,
                        .decimation = a->decimation,
                        /* 如实回显请求的格式 —— 主机按它解码（不是按它请求的值） */
                        .format = req.format,
                        /* 溢出的采集必须**如实标为无效** ——
                         * 主机靠这个标志判断这份数据能不能用。 */
                        .flags = (uint8_t)(((end >= a->window_len) ? CHUNK_FLAG_LAST : 0u) |
                                           (a->overrun ? CHUNK_FLAG_INVALID : 0u))};
    memcpy(buf, &h, sizeof(h));
    if (req.format == FMT_PACK12) {
        /* 2 样点 → 3 字节。n 为奇数时末样点与 0 凑对 —— 主机按 count
         * 截断，解出的第 count+1 个值忽略（见 docs/03-protocol.md 6.4）。
         * payload 比 RAW16 小（n×1.5 B），同一块 static buf 装得下。 */
        uint32_t pairs = (n + 1u) / 2u;
        for (uint32_t i = 0; i < pairs; i++) {
            uint32_t j = i * 2u;
            uint16_t s0 = acq_sample(a, 0, start + j);
            uint16_t s1 = (j + 1u < n) ? acq_sample(a, 0, start + j + 1u) : 0u;
            proto_pack12_pair(s0, s1, &buf[PROTO_CHUNK_HEADER_LEN + i * 3u]);
        }
        send_frame(pt, PROTO_FLAG_RESP, seq, CMD_READ_BUFFER, buf,
                   (uint16_t)(PROTO_CHUNK_HEADER_LEN + pairs * 3u));
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint16_t v = acq_sample(a, 0, start + i);
        buf[PROTO_CHUNK_HEADER_LEN + i * 2u] = (uint8_t)(v & 0xFFu);
        buf[PROTO_CHUNK_HEADER_LEN + i * 2u + 1u] = (uint8_t)((v >> 8) & 0xFFu);
    }
    send_frame(pt, PROTO_FLAG_RESP, seq, CMD_READ_BUFFER, buf,
               (uint16_t)(PROTO_CHUNK_HEADER_LEN + n * 2u));
}

static void cmd_reset(proto_task_t *pt, uint16_t seq, const proto_frame_t *f)
{
    if (f->hdr.len < sizeof(reset_req_t) || f->payload[0] != 0xA5u) {
        send_error(pt, seq, CMD_RESET, ERR_BAD_PARAM, "magic must be 0xA5");
        return;
    }
    acq_stop(pt->acq);
    /* 配置回出厂值。**用重新初始化而不是逐字段清零** —— 后者漏一个字段
     * 就会留下一个「上一台设备的配置」，而那种 bug 极难查。 */
    acq_init(pt->acq, pt->hal);
    send_ack(pt, seq, CMD_RESET);
}

static void cmd_measure(proto_task_t *pt, uint16_t seq)
{
    /* 定点测量（`App/measure.c`）还没实现。明确回 UNSUPPORTED ——
     * 回一个全 0 的 measure_resp_t 会让主机以为「测得是 0」，
     * 而那是完全不同的两件事。 */
    send_error(pt, seq, CMD_MEASURE, ERR_UNSUPPORTED, "measure not implemented");
}

/* ── 分发 ───────────────────────────────────────────────────────── */

static void handle_frame(proto_task_t *pt, const proto_frame_t *f)
{
    const uint16_t seq = f->hdr.seq;
    const uint16_t cmd = f->hdr.cmd;
    pt->frames_rx++;

    /* 免回复：连**错误**都不能回 —— 回一个错误应答同样是应答，
     * 会让主机的序号匹配错位。 */
    if (f->hdr.flags & PROTO_FLAG_NO_REPLY) {
        return;
    }

    /* 主版本不符：GET_INFO / PING 仍必须应答（主机靠它们确认对面是谁），
     * 其余回 VERSION_MISMATCH。 */
    if (f->hdr.ver != PROTO_VER) {
        if (cmd != CMD_GET_INFO && cmd != CMD_PING) {
            send_error(pt, seq, cmd, ERR_VERSION_MISMATCH, "ver mismatch");
            return;
        }
    }

    /* 状态机门禁 —— 与 `proto_cmd_allowed` 是同一张表的两次读取：
     * 那边是契约（两端跑同一份黄金向量），这里是执行。 */
    if (!proto_cmd_allowed(acq_state(pt->acq), cmd)) {
        send_error(pt, seq, cmd, ERR_BUSY, "busy: stop first");
        return;
    }

    switch (cmd) {
    case CMD_GET_INFO:
        cmd_get_info(pt, seq);
        break;
    case CMD_PING:
        /* 原样回显 payload —— 主机用它测往返与时钟差 */
        send_frame(pt, PROTO_FLAG_RESP, seq, CMD_PING, f->payload, f->hdr.len);
        break;
    case CMD_GET_STATUS:
        cmd_get_status(pt, seq);
        break;
    case CMD_RESET:
        cmd_reset(pt, seq, f);
        break;
    case CMD_GET_CONFIG:
        send_config(pt, seq);
        break;
    case CMD_SET_SAMPLE_RATE:
        cmd_set_sample_rate(pt, seq, f);
        break;
    case CMD_SET_CHANNEL:
        cmd_set_channel(pt, seq, f);
        break;
    case CMD_SET_TRIGGER:
        cmd_set_trigger(pt, seq, f);
        break;
    case CMD_SET_ACQ:
        cmd_set_acq(pt, seq, f);
        break;
    case CMD_ARM:
        cmd_arm(pt, seq);
        break;
    case CMD_STOP:
        cmd_stop(pt, seq);
        break;
    case CMD_FORCE_TRIGGER:
        cmd_force_trigger(pt, seq);
        break;
    case CMD_READ_BUFFER:
        cmd_read_buffer(pt, seq, f);
        break;
    case CMD_MEASURE:
        cmd_measure(pt, seq);
        break;
    case CMD_ECHO:
        send_frame(pt, PROTO_FLAG_RESP, seq, CMD_ECHO, f->payload, f->hdr.len);
        break;
    case CMD_SET_LOG_LEVEL:
    case CMD_GET_LAST_ERROR:
        send_ack(pt, seq, cmd);
        break;
    default:
        /* 未知命令码：**按 LEN 吞掉整帧**（解析器已经做了），这里回错误。
         * 不回的话，主机会一直等一个永远不会来的应答。 */
        send_error(pt, seq, cmd, ERR_UNKNOWN_CMD, "unknown cmd");
        break;
    }
}

/* ── 对外 ───────────────────────────────────────────────────────── */

void proto_task_init(proto_task_t *pt, const hal_t *hal, acq_t *acq)
{
    memset(pt, 0, sizeof(*pt));
    pt->hal = hal;
    pt->acq = acq;
    pt->event_seq = 1;
    proto_parser_init(&pt->parser);
}

void proto_task_poll(proto_task_t *pt)
{
    uint32_t n = pt->hal->link_read(pt->rx, (uint32_t)sizeof(pt->rx));
    if (n == 0u) {
        return;
    }

    /* 喂进去之后要**循环取帧** —— 一次读进来可能装了好几个。
     * 而且每一帧必须当场处理完：`f.payload` 指向解析器内部缓冲，
     * 下次 feed 就失效了。 */
    const uint8_t *data = pt->rx;
    size_t len = n;
    for (;;) {
        proto_frame_t f;
        proto_status_t st = proto_parser_feed(&pt->parser, data, len, &f);
        /* 只有第一次要把新字节交进去，后续是在排空解析器里攒下的帧 */
        data = NULL;
        len = 0;

        if (st == PROTO_OK || st == PROTO_BAD_VER) {
            handle_frame(pt, &f);
            continue;
        }
        /* CRC_ERR / BAD_LEN：解析器已经跳字节重同步并计了数，
         * 这里接着喂空数据把剩下的帧排空即可。 */
        if (st == PROTO_CRC_ERR || st == PROTO_BAD_LEN) {
            continue;
        }
        break; /* NEED_MORE */
    }
}

void proto_task_emit(proto_task_t *pt, const acq_out_t *ev)
{
    if (ev->kind == ACQ_EV_TRIGGERED) {
        send_frame(pt, PROTO_FLAG_RESP | PROTO_FLAG_EVENT, pt->event_seq++,
                   CMD_EVENT_TRIGGER, (const uint8_t *)&ev->trigger,
                   (uint16_t)sizeof(ev->trigger));
    } else if (ev->kind == ACQ_EV_OVERRUN) {
        send_frame(pt, PROTO_FLAG_RESP | PROTO_FLAG_EVENT, pt->event_seq++,
                   CMD_EVENT_OVERRUN, (const uint8_t *)&ev->overrun,
                   (uint16_t)sizeof(ev->overrun));
    }
}

uint16_t proto_task_last_error(const proto_task_t *pt)
{
    return pt->last_error_code;
}

uint32_t proto_task_crc_err(const proto_task_t *pt)
{
    return pt->parser.crc_err_count;
}

uint32_t proto_task_dropped(const proto_task_t *pt)
{
    return pt->parser.dropped_count;
}
