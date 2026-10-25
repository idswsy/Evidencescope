/**
 * @file    protocol.c
 * @brief   EvidenceScope 线协议编解码实现
 *
 * 纯 C，不依赖任何 HAL / 寄存器头文件 —— 可在 PC 上用 gcc 直接编译测试。
 * 无动态内存分配，所有状态由调用者持有。
 *
 * 设计要点：
 *   - 增量字节流状态机，天然处理粘包/拆包
 *   - 出错时以 +1 字节步进扫描 0xAA 0x55 重同步（不是按帧长跳过）
 *   - 解析顺序固定：读头 → 验 LEN → 读 payload → 算 CRC（先验长度，防越界）
 *   - CRC 增量计算，早失败早丢弃
 */

#include "protocol.h"

#include <string.h>

/* ══════════════════════════════════════════════════════════════════
 * CRC-16/CCITT-FALSE
 *   poly   0x1021
 *   init   0xFFFF
 *   refin  false / refout false / xorout 0x0000
 *   校验值 "123456789" → 0x29B1
 * 查表 256 项 = 512 B Flash，2 KB 帧约 0.2 ms @72MHz
 * ══════════════════════════════════════════════════════════════════ */

static uint16_t s_crc_table[256];
static bool     s_crc_table_ready = false;

static void crc16_build_table(void)
{
    for (uint32_t i = 0; i < 256u; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
        s_crc_table[i] = crc;
    }
    s_crc_table_ready = true;
}

uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    if (!s_crc_table_ready) {
        crc16_build_table();
    }

    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)(((crc >> 8) ^ data[i]) & 0xFFu);
        crc = (uint16_t)((crc << 8) ^ s_crc_table[idx]);
    }
    return crc;
}

/* ══════════════════════════════════════════════════════════════════
 * 小端读写helper
 * ══════════════════════════════════════════════════════════════════ */

static inline uint16_t rd_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void wr_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/* ══════════════════════════════════════════════════════════════════
 * 编码
 * ══════════════════════════════════════════════════════════════════ */

proto_status_t proto_encode(uint8_t *out, size_t out_cap,
                            uint8_t flags, uint16_t seq, uint16_t cmd,
                            const uint8_t *payload, uint16_t payload_len,
                            size_t *out_len)
{
    if (out == NULL || out_len == NULL) {
        return PROTO_BAD_LEN;
    }
    if (payload_len > PROTO_MAX_PAYLOAD_TX) {
        return PROTO_BAD_LEN;
    }
    if (payload_len > 0u && payload == NULL) {
        return PROTO_BAD_LEN;
    }

    const size_t frame_len = (size_t)PROTO_HEADER_LEN + (size_t)payload_len
                           + (size_t)PROTO_CRC_LEN;
    if (out_cap < frame_len) {
        return PROTO_BAD_LEN;
    }

    out[0] = PROTO_SYNC0;
    out[1] = PROTO_SYNC1;
    out[2] = PROTO_VER;
    out[3] = flags;
    wr_u16le(&out[4], seq);
    wr_u16le(&out[6], cmd);
    wr_u16le(&out[8], payload_len);

    if (payload_len > 0u) {
        memcpy(&out[PROTO_HEADER_LEN], payload, payload_len);
    }

    /* CRC 覆盖 VER..PAYLOAD 末尾（不含 SYNC） */
    const uint16_t crc = proto_crc16(&out[2], (size_t)PROTO_HEADER_LEN - 2u
                                              + (size_t)payload_len);
    wr_u16le(&out[PROTO_HEADER_LEN + payload_len], crc);

    *out_len = frame_len;
    return PROTO_OK;
}

proto_status_t proto_encode_error(uint8_t *out, size_t out_cap,
                                  uint16_t seq, uint16_t hash_cmd,
                                  uint16_t code, uint8_t severity,
                                  const char *msg, uint8_t msg_len,
                                  size_t *out_len)
{
    (void)hash_cmd;   /* 保留：将来可在错误 payload 里回显触发它的 CMD */

    error_resp_t e;
    memset(&e, 0, sizeof(e));
    e.code     = code;
    e.severity = severity;

    if (msg_len > sizeof(e.msg)) {
        msg_len = (uint8_t)sizeof(e.msg);
    }
    e.msg_len = msg_len;
    if (msg_len > 0u && msg != NULL) {
        memcpy(e.msg, msg, msg_len);
    }

    /* 注意：不能直接 (uint8_t*)&e —— 那会把结构体布局暴露到线上。
     * 逐字段显式序列化，布局由本函数锁死。 */
    uint8_t buf[2u + 1u + 1u + 32u];
    size_t  n = 0;
    wr_u16le(&buf[n], e.code);     n += 2u;
    buf[n++] = e.severity;
    buf[n++] = e.msg_len;
    if (e.msg_len > 0u) {
        memcpy(&buf[n], e.msg, e.msg_len);
        n += e.msg_len;
    }

    return proto_encode(out, out_cap,
                        (uint8_t)(PROTO_FLAG_RESP | PROTO_FLAG_ERROR),
                        seq, CMD_ERROR, buf, (uint16_t)n, out_len);
}

/* ══════════════════════════════════════════════════════════════════
 * PACK12：2 个 12-bit 样点 → 3 字节（省 25%）
 * ══════════════════════════════════════════════════════════════════ */

void proto_pack12_pair(uint16_t s0, uint16_t s1, uint8_t out[3])
{
    out[0] = (uint8_t)(s0 & 0xFFu);
    out[1] = (uint8_t)(((s0 >> 8) & 0x0Fu) | ((s1 & 0x0Fu) << 4));
    out[2] = (uint8_t)((s1 >> 4) & 0xFFu);
}

void proto_unpack12_pair(const uint8_t in[3], uint16_t *s0, uint16_t *s1)
{
    if (s0 != NULL) {
        *s0 = (uint16_t)(((uint16_t)in[0] | ((uint16_t)(in[1] & 0x0Fu) << 8)) & 0x0FFFu);
    }
    if (s1 != NULL) {
        *s1 = (uint16_t)((((uint16_t)in[1] >> 4) | ((uint16_t)in[2] << 4)) & 0x0FFFu);
    }
}

/* ══════════════════════════════════════════════════════════════════
 * 增量解析器
 * ══════════════════════════════════════════════════════════════════ */

void proto_parser_init(proto_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
}

/**
 * 丢弃缓冲区头部 n 字节
 */
static void parser_consume(proto_parser_t *p, uint16_t n)
{
    if (n >= p->len) {
        p->len = 0u;
        return;
    }
    memmove(p->buf, &p->buf[n], (size_t)(p->len - n));
    p->len = (uint16_t)(p->len - n);
}

/**
 * 尝试从缓冲区头部解析一个完整帧。
 * 返回 PROTO_OK 时，帧长度写入 *frame_len，但**不从缓冲区移除**
 * （由调用者在下次 feed 时惰性消费，以免让 out->payload 立刻失效）。
 */
static proto_status_t parser_try_parse(proto_parser_t *p,
                                       proto_frame_t *out,
                                       uint16_t *frame_len)
{
    /* 找同步头：以 +1 字节步进扫描，不是按帧长跳过 */
    while (p->len >= 2u &&
           !(p->buf[0] == PROTO_SYNC0 && p->buf[1] == PROTO_SYNC1)) {
        parser_consume(p, 1u);
        p->dropped_count++;
    }

    if (p->len < PROTO_HEADER_LEN) {
        return PROTO_NEED_MORE;
    }

    const uint16_t payload_len = rd_u16le(&p->buf[8]);

    /* 先验长度再读 payload —— 防越界 */
    if (payload_len > PROTO_MAX_PAYLOAD_TX) {
        parser_consume(p, 1u);
        p->dropped_count++;
        return PROTO_BAD_LEN;
    }

    const uint16_t need = (uint16_t)(PROTO_HEADER_LEN + payload_len + PROTO_CRC_LEN);
    if (p->len < need) {
        return PROTO_NEED_MORE;
    }

    /* CRC 覆盖 VER..PAYLOAD 末尾（不含 SYNC） */
    const uint16_t crc_calc = proto_crc16(&p->buf[2],
                                          (size_t)PROTO_HEADER_LEN - 2u
                                          + (size_t)payload_len);
    const uint16_t crc_wire = rd_u16le(&p->buf[PROTO_HEADER_LEN + payload_len]);

    if (crc_calc != crc_wire) {
        /* 不可信的帧不回应答，丢 1 字节重新找同步头 */
        parser_consume(p, 1u);
        p->crc_err_count++;
        return PROTO_CRC_ERR;
    }

    /* 主版本不符的帧仍然解析出来（调用者需要应答 GET_INFO / PING） */
    out->hdr.sync0 = p->buf[0];
    out->hdr.sync1 = p->buf[1];
    out->hdr.ver   = p->buf[2];
    out->hdr.flags = p->buf[3];
    out->hdr.seq   = rd_u16le(&p->buf[4]);
    out->hdr.cmd   = rd_u16le(&p->buf[6]);
    out->hdr.len   = payload_len;
    out->payload   = (payload_len > 0u) ? &p->buf[PROTO_HEADER_LEN] : NULL;

    *frame_len = need;

    if (((out->hdr.ver >> 4) & 0x0Fu) != PROTO_VER_MAJOR) {
        return PROTO_BAD_VER;
    }
    return PROTO_OK;
}

proto_status_t proto_parser_feed(proto_parser_t *p,
                                 const uint8_t *data, size_t len,
                                 proto_frame_t *out)
{
    if (p == NULL || out == NULL) {
        return PROTO_BAD_LEN;
    }

    /* 惰性消费上一次成功返回的帧 —— 此刻 out->payload 才正式失效 */
    if (p->pending > 0u) {
        parser_consume(p, p->pending);
        p->pending = 0u;
    }

    /* 追加新数据；缓冲区满时丢弃最旧的字节（说明流已失步） */
    if (data != NULL && len > 0u) {
        size_t incoming = len;
        while (incoming > (size_t)(PROTO_PARSER_BUF_SIZE - p->len)) {
            parser_consume(p, 1u);
            p->dropped_count++;
            if (p->len == 0u && incoming > PROTO_PARSER_BUF_SIZE) {
                /* 单次喂入就超过整个缓冲：丢弃前面的部分 */
                const size_t skip = incoming - PROTO_PARSER_BUF_SIZE;
                data += skip;
                incoming -= skip;
                p->dropped_count += (uint32_t)skip;
            }
        }
        memcpy(&p->buf[p->len], data, incoming);
        p->len = (uint16_t)(p->len + incoming);
    }

    /* 解析直到出一个帧或数据不够 */
    for (;;) {
        uint16_t frame_len = 0u;
        const proto_status_t st = parser_try_parse(p, out, &frame_len);

        switch (st) {
        case PROTO_OK:
        case PROTO_BAD_VER:
            p->pending = frame_len;   /* 下次 feed 时移除 */
            return st;

        case PROTO_NEED_MORE:
            return PROTO_NEED_MORE;

        case PROTO_CRC_ERR:
        case PROTO_BAD_LEN:
            /* 已跳过一个字节，继续扫描 */
            continue;
        }
        return PROTO_NEED_MORE;
    }
}

/* ══════════════════════════════════════════════════════════════════
 * 状态机：命令合法性矩阵
 * ══════════════════════════════════════════════════════════════════ */

bool proto_cmd_allowed(scope_state_t state, uint16_t cmd)
{
    /* 基础命令在任何状态都允许 */
    if (cmd == CMD_GET_INFO || cmd == CMD_PING ||
        cmd == CMD_GET_STATUS || cmd == CMD_STOP ||
        cmd == CMD_GET_LAST_ERROR ||
        cmd == CMD_EVENT_TRIGGER || cmd == CMD_EVENT_OVERRUN ||
        cmd == CMD_EVENT_LOG) {
        return true;
    }

    switch (state) {
    case STATE_IDLE:
    case STATE_DONE:
        /* IDLE / DONE 下**全部**允许 —— 包括 ARM。
         *
         * 注意：在 DONE 下再次 ARM 会丢弃当前采集的数据。这里选择放行而不是
         * 拒绝，是因为「采完想立刻再采一次」是最常见的用法，为它加一道状态
         * 检查会让主机的采集循环多一次 GET_STATUS 往返。数据被丢弃这件事
         * 由 READ_BUFFER 的 capture_id 校验兜住（对不上的 id 回 NO_DATA）。
         *
         * 回归：这句注释从前写的是「除 ARM 会丢数据外全部允许」——
         * 与它下面那行 `return true` 直接矛盾，而 docs/03 的状态机表照抄了
         * 注释的说法，于是契约页与实现讲了两件不同的事。 */
        return true;

    case STATE_ARMED:
    case STATE_STREAMING:
        /* 采集进行中只允许停止与强制触发，其余配置命令要回 BUSY */
        return (cmd == CMD_FORCE_TRIGGER);

    case STATE_FAULT:
        /* 故障态只允许复位 */
        return (cmd == CMD_RESET);

    default:
        return false;
    }
}
/* ── 采样率量化 ────────────────────────────────────────────────── */

uint32_t proto_quantize_rate_hz(uint32_t requested_hz)
{
    const uint32_t TIMER_HZ   = 72000000u;
    const uint32_t PERIOD_MAX = 65536u;

    if (requested_hz == 0u || requested_hz > PROTO_RATE_MAX_HZ) {
        return 0u;
    }

    /* ideal = 72e6 / requested，整数版四舍五入（+n/2 再除） */
    uint32_t base = (TIMER_HZ + requested_hz / 2u) / requested_hz;
    if (base < 1u) { base = 1u; }
    if (base > PERIOD_MAX) { base = PERIOD_MAX; }

    uint32_t best_rate = TIMER_HZ / base;
    uint32_t best_err  = (best_rate > requested_hz) ? (best_rate - requested_hz)
                                                    : (requested_hz - best_rate);

    /* 只扫 ±1 就够了：理想周期取整后，最近的两档一定在相邻周期里 */
    uint32_t cand[2];
    cand[0] = (base > 1u) ? (base - 1u) : 1u;
    cand[1] = ((base + 1u) < PERIOD_MAX) ? (base + 1u) : PERIOD_MAX;

    for (int i = 0; i < 2; i++) {
        uint32_t p    = cand[i];
        uint32_t rate = TIMER_HZ / p;
        uint32_t err  = (rate > requested_hz) ? (rate - requested_hz)
                                              : (requested_hz - rate);
        /* 误差相等时取**更快**的 —— 与上位机同一条 tie 规则 */
        if (err < best_err || (err == best_err && rate > best_rate)) {
            best_err  = err;
            best_rate = rate;
        }
    }

    if (best_rate == 0u || best_rate > PROTO_RATE_MAX_HZ) {
        return 0u;
    }
    return best_rate;
}
