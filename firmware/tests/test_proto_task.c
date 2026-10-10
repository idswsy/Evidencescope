/* tests/test_proto_task.c —— 命令分发的测试
 *
 * 判据是**往返**：把一条请求编码成线格式 → 喂给 `proto_task_poll` →
 * 把它写出去的字节再解析回来 → 断言响应体的内容。
 *
 * 为什么必须走完整的编解码，而不是直接调处理函数：
 * 主机侧读的就是这些字节。只调处理函数的话，「payload 拼错一个偏移」
 * 这类错误一个都抓不到 —— 而两端各按自己的偏移写，正是协议最容易出的问题。
 */

#include "acq.h"
#include "proto_task.h"
#include "test_util.h"

#include <string.h>

/* ══════════════════════════════════════════════════════════════════
 * 假 HAL：把发出去的字节攒起来，供测试解析
 * ══════════════════════════════════════════════════════════════════ */

#define TX_CAP 8192u

static uint16_t g_ring[ACQ_RING_SAMPLES];
static uint8_t g_tx[TX_CAP];
static uint32_t g_tx_len;
static uint8_t g_rx[512];
static uint32_t g_rx_len;
static uint32_t g_ms;
static uint32_t g_tick;
/* 已发布的半区（ACQ_HALF_FIRST / SECOND），由 acq_poll 领取。
 * 与 test_acq.c 的 g_pub 同一套路 —— 采集状态机只认**发布过**的半区。 */
static uint8_t g_pub;

static void m_write(const uint8_t *d, uint32_t n)
{
    if (g_tx_len + n <= TX_CAP) {
        memcpy(&g_tx[g_tx_len], d, n);
        g_tx_len += n;
    }
}
static uint32_t m_read(uint8_t *b, uint32_t cap)
{
    uint32_t n = (g_rx_len < cap) ? g_rx_len : cap;
    memcpy(b, g_rx, n);
    /* 削掉已读的部分 */
    memmove(g_rx, g_rx + n, g_rx_len - n);
    g_rx_len -= n;
    return n;
}
static uint32_t m_now_ms(void) { return g_ms; }
static uint32_t m_tick_us(void) { return g_tick; }
static uint32_t m_quantize(uint32_t hz)
{
    if (hz == 0 || hz > ACQ_RATE_MAX_HZ) return 0;
    uint32_t period = (72000000u + hz / 2u) / hz;
    if (period == 0) period = 1;
    uint32_t actual = 72000000u / period;
    return (actual == 0 || actual > ACQ_RATE_MAX_HZ) ? 0u : actual;
}
static void m_start(uint32_t hz) { (void)hz; }
static void m_stop(uint32_t extra) { (void)extra; }
static const uint16_t *m_ring(void) { return g_ring; }
static uint8_t m_take(void)
{
    uint8_t v = g_pub;
    g_pub = 0;
    return v;
}

static const hal_t MOCK = {
    .link_write = m_write,
    .link_read = m_read,
    .now_ms = m_now_ms,
    .quantize_rate_hz = m_quantize,
    .acq_start = m_start,
    .acq_stop = m_stop,
    .ring_base = m_ring,
    .take_published_halves = m_take,
    .tick_us = m_tick_us,
};

/* ══════════════════════════════════════════════════════════════════
 * 工具
 * ══════════════════════════════════════════════════════════════════ */

static void reset_world(void)
{
    memset(g_ring, 0, sizeof(g_ring));
    g_tx_len = 0;
    g_rx_len = 0;
    g_ms = 0;
    g_tick = 1000;
    g_pub = 0;
}

/* 往环里填第一个半区并「发布」它。
 *
 * 只有在发布之后，`acq_poll` 才会把这块数据算进采集进度 ——
 * 这是 `hal_t.take_published_halves` 的语义（见 test_acq.c 的 feed()）。
 * 本文件只需要一个半区：窗口 5 点，auto 超时强制完成后数据够用。 */
static void publish_first_half(uint16_t (*gen)(uint32_t i))
{
    for (uint32_t i = 0; i < ACQ_HALF_SAMPLES; i++) {
        g_ring[i] = gen(i);
    }
    g_pub |= ACQ_HALF_FIRST;
}

/* 把一条请求编码进「主机→设备」的接收缓冲。 */
static void push_request(uint16_t seq, uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[PROTO_MAX_FRAME_TX];
    size_t flen = 0;
    proto_status_t st = proto_encode(frame, sizeof(frame), 0u, seq, cmd, payload, len, &flen);
    if (st != PROTO_OK || g_rx_len + flen > sizeof(g_rx)) {
        FAIL("测试自身构造请求失败（st=%d len=%zu）", (int)st, flen);
        return;
    }
    memcpy(&g_rx[g_rx_len], frame, flen);
    g_rx_len += (uint32_t)flen;
}

/* 把设备写出去的字节解析成帧。返回帧数，最多 `cap` 个。 */
static uint32_t drain_tx(proto_frame_t *out, uint32_t cap, uint8_t *storage,
                         size_t storage_cap)
{
    proto_parser_t p;
    proto_parser_init(&p);
    /* 解析器的缓冲是它自己内部的，payload 指向它 —— 这里直接把整段
     * 喂进去并逐个记下 header，payload 需要时当场拷走。 */
    (void)storage;
    (void)storage_cap;
    uint32_t n = 0;
    const uint8_t *d = g_tx;
    size_t len = g_tx_len;
    for (;;) {
        proto_frame_t f;
        proto_status_t st = proto_parser_feed(&p, d, len, &f);
        d = NULL;
        len = 0;
        if (st == PROTO_OK) {
            if (n < cap) {
                out[n] = f;
                n++;
            }
            continue;
        }
        break;
    }
    return n;
}

/* ══════════════════════════════════════════════════════════════════
 * 用例
 * ══════════════════════════════════════════════════════════════════ */

static void case_get_info_roundtrip(void)
{
    GROUP("GET_INFO 往返");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    push_request(7, CMD_GET_INFO, NULL, 0);
    proto_task_poll(&pt);

    proto_frame_t got[4];
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1, "应当回一帧，实测 %u", n);
    if (n != 1) return;

    CHECK(got[0].hdr.seq == 7, "序号应当回显，实测 %u", got[0].hdr.seq);
    CHECK(got[0].hdr.cmd == CMD_GET_INFO, "命令码应当回显");
    CHECK((got[0].hdr.flags & PROTO_FLAG_RESP) != 0, "应当置 RESP 标志");
    CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) == 0, "不该是错误帧");
    CHECK(got[0].hdr.len == sizeof(info_resp_t),
          "payload 长度应为 %zu，实测 %u", sizeof(info_resp_t), got[0].hdr.len);

    if (got[0].hdr.len == sizeof(info_resp_t)) {
        info_resp_t r;
        memcpy(&r, got[0].payload, sizeof(r));
        CHECK(r.proto_ver == PROTO_VER, "协议版本");
        CHECK(r.adc_bits == 12, "12-bit ADC");
        CHECK(r.rate_max_hz == ACQ_RATE_MAX_HZ, "采样率上限应为 857142，实测 %u",
              r.rate_max_hz);
        CHECK(r.capture_max_samples == 4096u, "采集深度上限");
        CHECK(r.ch_count == 1u, "**本板只有 1 路模拟通道**（真双通道要等 P4）");
        /* 不能声明还没有的能力 —— 声明了主机就会去用它 */
        CHECK((r.caps & CAP_CH_DUAL) == 0, "不该声明双通道能力");
        CHECK((r.caps & CAP_COUPLING_AC) == 0,
              "耦合是手拨开关 SW2，不该声明 AC 耦合能力");
        /* 同上：这两条能力没实现，声明了 Agent 就会去调必然失败的命令。 */
        CHECK((r.caps & CAP_TRIG_SOFT) == 0,
              "软触发未实现（FORCE_TRIGGER 恒回 ERR_UNSUPPORTED），不该声明");
        CHECK((r.caps & CAP_FMT_MINMAX) == 0,
              "MINMAX 打包未实现（READ_BUFFER 拒绝该格式），不该声明");
        CHECK((r.caps & CAP_TRIG_AUTO) != 0, "auto 触发是实现了的");
        CHECK((r.caps & CAP_FMT_PACK12) != 0, "PACK12 是实现了的");
    }
}

static void case_set_rate_echoes_actual(void)
{
    GROUP("SET_SAMPLE_RATE 回显量化值");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    set_rate_req_t req = {.requested_hz = 100001u};
    push_request(1, CMD_SET_SAMPLE_RATE, (const uint8_t *)&req, sizeof(req));
    proto_task_poll(&pt);

    proto_frame_t got[4];
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1, "应当回一帧，实测 %u", n);
    if (n != 1) return;
    CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) == 0, "不该是错误帧");
    if (got[0].hdr.len == sizeof(set_rate_resp_t)) {
        set_rate_resp_t r;
        memcpy(&r, got[0].payload, sizeof(r));
        CHECK(r.actual_hz == 100000u,
              "必须回显**量化后**的值（协议纪律：主机时间轴以它为准），实测 %u",
              r.actual_hz);
    } else {
        FAIL("payload 长度应为 %zu，实测 %u", sizeof(set_rate_resp_t), got[0].hdr.len);
    }

    /* 达不到的采样率要报错，不能静默吸附 */
    reset_world();
    set_rate_req_t bad = {.requested_hz = 10000000u};
    push_request(2, CMD_SET_SAMPLE_RATE, (const uint8_t *)&bad, sizeof(bad));
    proto_task_poll(&pt);
    n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1, "应当回一帧");
    if (n == 1) {
        CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) != 0,
              "10 MHz 达不到，应当回错误帧而不是吸到一个近的档位");
    }
}

static void case_state_machine_gate(void)
{
    GROUP("状态机门禁");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    proto_frame_t got[4];

    /* IDLE 下 ARM 应当成功 */
    push_request(1, CMD_ARM, NULL, 0);
    proto_task_poll(&pt);
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1 && (got[0].hdr.flags & PROTO_FLAG_ERROR) == 0, "ARM 应当成功");
    CHECK(acq_state(&a) == STATE_ARMED, "ARM 之后应当是 ARMED");

    /* ARMED 下改配置 → BUSY */
    reset_world();
    set_trigger_req_t tr = {.mode = TRIG_MODE_AUTO,
                            .source = TRIG_SRC_CH1,
                            .edge = TRIG_EDGE_RISING,
                            .level_lsb = 2048,
                            .pre_samples = 512,
                            .holdoff_us = 1000};
    push_request(2, CMD_SET_TRIGGER, (const uint8_t *)&tr, sizeof(tr));
    proto_task_poll(&pt);
    n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1 && (got[0].hdr.flags & PROTO_FLAG_ERROR) != 0,
          "ARMED 下改触发应当回错误");
    if (n == 1) {
        error_resp_t e;
        memcpy(&e, got[0].payload, sizeof(e));
        CHECK(e.code == ERR_BUSY, "错误码应当是 BUSY(4)，实测 %u", e.code);
    }

    /* STOP 之后又能改了 */
    reset_world();
    push_request(3, CMD_STOP, NULL, 0);
    proto_task_poll(&pt);
    n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1 && (got[0].hdr.flags & PROTO_FLAG_ERROR) == 0,
          "STOP 任何时候都该成功（协议规定「已停也回 Ack」）");
    CHECK(acq_state(&a) == STATE_IDLE, "STOP 之后应当是 IDLE");

    reset_world();
    push_request(4, CMD_SET_TRIGGER, (const uint8_t *)&tr, sizeof(tr));
    proto_task_poll(&pt);
    n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1 && (got[0].hdr.flags & PROTO_FLAG_ERROR) == 0,
          "STOP 之后改触发应当成功");
}

static void case_unknown_command(void)
{
    GROUP("未知命令码");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    /* 一个没有定义的命令码 */
    push_request(9, 0x9999u, NULL, 0);
    proto_task_poll(&pt);

    proto_frame_t got[4];
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1, "未知命令也要回一帧（否则主机会一直等），实测 %u", n);
    if (n == 1) {
        CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) != 0, "应当是错误帧");
        error_resp_t e;
        memcpy(&e, got[0].payload, sizeof(e));
        CHECK(e.code == ERR_UNKNOWN_CMD, "错误码应当是 UNKNOWN_CMD(1)，实测 %u", e.code);
    }
}

static void case_read_buffer_rejects_stale_id(void)
{
    GROUP("READ_BUFFER 拒绝过期的 capture_id");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    /* 还没有采集过，任何 id 都不该匹配 */
    read_buffer_req_t req = {.capture_id = 1u,
                             .start_sample = 0u,
                             .count = 16u,
                             .format = FMT_RAW16,
                             .ch = 0u};
    push_request(1, CMD_READ_BUFFER, (const uint8_t *)&req, sizeof(req));
    proto_task_poll(&pt);

    proto_frame_t got[4];
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1, "应当回一帧");
    if (n == 1) {
        CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) != 0,
              "没有采集时读缓冲应当报错，而不是回一帧空数据让主机以为读到 0");
        error_resp_t e;
        memcpy(&e, got[0].payload, sizeof(e));
        CHECK(e.code == ERR_NO_DATA, "错误码应当是 NO_DATA(6)，实测 %u", e.code);
    }
}

/* 窗口数据生成器：恒低于触发电平（不会真的触发），但逐点不同。
 *
 * 「逐点不同」是这条测试能证伪的关键：把 3 字节当 2 字节解、或把高低
 * 半字节拼反，解出来的值都不会与窗口一致（全等的假数据抓不住这些错）。 */
static uint16_t gen_distinct_low(uint32_t i)
{
    return (uint16_t)(i % 1500u);
}

static void case_read_buffer_pack12_roundtrip(void)
{
    GROUP("READ_BUFFER PACK12 往返");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    proto_frame_t got[8];

    /* ── 1) 通过线格式配 5 点窗口并 ARM ── */
    set_acq_req_t ar = {.mode = ACQ_MODE_SINGLE, .capture_samples = 5u,
                        .format = FMT_RAW16, .decimation = 1u};
    set_trigger_req_t tr = {.mode = TRIG_MODE_AUTO, .source = TRIG_SRC_CH1,
                            .edge = TRIG_EDGE_RISING, .level_lsb = 2048u,
                            .pre_samples = 0u, .holdoff_us = 1000u};
    push_request(1, CMD_SET_ACQ, (const uint8_t *)&ar, sizeof(ar));
    push_request(2, CMD_SET_TRIGGER, (const uint8_t *)&tr, sizeof(tr));
    push_request(3, CMD_ARM, NULL, 0);
    proto_task_poll(&pt);
    uint32_t n = drain_tx(got, 8, NULL, 0);
    CHECK(n == 3, "三条请求应当回三帧，实测 %u", n);
    for (uint32_t i = 0; i < n && i < 3u; i++) {
        CHECK((got[i].hdr.flags & PROTO_FLAG_ERROR) == 0,
              "配置第 %u 条不该是错误：cmd=0x%04X", i, got[i].hdr.cmd);
    }

    /* ── 2) 完成一次采集 ──
     *
     * `proto_task_poll` 只跑链路；采集状态机由主循环里的 `acq_poll` 驱动，
     * 完成事件再由 `proto_task_emit` 发出（见 App/main.c 的主循环）。
     * auto 模式 + 200 ms 超时 = 不需要真的等到一个边沿。 */
    publish_first_half(gen_distinct_low);
    acq_out_t ev;
    CHECK(acq_poll(&a, &ev) == ACQ_EV_NONE, "恒低于电平的信号不该触发");
    g_ms += ACQ_AUTO_TIMEOUT_MS;
    CHECK(acq_poll(&a, &ev) == ACQ_EV_TRIGGERED, "auto 超时应当强制完成");
    if (acq_state(&a) != STATE_DONE) {
        FAIL("采集没进 DONE，后面的断言没有意义");
        return;
    }

    /* capture_id 只存在于 EVENT_TRIGGER 里 —— 主机就是从这拿的
     * （host/crates/core/src/acquire.rs）。这里也走同一条路，
     * 顺带验证事件里的 id 与 READ_BUFFER 认的是同一个。 */
    g_tx_len = 0; /* 清掉上面的三帧应答，只解析事件帧 */
    proto_task_emit(&pt, &ev);
    n = drain_tx(got, 8, NULL, 0);
    uint16_t cap_id = 0;
    if (n == 1) {
        /* payload 指向解析器的内部缓冲，下一次 feed 即失效 —— 先拷走 */
        cap_id = (uint16_t)(got[0].payload[0] | ((uint16_t)got[0].payload[1] << 8));
    }
    CHECK(n == 1 && got[0].hdr.cmd == CMD_EVENT_TRIGGER,
          "采集完成应当发一帧 EVENT_TRIGGER，实测 %u 帧", n);
    if (n != 1) {
        return;
    }
    CHECK(cap_id == a.capture_id, "事件里的 capture_id 应当是设备分配的那个");

    /* ── 3) 请求 PACK12 分片：count=5，奇数，末样点与 0 凑对 ── */
    read_buffer_req_t rb = {.capture_id = cap_id, .start_sample = 0u,
                            .count = 5u, .format = FMT_PACK12, .ch = 0u};
    g_tx_len = 0;
    push_request(4, CMD_READ_BUFFER, (const uint8_t *)&rb, sizeof(rb));
    proto_task_poll(&pt);

    n = drain_tx(got, 8, NULL, 0);
    CHECK(n == 1, "应当回一帧，实测 %u", n);
    if (n != 1) {
        return;
    }
    CHECK(got[0].hdr.cmd == CMD_READ_BUFFER, "命令码应当回显");
    CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) == 0,
          "不该是错误帧 —— 设备必须支持 PACK12（未实现时这里是 UNSUPPORTED）");
    CHECK(got[0].hdr.len == (uint16_t)(12u + 9u),
          "payload = 12 + ceil(5*3/2) = 21 字节，实测 %u", got[0].hdr.len);
    if (got[0].hdr.len != 21u) {
        return;
    }

    /* 解析器缓冲在下一次 feed 后就失效 —— 当场拷走再断言。 */
    uint8_t payload[64];
    memcpy(payload, got[0].payload, got[0].hdr.len);
    chunk_header_t h;
    memcpy(&h, payload, sizeof(h));
    CHECK(h.format == FMT_PACK12, "分片头必须**如实回显**请求的格式，实测 %u", h.format);
    CHECK(h.count == 5u, "count 应当是 5，实测 %u", h.count);
    CHECK(h.start_sample == 0u, "start_sample 应当回显请求值，实测 %u", h.start_sample);
    CHECK((h.flags & CHUNK_FLAG_LAST) != 0,
          "这一片覆盖到窗口末尾，应当置 LAST（flags=0x%02X）", h.flags);

    /* 每对解出来与窗口里的 acq_sample 逐点相等。第 5 个样点与 0 凑对，
     * 解出的第 6 个值超出 count，忽略。 */
    for (uint32_t i = 0; i < 5u; i += 2u) {
        uint16_t s0 = 0, s1 = 0;
        proto_unpack12_pair(&payload[PROTO_CHUNK_HEADER_LEN + (i / 2u) * 3u], &s0, &s1);
        CHECK(s0 == acq_sample(&a, 0, i), "第 %u 点：PACK12 解出 %u，窗口里是 %u",
              i, s0, acq_sample(&a, 0, i));
        if (i + 1u < 5u) {
            CHECK(s1 == acq_sample(&a, 0, i + 1u),
                  "第 %u 点：PACK12 解出 %u，窗口里是 %u", i + 1u, s1,
                  acq_sample(&a, 0, i + 1u));
        } else {
            /* 末样点与 0 凑对 —— 这是协议约定（docs/03-protocol.md §6.4），
             * 不是「随便填一个都行」：主机按 count 截断虽然会丢掉它，
             * 但两条线（固件/主机）对填充值的约定必须一致且被钉住。 */
            CHECK(s1 == 0u, "末样点应与 0 凑对，填充值实测 %u", s1);
        }
    }
}

static void case_no_reply_flag_is_honoured(void)
{
    GROUP("免回复标志");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    /* 带 NO_REPLY 的 PING —— 一个字节都不该回 */
    uint8_t frame[PROTO_MAX_FRAME_TX];
    size_t flen = 0;
    proto_encode(frame, sizeof(frame), PROTO_FLAG_NO_REPLY, 5, CMD_PING, NULL, 0, &flen);
    memcpy(g_rx, frame, flen);
    g_rx_len = (uint32_t)flen;

    proto_task_poll(&pt);
    CHECK(g_tx_len == 0, "置了免回复标志还回了 %u 字节", g_tx_len);
}

static void case_measure_says_unsupported(void)
{
    GROUP("未实现的命令要明说，不能回假数据");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    measure_req_t req = {.capture_id = 1u,
                         .start_sample = 0u,
                         .window_samples = 100u,
                         .md_mask = 0xFFFFFFFFu};
    push_request(1, CMD_MEASURE, (const uint8_t *)&req, sizeof(req));
    proto_task_poll(&pt);

    proto_frame_t got[4];
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1, "应当回一帧");
    if (n == 1) {
        CHECK((got[0].hdr.flags & PROTO_FLAG_ERROR) != 0,
              "measure 还没实现 —— 回一个全 0 的测量结果会让主机以为「测得是 0」，"
              "而那是完全不同的两件事");
        error_resp_t e;
        memcpy(&e, got[0].payload, sizeof(e));
        CHECK(e.code == ERR_UNSUPPORTED, "错误码应当是 UNSUPPORTED(10)，实测 %u", e.code);
    }
}

static void case_set_channel_rejects_ch2(void)
{
    GROUP("SET_CHANNEL 拒绝不存在的通道");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    proto_frame_t got[4];

    /* CH1（索引 0）存在 */
    set_channel_req_t ok = {.ch = 0u, .enable = 1u, .range_idx = 0u,
                            .coupling = COUPLING_DC, .offset_lsb = 0};
    push_request(1, CMD_SET_CHANNEL, (const uint8_t *)&ok, sizeof(ok));
    proto_task_poll(&pt);
    uint32_t n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1 && (got[0].hdr.flags & PROTO_FLAG_ERROR) == 0, "CH0 应当被接受");

    /* 索引 1 不存在 —— 收下并回显会让主机以为配成功了 */
    reset_world();
    set_channel_req_t bad = {.ch = 1u, .enable = 1u, .range_idx = 0u,
                             .coupling = COUPLING_DC, .offset_lsb = 0};
    push_request(2, CMD_SET_CHANNEL, (const uint8_t *)&bad, sizeof(bad));
    proto_task_poll(&pt);
    n = drain_tx(got, 4, NULL, 0);
    CHECK(n == 1 && (got[0].hdr.flags & PROTO_FLAG_ERROR) != 0,
          "本板只有一路模拟通道，配 CH2 应当被拒");
}

static void case_set_acq_echoes_and_arms(void)
{
    GROUP("SET_ACQ + ARM + READ_BUFFER 全链路");

    reset_world();
    acq_t a;
    acq_init(&a, &MOCK);
    proto_task_t pt;
    proto_task_init(&pt, &MOCK, &a);

    proto_frame_t got[8];

    set_acq_req_t ar = {.mode = ACQ_MODE_SINGLE, .capture_samples = 256,
                        .format = FMT_RAW16, .decimation = 1};
    push_request(1, CMD_SET_ACQ, (const uint8_t *)&ar, sizeof(ar));
    set_trigger_req_t tr = {.mode = TRIG_MODE_AUTO, .source = TRIG_SRC_CH1,
                            .edge = TRIG_EDGE_RISING, .level_lsb = 2048,
                            .pre_samples = 128, .holdoff_us = 1000};
    push_request(2, CMD_SET_TRIGGER, (const uint8_t *)&tr, sizeof(tr));
    push_request(3, CMD_ARM, NULL, 0);
    proto_task_poll(&pt);

    uint32_t n = drain_tx(got, 8, NULL, 0);
    CHECK(n == 3, "三条请求应当回三帧，实测 %u", n);
    for (uint32_t i = 0; i < n; i++) {
        CHECK((got[i].hdr.flags & PROTO_FLAG_ERROR) == 0,
              "第 %u 条不该是错误：cmd=0x%04X", i, got[i].hdr.cmd);
    }

    /* 读回配置，确认 SET_ACQ 真的落到了状态机里。
     *
     * ⚠ 必须先 STOP —— 设备此刻还在 ARMED，而 `proto_cmd_allowed` 规定
     * ARMED 下不收 GET_CONFIG（回 BUSY）。第一版就是漏了这一步，
     * 拿一个 20 字节的错误帧去当 17 字节的配置解。 */
    reset_world();
    push_request(4, CMD_STOP, NULL, 0);
    push_request(5, CMD_GET_CONFIG, NULL, 0);
    proto_task_poll(&pt);
    n = drain_tx(got, 8, NULL, 0);
    CHECK(n == 2, "STOP + GET_CONFIG 应当回两帧，实测 %u", n);
    if (n == 2) {
        CHECK((got[1].hdr.flags & PROTO_FLAG_ERROR) == 0,
              "STOP 之后 GET_CONFIG 应当成功");
        CHECK(got[1].hdr.len == 17u, "GET_CONFIG payload 应当是 17 字节，实测 %u",
              got[1].hdr.len);
        if (got[1].hdr.len == 17u) {
            const uint8_t *p = got[1].payload;
            uint16_t cs = (uint16_t)(p[5] | ((uint16_t)p[6] << 8));
            CHECK(cs == 256u, "capture_samples 应当是 256，实测 %u", cs);
            CHECK(p[10] == TRIG_MODE_AUTO, "触发模式应当是 auto，实测 %u", p[10]);
        }
    }
}

int main(void)
{
    case_get_info_roundtrip();
    case_set_rate_echoes_actual();
    case_state_machine_gate();
    case_unknown_command();
    case_read_buffer_rejects_stale_id();
    case_read_buffer_pack12_roundtrip();
    case_no_reply_flag_is_honoured();
    case_measure_says_unsupported();
    case_set_channel_rejects_ch2();
    case_set_acq_echoes_and_arms();
    return test_summary();
}
