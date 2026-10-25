/**
 * @file    protocol.h
 * @brief   EvidenceScope 线协议定义 —— C 端唯一真相源
 *
 * 本文件与 docs/03-protocol.md、host/crates/proto/src/lib.rs、
 * proto/tests/vectors.json 必须保持同步。改协议 = 改四处 + 一次提交。
 *
 * 设计纪律：
 *   - 所有多字节整数一律小端
 *   - 线上格式不依赖编译器结构体布局：pack(1) + _Static_assert 锁偏移
 *   - 控制链路永不出现 f32（电平用 LSB 整数，测量用带缩放的定点）
 *   - 未知 CMD 必须按 LEN 吞掉整帧再回 UNKNOWN_CMD
 *
 * @note    本文件与 protocol.c 均不依赖任何 HAL / 寄存器头文件，
 *          可在 PC 上用 gcc 直接编译测试（见 Makefile）。
 */

#ifndef SCOPE_PROTOCOL_H
#define SCOPE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════════
 * 常量
 * ══════════════════════════════════════════════════════════════════ */

#define PROTO_SYNC0              0xAAu
#define PROTO_SYNC1              0x55u

#define PROTO_VER_MAJOR          1u
#define PROTO_VER_MINOR          0u
#define PROTO_VER                ((uint8_t)((PROTO_VER_MAJOR << 4) | PROTO_VER_MINOR))

/** 固定帧头长度：SYNC(2) + VER(1) + FLAGS(1) + SEQ(2) + CMD(2) + LEN(2) */
#define PROTO_HEADER_LEN         10u
/** CRC16 长度 */
#define PROTO_CRC_LEN            2u
/** 最小帧长（LEN = 0） */
#define PROTO_FRAME_MIN          (PROTO_HEADER_LEN + PROTO_CRC_LEN)   /* 12 */

/** 分片头长度（READ_BUFFER 响应的前 12 字节） */
#define PROTO_CHUNK_HEADER_LEN   12u

/**
 * 一个分片最多装多少样点（RAW16 格式）。
 *
 * 1024 样点 × u16 = 2048 B，再加 12 B 分片头 = 2060 B payload。
 * **分片头是算在 payload 之内的** —— 这一点很容易算漏，
 * 写成「MAX_PAYLOAD = 2048」会导致 1024 点的分片超限。
 */
#define PROTO_CHUNK_SAMPLES_MAX  1024u

/**
 * 双向上限不对称：
 *   设备 → 主机：2060 B（12 B 分片头 + 1024 样点 × u16）
 *   主机 → 设备：512 B（实际命令都 ≤ 64 B）
 */
#define PROTO_MAX_PAYLOAD_TX     (PROTO_CHUNK_HEADER_LEN + PROTO_CHUNK_SAMPLES_MAX * 2u)  /* 2060 */
#define PROTO_MAX_PAYLOAD_RX     512u    /* 设备接收 */
#define PROTO_MAX_FRAME_TX       (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD_TX + PROTO_CRC_LEN)  /* 2072 */
#define PROTO_MAX_FRAME_RX       (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD_RX + PROTO_CRC_LEN)  /* 524 */

/**
 * 解析器缓冲大小。
 *   设备侧默认 PROTO_MAX_FRAME_RX（524 B，因为设备只收 ≤512 B payload）
 *   主机侧应编译为 PROTO_MAX_FRAME_TX（2072 B，要收设备发来的波形分片）
 *   覆盖方式：-DPROTO_PARSER_BUF_SIZE=2072
 */
#ifndef PROTO_PARSER_BUF_SIZE
#define PROTO_PARSER_BUF_SIZE    PROTO_MAX_FRAME_RX
#endif

/** 采集深度上限（8 KB 环 = 4096 点 × u16） */
#define PROTO_CAPTURE_MAX_SAMPLES 4096u
/** 默认分片大小（RAW16 时 payload = 12 分片头 + 1024×2 = 2060 B） */
#define PROTO_PREFERRED_CHUNK     1024u

/** 设备时钟频率：1 MHz（u32 tick_us，约 71.6 分钟回绕） */
#define PROTO_TICK_HZ            1000000u

/**
 * 单 ADC 的采样率上限（`docs/04-performance.md`：12 MHz ADCCLK ÷ 14 周期）。
 * **不是 1 MSPS**。与 `App/hal.h` 的 `ACQ_RATE_MAX_HZ` 是同一个数。
 */
#define PROTO_RATE_MAX_HZ 857142u

/* ══════════════════════════════════════════════════════════════════
 * FLAGS 位
 * ══════════════════════════════════════════════════════════════════ */

#define PROTO_FLAG_RESP           (1u << 0)  /* 1 = 响应 */
#define PROTO_FLAG_ERROR          (1u << 1)  /* 1 = 错误 */
#define PROTO_FLAG_MORE           (1u << 2)  /* 分片未结束 */
#define PROTO_FLAG_LAST           (1u << 3)  /* 分片结束 */
#define PROTO_FLAG_EVENT          (1u << 4)  /* 设备主动上报 */
#define PROTO_FLAG_NO_REPLY       (1u << 5)  /* 免回复 */
/* bit 6-7 保留，写 0 读忽略 */

/* ══════════════════════════════════════════════════════════════════
 * 命令码：高字节 = 类，低字节 = 类内编号
 * 编号一经分配永不复用
 * ══════════════════════════════════════════════════════════════════ */

/* 系统类 0x01xx */
#define CMD_GET_INFO             0x0101u
#define CMD_PING                 0x0102u
#define CMD_GET_STATUS           0x0103u
#define CMD_RESET                0x0104u

/* 配置类 0x02xx */
#define CMD_GET_CONFIG           0x0200u
#define CMD_SET_SAMPLE_RATE      0x0201u
#define CMD_SET_CHANNEL          0x0202u
#define CMD_SET_TRIGGER          0x0203u
#define CMD_SET_ACQ              0x0204u

/* 采集类 0x03xx */
#define CMD_ARM                  0x0301u
#define CMD_STOP                 0x0302u
#define CMD_FORCE_TRIGGER        0x0303u

/* 读取类 0x04xx */
#define CMD_READ_BUFFER          0x0401u

/* 测量类 0x05xx */
#define CMD_MEASURE              0x0501u

/* 调试与事件 0x06xx */
#define CMD_ECHO                 0x0601u
#define CMD_SET_LOG_LEVEL        0x0602u
#define CMD_GET_LAST_ERROR       0x0603u
#define CMD_MEM_READ             0x0610u   /* 仅 DEBUG 构建 */
#define CMD_MEM_WRITE            0x0611u   /* 仅 DEBUG 构建 */
#define CMD_EVENT_TRIGGER        0x0682u
#define CMD_EVENT_OVERRUN        0x0683u
#define CMD_EVENT_LOG            0x0684u

/* 错误 0x07xx */
#define CMD_ERROR                0x0701u

/** 命令分类（高字节） */
#define CMD_CLASS(cmd)           ((uint8_t)(((cmd) >> 8) & 0xFFu))
#define CMD_CLASS_SYSTEM         0x01u
#define CMD_CLASS_CONFIG         0x02u
#define CMD_CLASS_ACQ            0x03u
#define CMD_CLASS_READ           0x04u
#define CMD_CLASS_MEASURE        0x05u
#define CMD_CLASS_DEBUG          0x06u
#define CMD_CLASS_ERROR          0x07u

/* ══════════════════════════════════════════════════════════════════
 * 错误码
 * ══════════════════════════════════════════════════════════════════ */

typedef enum {
    ERR_OK               = 0x0000,
    ERR_UNKNOWN_CMD      = 0x0001,
    ERR_BAD_LEN          = 0x0002,
    ERR_BAD_PARAM        = 0x0003,
    ERR_BUSY             = 0x0004,
    ERR_BAD_STATE        = 0x0005,
    ERR_NO_DATA          = 0x0006,
    ERR_OVERRUN          = 0x0007,
    ERR_TIMEOUT          = 0x0008,
    ERR_VERSION_MISMATCH = 0x0009,
    ERR_UNSUPPORTED      = 0x000A,
    ERR_FLASH_ERR        = 0x000B,
    ERR_INTERNAL         = 0x000C,
} scope_err_t;

#define ERR_SEV_WARN             0u
#define ERR_SEV_ERR              1u
#define ERR_SEV_FATAL            2u

/* ══════════════════════════════════════════════════════════════════
 * 枚举
 * ══════════════════════════════════════════════════════════════════ */

/** 设备状态机 */
typedef enum {
    STATE_IDLE      = 0,
    STATE_ARMED     = 1,
    STATE_STREAMING = 2,
    STATE_DONE      = 3,
    STATE_FAULT     = 4,
} scope_state_t;

/** 触发模式 */
typedef enum {
    TRIG_MODE_AUTO   = 0,   /* 约 200 ms 无触发后强制完成，避免 Agent 永久阻塞 */
    TRIG_MODE_NORMAL = 1,
    TRIG_MODE_SINGLE = 2,
} trig_mode_t;

/** 触发源 */
typedef enum {
    TRIG_SRC_CH1  = 0,
    TRIG_SRC_CH2  = 1,
    TRIG_SRC_SOFT = 2,
} trig_src_t;

/** 触发边沿 */
typedef enum {
    TRIG_EDGE_RISING  = 0,
    TRIG_EDGE_FALLING = 1,
} trig_edge_t;

/** 耦合 */
typedef enum {
    COUPLING_DC = 0,
    COUPLING_AC = 1,
} coupling_t;

/** 采集模式 */
typedef enum {
    ACQ_MODE_SINGLE   = 0,  /* 单次缓存：触发后冻结，主机随机拉取 */
    ACQ_MODE_STREAM   = 1,  /* 连续流：按 format/decimation 持续推分片 */
} acq_mode_t;

/** 波形数据格式 */
typedef enum {
    FMT_RAW16  = 0,   /* 2 B/样点，4 字节对齐小端 u16 */
    FMT_PACK12 = 1,   /* 2 样点 → 3 B，省 25% */
    FMT_MINMAX = 2,   /* 每桶 D 个样点 → (min,max) u16 对，共 4 B。显示用 */
    /* 预留 FMT_DELTA = 3：64 样点块 {s0:u16, w:u4, 63 个 zigzag 增量按 w 位打包} */
} wf_format_t;

/** 分片头 flags */
#define CHUNK_FLAG_LAST       (1u << 0)
#define CHUNK_FLAG_INVALID    (1u << 1)

/** 测量 quality 位 */
#define MEAS_Q_CLIPPED        (1u << 0)  /* 削顶 */
#define MEAS_Q_LOW_SNR        (1u << 1)  /* 信噪比低 */
#define MEAS_Q_FEW_EDGES      (1u << 2)  /* 有效沿太少 */

/** GET_INFO.caps 位掩码 */
#define CAP_CH_DUAL           (1u << 0)  /* 支持双通道同步采样 */
#define CAP_TRIG_AUTO         (1u << 1)
#define CAP_TRIG_NORMAL       (1u << 2)
#define CAP_TRIG_SINGLE       (1u << 3)
#define CAP_TRIG_SOFT         (1u << 4)
#define CAP_COUPLING_AC       (1u << 5)
#define CAP_FMT_PACK12        (1u << 6)
#define CAP_FMT_MINMAX        (1u << 7)
#define CAP_MEAS_ON_DEVICE    (1u << 8)
#define CAP_DIGITAL_I2C       (1u << 9)  /* 支持数字通路 I2C 解码 */

/* ══════════════════════════════════════════════════════════════════
 * 帧结构
 * ══════════════════════════════════════════════════════════════════ */

#pragma pack(push, 1)

/** 线协议帧头 —— 与内存布局严格对应，偏移由 _Static_assert 锁定 */
typedef struct {
    uint8_t  sync0;     /*  0: 0xAA */
    uint8_t  sync1;     /*  1: 0x55 */
    uint8_t  ver;       /*  2: 0x10 */
    uint8_t  flags;     /*  3 */
    uint16_t seq;       /*  4: 小端 */
    uint16_t cmd;       /*  6: 小端 */
    uint16_t len;       /*  8: 小端，payload 字节数 */
} proto_header_t;       /* 10 字节 */

/* ── 5.1 系统类 ─────────────────────────────────────────────── */

typedef struct {
    uint8_t  proto_ver;              /* 0x10 */
    uint32_t fw_ver;
    uint16_t model;
    uint8_t  uid[12];
    uint32_t tick_hz;                /* 1000000 */
    uint8_t  adc_bits;               /* 12 */
    uint8_t  ch_count;               /* F103 立创板：1 或 2 */
    uint32_t rate_min_hz;
    uint32_t rate_max_hz;            /* 857142 */
    uint16_t capture_max_samples;    /* 4096 */
    uint16_t max_rx_payload;         /* 512 */
    uint16_t max_tx_payload;         /* 2060 = 12 分片头 + 1024 样点 × u16 */
    uint16_t preferred_chunk_samples;/* 1024 */
    uint32_t caps;
} info_resp_t;                       /* 45 字节 */

typedef struct {
    uint8_t  state;                  /* scope_state_t */
    uint16_t err_flags;
    uint16_t ring_fill_samples;
    uint16_t last_capture_id;
    uint32_t last_trigger_index;     /* 无触发 = 0xFFFFFFFF */
    uint32_t overrun_samples;
    uint16_t rx_crc_err;
    uint16_t rx_dropped;
    uint32_t tx_dropped;
    uint32_t uptime_ms;
    uint32_t tick_us;
    uint16_t last_error_code;
} status_resp_t;                     /* 33 字节 */

typedef struct {
    uint8_t magic;                   /* 必须 = 0xA5 */
} reset_req_t;

/* ── 5.2 配置类 ─────────────────────────────────────────────── */

typedef struct {
    uint32_t requested_hz;
} set_rate_req_t;

typedef struct {
    uint32_t actual_hz;              /* 量化后的实际值，主机以此建时间轴 */
} set_rate_resp_t;

typedef struct {
    uint8_t  ch;                     /* 0 起 */
    uint8_t  enable;
    uint8_t  range_idx;
    uint8_t  coupling;               /* coupling_t */
    int16_t  offset_lsb;             /* ADC LSB 整数，非伏特 */
} set_channel_req_t;                 /* 6 字节 */

typedef struct {
    uint8_t  mode;                   /* trig_mode_t */
    uint8_t  source;                 /* trig_src_t */
    uint8_t  edge;                   /* trig_edge_t */
    uint16_t level_lsb;              /* 0..4095，比较带 ±16 LSB 迟滞 */
    uint16_t pre_samples;
    uint32_t holdoff_us;
} set_trigger_req_t;                 /* 11 字节 */

typedef struct {
    uint8_t  mode;                   /* acq_mode_t */
    uint16_t capture_samples;        /* ≤ 4096 */
    uint8_t  format;                 /* wf_format_t */
    uint16_t decimation;             /* 1..256 */
} set_acq_req_t;                     /* 6 字节 */

/* ── 5.4 读取类 ─────────────────────────────────────────────── */

typedef struct {
    uint16_t capture_id;
    uint32_t start_sample;           /* 绝对样点号 */
    uint16_t count;                  /* RAW16/PACK12 ≤1024 样点；MINMAX ≤512 对 */
    uint8_t  format;
    uint8_t  ch;
} read_buffer_req_t;                 /* 10 字节 */

typedef struct {
    uint16_t capture_id;
    uint32_t start_sample;
    uint16_t count;
    uint16_t decimation;
    uint8_t  format;
    uint8_t  flags;                  /* CHUNK_FLAG_* */
} chunk_header_t;                    /* 12 字节 */

/* ── 5.5 测量类 ─────────────────────────────────────────────── */

typedef struct {
    uint16_t capture_id;
    uint32_t start_sample;
    uint32_t window_samples;
    uint32_t md_mask;
} measure_req_t;                     /* 14 字节 */

typedef struct {
    uint16_t min_lsb;
    uint16_t max_lsb;
    uint16_t pp_lsb;
    int32_t  mean_x256;
    uint32_t rms_x256;
    uint16_t rising;
    uint16_t falling;
    uint32_t period_samples_x256;
    uint16_t duty_x10000;
    uint16_t rise_time_samples;
    uint16_t fall_time_samples;
    uint16_t quality;                /* MEAS_Q_* */
} measure_resp_t;                    /* 30 字节 */

/* ── 5.6 事件 ───────────────────────────────────────────────── */

typedef struct {
    uint16_t capture_id;
    uint32_t trigger_index;          /* 捕获窗内相对样点 */
    uint32_t tick_us;
    uint32_t rate_hz;
    uint32_t n_samples;
} event_trigger_t;                   /* 18 字节 */

typedef struct {
    uint32_t dropped_samples;
    uint16_t capture_id;
} event_overrun_t;                   /* 6 字节 */

/* ── 5.7 错误 ───────────────────────────────────────────────── */

typedef struct {
    uint16_t code;
    uint8_t  severity;               /* ERR_SEV_* */
    uint8_t  msg_len;                /* ≤ 32 */
    char     msg[32];                /* 非 NUL 结尾，长度由 msg_len 决定 */
} error_resp_t;                      /* 36 字节 */

#pragma pack(pop)

/* ── 布局断言：任何编译器/平台都不能改变线上的字节布局 ────────── */

/* ── 编译期断言 ──────────────────────────────────────────────────
 * `_Static_assert` 是 **C11** 的。Keil MDK5 用的 ARMCC5 **没有 C11 模式**
 *（只有 C90/C99，工程里配的是 `uC99=1`），所以在固件工程里编不过。
 *
 * 下面给一个 C99 等价物：条件为假时数组大小为负，编译期立刻报错。
 * 效果一样；只是错误信息里没有那句 msg（C99 没地方放它）。
 *
 * ⚠ 回归（2026-10-05）：把 `proto/` 加进 Keil 的包含路径后**一次炸出 30 个错**，
 *   全在这个宏上。注意它影响的不只是新代码 —— `App/proto_task.c` 也包含本
 *   头文件，所以整条「固件 ↔ 协议」的路都靠它。
 *
 * ⚠ 检查方法：**`gcc -std=c99` 抓不到**（gcc 把 `_Static_assert` 当扩展收下了）。
 *   必须带 `-pedantic-errors` 才等价于 ARMCC5 的严格 C99。CI 里就是这么跑的。 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#  define PROTO_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#  define PROTO_SA_CAT_(a, b) a##b
#  define PROTO_SA_CAT(a, b)  PROTO_SA_CAT_(a, b)
#  define PROTO_STATIC_ASSERT(cond, msg)        typedef char PROTO_SA_CAT(proto_sa_, __LINE__)[(cond) ? 1 : -1]
#endif

PROTO_STATIC_ASSERT(sizeof(proto_header_t)    == 10, "proto_header_t must be 10 bytes");
PROTO_STATIC_ASSERT(offsetof(proto_header_t, sync0) == 0, "sync0 offset");
PROTO_STATIC_ASSERT(offsetof(proto_header_t, ver)   == 2, "ver offset");
PROTO_STATIC_ASSERT(offsetof(proto_header_t, flags) == 3, "flags offset");
PROTO_STATIC_ASSERT(offsetof(proto_header_t, seq)   == 4, "seq offset");
PROTO_STATIC_ASSERT(offsetof(proto_header_t, cmd)   == 6, "cmd offset");
PROTO_STATIC_ASSERT(offsetof(proto_header_t, len)   == 8, "len offset");

PROTO_STATIC_ASSERT(sizeof(chunk_header_t)    == 12, "chunk_header_t must be 12 bytes");
PROTO_STATIC_ASSERT(sizeof(read_buffer_req_t) == 10, "read_buffer_req_t must be 10 bytes");
PROTO_STATIC_ASSERT(sizeof(set_trigger_req_t) == 11, "set_trigger_req_t must be 11 bytes");
PROTO_STATIC_ASSERT(sizeof(set_channel_req_t) ==  6, "set_channel_req_t must be 6 bytes");
PROTO_STATIC_ASSERT(sizeof(set_acq_req_t)     ==  6, "set_acq_req_t must be 6 bytes");
PROTO_STATIC_ASSERT(sizeof(reset_req_t)       ==  1, "reset_req_t must be 1 byte");
PROTO_STATIC_ASSERT(sizeof(measure_req_t)     == 14, "measure_req_t must be 14 bytes");

/* 响应体布局：Rust 端 DeviceInfo::decode / measure 解析按这些偏移写死，
   任何改动都会让两端静默错位 —— 所以这里必须锁死 */
PROTO_STATIC_ASSERT(sizeof(info_resp_t)       == 45, "info_resp_t must be 45 bytes");
PROTO_STATIC_ASSERT(sizeof(status_resp_t)     == 33, "status_resp_t must be 33 bytes");
PROTO_STATIC_ASSERT(sizeof(measure_resp_t)    == 30, "measure_resp_t must be 30 bytes");
PROTO_STATIC_ASSERT(sizeof(event_trigger_t)   == 18, "event_trigger_t must be 18 bytes");
PROTO_STATIC_ASSERT(sizeof(event_overrun_t)   ==  6, "event_overrun_t must be 6 bytes");
PROTO_STATIC_ASSERT(sizeof(error_resp_t)      == 36, "error_resp_t must be 36 bytes");
PROTO_STATIC_ASSERT(offsetof(info_resp_t, uid)       ==  7, "uid offset");
PROTO_STATIC_ASSERT(offsetof(info_resp_t, rate_max_hz) == 29, "rate_max_hz offset");
PROTO_STATIC_ASSERT(offsetof(info_resp_t, caps)      == 41, "caps offset");

/* ══════════════════════════════════════════════════════════════════
 * API
 * ══════════════════════════════════════════════════════════════════ */

/** 解析结果 */
typedef enum {
    PROTO_OK           =  0,   /* 解析出一个完整帧 */
    PROTO_NEED_MORE    =  1,   /* 数据不足，等更多字节 */
    PROTO_CRC_ERR      =  2,   /* CRC 校验失败（已跳过一个字节重同步） */
    PROTO_BAD_LEN      =  3,   /* LEN 越界（已跳过） */
    PROTO_BAD_VER      =  4,   /* 主版本不符 */
} proto_status_t;

/** 已解析的帧（指向调用者的缓冲区，不拷贝） */
typedef struct {
    proto_header_t hdr;
    const uint8_t *payload;    /* 指向输入缓冲区内部，长度 = hdr.len */
} proto_frame_t;

/** 增量解析器状态 —— 调用者负责持有 */
typedef struct {
    uint8_t  buf[PROTO_PARSER_BUF_SIZE];
    uint16_t len;              /* 已缓存字节数 */
    /**
     * 上一次成功返回的帧长度。
     * 不立即从 buf 移除，是为了让 out->payload 在返回后仍然有效；
     * 直到下次 proto_parser_feed() 调用时才惰性消费。
     */
    uint16_t pending;
    uint32_t crc_err_count;
    uint32_t dropped_count;
} proto_parser_t;

/**
 * @brief CRC-16/CCITT-FALSE
 * @note  F103 硬件 CRC 只支持固定 CRC-32 多项式，做不了这个，用查表软件实现
 */
uint16_t proto_crc16(const uint8_t *data, size_t len);

/** 初始化解析器 */
void proto_parser_init(proto_parser_t *p);

/**
 * @brief 增量喂入数据并尝试解析出一个完整帧
 *
 * 内部维护字节流状态机，处理粘包/拆包：
 *   - 一次喂入可以是半个帧、一个帧或多个半帧
 *   - 出错时以 +1 字节步进扫描 0xAA 0x55 重同步（不是按帧长跳过）
 *   - 解析成功后从缓冲区移出该帧，剩余数据留给下次
 *
 * @param p       解析器
 * @param data    新到达的字节
 * @param len     data 长度
 * @param out     解析成功时填充（payload 指向 p->buf 内部）
 * @return PROTO_OK / NEED_MORE / CRC_ERR / BAD_LEN / BAD_VER
 *
 * @warning 一次调用最多返回一个帧。若一次喂入多帧，需循环调用直到 NEED_MORE。
 * @warning out->payload 在下次调用本函数后失效。
 */
proto_status_t proto_parser_feed(proto_parser_t *p,
                                 const uint8_t *data, size_t len,
                                 proto_frame_t *out);

/**
 * @brief 编码一个完整帧到 out
 *
 * @param out      输出缓冲，至少需要 PROTO_HEADER_LEN + payload_len + PROTO_CRC_LEN 字节
 * @param out_cap  输出缓冲容量
 * @param flags    0 表示请求帧（无 RESP/ERROR）
 * @param seq      序号
 * @param cmd      命令码
 * @param payload  可为 NULL（payload_len = 0）
 * @param payload_len 必须 ≤ PROTO_MAX_PAYLOAD_TX
 * @param out_len  实际写出的字节数
 * @return PROTO_OK / BAD_LEN（缓冲不足或 payload 超限）
 */
proto_status_t proto_encode(uint8_t *out, size_t out_cap,
                            uint8_t flags, uint16_t seq, uint16_t cmd,
                            const uint8_t *payload, uint16_t payload_len,
                            size_t *out_len);

/**
 * @brief 便捷构造：错误响应帧
 */
proto_status_t proto_encode_error(uint8_t *out, size_t out_cap,
                                  uint16_t seq, uint16_t hash_cmd,
                                  uint16_t code, uint8_t severity,
                                  const char *msg, uint8_t msg_len,
                                  size_t *out_len);

/**
 * @brief PACK12 编码：2 个 12-bit 样点 → 3 字节
 *        b0 = s0 低 8 位
 *        b1 = (s0 >> 8) & 0x0F | (s1 & 0x0F) << 4
 *        b2 = s1 >> 4
 */
void proto_pack12_pair(uint16_t s0, uint16_t s1, uint8_t out[3]);

/** PACK12 解码 */
void proto_unpack12_pair(const uint8_t in[3], uint16_t *s0, uint16_t *s1);

/**
 * @brief 状态机：该状态下是否允许执行该命令
 * @return true = 允许
 */
bool proto_cmd_allowed(scope_state_t state, uint16_t cmd);


/**
 * @brief 采样率量化：把请求值吸附到定时器真正能达到的档位
 *
 * **这条规则三端共用** —— 固件用它决定 ARR，上位机用它生成档位菜单，
 * 模拟器用它回显 `actual_hz`。放在协议层就是为了只有一份：
 * 各写一份的话，同一个请求在模拟器和真机上会得到不同的档位，而没人会发现。
 *
 * 规则：
 *   - 采样周期 = 72 MHz / requested，四舍五入到整数 `ARR+1`（范围 1..65536）
 *   - 再把周期 ±1 比一遍，取误差小的；**误差相等时取更快的那档**
 *   - 吸附之后仍超出 `PROTO_RATE_MAX_HZ` → 返回 0（**达不到**，不静默给个别的档位）
 *   - `requested == 0` 或已超上限 → 返回 0
 *
 * ⚠ **与上位机的 `nearest_achievable_rate` 有一处刻意的不同**：那个函数
 *   不做能力门控（超上限就返回超上限的档位），因为它的调用方 —— MCP 的
 *   schema —— 已经用 `range(max=...)` 拦住了。而固件这边必须自己拦：
 *   `App/acq.c` 的原则是「硬件说达不到就说达不到」，**不静默吸附**。
 *   所以这里多一道 `> PROTO_RATE_MAX_HZ → 0`。
 *   两者在**上限以内逐条一致**（已对拍过）。
 *
 * @return 实际能达到的采样率（Hz）；0 表示这个请求做不到
 */
uint32_t proto_quantize_rate_hz(uint32_t requested_hz);

#ifdef __cplusplus
}
#endif

#endif /* SCOPE_PROTOCOL_H */
