/* App/freq_meter.c —— 见 freq_meter.h 的说明。 */

#include "freq_meter.h"

/* 两个捕获值之间隔了多少个 tick。
 *
 * 计数器是自由运行的（0..ARR 循环），所以 `b < a` 时跨过一次回绕 ——
 * 补一个 `ARR + 1`。**超过一次回绕的情况这里分辨不出来**，那正是
 * 15.26 Hz 下限的来源：应报告"测不出"，不得输出虚构的周期。 */
static uint32_t tick_delta(uint32_t a, uint32_t b)
{
    if (b >= a) {
        return b - a;
    }
    return (FREQ_ARR + 1u - a) + b;
}

void freq_meter_reset(freq_meter_t *m)
{
    m->first_tick = 0u;
    m->last_tick = 0u;
    m->edges = 0u;
    m->have_first = false;
}

freq_result_t freq_meter_feed(freq_meter_t *m, uint32_t tick, uint32_t *out_hz)
{
    if (!m->have_first) {
        m->first_tick = tick;
        m->last_tick = tick;
        m->edges = 1u;
        m->have_first = true;
        return FREQ_PENDING;
    }

    /* 同一个 tick 上的两个边沿 —— 计时器的分辨率到头了。
     *
     * **整段丢掉、从下一个边沿重新起**，而不是拿当前这个 tick 当新起点：
     * 它很可能就是那对分不开的边沿里的**第二个**，用它当起点会把一个
     * 假的短间隔算进下一段里。回归：`tests/test_freq_meter.c` 的
     * 「超出量程」那组就是这么抓到的（1000 Hz 被算成 1219 Hz）。 */
    if (tick == m->last_tick) {
        freq_meter_reset(m);
        return FREQ_TOO_FAST;
    }

    m->last_tick = tick;
    m->edges++;

    {
        uint32_t span = tick_delta(m->first_tick, m->last_tick);

        if (span >= FREQ_MIN_SPAN_TICKS || m->edges >= FREQ_MAX_EDGES) {
            /* `edges - 1` 个整周期跨了 `span` 个 tick。
             * 中间量必须 64 位：32 个周期 × 1e6 会溢出 32 位。 */
            if (span == 0u) {
                /* 到不了这里（上面已经挡住了同一个 tick），但除法前
                 * 显式判一次 —— 这个仓库栽过"以为不会发生"的跟头。 */
                freq_meter_reset(m);
                return FREQ_PENDING;
            }
            *out_hz = (uint32_t)(((unsigned long long)(m->edges - 1u) *
                                  (unsigned long long)FREQ_TICK_HZ) / (unsigned long long)span);
            freq_meter_reset(m);
            return FREQ_OK;
        }
    }

    return FREQ_PENDING;
}
