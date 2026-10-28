/* Hardware/src/local_gen.c —— 见 local_gen.h 的说明。 */

#include "local_gen.h"

#include "main.h"
#include "tim.h"

/* TIM2 的计数频率：PSC=71 → 72 MHz / 72 = 1 MHz。
 * 与 `Core/Src/tim.c` 的 `MX_TIM2_Init` 对应，改那边就要改这里。 */
#define GEN_TICK_HZ 1000000u

static uint32_t s_arr;        /* 自动重装值（+1 = 一个周期的 tick 数） */
static uint16_t s_duty;       /* 千分比 */
static bool     s_enabled;

/* 把 ARR 与 CCR 一起写下去。
 *
 * 两个必须**一起**更新：先改 ARR 再改 CCR 的话，中间那一小会儿占空比是错的
 * —— 对一块被测的电路来说，那是它真的收到过的一个波形。 */
static void apply(void)
{
    uint32_t ccr = ((uint32_t)s_arr * (uint32_t)s_duty) / 1000u;

    if (ccr > s_arr) {
        ccr = s_arr;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)(s_arr - 1u));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr);
}

void LocalGen_Init(void)
{
    /* 上电默认：1 kHz、25%、**关**。
     * 与上游 `Init_Oscilloscope` 的 `pwmOut=500`（50%）不同：真机验证需要
     * 一个非半占空的波形，让「占空比测量」能和「100−duty 的反向误判」
     * 区分开（见 sim/waveform.rs 的变异体注释）。频率仍与上游一致为 1 kHz。 */
    s_duty = 250u;
    s_enabled = false;

    s_arr = GEN_TICK_HZ / 1000u;   /* 1 kHz */
    apply();

    /* 不启动。见 local_gen.h：`CC3E` 被 HAL 清掉了，PA2 此刻不输出。 */
}

void LocalGen_SetEnabled(bool on)
{
    if (on == s_enabled) {
        return;
    }

    if (on) {
        /* 先把参数写下去再开通道 —— 反过来的话，会先输出一小段旧参数
         * 的波形。 */
        apply();
        (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    } else {
        (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }

    s_enabled = on;
}

bool LocalGen_IsEnabled(void)
{
    return s_enabled;
}

uint32_t LocalGen_SetHz(uint32_t hz)
{
    uint32_t arr;
    uint32_t actual;

    if (hz < LOCAL_GEN_MIN_HZ || hz > LOCAL_GEN_MAX_HZ) {
        /* **达不到就报达不到**，不吸附到边界 —— 请求 600 kHz 却得到
         * 500 kHz，调用方会把那 500 kHz 当成真的。 */
        return 0u;
    }

    /* 四舍五入到最近的整数分频，与 `hal.h` 的 `quantize_rate_hz` 同一套语义。 */
    arr = (GEN_TICK_HZ + hz / 2u) / hz;
    if (arr < 2u) {
        arr = 2u;
    }
    if (arr > 65536u) {
        return 0u;
    }

    s_arr = arr;
    actual = GEN_TICK_HZ / arr;

    /* 已经开着的话立刻生效；没开就只记下来，等开的时候 apply。 */
    if (s_enabled) {
        apply();
    }

    return actual;
}

uint32_t LocalGen_GetHz(void)
{
    return GEN_TICK_HZ / s_arr;
}

void LocalGen_SetDutyPermille(uint16_t permille)
{
    if (permille < LOCAL_GEN_DUTY_MIN_PERMILLE) {
        permille = LOCAL_GEN_DUTY_MIN_PERMILLE;
    }
    if (permille > LOCAL_GEN_DUTY_MAX_PERMILLE) {
        permille = LOCAL_GEN_DUTY_MAX_PERMILLE;
    }

    s_duty = permille;

    if (s_enabled) {
        apply();
    }
}

uint16_t LocalGen_GetDutyPermille(void)
{
    return s_duty;
}
