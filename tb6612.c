#include "tb6612.h"

#include "ti_msp_dl_config.h"

static int8_t gLeftCommand;
static int8_t gRightCommand;

static int8_t clamp_percent(int8_t percent)
{
    if (percent > (int8_t) TB6612_MAX_DUTY_PERCENT) {
        return (int8_t) TB6612_MAX_DUTY_PERCENT;
    }
    if (percent < -(int8_t) TB6612_MAX_DUTY_PERCENT) {
        return -(int8_t) TB6612_MAX_DUTY_PERCENT;
    }
    return percent;
}

static uint32_t percent_to_compare(int8_t percent)
{
    uint32_t magnitude = (percent < 0) ? (uint32_t) (-percent) : (uint32_t) percent;
    uint32_t activeTicks = (TB6612_PWM_PERIOD_TICKS * magnitude) / 100U;

    return TB6612_PWM_PERIOD_TICKS - activeTicks;
}//把占空比变成比较寄存器的值
//我们是递减计数，所以类似递增计数，数了多少个数字就用它来算占空比

static void set_pwm(int8_t leftPercent, int8_t rightPercent)
{
    DL_TimerA_setCaptureCompareValue(PWM_TB1_INST,
        percent_to_compare(leftPercent), GPIO_PWM_TB1_C1_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_TB1_INST,
        percent_to_compare(rightPercent), GPIO_PWM_TB1_C2_IDX);
}

static void set_left_direction(int8_t percent)
{//先都清零，clear函数是清零的：确保没有一瞬间都是1，这个叫刹车状态（电机两端短接，强行按住）会有卡顿和异响
    //然后根据我们这个left的值来确定正反转和我们要的前进方向的关系
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    if (percent == 0) {
        return;
    }

    if ((percent > 0) == (TB6612_LEFT_FORWARD_IN1_HIGH != 0)) {
        DL_GPIO_setPins(A_PORT, A_PIN_AIN1_PIN);
    } else {
        DL_GPIO_setPins(A_PORT, A_PIN_AIN2_PIN);
    }
}

static void set_right_direction(int8_t percent)
{
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    if (percent == 0) {
        return;
    }

    if ((percent > 0) == (TB6612_RIGHT_FORWARD_IN1_HIGH != 0)) {
        DL_GPIO_setPins(B_PORT, B_PIN_BIN1_PIN);
    } else {
        DL_GPIO_setPins(B_PORT, B_PIN_BIN2_PIN);
    }
}

void TB6612_Init(void)
{
    gLeftCommand  = 0;
    gRightCommand = 0;
    set_pwm(0, 0);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    DL_TimerA_startCounter(PWM_TB1_INST);
}

void TB6612_Stop(void)
{
    set_pwm(0, 0);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    gLeftCommand  = 0;
    gRightCommand = 0;
}

void TB6612_SetMotors(int8_t leftPercent, int8_t rightPercent)
{
    leftPercent  = clamp_percent(leftPercent);
    rightPercent = clamp_percent(rightPercent);

    if ((leftPercent == 0) && (rightPercent == 0)) {
        TB6612_Stop();
        return;
    }

    /* Blank both PWM channels before touching direction and STBY. */
    set_pwm(0, 0);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    set_left_direction(leftPercent);
    set_right_direction(rightPercent);
    DL_GPIO_setPins(STBY_PORT, STBY_PIN_STBY_PIN);
    delay_cycles(CPUCLK_FREQ / 1000000U);
    set_pwm(leftPercent, rightPercent);

    gLeftCommand  = leftPercent;
    gRightCommand = rightPercent;
}

int8_t TB6612_GetLeftCommand(void)
{
    return gLeftCommand;
}

int8_t TB6612_GetRightCommand(void)
{
    return gRightCommand;
}

