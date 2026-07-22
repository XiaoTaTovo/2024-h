//线数就是一个齿，一个齿就是一个脉冲，最后输出两路相差90的信号去计数，然后我们要知道的是每一转的计数，这通常是线数乘以倍频数，2.看b是高电平，那就是反转，看b是低电平，那就是正转，但是实测可能有差距，就需要置标志位来补齐
//脉冲就是电平高低变化的一瞬间
//电机轴和输出轴
//我再说一下那个就是每转一圈的编码器计数怎么算首先要分电机轴和输出轴，电机轴就是一个线束对应一个这个脉冲然后他在里面对应传啊一圈就是要乘以那个减速就是比如说是一比28就相当于里面赚28圈外面才转一圈所以外面转了一圈里面对应的脉冲数也要长28乘以28之后要再乘以它的被评但是我其实对成被评这件事情不太清楚反正就是成吗然后左右现在我都配置的是二倍体就被频率高它分辨率越高它pid调参进行更细致是吧
#include "encoder.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

volatile int32_t gEncoderLeftCount;
volatile int32_t gEncoderRightCount;

static int32_t gLeftDeltaPrevious;
static int32_t gRightDeltaPrevious;

static int32_t count_delta(int32_t current, int32_t *previous)
{
    int32_t old = *previous;
    *previous = current;

    /* Unsigned subtraction keeps the result defined if the counter wraps. */
    return (int32_t) ((uint32_t) current - (uint32_t) old);
}

void Encoder_Init(void)
{
    NVIC_DisableIRQ(ENC_A_INT_IRQN);
    gEncoderLeftCount   = 0;
    gEncoderRightCount  = 0;
    gLeftDeltaPrevious  = 0;
    gRightDeltaPrevious = 0;

    DL_GPIO_clearInterruptStatus(
        ENC_A_PORT, ENC_A_PIN_AL_PIN | ENC_A_PIN_AR_PIN);
    NVIC_ClearPendingIRQ(ENC_A_INT_IRQN);
    NVIC_EnableIRQ(ENC_A_INT_IRQN);
}

void Encoder_ResetCounts(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    gEncoderLeftCount   = 0;
    gEncoderRightCount  = 0;
    gLeftDeltaPrevious  = 0;
    gRightDeltaPrevious = 0;
    if (interruptState == 0U) {
        __enable_irq();
    }
}

int32_t Encoder_GetLeftCount(void)
{
    return gEncoderLeftCount;
}

int32_t Encoder_GetRightCount(void)
{
    return gEncoderRightCount;
}

int32_t Encoder_GetLeftDelta(void)
{
    return count_delta(gEncoderLeftCount, &gLeftDeltaPrevious);
}

int32_t Encoder_GetRightDelta(void)
{
    return count_delta(gEncoderRightCount, &gRightDeltaPrevious);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        ENC_A_PORT, ENC_A_PIN_AL_PIN | ENC_A_PIN_AR_PIN);

    if ((pending & ENC_A_PIN_AL_PIN) != 0U) {
        bool aHigh;
        bool bHigh;

        DL_GPIO_clearInterruptStatus(ENC_A_PORT, ENC_A_PIN_AL_PIN);
        aHigh = DL_GPIO_readPins(ENC_A_PORT, ENC_A_PIN_AL_PIN) != 0U;
        bHigh = DL_GPIO_readPins(ENC_B_PORT, ENC_B_PIN_BL_PIN) != 0U;
        gEncoderLeftCount += (aHigh != bHigh) ? ENC_LEFT_SIGN : -ENC_LEFT_SIGN;
    }

    if ((pending & ENC_A_PIN_AR_PIN) != 0U) {
        bool aHigh;
        bool bHigh;

        DL_GPIO_clearInterruptStatus(ENC_A_PORT, ENC_A_PIN_AR_PIN);
        aHigh = DL_GPIO_readPins(ENC_A_PORT, ENC_A_PIN_AR_PIN) != 0U;
        bHigh = DL_GPIO_readPins(ENC_B_PORT, ENC_B_PIN_BR_PIN) != 0U;
        gEncoderRightCount += (aHigh != bHigh) ? ENC_RIGHT_SIGN : -ENC_RIGHT_SIGN;
    }
}
