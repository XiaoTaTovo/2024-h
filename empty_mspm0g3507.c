/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include <stdint.h>

#include "bluetooth_control.h"
#include "tb6612.h"
#include "vofa_telemetry.h"

#define TELEMETRY_PERIOD_MS (100U)

static volatile uint32_t gUptimeMs;

static void Delay_ms(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        ms--;
    }
}

int main(void)
{
    uint32_t lastTelemetryMs = 0;

    SYSCFG_DL_init();
    TB6612_Init();
    BluetoothControl_Init();

    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    Delay_ms(2000);              // 上电后等待 2 秒
    TB6612_SetMotors(0, 20);     // 只让左轮以正方向 20% 转动
    Delay_ms(1000);              // 转动 1 秒
    TB6612_Stop(); 


    while (1) {
        
        __WFI();
    }
}

void SysTick_Handler(void)
{
    gUptimeMs++;
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
                BluetoothControl_PushRxFromIsr(
                    (uint8_t) DL_UART_Main_receiveData(UART_BLUETOOTH_INST));
            }
            break;
        default:
            break;
    }
}
