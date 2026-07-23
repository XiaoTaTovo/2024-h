#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "project_mode.h"
#include "platform/ti_mspm0_platform.h"

static void DelayMs(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        ms--;
    }
}

#if PROJECT_MODE == PROJECT_MODE_GRAY_ADC_DEBUG

#include "app/gray_adc_debug.h"

#elif PROJECT_MODE == PROJECT_MODE_BLUETOOTH_TUNING

#include "bluetooth_control.h"
#include "encoder.h"
#include "tb6612.h"
#include "vofa_telemetry.h"

#define TELEMETRY_PERIOD_MS (100U)

static void RunBluetoothTuning(void)
{
    uint32_t lastTelemetryMs = 0;

    TB6612_Init();
    Encoder_Init();
    BluetoothControl_Init();

    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);

    VofaTelemetry_SendBanner();

    while (1) {
        BluetoothControlStatus status;
        uint32_t nowMs = TiMspm0Platform_Millis();
        bool sendNow;

        sendNow = BluetoothControl_ProcessPending(nowMs);

        if (BluetoothControl_CheckFailsafe(nowMs)) {
            sendNow = true;
        }

        BluetoothControl_Update(nowMs);

        if (sendNow ||
            ((uint32_t)(nowMs - lastTelemetryMs) >=
                TELEMETRY_PERIOD_MS)) {
            BluetoothControl_GetStatus(&status);
            VofaTelemetry_Send(&status, nowMs);
            lastTelemetryMs = nowMs;
        }

        __WFI();
    }
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
                BluetoothControl_PushRxFromIsr(
                    (uint8_t) DL_UART_Main_receiveData(
                        UART_BLUETOOTH_INST));
            }
            break;
        default:
            break;
    }
}

#else

#include "firmware.h"

static CarFirmware gFirmware;

static H2024Mode GetH2024Mode(void)
{
#if PROJECT_MODE == PROJECT_MODE_H2024_ITEM_1
    return H2024_MODE_ITEM_1;
#elif PROJECT_MODE == PROJECT_MODE_H2024_ITEM_2
    return H2024_MODE_ITEM_2;
#elif PROJECT_MODE == PROJECT_MODE_H2024_ITEM_3
    return H2024_MODE_ITEM_3;
#else
    return H2024_MODE_ITEM_4;
#endif
}

static void RunH2024Firmware(void)
{
    CarFirmwareConfig config;
    CarStatus status;

    TiMspm0Platform_Init();
    status = TiMspm0Platform_BuildConfig(&config, GetH2024Mode());
    if (status == CAR_OK) {
        status = CarFirmware_Init(
            &gFirmware, &config, TiMspm0Platform_Millis());
    }
    if (status != CAR_OK) {
        while (1) {
            __WFI();
        }
    }

    while (1) {
        TiMspm0Platform_PollMotorRx(&gFirmware);
        CarFirmware_Tick(&gFirmware, TiMspm0Platform_Millis());
        __WFI();
    }
}

#endif

int main(void)
{
    SYSCFG_DL_init();
    DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

#if PROJECT_MODE == PROJECT_MODE_GRAY_ADC_DEBUG
    GrayAdcDebug_Run();
#elif PROJECT_MODE == PROJECT_MODE_BLUETOOTH_TUNING
    RunBluetoothTuning();
#else
    RunH2024Firmware();
#endif

    return 0;
}

void SysTick_Handler(void)
{
    TiMspm0Platform_OnSysTick();
}
