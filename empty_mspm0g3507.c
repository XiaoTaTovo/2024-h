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

#include <stdio.h>

#include "OLED.h"
#include "drivers/gray_array.h"

static GrayArray gGrayDebugArray;
static volatile OLED_Status gGrayDebugOledStatus =
    OLED_STATUS_ERROR_NOT_INITIALIZED;
static volatile uint32_t gGrayDebugFrame;

static OLED_Status DisplayGrayRaw(const GrayArray *gray,
                                  bool adc_ok,
                                  uint32_t frame)
{
    char line[22];
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 0U, "GRAY ADC RAW");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%u:%4u %u:%4u",
                       first, gray->raw[first],
                       (uint8_t)(first + 1U), gray->raw[first + 1U]);
        status = OLED_ShowString(0U, (uint8_t)(row + 1U), line);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }
    (void)snprintf(
        line, sizeof(line), "EN:%u ERR:%u",
        (DL_GPIO_readPins(GPIO_GRAY_EN_PORT, GPIO_GRAY_EN_PIN) != 0U) ? 1U : 0U,
        (DL_GPIO_readPins(GPIO_GRAY_ERR_PORT, GPIO_GRAY_ERR_PIN) != 0U) ? 1U : 0U);
    status = OLED_ShowString(0U, 6U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line),
                   adc_ok ? "ADC OK F:%lu" : "ADC ERR F:%lu",
                   (unsigned long)frame);
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static void RunGrayAdcDebug(void)
{
    CarFirmwareConfig config;
    OLED_Config oled_config;

    TiMspm0Platform_Init();
    if (TiMspm0Platform_BuildConfig(&config, H2024_MODE_ITEM_1) != CAR_OK) {
        while (1) {
            __WFI();
        }
    }
    GrayArray_Init(&gGrayDebugArray, &config.gray);

    oled_config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);
    gGrayDebugOledStatus = OLED_Init(&oled_config);
    if (gGrayDebugOledStatus == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        oled_config = OLED_MakeSSD1306Config(0x3DU);
        gGrayDebugOledStatus = OLED_Init(&oled_config);
    }

    while (1) {
        bool adc_ok = GrayArray_Read(
            &gGrayDebugArray, TiMspm0Platform_Millis());

        gGrayDebugFrame++;
        if (OLED_IsInitialized()) {
            gGrayDebugOledStatus = DisplayGrayRaw(
                &gGrayDebugArray, adc_ok, gGrayDebugFrame);
        }
        DelayMs(100U);
    }
}

#elif PROJECT_MODE == PROJECT_MODE_BLUETOOTH_TUNING

#include "bluetooth_control.h"
#include "encoder.h"
#include "tb6612.h"
#include "vofa_telemetry.h"

#define TELEMETRY_PERIOD_MS (100U)

static void RunBluetoothTuning(void)
{
    TB6612_Init();

    DelayMs(2000);

    TB6612_SetMotors(0, 20);  // 只测试左轮逻辑正方向
    DelayMs(1000);

    TB6612_Stop();

    while (1) {
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
    RunGrayAdcDebug();
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
