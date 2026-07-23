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
#include "drivers/button.h"
#include "drivers/gray_array.h"

#define GRAY_CALIBRATION_FRAMES (16U)
#define GRAY_BLACK_THRESHOLD    (500U)

static GrayArray gGrayDebugArray;
static volatile OLED_Status gGrayDebugOledStatus =
    OLED_STATUS_ERROR_NOT_INITIALIZED;
static volatile uint32_t gGrayDebugFrame;

typedef enum {
    GRAY_CAL_WHITE_WAIT = 0,
    GRAY_CAL_BLACK_WAIT,
    GRAY_CAL_RUNNING,
    GRAY_CAL_ERROR,
} GrayCalibrationState;

static bool ReadStartButton(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadStartButtonLevel();
}

static OLED_Status DisplayCalibrationMessage(const char *message,
                                             const char *detail)
{
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 1U, message);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 3U, detail);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static OLED_Status DisplayGrayValues(const GrayArray *gray,
                                     bool adc_ok,
                                     uint32_t frame)
{
    char line[22];
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 0U, "GRAY ADC NORM");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%u:%4u %u:%4u",
                       (uint8_t)(first + 1U), gray->latest.normalized[first],
                       (uint8_t)(first + 2U),
                       gray->latest.normalized[first + 1U]);
        status = OLED_ShowString(0U, (uint8_t)(row + 1U), line);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }
    (void)snprintf(line, sizeof(line),
                   adc_ok ? "ADC OK F:%lu" : "ADC ERR F:%lu",
                   (unsigned long)frame);
    status = OLED_ShowString(0U, 5U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "1:%c 2:%c 3:%c 4:%c",
                   (gray->latest.normalized[0] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W',
                   (gray->latest.normalized[1] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W',
                   (gray->latest.normalized[2] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W',
                   (gray->latest.normalized[3] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W');
    status = OLED_ShowString(0U, 6U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "5:%c 6:%c 7:%c 8:%c",
                   (gray->latest.normalized[4] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W',
                   (gray->latest.normalized[5] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W',
                   (gray->latest.normalized[6] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W',
                   (gray->latest.normalized[7] >= GRAY_BLACK_THRESHOLD) ? 'B' : 'W');
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static bool CaptureCalibrationSurface(GrayArray *gray,
                                      uint16_t output[GRAY_ARRAY_CHANNELS],
                                      const char *label)
{
    uint32_t sums[GRAY_ARRAY_CHANNELS] = {0U};

    for (uint8_t sample = 0U; sample < GRAY_CALIBRATION_FRAMES; sample++) {
        if (!GrayArray_Read(gray, TiMspm0Platform_Millis())) {
            return false;
        }
        for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
            sums[channel] += gray->raw[channel];
        }
        (void)DisplayCalibrationMessage(label, "SAMPLING...");
        DelayMs(10U);
    }
    for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
        output[channel] = (uint16_t)(sums[channel] / GRAY_CALIBRATION_FRAMES);
    }
    return true;
}

static void RunGrayAdcDebug(void)
{
    CarFirmwareConfig config;
    OLED_Config oled_config;
    Button button;
    GrayCalibrationState state = GRAY_CAL_WHITE_WAIT;
    uint16_t white[GRAY_ARRAY_CHANNELS] = {0U};
    uint16_t black[GRAY_ARRAY_CHANNELS] = {0U};

    TiMspm0Platform_Init();
    if (TiMspm0Platform_BuildConfig(&config, H2024_MODE_ITEM_1) != CAR_OK) {
        while (1) {
            __WFI();
        }
    }
    GrayArray_Init(&gGrayDebugArray, &config.gray);
    Button_Init(&button, ReadStartButton, 0, true, 30U);

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
        Button_Update(&button, TiMspm0Platform_Millis());
        if (Button_TakePressedEvent(&button)) {
            if (state == GRAY_CAL_WHITE_WAIT) {
                state = CaptureCalibrationSurface(
                    &gGrayDebugArray, white, "WHITE") ?
                    GRAY_CAL_BLACK_WAIT : GRAY_CAL_ERROR;
            } else if (state == GRAY_CAL_BLACK_WAIT) {
                if (CaptureCalibrationSurface(
                        &gGrayDebugArray, black, "BLACK") &&
                    GrayArray_SetCalibration(&gGrayDebugArray, black, white)) {
                    state = GRAY_CAL_RUNNING;
                } else {
                    state = GRAY_CAL_ERROR;
                }
            } else if (state == GRAY_CAL_ERROR) {
                state = GRAY_CAL_WHITE_WAIT;
            }
        }
        if (OLED_IsInitialized()) {
            if (state == GRAY_CAL_WHITE_WAIT) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "PUT WHITE", "PRESS START");
            } else if (state == GRAY_CAL_BLACK_WAIT) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "PUT BLACK", "PRESS START");
            } else if (state == GRAY_CAL_ERROR) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "CAL ERROR", "PRESS TO RETRY");
            } else {
                gGrayDebugOledStatus = DisplayGrayValues(
                    &gGrayDebugArray, adc_ok, gGrayDebugFrame);
            }
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
