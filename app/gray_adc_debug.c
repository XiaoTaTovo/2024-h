#include "app/gray_adc_debug.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "OLED.h"
#include "drivers/button.h"
#include "drivers/gray_array.h"
#include "platform/ti_mspm0_platform.h"
#include "ti_msp_dl_config.h"

#define GRAY_CALIBRATION_FRAMES (16U)
#define GRAY_WHITE_THRESHOLD    (350U)
#define GRAY_BLACK_THRESHOLD    (650U)

static GrayArray gGrayDebugArray;
static OLED_Status gGrayDebugOledStatus = OLED_STATUS_ERROR_NOT_INITIALIZED;
static uint32_t gGrayDebugFrame;

typedef enum {
    GRAY_CAL_WHITE_WAIT = 0,
    GRAY_CAL_BLACK_WAIT,
    GRAY_CAL_RUNNING,
    GRAY_CAL_ERROR,
} GrayCalibrationState;

static void GrayAdcDebug_DelayMs(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        ms--;
    }
}

static bool ReadKey1(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey1Level();
}

static bool ReadKey2(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey2Level();
}

static bool ReadKey3(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey3Level();
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

static char GrayClassificationChar(
    const GrayArrayClassification *classification, uint8_t channel)
{
    uint8_t bit;

    if ((classification == 0) || (channel >= GRAY_ARRAY_CHANNELS)) {
        return '?';
    }
    bit = (uint8_t)(1U << channel);
    if ((classification->black_mask & bit) != 0U) {
        return 'B';
    }
    if ((classification->white_mask & bit) != 0U) {
        return 'W';
    }
    return '?';
}

static OLED_Status DisplayGrayValues(
    const GrayArray *gray,
    bool adc_ok,
    uint32_t frame,
    const GrayArrayClassification *classification)
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
                   GrayClassificationChar(classification, 0U),
                   GrayClassificationChar(classification, 1U),
                   GrayClassificationChar(classification, 2U),
                   GrayClassificationChar(classification, 3U));
    status = OLED_ShowString(0U, 6U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "5:%c 6:%c 7:%c 8:%c",
                   GrayClassificationChar(classification, 4U),
                   GrayClassificationChar(classification, 5U),
                   GrayClassificationChar(classification, 6U),
                   GrayClassificationChar(classification, 7U));
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static bool CaptureCalibrationSurface(
    GrayArray *gray,
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
        GrayAdcDebug_DelayMs(10U);
    }
    for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
        output[channel] = (uint16_t)(sums[channel] / GRAY_CALIBRATION_FRAMES);
    }
    return true;
}

void GrayAdcDebug_Run(void)
{
    CarFirmwareConfig config;
    OLED_Config oled_config;
    Button key1;
    Button key2;
    Button key3;
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
    Button_Init(&key1, ReadKey1, 0, true, 30U);
    Button_Init(&key2, ReadKey2, 0, true, 30U);
    Button_Init(&key3, ReadKey3, 0, true, 30U);

    oled_config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);
    gGrayDebugOledStatus = OLED_Init(&oled_config);
    if (gGrayDebugOledStatus == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        oled_config = OLED_MakeSSD1306Config(0x3DU);
        gGrayDebugOledStatus = OLED_Init(&oled_config);
    }

    while (1) {
        bool adc_ok = GrayArray_Read(
            &gGrayDebugArray, TiMspm0Platform_Millis());
        GrayArrayClassification classification = {0U};
        bool classification_ok = GrayArray_ClassifyLatest(
            &gGrayDebugArray, GRAY_WHITE_THRESHOLD, GRAY_BLACK_THRESHOLD,
            &classification);

        gGrayDebugFrame++;
        Button_Update(&key1, TiMspm0Platform_Millis());
        Button_Update(&key2, TiMspm0Platform_Millis());
        Button_Update(&key3, TiMspm0Platform_Millis());
        if ((state == GRAY_CAL_WHITE_WAIT) &&
            Button_TakePressedEvent(&key1)) {
            state = CaptureCalibrationSurface(
                &gGrayDebugArray, white, "WHITE") ?
                GRAY_CAL_BLACK_WAIT : GRAY_CAL_ERROR;
        } else if ((state == GRAY_CAL_BLACK_WAIT) &&
                   Button_TakePressedEvent(&key2)) {
            if (CaptureCalibrationSurface(
                    &gGrayDebugArray, black, "BLACK") &&
                GrayArray_SetCalibration(&gGrayDebugArray, black, white)) {
                state = GRAY_CAL_RUNNING;
            } else {
                state = GRAY_CAL_ERROR;
            }
        } else if ((state == GRAY_CAL_ERROR) &&
                   Button_TakePressedEvent(&key3)) {
            state = GRAY_CAL_WHITE_WAIT;
        }
        if (OLED_IsInitialized()) {
            if (state == GRAY_CAL_WHITE_WAIT) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "PUT WHITE", "PRESS KEY1");
            } else if (state == GRAY_CAL_BLACK_WAIT) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "PUT BLACK", "PRESS KEY2");
            } else if (state == GRAY_CAL_ERROR) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "CAL ERROR", "KEY3 TO RETRY");
            } else {
                gGrayDebugOledStatus = DisplayGrayValues(
                    &gGrayDebugArray, adc_ok, gGrayDebugFrame,
                    classification_ok ? &classification : 0);
            }
        }
        GrayAdcDebug_DelayMs(100U);
    }
}
