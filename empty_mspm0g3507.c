#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "project_mode.h"
#include "platform/ti_mspm0_platform.h"

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
#include "OLED.h"
#include "tb6612.h"

static CarFirmware gFirmware;
static OLED_Status gH2024OledStatus = OLED_STATUS_ERROR_NOT_INITIALIZED;

static const char *H2024_ModeName(H2024Mode mode)
{
    switch (mode) {
        case H2024_MODE_ITEM_1:
            return "ITEM1";
        case H2024_MODE_ITEM_2:
            return "ITEM2";
        case H2024_MODE_ITEM_3:
            return "ITEM3";
        case H2024_MODE_ITEM_4:
            return "ITEM4";
        case H2024_MODE_TURN_DEBUG:
            return "TURN";
        default:
            return "?????";
    }
}

static const char *H2024_ButtonActionName(CarButtonAction action)
{
    switch (action) {
        case CAR_BUTTON_ACTION_ARM_OK:
            return "ARM";
        case CAR_BUTTON_ACTION_ARM_REJECTED:
            return "REJ";
        case CAR_BUTTON_ACTION_EMERGENCY_STOP:
            return "STOP";
        case CAR_BUTTON_ACTION_NONE:
        default:
            return "NONE";
    }
}

static int32_t H2024_ToTenths(float value)
{
    float scaled = value * 10.0f;

    return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void H2024_ShowFixed1(uint8_t page, const char *prefix, float value)
{
    char line[22];
    int32_t scaled = H2024_ToTenths(value);
    uint32_t magnitude;
    char sign = (scaled < 0) ? '-' : '+';

    if (scaled < 0) {
        magnitude = (uint32_t)(-(scaled + 1)) + 1U;
    } else {
        magnitude = (uint32_t)scaled;
    }
    (void)snprintf(line, sizeof(line), "%s%c%lu.%lu", prefix, sign,
                   (unsigned long)(magnitude / 10U),
                   (unsigned long)(magnitude % 10U));
    (void)OLED_ShowString(0U, page, line);
}

static void H2024_InitOled(void)
{
    OLED_Config config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);

    gH2024OledStatus = OLED_Init(&config);
    if (gH2024OledStatus == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        config = OLED_MakeSSD1306Config(0x3DU);
        gH2024OledStatus = OLED_Init(&config);
    }
}

static void H2024_RefreshOled(const CarFirmware *firmware,
                              uint32_t now_ms)
{
    static uint32_t last_refresh_ms;
    static uint8_t page;
    char line[22];
    uint32_t faults;
    const char *segment_state;

    if ((firmware == 0) || !OLED_IsInitialized() ||
        (gH2024OledStatus != OLED_STATUS_OK) ||
        ((uint32_t)(now_ms - last_refresh_ms) < 50U)) {
        return;
    }
    last_refresh_ms = now_ms;

    if (page == 0U) {
        faults = firmware->hardware_faults | firmware->output.faults;
        segment_state = firmware->output.route_finished ? "DONE" :
                        (firmware->output.route_running ? "RUN" : "STOP");
        (void)OLED_Clear();
        (void)snprintf(line, sizeof(line), "MODE:%s TB",
                       H2024_ModeName(firmware->config.mode));
        (void)OLED_ShowString(0U, 0U, line);
        (void)OLED_ShowString(0U, 1U,
                              firmware->yaw.calibrated ?
                              "CAL:OK" : "CAL:WAIT");
        H2024_ShowFixed1(2U, "YAW:", firmware->imu_sample.yaw_deg);
        (void)snprintf(line, sizeof(line), "SEG:%02u %s",
                       (unsigned)firmware->output.route_index,
                       segment_state);
        (void)OLED_ShowString(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "L:%+ld R:%+ld",
                       (long)firmware->output.motor.left_mm_s,
                       (long)firmware->output.motor.right_mm_s);
        (void)OLED_ShowString(0U, 4U, line);
        (void)snprintf(line, sizeof(line), "PWM:%d/%d",
                       TB6612_GetLeftCommand(),
                       TB6612_GetRightCommand());
        (void)OLED_ShowString(0U, 5U, line);
        (void)snprintf(line, sizeof(line), "F:%08lX",
                       (unsigned long)faults);
        (void)OLED_ShowString(0U, 6U, line);
        (void)snprintf(line, sizeof(line), "KEYH:%u%u%u K1 S/S",
                       TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                       TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                       TiMspm0Platform_ReadKey3Level() ? 1U : 0U);
        (void)OLED_ShowString(0U, 7U, line);
    } else if (page == 1U) {
        (void)OLED_Clear();
        (void)snprintf(line, sizeof(line), "BTN:%lu %s",
                       (unsigned long)firmware->button_event_count,
                       H2024_ButtonActionName(firmware->last_button_action));
        (void)OLED_ShowString(0U, 0U, line);
        (void)snprintf(line, sizeof(line), "I:%c E:%c M:%c",
                       firmware->last_button_imu_valid ? 'Y' : 'N',
                       firmware->last_button_encoder_valid ? 'Y' : 'N',
                       firmware->last_button_motor_armed ? 'Y' : 'N');
        (void)OLED_ShowString(0U, 1U, line);
        (void)snprintf(line, sizeof(line), "STAT:%ld",
                       (long)firmware->last_arm_status);
        (void)OLED_ShowString(0U, 2U, line);
        (void)snprintf(line, sizeof(line), "ENC:%c",
                       firmware->encoder_valid_current ? 'Y' : 'N');
        (void)OLED_ShowString(0U, 3U, line);
        (void)snprintf(line, sizeof(line), "TB:%c SER:%c",
                       firmware->config.motor.direct_set_wheel_speeds != 0 ?
                       'Y' : 'N',
                       firmware->config.motor.send != 0 ? 'Y' : 'N');
        (void)OLED_ShowString(0U, 4U, line);
        (void)snprintf(line, sizeof(line), "APP:%c RUN:%c",
                       firmware->app.armed ? 'Y' : 'N',
                       firmware->app.executor.running ? 'Y' : 'N');
        (void)OLED_ShowString(0U, 5U, line);
        (void)snprintf(line, sizeof(line), "PWM:%d/%d",
                       TB6612_GetLeftCommand(),
                       TB6612_GetRightCommand());
        (void)OLED_ShowString(0U, 6U, line);
        H2024_ShowFixed1(7U, "BIAS:", firmware->yaw.bias_dps);
    }

    if (OLED_Update() != OLED_STATUS_OK) {
        gH2024OledStatus = OLED_STATUS_ERROR_I2C_BUS;
        return;
    }
    page = (uint8_t)((page + 1U) % 2U);
}

static H2024Mode GetH2024Mode(void)
{
#if PROJECT_MODE == PROJECT_MODE_H2024_ITEM_1
    return H2024_MODE_ITEM_1;
#elif PROJECT_MODE == PROJECT_MODE_H2024_ITEM_2
    return H2024_MODE_ITEM_2;
#elif PROJECT_MODE == PROJECT_MODE_H2024_ITEM_3
    return H2024_MODE_ITEM_3;
#elif PROJECT_MODE == PROJECT_MODE_TURN_DEBUG
    return H2024_MODE_TURN_DEBUG;
#else
    return H2024_MODE_ITEM_4;
#endif
}

static void RunH2024Firmware(void)
{
    CarFirmwareConfig config;
    CarStatus status;

    TiMspm0Platform_Init();
    H2024_InitOled();
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
        uint32_t now_ms = TiMspm0Platform_Millis();

        TiMspm0Platform_PollMotorRx(&gFirmware);
        CarFirmware_Tick(&gFirmware, now_ms);
        H2024_RefreshOled(&gFirmware, now_ms);
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
