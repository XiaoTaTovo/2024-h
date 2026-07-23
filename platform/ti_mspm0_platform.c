#include "platform/ti_mspm0_platform.h"

#include <string.h>

#include "platform/ti_mspm0_platform_config.h"
#include "encoder.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis;
static volatile bool g_adc_ready;
static volatile uint8_t g_motor_rx[H2024_UART_RX_BUFFER_SIZE];
static volatile uint8_t g_motor_rx_head;
static volatile uint8_t g_motor_rx_tail;
static volatile TiMspm0PlatformDiagnostics g_diagnostics;
static TB6612MotorBoardContext g_tb6612_context;

static uint32_t TiMspm0Platform_MillisAdapter(void *context)
{
    (void)context;
    return TiMspm0Platform_Millis();
}

static uint8_t TiMotor_NextRxIndex(uint8_t index)
{
    index++;
    return (index >= H2024_UART_RX_BUFFER_SIZE) ? 0U : index;
}

static void TiMotor_QueueRxByte(uint8_t byte)
{
    uint8_t next = TiMotor_NextRxIndex(g_motor_rx_head);

    g_diagnostics.motor_rx_bytes++;
    if (next == g_motor_rx_tail) {
        g_diagnostics.motor_rx_overflows++;
        return;
    }
    g_motor_rx[g_motor_rx_head] = byte;
    g_motor_rx_head = next;
}

static bool TiMotor_ReadRxByte(uint8_t *byte)
{
    uint8_t tail;

    if (byte == 0) {
        return false;
    }
    tail = g_motor_rx_tail;
    if (tail == g_motor_rx_head) {
        return false;
    }
    *byte = g_motor_rx[tail];
    g_motor_rx_tail = TiMotor_NextRxIndex(tail);
    return true;
}

static bool TiMotor_Send(const uint8_t *data, uint8_t length, void *context)
{
    uint32_t timeout;

    (void)context;
    if ((data == 0) || (length == 0U)) {
        return false;
    }
    for (uint8_t i = 0U; i < length; i++) {
        timeout = H2024_UART_TX_TIMEOUT_LOOPS;
        while (DL_UART_isBusy(UART_MOTOR_INST) && (timeout > 0U)) {
            timeout--;
        }
        if (timeout == 0U) {
            g_diagnostics.motor_tx_timeouts++;
            return false;
        }
        DL_UART_Main_transmitData(UART_MOTOR_INST, data[i]);
    }
    timeout = H2024_UART_TX_TIMEOUT_LOOPS;
    while (DL_UART_isBusy(UART_MOTOR_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.motor_tx_timeouts++;
        return false;
    }
    return true;
}

static uint8_t TiImu_Transfer(uint8_t value, void *context)
{
    uint32_t timeout = H2024_SPI_TIMEOUT_LOOPS;

    (void)context;
    while (DL_SPI_isTXFIFOFull(SPI_IMU_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.imu_spi_timeouts++;
        return 0xFFU;
    }
    DL_SPI_transmitData8(SPI_IMU_INST, value);
    timeout = H2024_SPI_TIMEOUT_LOOPS;
    while (DL_SPI_isRXFIFOEmpty(SPI_IMU_INST) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        g_diagnostics.imu_spi_timeouts++;
        return 0xFFU;
    }
    return DL_SPI_receiveData8(SPI_IMU_INST);
}

static void TiImu_Select(bool active, void *context)
{
    (void)context;
    if (active) {
        while (!DL_SPI_isRXFIFOEmpty(SPI_IMU_INST)) {
            (void)DL_SPI_receiveData8(SPI_IMU_INST);
        }
        DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    } else {
        DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    }
}

static void TiDelayMs(uint32_t delay_ms, void *context)
{
    (void)context;
    while (delay_ms > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
        delay_ms--;
    }
}

static void TiDelayUs(uint32_t delay_us, void *context)
{
    (void)context;
    while (delay_us > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000000U);
        delay_us--;
    }
}

static bool TiGray_Select(uint8_t channel, void *context)
{
    (void)context;
    if (channel >= GRAY_ARRAY_CHANNELS) {
        return false;
    }
    if ((channel & 0x01U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
    }
    if ((channel & 0x02U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
    }
    if ((channel & 0x04U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
    }
    return true;
}

static bool TiGray_ReadAdc(uint16_t *value, void *context)
{
    uint32_t timeout = H2024_ADC_TIMEOUT_LOOPS;

    (void)context;
    if (value == 0) {
        return false;
    }
    g_adc_ready = false;
    DL_ADC12_startConversion(ADC_GRAY_INST);
    while (!g_adc_ready && (timeout > 0U)) {
        timeout--;
    }
    if (!g_adc_ready) {
        g_diagnostics.gray_adc_timeouts++;
        return false;
    }
    *value = (uint16_t)DL_ADC12_getMemResult(
        ADC_GRAY_INST, ADC_GRAY_ADCMEM_GRAY_OUT);
    return true;
}

static bool TiButton_Read(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey1Level();
}

static void TiBuzzer_Set(bool enabled, void *context)
{
    (void)context;
    if (enabled) {
        DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    }
}

void TiMspm0Platform_OnSysTick(void)
{
    g_millis++;
}

void UART_MOTOR_INST_IRQHandler(void)
{
    (void)DL_UART_Main_getPendingInterrupt(UART_MOTOR_INST);
    while (!DL_UART_Main_isRXFIFOEmpty(UART_MOTOR_INST)) {
        TiMotor_QueueRxByte(DL_UART_Main_receiveData(UART_MOTOR_INST));
    }
}

void ADC_GRAY_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC_GRAY_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        g_adc_ready = true;
    }
}

void TiMspm0Platform_Init(void)
{
    g_adc_ready = false;
    g_motor_rx_head = 0U;
    g_motor_rx_tail = 0U;
    g_diagnostics = (TiMspm0PlatformDiagnostics){0};
    /* Keep the motor driver electrically disabled during every startup path. */
    DL_TimerA_stopCounter(PWM_TB1_INST);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(GPIO_GRAY_EN_PORT, GPIO_GRAY_EN_PIN);
    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    TB6612_MotorBoardContextInit(
        &g_tb6612_context, TiMspm0Platform_MillisAdapter, 0,
        H2024_TB6612_SPEED_UNITS_AT_MAX_DUTY);
    TB6612_Init();
    Encoder_Init();
    while (!DL_UART_Main_isRXFIFOEmpty(UART_MOTOR_INST)) {
        (void)DL_UART_Main_receiveData(UART_MOTOR_INST);
    }
    DL_ADC12_disableConversions(ADC_GRAY_INST);
    DL_ADC12_initSingleSample(
        ADC_GRAY_INST,
        DL_ADC12_REPEAT_MODE_ENABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_setSampleTime0(ADC_GRAY_INST, 8U);
    DL_ADC12_clearInterruptStatus(
        ADC_GRAY_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(ADC_GRAY_INST);
    NVIC_ClearPendingIRQ(UART_MOTOR_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC_GRAY_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_MOTOR_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_GRAY_INST_INT_IRQN);
}

uint32_t TiMspm0Platform_Millis(void)
{
    return g_millis;
}

bool TiMspm0Platform_ReadKey1Level(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY1_PIN) != 0U);
}

bool TiMspm0Platform_ReadKey2Level(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY2_PIN) != 0U);
}

bool TiMspm0Platform_ReadKey3Level(void)
{
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_KEY3_PIN) != 0U);
}

void TiMspm0Platform_PollMotorRx(CarFirmware *firmware)
{
    uint32_t now_ms = TiMspm0Platform_Millis();
    uint8_t byte;

    while (TiMotor_ReadRxByte(&byte)) {
        CarFirmware_OnMotorRxByte(firmware, byte, now_ms);
    }
}

void TiMspm0Platform_GetDiagnostics(TiMspm0PlatformDiagnostics *diagnostics)
{
    if (diagnostics == 0) {
        return;
    }
    diagnostics->motor_rx_bytes = g_diagnostics.motor_rx_bytes;
    diagnostics->motor_rx_overflows = g_diagnostics.motor_rx_overflows;
    diagnostics->motor_tx_timeouts = g_diagnostics.motor_tx_timeouts;
    diagnostics->imu_spi_timeouts = g_diagnostics.imu_spi_timeouts;
    diagnostics->gray_adc_timeouts = g_diagnostics.gray_adc_timeouts;
}

CarStatus TiMspm0Platform_BuildConfig(CarFirmwareConfig *config,
                                      H2024Mode mode)
{
    if (config == 0) {
        return CAR_ERROR_ARG;
    }
    *config = (CarFirmwareConfig){0};
    config->car = CarConfig_MakeDefault();
    config->car.wheel_diameter_mm = H2024_WHEEL_DIAMETER_MM;
    config->car.track_width_mm = H2024_TRACK_WIDTH_MM;
    config->car.encoder_counts_per_wheel_rev =
        H2024_ENCODER_COUNTS_PER_WHEEL_REV;
    config->car.arc_line_kp = H2024_LINE_PID_KP;
    config->car.arc_line_ki = H2024_LINE_PID_KI;
    config->car.arc_line_kd = H2024_LINE_PID_KD;
    config->car.arc_line_integral_limit = H2024_LINE_PID_INTEGRAL_LIMIT;
    config->mode = mode;

#if H2024_MOTOR_BACKEND_TB6612
    config->motor = (MotorBoardConfig){0};
    config->motor.left_channel = MOTOR_BOARD_CHANNEL_B;
    config->motor.right_channel = MOTOR_BOARD_CHANNEL_D;
    config->motor.left_inverted = false;
    config->motor.right_inverted = false;
    config->motor.direct_set_wheel_speeds =
        TB6612_MotorBoard_SetWheelSpeeds;
    config->motor.direct_get_encoder = TB6612_MotorBoard_GetEncoder;
    config->motor.direct_context = &g_tb6612_context;
#else
    config->motor = (MotorBoardConfig){
        TiMotor_Send, 0,
        MOTOR_BOARD_CHANNEL_B, MOTOR_BOARD_CHANNEL_D,
        false, true, 5U,
        0, 0, 0
    };
#endif
    config->imu = (Icm42688Port){
        TiImu_Transfer, TiImu_Select, TiDelayMs, 0
    };
    config->gray = (GrayArrayPort){
        TiGray_Select, TiGray_ReadAdc, TiDelayUs, 0,
        H2024_GRAY_SETTLE_US, H2024_GRAY_SAMPLES_PER_CHANNEL
    };
    config->button_read = TiButton_Read;
    config->buzzer_set = TiBuzzer_Set;
    config->motor_units_per_mm_s = H2024_MOTOR_UNITS_PER_MM_S;
    config->yaw_axis = CAR_IMU_AXIS_Z;
    config->yaw_sign = H2024_IMU_YAW_SIGN;
    config->yaw_bias_dps = H2024_IMU_BIAS_DPS;
    config->yaw_bias_fixed = H2024_IMU_USE_FIXED_BIAS != 0U;
    config->imu_calibration_samples = H2024_IMU_CALIBRATION_SAMPLES;
    config->imu_max_step_ms = 20U;
    config->button_debounce_ms = 20U;
    config->button_active_low = true;

    config->motor_command_spacing_ms = H2024_MOTOR_COMMAND_SPACING_MS;
    config->set_encoder_polarity_on_arm = true;
    config->set_speed_pid_on_arm = true;
    for (uint8_t channel = 0U; channel < MOTOR_BOARD_CHANNEL_COUNT; channel++) {
        config->encoder_polarity[channel] = true;
        config->speed_pid[channel] = (MotorBoardPid){
            H2024_MOTOR_PID_KP,
            H2024_MOTOR_PID_KI,
            H2024_MOTOR_PID_KD
        };
    }

    /* Measured full-white/full-black values for the current sensor board. */
    {
        static const uint16_t gray_black[GRAY_ARRAY_CHANNELS] = {
            108U, 112U, 113U, 114U, 114U, 114U, 114U, 114U
        };
        static const uint16_t gray_white[GRAY_ARRAY_CHANNELS] = {
            2415U, 1677U, 2310U, 2001U, 1693U, 1885U, 1869U, 1141U
        };

        memcpy(config->gray_black, gray_black, sizeof(gray_black));
        memcpy(config->gray_white, gray_white, sizeof(gray_white));
        config->gray_calibration_valid = true;
    }
    return CAR_OK;
}
