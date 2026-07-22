#include "platform/ti_mspm0_platform.h"

#include <string.h>

#include "platform/ti_mspm0_platform_config.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis;
static volatile bool g_adc_ready;
static volatile uint8_t g_motor_rx[H2024_UART_RX_BUFFER_SIZE];
static volatile uint8_t g_motor_rx_head;
static volatile uint8_t g_motor_rx_tail;
static volatile TiMspm0PlatformDiagnostics g_diagnostics;

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
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_AD0_PIN);
    }
    if ((channel & 0x02U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_AD1_PIN);
    }
    if ((channel & 0x04U) != 0U) {
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_AD2_PIN);
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
    return (DL_GPIO_readPins(GPIO_KEYS_PORT, GPIO_KEYS_START_PIN) != 0U);
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
    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    while (!DL_UART_Main_isRXFIFOEmpty(UART_MOTOR_INST)) {
        (void)DL_UART_Main_receiveData(UART_MOTOR_INST);
    }
    NVIC_ClearPendingIRQ(UART_MOTOR_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_MOTOR_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_GRAY_INST_INT_IRQN);
}

uint32_t TiMspm0Platform_Millis(void)
{
    return g_millis;
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
    config->mode = mode;

    config->motor = (MotorBoardConfig){
        TiMotor_Send, 0,
        MOTOR_BOARD_CHANNEL_B, MOTOR_BOARD_CHANNEL_D,
        false, true, 5U
    };
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
    config->imu_calibration_samples = 400U;
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

    /* Item 1 does not need gray. Fill measured values before item 2-4. */
    config->gray_calibration_valid = false;
    return CAR_OK;
}
