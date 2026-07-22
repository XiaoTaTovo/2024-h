#include "vofa_telemetry.h"

#include "ti_msp_dl_config.h"

static void send_text(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(
            UART_BLUETOOTH_INST, (uint8_t) *text++);
    }
}

static char *append_unsigned(char *out, uint32_t value)
{
    char reversed[10];
    uint32_t count = 0;

    do {
        reversed[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count != 0U) {
        *out++ = reversed[--count];
    }
    return out;
}

static char *append_signed(char *out, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        *out++ = '-';
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    return append_unsigned(out, magnitude);
}

static char *append_separator(char *out)
{
    *out++ = ',';
    return out;
}

static char *append_u32_field(char *out, uint32_t value)
{
    out = append_unsigned(out, value);
    return append_separator(out);
}

static char *append_i32_field(char *out, int32_t value)
{
    out = append_signed(out, value);
    return append_separator(out);
}

void VofaTelemetry_SendBanner(void)
{
    send_text("#READY Bluetooth encoder and speed-loop debug\r\n");
    send_text("#CSV mode,motion,target_l,target_r,rpm_l,rpm_r,"
              "duty_l,duty_r,delta_l,delta_r,total_l,total_r,"
              "ppr_l,ppr_r,kp_x1000,ki_x1000,kd_x1000,"
              "limit,duty_step,rx,errors,failsafe,ms\r\n");
    send_text("#Send HELP for commands\r\n");
}

void VofaTelemetry_Send(
    const BluetoothControlStatus *status, uint32_t uptimeMs)
{
    char line[256];
    char *out = line;
    char *cursor;

    out = append_u32_field(out, (uint32_t) status->mode);
    out = append_u32_field(out, (uint32_t) status->motion);
    out = append_i32_field(out, status->targetLeftRpm);
    out = append_i32_field(out, status->targetRightRpm);
    out = append_i32_field(out, status->measuredLeftRpm);
    out = append_i32_field(out, status->measuredRightRpm);
    out = append_i32_field(out, status->leftCommandPercent);
    out = append_i32_field(out, status->rightCommandPercent);
    out = append_i32_field(out, status->leftDeltaCount);
    out = append_i32_field(out, status->rightDeltaCount);
    out = append_i32_field(out, status->leftTotalCount);
    out = append_i32_field(out, status->rightTotalCount);
    out = append_u32_field(out, status->leftCountsPerRev);
    out = append_u32_field(out, status->rightCountsPerRev);
    out = append_u32_field(out, status->kpMilli);
    out = append_u32_field(out, status->kiMilli);
    out = append_u32_field(out, status->kdMilli);
    out = append_u32_field(out, status->speedLimitPercent);
    out = append_u32_field(out, status->dutyPercent);
    out = append_u32_field(out, status->rxCount);
    out = append_u32_field(out, status->errorCount);
    out = append_u32_field(out, status->failsafeCount);
    out = append_unsigned(out, uptimeMs);
    *out++ = '\r';
    *out++ = '\n';

    for (cursor = line; cursor < out; cursor++) {
        DL_UART_Main_transmitDataBlocking(
            UART_BLUETOOTH_INST, (uint8_t) *cursor);
    }
}
