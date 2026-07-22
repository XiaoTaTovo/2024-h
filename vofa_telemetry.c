#include "vofa_telemetry.h"

#include "ti_msp_dl_config.h"

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
    if (value < 0) {
        *out++ = '-';
        value = -value;
    }
    return append_unsigned(out, (uint32_t) value);
}

static char *append_separator(char *out)
{
    *out++ = ',';
    return out;
}

void VofaTelemetry_Send(const BluetoothControlStatus *status, uint32_t uptimeMs)
{
    char line[96];
    char *out = line;
    char *cursor;

    out = append_unsigned(out, (uint32_t) status->motion);
    out = append_separator(out);
    out = append_signed(out, status->leftCommandPercent);
    out = append_separator(out);
    out = append_signed(out, status->rightCommandPercent);
    out = append_separator(out);
    out = append_unsigned(out, status->dutyPercent);
    out = append_separator(out);
    out = append_unsigned(out, status->rxCount);
    out = append_separator(out);
    out = append_unsigned(out, status->errorCount);
    out = append_separator(out);
    out = append_unsigned(out, status->failsafeCount);
    out = append_separator(out);
    out = append_unsigned(out, uptimeMs);
    *out++ = '\r';
    *out++ = '\n';

    for (cursor = line; cursor < out; cursor++) {
        DL_UART_Main_transmitDataBlocking(UART_DEBUG_INST, (uint8_t) *cursor);
    }
}
