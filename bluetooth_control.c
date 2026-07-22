#include "bluetooth_control.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "encoder.h"
#include "tb6612.h"
#include "ti_msp_dl_config.h"

#define BLUETOOTH_RX_BUFFER_SIZE         (128U)
#define BLUETOOTH_COMMAND_BUFFER_SIZE    (64U)
#define BLUETOOTH_COMMAND_IDLE_MS        (100U)
#define BLUETOOTH_INITIAL_DUTY_PERCENT   (20U)
#define BLUETOOTH_MIN_DUTY_PERCENT       (10U)
#define BLUETOOTH_DUTY_STEP_PERCENT      (5U)
#define BLUETOOTH_FAILSAFE_MS            (2000U)
#define SPEED_CONTROL_PERIOD_MS          (50U)
#define SPEED_INITIAL_LIMIT_PERCENT      (35U)
#define SPEED_MAX_TARGET_RPM             (1000)
#define SPEED_MAX_COUNTS_PER_REV         (1000000U)
#define SPEED_DEFAULT_KP_MILLI           (250U)
#define SPEED_DEFAULT_KI_MILLI           (600U)
#define SPEED_DEFAULT_KD_MILLI           (0U)
#define SPEED_MAX_GAIN_MILLI             (100000U)

static volatile uint8_t gRxBuffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint8_t gRxHead;
static volatile uint8_t gRxTail;
static volatile uint32_t gRxCount;
static volatile uint32_t gErrorCount;

static char gCommandBuffer[BLUETOOTH_COMMAND_BUFFER_SIZE];
static uint8_t gCommandLength;
static uint32_t gLastCommandByteMs;

static BluetoothControlMode gMode;
static BluetoothMotion gMotion;
static uint8_t gDutyPercent;
static uint8_t gSpeedLimitPercent;
static uint32_t gLastMotionCommandMs;
static uint32_t gFailsafeCount;

static int32_t gTargetLeftRpm;
static int32_t gTargetRightRpm;
static int32_t gMeasuredLeftRpm;
static int32_t gMeasuredRightRpm;
static int32_t gLeftDeltaCount;
static int32_t gRightDeltaCount;
static uint32_t gLeftCountsPerRev;
static uint32_t gRightCountsPerRev;
static uint32_t gKpMilli;
static uint32_t gKiMilli;
static uint32_t gKdMilli;
static int64_t gLeftIntegralMilli;
static int64_t gRightIntegralMilli;
static int32_t gLeftPreviousError;
static int32_t gRightPreviousError;
static uint32_t gLastSpeedSampleMs;

static void send_text(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(
            UART_BLUETOOTH_INST, (uint8_t) *text++);
    }
}

static void send_help(void)
{
    send_text("#HELP commands require CR/LF or 100 ms idle\r\n");
    send_text("#CAL | STATUS | PPR left [right]\r\n");
    send_text("#OPEN left_pct right_pct | F | B | L | R | S | + | -\r\n");
    send_text("#SPEED left_rpm right_rpm | KEEP\r\n");
    send_text("#PID kp ki [kd] | LIMIT percent | DUTY percent\r\n");
    send_text("#Example: PID 0.250 0.600 0\r\n");
}

static void report_command_error(const char *message)
{
    gErrorCount++;
    send_text("#ERR ");
    send_text(message);
    send_text("\r\n");
}

static int8_t clamp_motor_percent(int32_t percent)
{
    if (percent > (int32_t) TB6612_MAX_DUTY_PERCENT) {
        return (int8_t) TB6612_MAX_DUTY_PERCENT;
    }
    if (percent < -(int32_t) TB6612_MAX_DUTY_PERCENT) {
        return -(int8_t) TB6612_MAX_DUTY_PERCENT;
    }
    return (int8_t) percent;
}

static BluetoothMotion classify_motion(int32_t left, int32_t right)
{
    if ((left == 0) && (right == 0)) {
        return BLUETOOTH_MOTION_STOP;
    }
    if ((left > 0) && (right > 0)) {
        return BLUETOOTH_MOTION_FORWARD;
    }
    if ((left < 0) && (right < 0)) {
        return BLUETOOTH_MOTION_BACKWARD;
    }
    if ((left < 0) && (right > 0)) {
        return BLUETOOTH_MOTION_LEFT;
    }
    if ((left > 0) && (right < 0)) {
        return BLUETOOTH_MOTION_RIGHT;
    }
    return BLUETOOTH_MOTION_DIRECT;
}

static void reset_controller_state(void)
{
    gLeftIntegralMilli  = 0;
    gRightIntegralMilli = 0;
    gLeftPreviousError  = 0;
    gRightPreviousError = 0;
}

static void stop_motors(void)
{
    TB6612_Stop();
    gMode           = BLUETOOTH_MODE_STOP;
    gMotion         = BLUETOOTH_MOTION_STOP;
    gTargetLeftRpm  = 0;
    gTargetRightRpm = 0;
    reset_controller_state();
}

static void apply_open_loop(
    int32_t leftPercent, int32_t rightPercent, uint32_t nowMs)
{
    int8_t left  = clamp_motor_percent(leftPercent);
    int8_t right = clamp_motor_percent(rightPercent);

    if ((left == 0) && (right == 0)) {
        stop_motors();
        return;
    }

    gMode                = BLUETOOTH_MODE_OPEN_LOOP;
    gMotion              = classify_motion(left, right);
    gTargetLeftRpm       = 0;
    gTargetRightRpm      = 0;
    gLastMotionCommandMs = nowMs;
    reset_controller_state();
    TB6612_SetMotors(left, right);
}

static char *next_token(char **cursor)
{
    char *token;

    while ((**cursor == ' ') || (**cursor == '\t') || (**cursor == ',')) {
        (*cursor)++;
    }
    if (**cursor == '\0') {
        return NULL;
    }

    token = *cursor;
    while ((**cursor != '\0') && (**cursor != ' ') &&
           (**cursor != '\t') && (**cursor != ',')) {
        (*cursor)++;
    }
    if (**cursor != '\0') {
        **cursor = '\0';
        (*cursor)++;
    }
    return token;
}

static void uppercase_ascii(char *text)
{
    while (*text != '\0') {
        if ((*text >= 'a') && (*text <= 'z')) {
            *text = (char) (*text - ('a' - 'A'));
        }
        text++;
    }
}

static bool parse_int32(const char *text, int32_t *value)
{
    bool negative = false;
    int64_t result = 0;

    if ((*text == '+') || (*text == '-')) {
        negative = (*text == '-');
        text++;
    }
    if ((*text < '0') || (*text > '9')) {
        return false;
    }

    while ((*text >= '0') && (*text <= '9')) {
        result = (result * 10) + (int64_t) (*text - '0');
        if (result > ((int64_t) INT32_MAX + (negative ? 1 : 0))) {
            return false;
        }
        text++;
    }
    if (*text != '\0') {
        return false;
    }

    *value = negative ? (int32_t) -result : (int32_t) result;
    return true;
}

static bool parse_gain_milli(const char *text, uint32_t *value)
{
    uint32_t whole = 0;
    uint32_t fraction = 0;
    uint32_t fractionDigits = 0;

    if (*text == '+') {
        text++;
    }
    if ((*text < '0') || (*text > '9')) {
        return false;
    }

    while ((*text >= '0') && (*text <= '9')) {
        whole = (whole * 10U) + (uint32_t) (*text - '0');
        if (whole > (SPEED_MAX_GAIN_MILLI / 1000U)) {
            return false;
        }
        text++;
    }

    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            if (fractionDigits >= 3U) {
                return false;
            }
            fraction = (fraction * 10U) + (uint32_t) (*text - '0');
            fractionDigits++;
            text++;
        }
    }
    while (fractionDigits < 3U) {
        fraction *= 10U;
        fractionDigits++;
    }
    if (*text != '\0') {
        return false;
    }

    *value = (whole * 1000U) + fraction;
    return *value <= SPEED_MAX_GAIN_MILLI;
}

static bool has_no_more_tokens(char **cursor)
{
    return next_token(cursor) == NULL;
}

static bool command_open(char **cursor, uint32_t nowMs)
{
    char *leftText = next_token(cursor);
    char *rightText = next_token(cursor);
    int32_t left;
    int32_t right;

    if ((leftText == NULL) || (rightText == NULL) ||
        !parse_int32(leftText, &left) || !parse_int32(rightText, &right) ||
        !has_no_more_tokens(cursor) ||
        (left < -(int32_t) TB6612_MAX_DUTY_PERCENT) ||
        (left > (int32_t) TB6612_MAX_DUTY_PERCENT) ||
        (right < -(int32_t) TB6612_MAX_DUTY_PERCENT) ||
        (right > (int32_t) TB6612_MAX_DUTY_PERCENT)) {
        report_command_error("OPEN needs left/right percent in -60..60");
        return false;
    }

    apply_open_loop(left, right, nowMs);
    send_text("#OK OPEN\r\n");
    return true;
}

static bool command_ppr(char **cursor)
{
    char *leftText = next_token(cursor);
    char *rightText = next_token(cursor);
    int32_t left;
    int32_t right;

    if ((leftText == NULL) || !parse_int32(leftText, &left)) {
        report_command_error("PPR needs one or two positive counts");
        return false;
    }
    if (rightText == NULL) {
        right = left;
    } else if (!parse_int32(rightText, &right)) {
        report_command_error("PPR needs one or two positive counts");
        return false;
    }
    if (!has_no_more_tokens(cursor) || (left <= 0) || (right <= 0) ||
        ((uint32_t) left > SPEED_MAX_COUNTS_PER_REV) ||
        ((uint32_t) right > SPEED_MAX_COUNTS_PER_REV)) {
        report_command_error("PPR range is 1..1000000");
        return false;
    }

    gLeftCountsPerRev  = (uint32_t) left;
    gRightCountsPerRev = (uint32_t) right;
    send_text("#OK PPR\r\n");
    return true;
}

static bool command_pid(char **cursor)
{
    char *kpText = next_token(cursor);
    char *kiText = next_token(cursor);
    char *kdText = next_token(cursor);
    uint32_t kp;
    uint32_t ki;
    uint32_t kd = 0;

    if ((kpText == NULL) || (kiText == NULL) ||
        !parse_gain_milli(kpText, &kp) || !parse_gain_milli(kiText, &ki) ||
        ((kdText != NULL) && !parse_gain_milli(kdText, &kd)) ||
        !has_no_more_tokens(cursor)) {
        report_command_error("PID example: PID 0.250 0.600 0");
        return false;
    }

    gKpMilli = kp;
    gKiMilli = ki;
    gKdMilli = kd;
    reset_controller_state();
    send_text("#OK PID\r\n");
    return true;
}

static bool command_speed(char **cursor, uint32_t nowMs)
{
    char *leftText = next_token(cursor);
    char *rightText = next_token(cursor);
    int32_t left;
    int32_t right;
    bool targetChanged;

    if ((leftText == NULL) || (rightText == NULL) ||
        !parse_int32(leftText, &left) || !parse_int32(rightText, &right) ||
        !has_no_more_tokens(cursor) ||
        (left < -SPEED_MAX_TARGET_RPM) || (left > SPEED_MAX_TARGET_RPM) ||
        (right < -SPEED_MAX_TARGET_RPM) || (right > SPEED_MAX_TARGET_RPM)) {
        report_command_error("SPEED needs left/right RPM in -1000..1000");
        return false;
    }
    if ((left == 0) && (right == 0)) {
        stop_motors();
        send_text("#OK STOP\r\n");
        return true;
    }
    if ((gLeftCountsPerRev == 0U) || (gRightCountsPerRev == 0U)) {
        report_command_error("set PPR before SPEED");
        return false;
    }

    targetChanged = (gMode != BLUETOOTH_MODE_SPEED_LOOP) ||
                    (left != gTargetLeftRpm) ||
                    (right != gTargetRightRpm);
    if (targetChanged) {
        reset_controller_state();
    }
    gMode                = BLUETOOTH_MODE_SPEED_LOOP;
    gMotion              = classify_motion(left, right);
    gTargetLeftRpm       = left;
    gTargetRightRpm      = right;
    gLastMotionCommandMs = nowMs;
    send_text("#OK SPEED\r\n");
    return true;
}

static bool execute_command(uint32_t nowMs)
{
    char *cursor = gCommandBuffer;
    char *command = next_token(&cursor);
    int32_t value;

    if (command == NULL) {
        return false;
    }
    uppercase_ascii(command);

    if ((strcmp(command, "HELP") == 0) || (strcmp(command, "?") == 0)) {
        send_help();
        return true;
    }
    if ((strcmp(command, "S") == 0) || (strcmp(command, "STOP") == 0) ||
        (strcmp(command, "X") == 0) || (strcmp(command, "0") == 0)) {
        if (!has_no_more_tokens(&cursor)) {
            report_command_error("STOP takes no arguments");
            return false;
        }
        stop_motors();
        send_text("#OK STOP\r\n");
        return true;
    }
    if (strcmp(command, "F") == 0) {
        apply_open_loop(gDutyPercent, gDutyPercent, nowMs);
        send_text("#OK FORWARD\r\n");
        return true;
    }
    if (strcmp(command, "B") == 0) {
        apply_open_loop(-(int32_t) gDutyPercent, -(int32_t) gDutyPercent, nowMs);
        send_text("#OK BACKWARD\r\n");
        return true;
    }
    if (strcmp(command, "L") == 0) {
        apply_open_loop(-(int32_t) gDutyPercent, gDutyPercent, nowMs);
        send_text("#OK LEFT\r\n");
        return true;
    }
    if (strcmp(command, "R") == 0) {
        apply_open_loop(gDutyPercent, -(int32_t) gDutyPercent, nowMs);
        send_text("#OK RIGHT\r\n");
        return true;
    }
    if (strcmp(command, "+") == 0) {
        if (gDutyPercent <=
            (TB6612_MAX_DUTY_PERCENT - BLUETOOTH_DUTY_STEP_PERCENT)) {
            gDutyPercent += BLUETOOTH_DUTY_STEP_PERCENT;
        } else {
            gDutyPercent = TB6612_MAX_DUTY_PERCENT;
        }
        send_text("#OK DUTY+\r\n");
        return true;
    }
    if (strcmp(command, "-") == 0) {
        if (gDutyPercent >=
            (BLUETOOTH_MIN_DUTY_PERCENT + BLUETOOTH_DUTY_STEP_PERCENT)) {
            gDutyPercent -= BLUETOOTH_DUTY_STEP_PERCENT;
        } else {
            gDutyPercent = BLUETOOTH_MIN_DUTY_PERCENT;
        }
        send_text("#OK DUTY-\r\n");
        return true;
    }
    if (strcmp(command, "OPEN") == 0) {
        return command_open(&cursor, nowMs);
    }
    if ((strcmp(command, "CAL") == 0) || (strcmp(command, "RESET") == 0)) {
        if (!has_no_more_tokens(&cursor)) {
            report_command_error("CAL takes no arguments");
            return false;
        }
        stop_motors();
        Encoder_ResetCounts();
        gLeftDeltaCount    = 0;
        gRightDeltaCount   = 0;
        gMeasuredLeftRpm   = 0;
        gMeasuredRightRpm  = 0;
        gLastSpeedSampleMs = nowMs;
        send_text("#CAL rotate each wheel exactly one output revolution\r\n");
        send_text("#CAL then send STATUS and use absolute total counts as PPR\r\n");
        return true;
    }
    if (strcmp(command, "STATUS") == 0) {
        if (!has_no_more_tokens(&cursor)) {
            report_command_error("STATUS takes no arguments");
            return false;
        }
        return true;
    }
    if (strcmp(command, "PPR") == 0) {
        return command_ppr(&cursor);
    }
    if (strcmp(command, "PID") == 0) {
        return command_pid(&cursor);
    }
    if (strcmp(command, "SPEED") == 0) {
        return command_speed(&cursor, nowMs);
    }
    if (strcmp(command, "KEEP") == 0) {
        if (!has_no_more_tokens(&cursor) ||
            (gMode == BLUETOOTH_MODE_STOP)) {
            report_command_error("KEEP requires a running motor");
            return false;
        }
        gLastMotionCommandMs = nowMs;
        send_text("#OK KEEP\r\n");
        return true;
    }
    if (strcmp(command, "DUTY") == 0) {
        char *valueText = next_token(&cursor);

        if ((valueText == NULL) || !parse_int32(valueText, &value) ||
            !has_no_more_tokens(&cursor) ||
            (value < (int32_t) BLUETOOTH_MIN_DUTY_PERCENT) ||
            (value > (int32_t) TB6612_MAX_DUTY_PERCENT)) {
            report_command_error("DUTY range is 10..60");
            return false;
        }
        gDutyPercent = (uint8_t) value;
        send_text("#OK DUTY\r\n");
        return true;
    }
    if (strcmp(command, "LIMIT") == 0) {
        char *valueText = next_token(&cursor);

        if ((valueText == NULL) || !parse_int32(valueText, &value) ||
            !has_no_more_tokens(&cursor) || (value < 1) ||
            (value > (int32_t) TB6612_MAX_DUTY_PERCENT)) {
            report_command_error("LIMIT range is 1..60");
            return false;
        }
        gSpeedLimitPercent = (uint8_t) value;
        reset_controller_state();
        send_text("#OK LIMIT\r\n");
        return true;
    }

    report_command_error("unknown command; send HELP");
    return false;
}

static int32_t calculate_rpm(
    int32_t deltaCount, uint32_t countsPerRev, uint32_t elapsedMs)
{
    int64_t numerator;
    int64_t denominator;

    if ((countsPerRev == 0U) || (elapsedMs == 0U)) {
        return 0;
    }

    numerator   = (int64_t) deltaCount * 60000;
    denominator = (int64_t) countsPerRev * elapsedMs;
    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
        numerator -= denominator / 2;
    }
    numerator /= denominator;

    if (numerator > INT32_MAX) {
        return INT32_MAX;
    }
    if (numerator < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t) numerator;
}

static int8_t run_pid(int32_t targetRpm, int32_t measuredRpm,
    uint32_t elapsedMs, int64_t *integralMilli, int32_t *previousError)
{
    int32_t error;
    int64_t proportional;
    int64_t derivative;
    int64_t output;
    int64_t limitMilli = (int64_t) gSpeedLimitPercent * 1000;

    if (targetRpm == 0) {
        *integralMilli = 0;
        *previousError = 0;
        return 0;
    }

    error = targetRpm - measuredRpm;
    proportional = (int64_t) gKpMilli * error;
    *integralMilli +=
        ((int64_t) gKiMilli * error * elapsedMs) / 1000;
    if (*integralMilli > limitMilli) {
        *integralMilli = limitMilli;
    } else if (*integralMilli < -limitMilli) {
        *integralMilli = -limitMilli;
    }

    derivative = ((int64_t) gKdMilli * (error - *previousError) * 1000) /
                 elapsedMs;
    *previousError = error;
    output = proportional + *integralMilli + derivative;

    if (targetRpm > 0) {
        if (output < 0) {
            output = 0;
        } else if (output > limitMilli) {
            output = limitMilli;
        }
    } else {
        if (output > 0) {
            output = 0;
        } else if (output < -limitMilli) {
            output = -limitMilli;
        }
    }

    if (output >= 0) {
        output = (output + 500) / 1000;
    } else {
        output = (output - 500) / 1000;
    }
    return (int8_t) output;
}

void BluetoothControl_Init(void)
{
    gRxHead              = 0;
    gRxTail              = 0;
    gRxCount             = 0;
    gErrorCount          = 0;
    gCommandLength       = 0;
    gLastCommandByteMs   = 0;
    gDutyPercent         = BLUETOOTH_INITIAL_DUTY_PERCENT;
    gSpeedLimitPercent   = SPEED_INITIAL_LIMIT_PERCENT;
    gLastMotionCommandMs = 0;
    gFailsafeCount       = 0;
    gTargetLeftRpm       = 0;
    gTargetRightRpm      = 0;
    gMeasuredLeftRpm     = 0;
    gMeasuredRightRpm    = 0;
    gLeftDeltaCount      = 0;
    gRightDeltaCount     = 0;
    gLeftCountsPerRev    = ENC_LEFT_COUNTS_PER_REV;
    gRightCountsPerRev   = ENC_RIGHT_COUNTS_PER_REV;
    gKpMilli             = SPEED_DEFAULT_KP_MILLI;
    gKiMilli             = SPEED_DEFAULT_KI_MILLI;
    gKdMilli             = SPEED_DEFAULT_KD_MILLI;
    gLastSpeedSampleMs   = 0;
    stop_motors();
}

void BluetoothControl_PushRxFromIsr(uint8_t byte)
{
    uint8_t next =
        (uint8_t) ((gRxHead + 1U) % BLUETOOTH_RX_BUFFER_SIZE);

    gRxCount++;
    if (next == gRxTail) {
        gErrorCount++;
        return;
    }
    gRxBuffer[gRxHead] = byte;
    gRxHead            = next;
}

bool BluetoothControl_ProcessPending(uint32_t nowMs)
{
    bool changed = false;

    while (gRxTail != gRxHead) {
        uint8_t byte = gRxBuffer[gRxTail];
        gRxTail =
            (uint8_t) ((gRxTail + 1U) % BLUETOOTH_RX_BUFFER_SIZE);
        gLastCommandByteMs = nowMs;

        if ((byte == '\r') || (byte == '\n')) {
            if (gCommandLength != 0U) {
                gCommandBuffer[gCommandLength] = '\0';
                changed = execute_command(nowMs) || changed;
                gCommandLength = 0;
            }
        } else if ((byte == '\b') || (byte == 0x7FU)) {
            if (gCommandLength != 0U) {
                gCommandLength--;
            }
        } else if ((byte >= 32U) && (byte <= 126U)) {
            if (gCommandLength < (BLUETOOTH_COMMAND_BUFFER_SIZE - 1U)) {
                gCommandBuffer[gCommandLength++] = (char) byte;
            } else {
                gCommandLength = 0;
                report_command_error("command too long");
            }
        }
    }

    if ((gCommandLength != 0U) &&
        ((uint32_t) (nowMs - gLastCommandByteMs) >=
            BLUETOOTH_COMMAND_IDLE_MS)) {
        gCommandBuffer[gCommandLength] = '\0';
        changed = execute_command(nowMs) || changed;
        gCommandLength = 0;
    }
    return changed;
}

void BluetoothControl_Update(uint32_t nowMs)
{
    uint32_t elapsedMs = (uint32_t) (nowMs - gLastSpeedSampleMs);

    if (elapsedMs < SPEED_CONTROL_PERIOD_MS) {
        return;
    }
    gLastSpeedSampleMs = nowMs;

    gLeftDeltaCount  = Encoder_GetLeftDelta();
    gRightDeltaCount = Encoder_GetRightDelta();
    gMeasuredLeftRpm =
        calculate_rpm(gLeftDeltaCount, gLeftCountsPerRev, elapsedMs);
    gMeasuredRightRpm =
        calculate_rpm(gRightDeltaCount, gRightCountsPerRev, elapsedMs);

    if (gMode == BLUETOOTH_MODE_SPEED_LOOP) {
        int8_t leftCommand = run_pid(gTargetLeftRpm, gMeasuredLeftRpm,
            elapsedMs, &gLeftIntegralMilli, &gLeftPreviousError);
        int8_t rightCommand = run_pid(gTargetRightRpm, gMeasuredRightRpm,
            elapsedMs, &gRightIntegralMilli, &gRightPreviousError);

        if ((leftCommand != TB6612_GetLeftCommand()) ||
            (rightCommand != TB6612_GetRightCommand())) {
            TB6612_SetMotors(leftCommand, rightCommand);
        }
    }
}

bool BluetoothControl_CheckFailsafe(uint32_t nowMs)
{
    if ((gMode != BLUETOOTH_MODE_STOP) &&
        ((uint32_t) (nowMs - gLastMotionCommandMs) >=
            BLUETOOTH_FAILSAFE_MS)) {
        stop_motors();
        gFailsafeCount++;
        send_text("#FAILSAFE STOP\r\n");
        return true;
    }
    return false;
}

void BluetoothControl_GetStatus(BluetoothControlStatus *status)
{
    status->mode                = gMode;
    status->motion              = gMotion;
    status->targetLeftRpm       = gTargetLeftRpm;
    status->targetRightRpm      = gTargetRightRpm;
    status->measuredLeftRpm     = gMeasuredLeftRpm;
    status->measuredRightRpm    = gMeasuredRightRpm;
    status->leftCommandPercent  = TB6612_GetLeftCommand();
    status->rightCommandPercent = TB6612_GetRightCommand();
    status->leftDeltaCount      = gLeftDeltaCount;
    status->rightDeltaCount     = gRightDeltaCount;
    status->leftTotalCount      = Encoder_GetLeftCount();
    status->rightTotalCount     = Encoder_GetRightCount();
    status->leftCountsPerRev    = gLeftCountsPerRev;
    status->rightCountsPerRev   = gRightCountsPerRev;
    status->kpMilli             = gKpMilli;
    status->kiMilli             = gKiMilli;
    status->kdMilli             = gKdMilli;
    status->speedLimitPercent   = gSpeedLimitPercent;
    status->dutyPercent         = gDutyPercent;
    status->rxCount             = gRxCount;
    status->errorCount          = gErrorCount;
    status->failsafeCount       = gFailsafeCount;
}
