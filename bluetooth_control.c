#include "bluetooth_control.h"

#include "tb6612.h"

#define BLUETOOTH_RX_BUFFER_SIZE       (32U)
#define BLUETOOTH_INITIAL_DUTY_PERCENT (20U)
#define BLUETOOTH_MIN_DUTY_PERCENT     (10U)
#define BLUETOOTH_DUTY_STEP_PERCENT    (5U)
#define BLUETOOTH_FAILSAFE_MS          (2000U)

static volatile uint8_t gRxBuffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint8_t gRxHead;
static volatile uint8_t gRxTail;
static volatile uint32_t gRxCount;
static volatile uint32_t gErrorCount;

static BluetoothMotion gMotion;
static uint8_t gDutyPercent;
static uint32_t gLastMotionCommandMs;
static uint32_t gFailsafeCount;

static void apply_motion(BluetoothMotion motion)
{
    int8_t duty = (int8_t) gDutyPercent;

    switch (motion) {
        case BLUETOOTH_MOTION_FORWARD:
            TB6612_SetMotors(duty, duty);
            break;
        case BLUETOOTH_MOTION_BACKWARD:
            TB6612_SetMotors(-duty, -duty);
            break;
        case BLUETOOTH_MOTION_LEFT:
            TB6612_SetMotors(-duty, duty);
            break;
        case BLUETOOTH_MOTION_RIGHT:
            TB6612_SetMotors(duty, -duty);
            break;
        case BLUETOOTH_MOTION_STOP:
        default:
            TB6612_Stop();
            motion = BLUETOOTH_MOTION_STOP;
            break;
    }
    gMotion = motion;
}

static bool handle_byte(uint8_t byte, uint32_t nowMs)
{
    if ((byte >= (uint8_t) 'a') && (byte <= (uint8_t) 'z')) {
        byte = (uint8_t) (byte - ((uint8_t) 'a' - (uint8_t) 'A'));
    }

    switch (byte) {
        case 'F':
            gLastMotionCommandMs = nowMs;
            apply_motion(BLUETOOTH_MOTION_FORWARD);
            return true;
        case 'B':
            gLastMotionCommandMs = nowMs;
            apply_motion(BLUETOOTH_MOTION_BACKWARD);
            return true;
        case 'L':
            gLastMotionCommandMs = nowMs;
            apply_motion(BLUETOOTH_MOTION_LEFT);
            return true;
        case 'R':
            gLastMotionCommandMs = nowMs;
            apply_motion(BLUETOOTH_MOTION_RIGHT);
            return true;
        case 'S':
        case 'X':
        case '0':
            apply_motion(BLUETOOTH_MOTION_STOP);
            return true;
        case '+':
            if (gDutyPercent <= (TB6612_MAX_DUTY_PERCENT - BLUETOOTH_DUTY_STEP_PERCENT)) {
                gDutyPercent += BLUETOOTH_DUTY_STEP_PERCENT;
            } else {
                gDutyPercent = TB6612_MAX_DUTY_PERCENT;
            }
            if (gMotion != BLUETOOTH_MOTION_STOP) {
                gLastMotionCommandMs = nowMs;
                apply_motion(gMotion);
            }
            return true;
        case '-':
            if (gDutyPercent >= (BLUETOOTH_MIN_DUTY_PERCENT + BLUETOOTH_DUTY_STEP_PERCENT)) {
                gDutyPercent -= BLUETOOTH_DUTY_STEP_PERCENT;
            } else {
                gDutyPercent = BLUETOOTH_MIN_DUTY_PERCENT;
            }
            if (gMotion != BLUETOOTH_MOTION_STOP) {
                gLastMotionCommandMs = nowMs;
                apply_motion(gMotion);
            }
            return true;
        case '\r':
        case '\n':
        case ' ':
        case '\t':
            return false;
        default:
            gErrorCount++;
            return false;
    }
}

void BluetoothControl_Init(void)
{
    gRxHead              = 0;
    gRxTail              = 0;
    gRxCount             = 0;
    gErrorCount          = 0;
    gMotion              = BLUETOOTH_MOTION_STOP;
    gDutyPercent         = BLUETOOTH_INITIAL_DUTY_PERCENT;
    gLastMotionCommandMs = 0;
    gFailsafeCount       = 0;
    TB6612_Stop();
}

void BluetoothControl_PushRxFromIsr(uint8_t byte)
{
    uint8_t next = (uint8_t) ((gRxHead + 1U) % BLUETOOTH_RX_BUFFER_SIZE);

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
        gRxTail = (uint8_t) ((gRxTail + 1U) % BLUETOOTH_RX_BUFFER_SIZE);
        changed = handle_byte(byte, nowMs) || changed;
    }
    return changed;
}

bool BluetoothControl_CheckFailsafe(uint32_t nowMs)
{
    if ((gMotion != BLUETOOTH_MOTION_STOP) &&
        ((uint32_t) (nowMs - gLastMotionCommandMs) >= BLUETOOTH_FAILSAFE_MS)) {
        apply_motion(BLUETOOTH_MOTION_STOP);
        gFailsafeCount++;
        return true;
    }
    return false;
}

void BluetoothControl_GetStatus(BluetoothControlStatus *status)
{
    status->motion              = gMotion;
    status->leftCommandPercent  = TB6612_GetLeftCommand();
    status->rightCommandPercent = TB6612_GetRightCommand();
    status->dutyPercent         = gDutyPercent;
    status->rxCount             = gRxCount;
    status->errorCount          = gErrorCount;
    status->failsafeCount       = gFailsafeCount;
}
