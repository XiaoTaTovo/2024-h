#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BLUETOOTH_MOTION_STOP = 0,
    BLUETOOTH_MOTION_FORWARD = 1,
    BLUETOOTH_MOTION_BACKWARD = 2,
    BLUETOOTH_MOTION_LEFT = 3,
    BLUETOOTH_MOTION_RIGHT = 4
} BluetoothMotion;

typedef struct {
    BluetoothMotion motion;
    int8_t leftCommandPercent;
    int8_t rightCommandPercent;
    uint8_t dutyPercent;
    uint32_t rxCount;
    uint32_t errorCount;
    uint32_t failsafeCount;
} BluetoothControlStatus;

void BluetoothControl_Init(void);
void BluetoothControl_PushRxFromIsr(uint8_t byte);
bool BluetoothControl_ProcessPending(uint32_t nowMs);
bool BluetoothControl_CheckFailsafe(uint32_t nowMs);
void BluetoothControl_GetStatus(BluetoothControlStatus *status);

#endif
