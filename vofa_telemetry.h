#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include <stdint.h>

#include "bluetooth_control.h"

void VofaTelemetry_Send(const BluetoothControlStatus *status, uint32_t uptimeMs);

#endif
