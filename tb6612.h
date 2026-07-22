#ifndef TB6612_H
#define TB6612_H

#include <stdint.h>

#define TB6612_PWM_PERIOD_TICKS       (1600U)
#define TB6612_MAX_DUTY_PERCENT       (60U)

/* Set a value to 0 if that wheel's vehicle-forward direction is reversed. */
#define TB6612_LEFT_FORWARD_IN1_HIGH  (0)
#define TB6612_RIGHT_FORWARD_IN1_HIGH (0)

void TB6612_Init(void);
void TB6612_Stop(void);
void TB6612_SetMotors(int8_t leftPercent, int8_t rightPercent);
int8_t TB6612_GetLeftCommand(void);
int8_t TB6612_GetRightCommand(void);

#endif
