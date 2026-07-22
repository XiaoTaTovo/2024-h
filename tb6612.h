#ifndef TB6612_H
#define TB6612_H

#include <stdint.h>

#define TB6612_PWM_PERIOD_TICKS       (1600U)//占空比的分辨率
#define TB6612_MAX_DUTY_PERCENT       (60U)//最大占空比

/* Change only the affected value to 0 if a vehicle-forward command is reversed. */
#define TB6612_LEFT_FORWARD_IN1_HIGH  (1)//左轮前进AIN1高电平
//都需要实测，软件方面实现就是gpio给高和低
#define TB6612_RIGHT_FORWARD_IN1_HIGH (1)//右轮前进BIN1高电平

void TB6612_Init(void);
void TB6612_Stop(void);
void TB6612_SetMotors(int8_t leftPercent, int8_t rightPercent);
int8_t TB6612_GetLeftCommand(void);
int8_t TB6612_GetRightCommand(void);

#endif
