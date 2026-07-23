#ifndef TB6612_H
#define TB6612_H

#include <stdbool.h>
#include <stdint.h>

#define TB6612_PWM_PERIOD_TICKS       (1600U)
#define TB6612_MAX_DUTY_PERCENT       (80U)//最大占空比

/* Set a value to 0 if that wheel's vehicle-forward direction is reversed. */
#define TB6612_LEFT_FORWARD_IN1_HIGH  (0)
#define TB6612_RIGHT_FORWARD_IN1_HIGH (0)

typedef uint32_t (*TB6612NowFn)(void *context);

typedef struct {
    TB6612NowFn now_ms;
    void *now_context;
    int16_t speed_units_at_max_duty;
} TB6612MotorBoardContext;

void TB6612_Init(void);
void TB6612_Stop(void);
void TB6612_SetMotors(int8_t leftPercent, int8_t rightPercent);
int8_t TB6612_GetLeftCommand(void);
int8_t TB6612_GetRightCommand(void);

void TB6612_MotorBoardContextInit(TB6612MotorBoardContext *context,
                                  TB6612NowFn now_ms,
                                  void *now_context,
                                  int16_t speed_units_at_max_duty);
bool TB6612_MotorBoard_SetWheelSpeeds(int16_t left,
                                      int16_t right,
                                      void *context);
bool TB6612_MotorBoard_GetEncoder(int16_t *left,
                                  int16_t *right,
                                  uint32_t *timestamp_ms,
                                  void *context);

#endif
