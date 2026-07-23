#ifndef H2024_TI_PLATFORM_CONFIG_H
#define H2024_TI_PLATFORM_CONFIG_H

/* Replace these placeholders with measured values before ground testing. */
#define H2024_WHEEL_DIAMETER_MM              (65.0f)
#define H2024_TRACK_WIDTH_MM                 (140.0f)
#define H2024_ENCODER_COUNTS_PER_WHEEL_REV   (1000.0f)
#define H2024_MOTOR_UNITS_PER_MM_S           (1.0f)
#define H2024_MOTOR_BACKEND_TB6612           (1U)
#define H2024_TB6612_SPEED_UNITS_AT_MAX_DUTY (350)
#define H2024_IMU_BIAS_DPS                   (-0.45f)
#define H2024_IMU_USE_FIXED_BIAS             (0U)
#define H2024_IMU_CALIBRATION_SAMPLES       (400U)
#define H2024_IMU_YAW_SIGN                   (1)//控制yaw的方向，目前假设向左转yaw增大

/* Reserved for the retained serial motor-board backend. TB6612 task mode
 * ignores these register/PID values and uses the direct adapter below. */
#define H2024_MOTOR_PID_KP                   (40.0f)
#define H2024_MOTOR_PID_KI                   (4.9f)
#define H2024_MOTOR_PID_KD                   (0.0f)
#define H2024_MOTOR_COMMAND_SPACING_MS       (50U)

/* Start line control with P only; tune Ki/Kd after the TB6612 baseline. */
#define H2024_LINE_PID_KP                    (0.020f)
#define H2024_LINE_PID_KI                    (0.0f)
#define H2024_LINE_PID_KD                    (0.0f)
#define H2024_LINE_PID_INTEGRAL_LIMIT        (5000.0f)

#define H2024_GRAY_SETTLE_US                 (10U)
#define H2024_GRAY_SAMPLES_PER_CHANNEL       (4U)
#define H2024_ADC_TIMEOUT_LOOPS              (100000U)

/* These bounds prevent a failed peripheral from hanging the whole car. */
#define H2024_UART_TX_TIMEOUT_LOOPS          (100000U)
#define H2024_SPI_TIMEOUT_LOOPS              (100000U)
#define H2024_UART_RX_BUFFER_SIZE            (64U)

#endif
