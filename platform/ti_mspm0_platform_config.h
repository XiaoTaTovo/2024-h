#ifndef H2024_TI_PLATFORM_CONFIG_H
#define H2024_TI_PLATFORM_CONFIG_H

/* Replace these placeholders with measured values before ground testing. */
#define H2024_WHEEL_DIAMETER_MM              (65.0f)
#define H2024_TRACK_WIDTH_MM                 (140.0f)
#define H2024_ENCODER_COUNTS_PER_WHEEL_REV   (1000.0f)
#define H2024_MOTOR_UNITS_PER_MM_S           (1.0f)
#define H2024_IMU_YAW_SIGN                   (1)//控制yaw的方向，目前假设向左转yaw增大

/* Motor board speed-loop PID (verified values Kp=40.0 Ki=4.9 Kd=0.0),
 * downloaded to the board during arm. Frames are spaced by this many ms. */
#define H2024_MOTOR_PID_KP                   (40.0f)
#define H2024_MOTOR_PID_KI                   (4.9f)
#define H2024_MOTOR_PID_KD                   (0.0f)
#define H2024_MOTOR_COMMAND_SPACING_MS       (50U)

#define H2024_GRAY_SETTLE_US                 (10U)
#define H2024_GRAY_SAMPLES_PER_CHANNEL       (4U)
#define H2024_ADC_TIMEOUT_LOOPS              (100000U)

/* These bounds prevent a failed peripheral from hanging the whole car. */
#define H2024_UART_TX_TIMEOUT_LOOPS          (100000U)
#define H2024_SPI_TIMEOUT_LOOPS              (100000U)
#define H2024_UART_RX_BUFFER_SIZE            (64U)

#endif
