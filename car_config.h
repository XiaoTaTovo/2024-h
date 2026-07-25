#ifndef H2024_CAR_CONFIG_H
#define H2024_CAR_CONFIG_H

#include <stdint.h>

typedef struct {
    float wheel_diameter_mm;
    float track_width_mm;
    float encoder_counts_per_wheel_rev;

    float straight_speed_mm_s;//直线基础速度，注意和现在已知的这个基准做转换，比如现在是350，所以用
    float arc_speed_mm_s;
    float turn_wheel_speed_mm_s;//转向速度上限
    float max_wheel_speed_mm_s;

    float distance_tolerance_mm;
    float angle_tolerance_deg;//允许的误差最小角度
    float straight_heading_kp;//直线yaw的kp
    float arc_line_kp;
    float turn_heading_kp;//yaw转一定方向的kp
    float turn_min_speed_mm_s;//克服静摩擦力加机械死区

    uint32_t encoder_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t gray_timeout_ms;

    uint32_t straight_timeout_ms;
    uint32_t arc_timeout_ms;
    uint32_t turn_timeout_ms;

    uint16_t gray_min_signal;
    uint16_t gray_min_confidence;
    float arc_line_ki;
    float arc_line_kd;
    float arc_line_integral_limit;
} CarConfig;

static inline CarConfig CarConfig_MakeDefault(void)
{
    CarConfig config = {
        65.0f,
        140.0f,
        1000.0f,
        180.0f,//直线基础速度，注意和现在已知的这个基准做转换，比如现在是350，所以用
        140.0f,
        90.0f,
        350.0f,//转向速度上限
        5.0f,
        3.0f,//允许的误差最小角度
        2.0f,
        0.0f,
        2.0f,//yaw转向的kp，转速等于误差乘这个值
        70.0f,//克服静摩擦力加机械死区
        150U,
        150U,
        100U,
        15000U,
        15000U,
        6000U,
        80U,
        200U,
        0.0f,
        0.0f,
        0.0f
    };
    return config;
}

#endif
