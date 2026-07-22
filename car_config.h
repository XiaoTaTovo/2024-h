#ifndef H2024_CAR_CONFIG_H
#define H2024_CAR_CONFIG_H

#include <stdint.h>

typedef struct {
    float wheel_diameter_mm;
    float track_width_mm;
    float encoder_counts_per_wheel_rev;

    float straight_speed_mm_s;
    float arc_speed_mm_s;
    float turn_wheel_speed_mm_s;
    float max_wheel_speed_mm_s;

    float distance_tolerance_mm;
    float angle_tolerance_deg;
    float straight_heading_kp;
    float arc_line_kp;

    uint32_t encoder_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t gray_timeout_ms;

    uint32_t straight_timeout_ms;
    uint32_t arc_timeout_ms;
    uint32_t turn_timeout_ms;

    uint16_t gray_min_signal;
    uint16_t gray_min_confidence;
} CarConfig;

static inline CarConfig CarConfig_MakeDefault(void)
{
    CarConfig config = {
        65.0f,
        140.0f,
        1000.0f,
        180.0f,
        140.0f,
        90.0f,
        350.0f,
        5.0f,
        3.0f,
        2.0f,
        0.0f,
        150U,
        150U,
        100U,
        15000U,
        15000U,
        6000U,
        80U,
        200U
    };
    return config;
}

#endif
