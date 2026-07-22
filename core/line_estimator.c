#include "core/line_estimator.h"

CarStatus CarLineEstimator_Update(const CarConfig *config,
                                  const CarGraySample *sample,
                                  CarLineEstimate *estimate)
//sample是读取的原始数据，包括  8个灰度传感器的归一化值、时间戳、有效否
//estimate是线的估计数据，包括  线的位置、置信度、活跃计数、时间戳、有效否
{
    static const int16_t weights[8] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    uint32_t sum = 0U;
    int32_t weighted_sum = 0;
    uint8_t active_count = 0U;

    if ((config == 0) || (sample == 0) || (estimate == 0)) {
        return CAR_ERROR_ARG;
    }

    *estimate = (CarLineEstimate){0};
    estimate->timestamp_ms = sample->timestamp_ms;

    if (!sample->valid) {
        return CAR_OK;
    }

    for (uint8_t i = 0U; i < 8U; i++) {
        uint16_t value = sample->normalized[i];
        if (value >= config->gray_min_signal) {
            sum += value;
            weighted_sum += (int32_t)value * weights[i];
            active_count++;
        }
    }

    estimate->confidence = (sum > UINT16_MAX) ? UINT16_MAX : (uint16_t)sum;
    estimate->active_count = active_count;
    if ((sum >= config->gray_min_confidence) && (active_count > 0U)) {
        estimate->position = (int16_t)(weighted_sum / (int32_t)sum);
        estimate->valid = true;
    }

    return CAR_OK;
}
