#ifndef H2024_LINE_ESTIMATOR_H
#define H2024_LINE_ESTIMATOR_H

#include "car_config.h"
#include "car_types.h"

typedef struct {
    int16_t position;//线的位置，左边为负，右边为正，中间为0，记忆方法是想象数轴，以后也这样
    uint16_t confidence;
    uint8_t active_count;
    uint32_t timestamp_ms;
    bool valid;
} CarLineEstimate;

CarStatus CarLineEstimator_Update(const CarConfig *config,
                                  const CarGraySample *sample,
                                  CarLineEstimate *estimate);

#endif
