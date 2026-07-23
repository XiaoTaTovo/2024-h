#ifndef H2024_ROUTE_EXECUTOR_H
#define H2024_ROUTE_EXECUTOR_H

#include "car_config.h"
#include "car_types.h"
#include "core/line_estimator.h"
#include "core/odometry.h"

#define CAR_ROUTE_MAX_SEGMENTS (64U)

typedef enum {
    CAR_SEGMENT_STRAIGHT = 0,
    CAR_SEGMENT_TURN,
    CAR_SEGMENT_ARC,
    CAR_SEGMENT_CUE,
    CAR_SEGMENT_STOP
} CarSegmentType;

typedef struct {
    CarSegmentType type;
    float value;
    float speed;
    uint32_t timeout_ms;
    bool use_line;
    CarCue cue;
} CarRouteSegment;

typedef struct {
    CarRouteSegment segments[CAR_ROUTE_MAX_SEGMENTS];
    uint16_t count;
} CarRoute;

typedef struct {
    const CarRoute *route;
    uint16_t index;
    uint32_t segment_start_ms;
    float segment_start_distance_mm;
    float segment_start_yaw_deg;
    float line_integral;
    float line_previous_error;
    uint32_t line_previous_ms;
    bool running;
    bool finished;
} CarRouteExecutor;

void CarRouteExecutor_Init(CarRouteExecutor *executor);
CarStatus CarRouteExecutor_Start(CarRouteExecutor *executor,
                                 const CarRoute *route,
                                 uint32_t now_ms,
                                 const CarOdometry *odometry,
                                 float yaw_deg);
CarStatus CarRouteExecutor_Update(CarRouteExecutor *executor,
                                  const CarConfig *config,
                                  uint32_t now_ms,
                                  const CarOdometry *odometry,
                                  float yaw_deg,
                                  const CarLineEstimate *line,
                                  CarMotorCommand *motor,
                                  CarCue *cue,
                                  uint32_t *faults);
bool CarRouteExecutor_GrayRequired(const CarRouteExecutor *executor);

#endif
