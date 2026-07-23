#include "app/h2024_task.h"

#define H2024_AB_MM (1000.0f)
#define H2024_BC_MM (800.0f)
#define H2024_ARC_RADIUS_MM (400.0f)
#define H2024_DIAGONAL_MM (1280.6248f)
#define H2024_DIAGONAL_TURN_DEG (38.6598f)
#define H2024_LOOP_TURN_DEG (141.3402f)
#define H2024_DEBUG_TURN_DEG (30.0f)

static CarStatus H2024_Add(CarRoute *route,
                           CarSegmentType type,
                           float value,
                           float speed,
                           uint32_t timeout_ms,
                           bool use_line,
                           CarCue cue)
{
    CarRouteSegment *segment;

    if ((route == 0) || (route->count >= CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_CAPACITY;
    }
    segment = &route->segments[route->count++];
    *segment = (CarRouteSegment){type, value, speed, timeout_ms, use_line, cue};
    return CAR_OK;
}

static CarStatus H2024_AddCue(CarRoute *route, CarCue cue)
{
    return H2024_Add(route, CAR_SEGMENT_CUE, 0.0f, 0.0f, 0U, false, cue);
}

static CarStatus H2024_AddItem3Loop(CarRoute *route,
                                    const CarConfig *config,
                                    bool add_entry_turn)
{
    if (add_entry_turn &&
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_LOOP_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    if ((H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_DIAGONAL_MM,
                   config->straight_speed_mm_s, config->straight_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_ARC, H2024_ARC_RADIUS_MM,
                   config->arc_speed_mm_s, config->arc_timeout_ms,
                   true, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_DIAGONAL_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_DIAGONAL_MM,
                   config->straight_speed_mm_s, config->straight_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_TURN, H2024_LOOP_TURN_DEG,
                   config->turn_wheel_speed_mm_s, config->turn_timeout_ms,
                   false, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_Add(route, CAR_SEGMENT_ARC, H2024_ARC_RADIUS_MM,
                   config->arc_speed_mm_s, config->arc_timeout_ms,
                   true, CAR_CUE_NONE) != CAR_OK) ||
        (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}

CarStatus H2024_BuildRoute(H2024Mode mode,
                           const CarConfig *config,
                           CarRoute *route)
{
    if ((config == 0) || (route == 0)) {
        return CAR_ERROR_ARG;
    }
    *route = (CarRoute){0};

    if (H2024_AddCue(route, CAR_CUE_START) != CAR_OK) {
        return CAR_ERROR_CAPACITY;
    }

    switch (mode) {
        case H2024_MODE_ITEM_1:
            if ((H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_AB_MM,
                           config->straight_speed_mm_s,
                           config->straight_timeout_ms, false,
                           CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2024_MODE_ITEM_2:
            if ((H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_AB_MM,
                           config->straight_speed_mm_s,
                           config->straight_timeout_ms, false,
                           CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
                (H2024_Add(route, CAR_SEGMENT_ARC, -H2024_ARC_RADIUS_MM,
                           config->arc_speed_mm_s, config->arc_timeout_ms,
                           true, CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
                (H2024_Add(route, CAR_SEGMENT_STRAIGHT, H2024_AB_MM,
                           config->straight_speed_mm_s,
                           config->straight_timeout_ms, false,
                           CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK) ||
                (H2024_Add(route, CAR_SEGMENT_ARC, -H2024_ARC_RADIUS_MM,
                           config->arc_speed_mm_s, config->arc_timeout_ms,
                           true, CAR_CUE_NONE) != CAR_OK) ||
                (H2024_AddCue(route, CAR_CUE_CHECKPOINT) != CAR_OK)) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2024_MODE_ITEM_3:
            if (H2024_AddItem3Loop(route, config, false) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        case H2024_MODE_ITEM_4:
            for (uint8_t loop = 0U; loop < 4U; loop++) {
                if (H2024_AddItem3Loop(route, config, loop > 0U) != CAR_OK) {
                    return CAR_ERROR_CAPACITY;
                }
            }
            break;

        case H2024_MODE_TURN_DEBUG:
            if (H2024_Add(route, CAR_SEGMENT_TURN, H2024_DEBUG_TURN_DEG,
                          config->turn_wheel_speed_mm_s,
                          config->turn_timeout_ms, false,
                          CAR_CUE_NONE) != CAR_OK) {
                return CAR_ERROR_CAPACITY;
            }
            break;

        default:
            return CAR_ERROR_ARG;
    }

    if (H2024_Add(route, CAR_SEGMENT_STOP, 0.0f, 0.0f, 0U,
                  false, CAR_CUE_FINISH) != CAR_OK) {
        return CAR_ERROR_CAPACITY;
    }
    return CAR_OK;
}
