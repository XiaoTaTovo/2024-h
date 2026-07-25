#include "core/route_executor.h"

static float CarRoute_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}
//abs 绝对值
static float CarRoute_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}
//clamp 限幅
static void CarRoute_Stop(CarMotorCommand *motor)
{
    *motor = (CarMotorCommand){0.0f, 0.0f, false};
}
//stop 停车
static void CarRoute_Advance(CarRouteExecutor *executor,
                             uint32_t now_ms,
                             const CarOdometry *odometry,
                             float yaw_deg)
{
    executor->index++;
    executor->segment_start_ms = now_ms;
    executor->segment_start_distance_mm = odometry->center_distance_mm;
    executor->segment_start_yaw_deg = yaw_deg;
    executor->line_integral = 0.0f;
    executor->line_previous_error = 0.0f;
    executor->line_previous_ms = now_ms;
    if (executor->index >= executor->route->count) {
        executor->running = false;
        executor->finished = true;
    }
}

void CarRouteExecutor_Init(CarRouteExecutor *executor)
{
    if (executor != 0) {
        *executor = (CarRouteExecutor){0};
    }
}

CarStatus CarRouteExecutor_Start(CarRouteExecutor *executor,
                                 const CarRoute *route,
                                 uint32_t now_ms,
                                 const CarOdometry *odometry,
                                 float yaw_deg)
{
    if ((executor == 0) || (route == 0) || (odometry == 0) ||
        (route->count == 0U) || (route->count > CAR_ROUTE_MAX_SEGMENTS)) {
        return CAR_ERROR_ARG;
    }

    executor->route = route;
    executor->index = 0U;
    executor->segment_start_ms = now_ms;
    executor->segment_start_distance_mm = odometry->center_distance_mm;
    executor->segment_start_yaw_deg = yaw_deg;
    executor->line_integral = 0.0f;
    executor->line_previous_error = 0.0f;
    executor->line_previous_ms = now_ms;
    executor->running = true;
    executor->finished = false;
    return CAR_OK;
}

CarStatus CarRouteExecutor_Update(CarRouteExecutor *executor,
                                  const CarConfig *config,
                                  uint32_t now_ms,
                                  const CarOdometry *odometry,
                                  float yaw_deg,
                                  const CarLineEstimate *line,
                                  CarMotorCommand *motor,
                                  CarCue *cue,
                                  uint32_t *faults)
{
    const CarRouteSegment *segment;
    float progress;
    float error;
    float correction;

    if ((executor == 0) || (config == 0) || (odometry == 0) ||
        (motor == 0) || (cue == 0) || (faults == 0)) {
        return CAR_ERROR_ARG;
    }

    *cue = CAR_CUE_NONE;
    CarRoute_Stop(motor);
    //默认停车
    if (!executor->running) {
        return CAR_OK;
    }
    //如果没有跑，直接返回
    //后续根据状态去覆盖
    if ((executor->route == 0) || (executor->index >= executor->route->count)) {
        *faults |= CAR_FAULT_ROUTE_INVALID;
        executor->running = false;
        return CAR_ERROR_STATE;
    }

    segment = &executor->route->segments[executor->index];
    if ((segment->timeout_ms > 0U) &&
        ((uint32_t)(now_ms - executor->segment_start_ms) > segment->timeout_ms)) {
        *faults |= CAR_FAULT_SEGMENT_TIMEOUT;
        executor->running = false;
        return CAR_ERROR_STATE;
    }

    switch (segment->type) {
        case CAR_SEGMENT_STRAIGHT:
            progress = odometry->center_distance_mm -
                       executor->segment_start_distance_mm;
            if (CarRoute_Abs(progress) + config->distance_tolerance_mm >=
                CarRoute_Abs(segment->value)) {
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg);
                break;
            }
            error = executor->segment_start_yaw_deg - yaw_deg;
            correction = CarRoute_Clamp(error * config->straight_heading_kp,
                                        config->max_wheel_speed_mm_s * 0.4f);
            motor->left_mm_s = segment->speed - correction;
            motor->right_mm_s = segment->speed + correction;
            motor->enable = true;
            break;
        //直线段行驶逻辑，现在的假设是车头向左偏，yaw增大
        //error为原来0-现在，为负，pid计算correction为负
        //左轮减一个负数就是加速，所以向哪边偏说明哪边慢了，要加速


        case CAR_SEGMENT_TURN:
        {
            float target_yaw = executor->segment_start_yaw_deg + segment->value;
            float turn_direction = (segment->value >= 0.0f) ? 1.0f : -1.0f;//控制方向，我们现在的逻辑是左转为正，右转为负
            float turn_error = (target_yaw - yaw_deg) * turn_direction;
            float turn_speed;

            if (turn_error <= config->angle_tolerance_deg) {
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg);
                break;
            }//最小的这个角度误差，就不用动了

            /* Proportional yaw loop with a small floor to overcome stiction. */
            turn_speed = turn_error * config->turn_heading_kp;
            if (turn_speed < config->turn_min_speed_mm_s) {
                turn_speed = config->turn_min_speed_mm_s;
            }//克服静摩擦力加机械死区
            turn_speed = CarRoute_Clamp(
                turn_speed, CarRoute_Abs(segment->speed));
            if (segment->value > 0.0f) {
                motor->left_mm_s = 0.0f;
                motor->right_mm_s = turn_speed;
            } else {
                motor->left_mm_s = turn_speed;
                motor->right_mm_s = 0.0f;
            }//左转为正，所以左轮速度为0
            motor->enable = true;
            break;
        }
        //转弯段逻辑：先算目标值，就是起点加要转到的角度
        //error 差的角度
        case CAR_SEGMENT_ARC:
        {
            float radius = CarRoute_Abs(segment->value);
            float direction = (segment->value >= 0.0f) ? 1.0f : -1.0f;
            float inner_ratio;

            progress = yaw_deg - executor->segment_start_yaw_deg;
            if (CarRoute_Abs(progress) + config->angle_tolerance_deg >= 180.0f) {
                CarRoute_Advance(executor, now_ms, odometry, yaw_deg);
                break;
            }
            if (radius <= config->track_width_mm * 0.5f) {
                *faults |= CAR_FAULT_ROUTE_INVALID;
                executor->running = false;
                return CAR_ERROR_STATE;
            }

            inner_ratio = (radius - config->track_width_mm * 0.5f) /
                          (radius + config->track_width_mm * 0.5f);
            if (direction > 0.0f) {
                motor->left_mm_s = segment->speed * inner_ratio;
                motor->right_mm_s = segment->speed;
            } else {
                motor->left_mm_s = segment->speed;
                motor->right_mm_s = segment->speed * inner_ratio;
            }
            if (segment->use_line && (line != 0) && line->valid) {
                float line_error = (float)line->position;
                float dt_s = (float)(now_ms - executor->line_previous_ms) /
                             1000.0f;
                float derivative = 0.0f;

                if ((dt_s > 0.0f) && (dt_s <= 0.25f)) {
                    derivative = (line_error - executor->line_previous_error) /
                                 dt_s;
                    executor->line_integral += line_error * dt_s;
                } else {
                    dt_s = 0.01f;
                }
                if (config->arc_line_integral_limit > 0.0f) {
                    executor->line_integral = CarRoute_Clamp(
                        executor->line_integral,
                        config->arc_line_integral_limit);
                } else {
                    executor->line_integral = 0.0f;
                }
                correction = line_error * config->arc_line_kp +
                             executor->line_integral * config->arc_line_ki +
                             derivative * config->arc_line_kd;
                executor->line_previous_error = line_error;
                executor->line_previous_ms = now_ms;
                correction = CarRoute_Clamp(
                    correction, config->max_wheel_speed_mm_s * 0.25f);
                /* Positive line position means the line is on the right.
                 * Increase the left wheel and slow the right wheel to steer
                 * toward it; this matches the TB6612 forward convention. */
                motor->left_mm_s += correction;
                motor->right_mm_s -= correction;
            }
            motor->enable = true;
            break;
        }
        //圆弧段
        case CAR_SEGMENT_CUE:
            *cue = segment->cue;
            CarRoute_Advance(executor, now_ms, odometry, yaw_deg);
            break;
        //触发声光提示段
        case CAR_SEGMENT_STOP:
            *cue = segment->cue;
            CarRoute_Advance(executor, now_ms, odometry, yaw_deg);
            break;
        //停止段
        default:
            *faults |= CAR_FAULT_ROUTE_INVALID;
            executor->running = false;
            return CAR_ERROR_STATE;
    }

    motor->left_mm_s = CarRoute_Clamp(motor->left_mm_s,
                                      config->max_wheel_speed_mm_s);
    motor->right_mm_s = CarRoute_Clamp(motor->right_mm_s,
                                       config->max_wheel_speed_mm_s);
    return CAR_OK;
}

bool CarRouteExecutor_GrayRequired(const CarRouteExecutor *executor)
{
    if ((executor == 0) || !executor->running ||
        (executor->route == 0) || (executor->index >= executor->route->count)) {
        return false;
    }
    return executor->route->segments[executor->index].use_line;
}
