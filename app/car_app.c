#include "app/car_app.h"

#include "core/safety_supervisor.h"

CarStatus CarApp_Init(CarApp *app, const CarConfig *config)
{
    if ((app == 0) || (config == 0)) {
        return CAR_ERROR_ARG;
    }
    *app = (CarApp){0};
    app->config = *config;
    CarOdometry_Init(&app->odometry);
    CarRouteExecutor_Init(&app->executor);
    return CAR_OK;
}

CarStatus CarApp_Arm(CarApp *app,
                     H2024Mode mode,
                     uint32_t now_ms,
                     const CarInputSnapshot *input)
{
    CarStatus status;

    if ((app == 0) || (input == 0) || !input->encoder.valid || !input->imu.valid) {
        return CAR_ERROR_ARG;
    }
    CarOdometry_Init(&app->odometry);
    status = CarOdometry_Update(&app->odometry, &app->config, &input->encoder);
    if (status != CAR_OK) {
        return status;
    }
    status = H2024_BuildRoute(mode, &app->config, &app->route);
    if (status != CAR_OK) {
        app->faults |= CAR_FAULT_ROUTE_INVALID;
        return status;
    }
    status = CarRouteExecutor_Start(&app->executor, &app->route, now_ms,
                                    &app->odometry, input->imu.yaw_deg);
    if (status == CAR_OK) {
        app->faults = CAR_FAULT_NONE;
        app->armed = true;
    }
    return status;
}

CarStatus CarApp_Update(CarApp *app,
                        uint32_t now_ms,
                        const CarInputSnapshot *input,
                        CarOutputSnapshot *output)
{
    CarStatus status;
    CarMotorCommand motor = {0};
    CarCue cue = CAR_CUE_NONE;
    bool gray_required;
    uint32_t current_faults;

    if ((app == 0) || (input == 0) || (output == 0)) {
        return CAR_ERROR_ARG;
    }
    *output = (CarOutputSnapshot){0};

    if (input->encoder.valid) {
        (void)CarOdometry_Update(&app->odometry, &app->config, &input->encoder);
    }
    if (input->gray.valid) {
        (void)CarLineEstimator_Update(&app->config, &input->gray, &app->line);
    }

    gray_required = CarRouteExecutor_GrayRequired(&app->executor);
    current_faults = CarSafety_Evaluate(&app->config, now_ms, input,
                                        app->executor.running, gray_required);
    app->faults |= current_faults;
    if (app->faults != CAR_FAULT_NONE) {
        app->armed = false;
        app->executor.running = false;
        output->cue = CAR_CUE_FAULT;
        output->faults = app->faults;
        return CAR_ERROR_STATE;
    }

    if (!app->armed) {
        output->faults = app->faults;
        return CAR_OK;
    }

    status = CarRouteExecutor_Update(&app->executor, &app->config, now_ms,
                                     &app->odometry, input->imu.yaw_deg,
                                     &app->line, &motor, &cue, &app->faults);
    if (app->faults != CAR_FAULT_NONE) {
        motor = (CarMotorCommand){0};
        cue = CAR_CUE_FAULT;
        app->armed = false;
    }

    output->motor = motor;
    output->cue = cue;
    output->faults = app->faults;
    output->route_index = app->executor.index;
    output->route_running = app->executor.running;
    output->route_finished = app->executor.finished;
    if (app->executor.finished) {
        app->armed = false;
    }
    return status;
}

void CarApp_Stop(CarApp *app, uint32_t fault)
{
    if (app == 0) {
        return;
    }
    app->faults |= fault;
    app->armed = false;
    app->executor.running = false;
}
