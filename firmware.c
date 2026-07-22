#include "firmware.h"

static float CarFirmware_SelectGyro(const CarFirmware *firmware,
                                    const Icm42688Sample *sample)
{
    int16_t raw;

    switch (firmware->config.yaw_axis) {
        case CAR_IMU_AXIS_X:
            raw = sample->gyro_x;
            break;
        case CAR_IMU_AXIS_Y:
            raw = sample->gyro_y;
            break;
        case CAR_IMU_AXIS_Z:
        default:
            raw = sample->gyro_z;
            break;
    }
    return ((float)raw / firmware->imu.gyro_lsb_per_dps) *
           (float)firmware->config.yaw_sign;
}

static int16_t CarFirmware_ToMotorUnits(const CarFirmware *firmware,
                                        float speed_mm_s)
{
    float scaled = speed_mm_s * firmware->config.motor_units_per_mm_s;

    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static bool CarFirmware_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void CarFirmware_ScheduleMotorPrepare(CarFirmware *firmware,
                                             uint32_t now_ms)
{
    firmware->motor_prepare_next_ms =
        now_ms + firmware->config.motor_command_spacing_ms;
}

static bool CarFirmware_BeginMotorPrepare(CarFirmware *firmware,
                                          uint32_t now_ms)
{
    if (!MotorBoard_Stop(&firmware->motor)) {
        return false;
    }
    firmware->motor_armed = false;
    firmware->motor_prepare_active = true;
    firmware->motor_prepare_step = CAR_MOTOR_PREP_POLARITY_A;
    CarFirmware_ScheduleMotorPrepare(firmware, now_ms);
    return true;
}

static bool CarFirmware_StepMotorPrepare(CarFirmware *firmware,
                                         uint32_t now_ms)
{
    bool sent;

    if (!firmware->motor_prepare_active ||
        !CarFirmware_TimeReached(now_ms, firmware->motor_prepare_next_ms)) {
        return true;
    }

    /*
     * The verified example enables closed loop first. The tested board can
     * run away at a zero target when polarity is still at its power-on value,
     * so this port keeps the verified values but applies polarity before
     * enabling closed loop.
     */
    while (firmware->motor_prepare_active) {
        switch (firmware->motor_prepare_step) {
            case CAR_MOTOR_PREP_POLARITY_A:
            case CAR_MOTOR_PREP_POLARITY_B:
            case CAR_MOTOR_PREP_POLARITY_C:
            case CAR_MOTOR_PREP_POLARITY_D:
            {
                MotorBoardChannel channel = (MotorBoardChannel)(
                    firmware->motor_prepare_step -
                    CAR_MOTOR_PREP_POLARITY_A);

                firmware->motor_prepare_step = (CarMotorPrepareStep)(
                    firmware->motor_prepare_step + 1);
                if (!firmware->config.set_encoder_polarity_on_arm) {
                    continue;
                }
                sent = MotorBoard_SetEncoderPolarity(
                    &firmware->motor, channel,
                    firmware->config.encoder_polarity[channel]);
                break;
            }
            case CAR_MOTOR_PREP_PID:
                firmware->motor_prepare_step =
                    CAR_MOTOR_PREP_ENABLE_CLOSED_LOOP;
                if (!firmware->config.set_speed_pid_on_arm) {
                    continue;
                }
                sent = MotorBoard_SetAllPid(
                    &firmware->motor, firmware->config.speed_pid);
                break;
            case CAR_MOTOR_PREP_ENABLE_CLOSED_LOOP:
                sent = MotorBoard_SetClosedLoop(&firmware->motor, true);
                firmware->motor_prepare_step = CAR_MOTOR_PREP_FINAL_ZERO;
                break;
            case CAR_MOTOR_PREP_FINAL_ZERO:
                sent = MotorBoard_Stop(&firmware->motor);
                if (sent) {
                    firmware->motor_prepare_active = false;
                    firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
                    firmware->motor_armed = true;
                    return true;
                }
                break;
            case CAR_MOTOR_PREP_IDLE:
            default:
                sent = false;
                break;
        }

        if (!sent) {
            firmware->motor_prepare_active = false;
            firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
            return false;
        }
        CarFirmware_ScheduleMotorPrepare(firmware, now_ms);
        return true;
    }
    return true;
}

static void CarFirmware_ApplyOutput(CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware->output.faults != CAR_FAULT_NONE) ||
        (firmware->hardware_faults != CAR_FAULT_NONE)) {
        firmware->motor_prepare_active = false;
        firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
        (void)MotorBoard_EmergencyStop(&firmware->motor);
        firmware->motor_armed = false;
        return;
    }

    if (firmware->output.motor.enable) {
        if (!firmware->motor_armed) {
            if (!firmware->motor_prepare_active &&
                !CarFirmware_BeginMotorPrepare(firmware, now_ms)) {
                CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
            }
            return;
        }
        if (!MotorBoard_SetWheelSpeeds(
                &firmware->motor,
                CarFirmware_ToMotorUnits(firmware,
                                         firmware->output.motor.left_mm_s),
                CarFirmware_ToMotorUnits(firmware,
                                         firmware->output.motor.right_mm_s))) {
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
        }
    } else if (firmware->motor_prepare_active) {
        return;
    } else if (firmware->motor_armed ||
               CarPeriodicTask_Due(&firmware->stop_refresh_task, now_ms)) {
        if (!MotorBoard_Stop(&firmware->motor)) {
            CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
        }
    }
}

static void CarFirmware_RunImu(CarFirmware *firmware, uint32_t now_ms)
{
    Icm42688Sample raw = {0};

    if (!Icm42688_ReadSample(&firmware->imu, now_ms, &raw)) {
        firmware->imu_sample.valid = false;
        return;
    }
    (void)CarYawEstimator_Update(
        &firmware->yaw, CarFirmware_SelectGyro(firmware, &raw), now_ms);
    (void)CarYawEstimator_GetSample(&firmware->yaw, &firmware->imu_sample);
}

static void CarFirmware_RunControl(CarFirmware *firmware, uint32_t now_ms)
{
    CarInputSnapshot input = {0};
    bool start_event = Button_TakePressedEvent(&firmware->button);

    (void)MotorBoard_GetEncoderSample(&firmware->motor, &input.encoder);
    input.imu = firmware->imu_sample;
    input.gray = firmware->gray_sample;
    input.start_pressed = start_event;

    if (start_event && firmware->app.executor.running) {
        input.emergency_stop = true;
    } else if (start_event && !firmware->app.armed) {
        CarYawEstimator_ResetYaw(&firmware->yaw, 0.0f);
        input.imu.yaw_deg = 0.0f;
        if (CarApp_Arm(&firmware->app, firmware->config.mode,
                       now_ms, &input) != CAR_OK) {
            Buzzer_PlayCue(&firmware->buzzer, CAR_CUE_FAULT, now_ms);
        }
    }

    (void)CarApp_Update(&firmware->app, now_ms, &input, &firmware->output);
    firmware->output.faults |= firmware->hardware_faults;
    if ((firmware->output.cue != CAR_CUE_NONE) &&
        (firmware->output.cue != firmware->last_output_cue)) {
        Buzzer_PlayCue(&firmware->buzzer, firmware->output.cue, now_ms);
    }
    firmware->last_output_cue = firmware->output.cue;
    CarFirmware_ApplyOutput(firmware, now_ms);
}

CarStatus CarFirmware_Init(CarFirmware *firmware,
                           const CarFirmwareConfig *config,
                           uint32_t now_ms)
{
    if ((firmware == 0) || (config == 0) ||
        (config->motor_units_per_mm_s <= 0.0f) ||
        (config->motor_command_spacing_ms == 0U) ||
        ((config->yaw_sign != 1) && (config->yaw_sign != -1))) {
        return CAR_ERROR_ARG;
    }

    *firmware = (CarFirmware){0};
    firmware->config = *config;
    MotorBoard_Init(&firmware->motor, &config->motor);
    Icm42688_InitObject(&firmware->imu, &config->imu);
    GrayArray_Init(&firmware->gray, &config->gray);
    Button_Init(&firmware->button, config->button_read,
                config->button_context, config->button_active_low,
                config->button_debounce_ms);
    Buzzer_Init(&firmware->buzzer, config->buzzer_set,
                config->buzzer_context);
    CarYawEstimator_Init(&firmware->yaw,
                         config->imu_calibration_samples,
                         config->imu_max_step_ms);
    if (CarApp_Init(&firmware->app, &config->car) != CAR_OK) {
        return CAR_ERROR_ARG;
    }

    if (!Icm42688_Initialize(&firmware->imu)) {
        firmware->hardware_faults |= CAR_FAULT_IMU_INIT;
    }
    if ((!config->gray_calibration_valid ||
         !GrayArray_SetCalibration(&firmware->gray,
                                   config->gray_black,
                                   config->gray_white)) &&
        (config->mode != H2024_MODE_ITEM_1)) {
        firmware->hardware_faults |= CAR_FAULT_GRAY_NOT_CALIBRATED;
    }

    CarPeriodicTask_Init(&firmware->imu_task, 5U, now_ms);
    CarPeriodicTask_Init(&firmware->gray_task, 10U, now_ms);
    CarPeriodicTask_Init(&firmware->control_task, 20U, now_ms);
    CarPeriodicTask_Init(&firmware->stop_refresh_task, 100U, now_ms);
    firmware->initialized = true;
    if ((firmware->hardware_faults == CAR_FAULT_NONE) &&
        !CarFirmware_BeginMotorPrepare(firmware, now_ms)) {
        CarFirmware_ForceStop(firmware, CAR_FAULT_MOTOR_IO);
    } else if (firmware->hardware_faults != CAR_FAULT_NONE) {
        (void)MotorBoard_Stop(&firmware->motor);
    }
    return CAR_OK;
}

void CarFirmware_OnMotorRxByte(CarFirmware *firmware,
                               uint8_t byte,
                               uint32_t now_ms)
{
    if ((firmware != 0) && firmware->initialized) {
        MotorBoard_OnRxByte(&firmware->motor, byte, now_ms);
    }
}

void CarFirmware_Tick(CarFirmware *firmware, uint32_t now_ms)
{
    if ((firmware == 0) || !firmware->initialized) {
        return;
    }
    (void)CarFirmware_StepMotorPrepare(firmware, now_ms);
    Button_Update(&firmware->button, now_ms);
    Buzzer_Update(&firmware->buzzer, now_ms);

    if (CarPeriodicTask_Due(&firmware->imu_task, now_ms)) {
        CarFirmware_RunImu(firmware, now_ms);
    }
    if (CarPeriodicTask_Due(&firmware->gray_task, now_ms)) {
        if (GrayArray_Read(&firmware->gray, now_ms)) {
            (void)GrayArray_GetLatest(&firmware->gray, &firmware->gray_sample);
        }
    }
    if (CarPeriodicTask_Due(&firmware->control_task, now_ms)) {
        CarFirmware_RunControl(firmware, now_ms);
    }
}

void CarFirmware_ForceStop(CarFirmware *firmware, uint32_t fault)
{
    if (firmware == 0) {
        return;
    }
    firmware->hardware_faults |= fault;
    CarApp_Stop(&firmware->app, fault);
    firmware->output = (CarOutputSnapshot){0};
    firmware->output.faults = firmware->hardware_faults;
    firmware->output.cue = CAR_CUE_FAULT;
    firmware->motor_prepare_active = false;
    firmware->motor_prepare_step = CAR_MOTOR_PREP_IDLE;
    (void)MotorBoard_EmergencyStop(&firmware->motor);
    firmware->motor_armed = false;
}

const CarOutputSnapshot *CarFirmware_GetOutput(const CarFirmware *firmware)
{
    return (firmware == 0) ? 0 : &firmware->output;
}
