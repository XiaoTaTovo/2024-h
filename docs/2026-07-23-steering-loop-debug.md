# Steering loop debug record

## Implemented path

- The IMU task runs every 5 ms and integrates the calibrated yaw rate.
- The control task runs every 20 ms and calls `CarRouteExecutor_Update`.
- A `CAR_SEGMENT_TURN` now uses a proportional yaw loop. The remaining angle
  selects the turn speed, while the motor board speed loop remains the inner loop.
- Positive route angles keep the existing convention: left wheel is stopped and
  right wheel moves forward. Negative angles use the opposite wheel.

The initial parameters are `turn_heading_kp = 2.0` and
`turn_min_speed_mm_s = 30.0`. They are starting values, not measured results.

## Required bench sequence

1. Keep the car still and level after power-up. Wait for 400 IMU samples
   (about 2 seconds at the current 5 ms period).
2. Confirm the estimator is calibrated and yaw is stable. Rotate the chassis by
   hand in both directions and confirm the sign of `yaw_deg`.
3. In CCS select `PROJECT_MODE_TURN_DEBUG`. Lift the wheels and run the
   built-in 30 degree positive turn. Check that the commanded wheel and the yaw
   sign agree. Stop immediately if the yaw moves away from the target. Repeat
   with a negative value in `H2024_DEBUG_TURN_DEG`.
4. Put the car on the floor and run the same low-speed 30 degree turn. Record the
   start yaw, target yaw, final yaw, overshoot, elapsed time, and any timeout.
5. Repeat the same turn ten times. Only after this passes should the turn
   segments be used in the H2024 route.

Change one parameter per test: first `turn_heading_kp`, then
`turn_min_speed_mm_s`, then `angle_tolerance_deg`. Do not retune the motor-board
speed PID during this step.

## Blocking conditions

- If the yaw sign is wrong, change the platform `H2024_IMU_YAW_SIGN` after
  rechecking the physical axis; do not compensate with the turn gain.
- If yaw is not calibrated or becomes stale, the safety supervisor must stop
  the route. A successful host build does not prove the IMU wiring or motor
  polarity on the real car.
- The PCB-verified mapping is `KEY1=PB23`, `KEY2=PB26`, `KEY3=PB27`.
  Regenerate `ti_msp_dl_config.h/.c` in CCS and verify these generated macros
  before burning the board.

## Telemetry naming

The VOFA CSV column order is unchanged. Only the two labels at positions 13 and
14 changed from `ppr_l,ppr_r` to `cpr_l,cpr_r`; the Bluetooth command accepts
`CPR` and keeps `PPR` as a compatibility alias.

## Buttons and OLED

The button driver currently implements a debounced press event only. A short
press and a long press both produce one event after debounce; holding a button
does not auto-repeat and does not change parameters.

| Mode | KEY1 | KEY2 | KEY3 |
| --- | --- | --- | --- |
| Gray debug | capture white | capture black | retry after calibration error |
| H2024/turn debug | start route; press again for emergency stop | unused | unused |

H2024 and turn-debug modes now refresh one OLED page every 50 ms. The status
page shows mode, calibration state, gyro bias, yaw, route segment, wheel
commands, faults, and `KEYH:123`. `KEYH` is the raw level of KEY1/KEY2/KEY3:
an unpressed key is `1`, and a held key is `0`. This provides the first
hardware pin check before relying on a button action. The current firmware
uses OLED `SDA=PA0` and `SCL=PA1`; PB2/PB3 are reserved for `UART_BLUETOOTH`.
The gray calibration prompts also show `KEYH:123`, so all three keys can be
verified while the motor outputs remain disabled.

`H2024_MODE_TURN_DEBUG` does not require gray calibration because its route
contains no line-following segment. Gray calibration remains mandatory for
H2024 items 2 through 4.
