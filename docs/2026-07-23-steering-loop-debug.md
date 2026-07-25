# Steering loop debug record

## Implemented path

- The IMU task runs every 5 ms and integrates the calibrated yaw rate.
- The control task runs every 20 ms and calls `CarRouteExecutor_Update`.
- A `CAR_SEGMENT_TURN` uses a proportional yaw loop. The remaining angle
  selects the requested wheel speed. In the current TB6612 task backend this
  request is mapped directly from mm/s units to PWM; there is no task speed PID.
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
   built-in 30 degree positive turn only long enough to confirm the commanded
   wheel and PWM. The chassis cannot produce yaw while supported, so stop or
   reset before the six-second segment timeout.
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
| H2024/turn debug | start route; press again for emergency stop | switch OLED P1/P2 | reserved |

H2024 and turn-debug modes render status at 200 ms intervals and transfer one
OLED hardware page per main-loop pass. The display no longer alternates pages
automatically. P1 is the route/output page and P2 is the arm/backend diagnostic
page. KEY2 selects the page. `K:123` is the raw level of KEY1/KEY2/KEY3: an
unpressed key is `1`, and a held key is `0`. The current firmware uses OLED
`SDA=PA0` and `SCL=PA1`; PB2/PB3 are reserved for `UART_BLUETOOTH`.

`H2024_MODE_TURN_DEBUG` does not require gray calibration because its route
contains no line-following segment. Gray calibration remains mandatory for
H2024 items 2 through 4.

## Verified hardware result (2026-07-24)

- The positive debug target is 30 degrees.
- The real chassis turns left and stops at about +27 degrees.
- Repeated turn behavior has been accepted by the user as normal.
- Measured odometry inputs are wheel diameter 65 mm and encoder CPR 724.
- The first ITEM1 floor baseline completed successfully with good straight-line
  behavior; parameter optimization is intentionally deferred to the next day.
- This is the intended result: `angle_tolerance_deg` is 3 degrees, so the turn
  segment advances when the remaining error is less than or equal to 3 degrees.
- At the start of the turn the command is approximately `L=0, R=60 mm/s` and
  the TB6612 adapter displays approximately `PWM=0/14`.
- The successful run proves the complete path: KEY1 press -> arm -> safety
  gate -> route executor -> yaw P control -> TB6612 -> IMU feedback -> stop.

## Confirmed incident log

### I01: Correct source changed but wrong firmware behavior remained

- Symptom: only the Bluetooth `I/E/M` page appeared and task PWM stayed zero.
- Cause: the C-drive CCS project and the D-drive repository copy had diverged;
  the actual C-drive build still selected Bluetooth mode.
- Evidence: the map contained `BluetoothControl_Update` but not
  `CarFirmware_Tick` or `CarRouteExecutor_Update`.
- Prevention: after every mode change, clean-build the C-drive project and
  verify the task symbols in the generated map before flashing.

### I02: Button pin assumptions did not match the PCB

- Symptom: only one physical key affected the display and calibration actions
  appeared attached to the wrong key.
- Cause: an earlier pin list used the wrong GPIO mapping.
- Verified mapping: KEY1=PB23 (middle), KEY2=PB26 (right), KEY3=PB27 (left),
  all active-low. Raw pressed states are 011, 101, and 110 respectively.

### I03: OLED refresh starved button and control polling

- Symptom: P1/P2 alternated continuously, short KEY1 presses were missed, and
  the control loop was difficult to observe.
- Cause: at 100 kHz I2C the old code cleared and sent all 1024 framebuffer bytes
  every 50 ms. The blocking transfer consumed most of the main loop.
- Fix: KEY2 now selects a stable page; rendering is every 200 ms and transfer
  uses `OLED_UpdatePages(tx_page, 1)` so each pass sends only one hardware page.

### I04: False `F:00000002` immediately after a successful arm

- Symptom: `BTN:1 ARM`, `STAT:0`, then `APP:N RUN:N`, `M:N`, `PWM:0/0`, while
  the current encoder field still displayed `E:Y`.
- Cause: the main loop captured `now_ms`, then the direct TB6612 encoder adapter
  captured a timestamp one SysTick later. Unsigned `now_ms - timestamp_ms`
  underflowed to a huge age and falsely raised `CAR_FAULT_ENCODER_STALE`.
- Fix: the stale comparison now uses signed wrap-safe elapsed-time arithmetic.
- Regression coverage: normal age, timestamp one tick in the future, timer
  wraparound, and genuinely stale samples are covered by
  `tests/test_safety_timestamp.c.reference`.

## OLED field reference

### P1 route/output page

| Field | Meaning | Expected or possible values |
| --- | --- | --- |
| `MODE` | Selected route mode | `TURN`, `ITEM1` to `ITEM4`, or `?????` |
| `TB` | Static intended-backend label | Confirm the real callback with P2 `TB:Y` |
| `CAL` | Yaw calibration | `WAIT` before 400 stationary samples, then `OK` |
| `YAW` | Integrated yaw in degrees | Positive is left; reset to zero at arm |
| `SEG` | Route index and executor state | Index plus `STOP`, `RUN`, or `DONE` |
| `L/R` | Requested wheel speeds | Signed mm/s commands, not measured speeds |
| `PWM` | Last TB6612 commands | Signed duty percent, limited to +/-80 |
| `F` | Combined fault mask | Eight hexadecimal digits; zero is normal |
| `K:123` | Raw KEY1/KEY2/KEY3 levels | 111 released; 011/101/110 for middle/right/left |

TURN normally progresses from `SEG:00 STOP` to `SEG:01 RUN` and finally
`SEG:03 DONE`. ITEM1 normally runs at segment 01 and finishes at index 04.
ITEM2 motion segments are 01 straight, 03 half-circle, 05 straight, and 07
half-circle; it finishes after index 09 advances to 10.

### P2 arm/backend page

| Field | Meaning | Expected or possible values |
| --- | --- | --- |
| `BTN` | Debounced KEY1 event count and result | Action is `NONE`, `ARM`, `REJ`, or `STOP` |
| `I` | Current IMU/yaw sample valid | `Y` after calibration; `N` on wait/read failure |
| `E` | Current encoder API sample valid | `Y` does not prove counts change or signs are correct |
| `M` | Motor output layer prepared | `Y` after prepare; `N` before prepare or after output fault |
| `STAT` | Last arm return value | 0 OK, -1 input/argument, -2 state, -3 capacity |
| `ENC` | Duplicate current encoder-valid field | Normally identical to `E`; not an encoder count |
| `TB` | Direct motor callback installed | `Y` for the current backend |
| `SER` | Serial/Modbus callback installed | `N` for the current backend; code is retained as spare |
| `APP` | High-level application armed | `Y` while a valid route owns motion |
| `RUN` | Route executor running | `Y` only during active route execution |
| `PWM` | Same physical command as P1 | 0/0 stopped; TURN starts near 0/14 |
| `BIAS` | Gyro zero-rate bias, dps | Currently expected around -0.5 to -0.4 |

The normal pre-start state is `I:Y E:Y M:Y`, `STAT:0`, `TB:Y SER:N`,
`APP:N RUN:N`, and `PWM:0/0`. A normal start gives `BTN:n ARM`, `APP:Y RUN:Y`.

### Fault values

| Hex | Meaning |
| --- | --- |
| `00000000` | No fault |
| `00000001` | Emergency stop |
| `00000002` | Encoder invalid/stale |
| `00000004` | IMU invalid/stale |
| `00000008` | Gray sample invalid/stale while required |
| `00000010` | Segment timeout |
| `00000020` | Invalid route/geometry |
| `00000040` | Motor preparation/output failure |
| `00000080` | IMU initialization failure |
| `00000100` | Gray calibration missing for a gray-dependent task |

Faults are bit masks and can combine; for example, `00000006` means encoder
and IMU stale together.

## Next acceptance sequence

1. Repeat +30 degrees ten times. Require 10/10 no-fault completion and record
   final yaw and elapsed time; the initial acceptance band is 27 to 33 degrees.
2. Change only `H2024_DEBUG_TURN_DEG` to -30. Expect `L:+60 R:+0`, approximately
   `PWM:14/0`, decreasing yaw, and a final yaw near -27 degrees.
3. Select `PROJECT_MODE_H2024_ITEM_1` and lift the wheels first. Expect
   `SEG:01 RUN`, approximately `L/R=180/180`, and `PWM=41/41` before correction.
4. Before judging the one-metre stop, replace placeholder wheel diameter and
   CPR with measured values. Record distance, lateral error, final yaw, time,
   battery voltage, and fault mask for every floor run.
5. Start ITEM2 only after ITEM1 is repeatable. Its line PID is active on arc
   segments 03 and 07, not straight segments 01 and 05.

## Deferred improvements

- Add a TB6612 wheel-speed inner loop using the existing encoder feedback while
  retaining the current direct/open-loop fallback and resident Bluetooth
  tuning path.
- Add an optional 100 cm finish-marker stop for ITEM1. The intended marker is
  a transverse black line seen by most or all gray channels, not a normal
  single line sample. Gate detection by a minimum travelled distance and
  require several consecutive frames so the start area and noise cannot stop
  the car early. Keep encoder distance as a fallback and diagnostic cross-check.
