#ifndef PROJECT_MODE_H
#define PROJECT_MODE_H

#define PROJECT_MODE_BLUETOOTH_TUNING (0)
#define PROJECT_MODE_H2024_ITEM_1      (1)
#define PROJECT_MODE_H2024_ITEM_2      (2)
#define PROJECT_MODE_H2024_ITEM_3      (3)
#define PROJECT_MODE_H2024_ITEM_4      (4)
#define PROJECT_MODE_GRAY_ADC_DEBUG    (5)
#define PROJECT_MODE_TURN_DEBUG        (6)

/*
 * Keep the gray ADC page as the default while the sensor array is being
 * validated. Select another mode here after gray calibration is complete.
 */
#ifndef PROJECT_MODE
#define PROJECT_MODE PROJECT_MODE_H2024_ITEM_2

#endif

#if (PROJECT_MODE < PROJECT_MODE_BLUETOOTH_TUNING) || \
    (PROJECT_MODE > PROJECT_MODE_TURN_DEBUG)
#error "PROJECT_MODE is invalid"
#endif

#endif
