#ifndef PROJECT_MODE_H
#define PROJECT_MODE_H

#define PROJECT_MODE_BLUETOOTH_TUNING (0)
#define PROJECT_MODE_H2024_ITEM_1      (1)
#define PROJECT_MODE_H2024_ITEM_2      (2)
#define PROJECT_MODE_H2024_ITEM_3      (3)
#define PROJECT_MODE_H2024_ITEM_4      (4)

/*
 * Keep Bluetooth tuning as the default until encoder PPR and speed-loop
 * parameters have been measured on the real car.
 */
#ifndef PROJECT_MODE
#define PROJECT_MODE PROJECT_MODE_BLUETOOTH_TUNING
#endif

#if (PROJECT_MODE < PROJECT_MODE_BLUETOOTH_TUNING) || \
    (PROJECT_MODE > PROJECT_MODE_H2024_ITEM_4)
#error "PROJECT_MODE is invalid"
#endif

#endif
