#ifndef PROJECT_MODE_H
#define PROJECT_MODE_H

#define PROJECT_MODE_BLUETOOTH_TUNING (0)
#define PROJECT_MODE_H2024_ITEM_1      (1)
#define PROJECT_MODE_H2024_ITEM_2      (2)
#define PROJECT_MODE_H2024_ITEM_3      (3)
#define PROJECT_MODE_H2024_ITEM_4      (4)
#define PROJECT_MODE_GRAY_ADC_DEBUG    (5)
#define PROJECT_MODE_TURN_DEBUG        (6)
/* 2026 积分赛任务独立选择：ITEM_1=A->B，ITEM_2=A->B->C->D->A，
 * ITEM_3=固定正向起步的变序单圈，ITEM_4=同路线连续三圈。
 * MAIN 指向当前最终三圈路线。 */
#define PROJECT_MODE_H2026_ITEM_1      (7)
#define PROJECT_MODE_H2026_ITEM_2      (8)
#define PROJECT_MODE_H2026_ITEM_3      (9)
#define PROJECT_MODE_H2026_ITEM_4      (10)
#define PROJECT_MODE_H2026_MAIN        PROJECT_MODE_H2026_ITEM_4

/*
 * Keep the gray ADC page as the default while the sensor array is being
 * validated. Select another mode here after gray calibration is complete.
 */
#ifndef PROJECT_MODE
#define PROJECT_MODE PROJECT_MODE_H2026_ITEM_4

#endif

/* ITEM_4 是当前编号最大的可烧录模式。 */
#if (PROJECT_MODE < PROJECT_MODE_BLUETOOTH_TUNING) || \
    (PROJECT_MODE > PROJECT_MODE_H2026_ITEM_4)
#error "PROJECT_MODE is invalid"
#endif

#endif
