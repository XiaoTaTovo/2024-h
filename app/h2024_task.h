#ifndef H2024_TASK_H
#define H2024_TASK_H

#include "car_config.h"
#include "core/route_executor.h"

typedef enum {
    H2024_MODE_ITEM_1 = 1,
    H2024_MODE_ITEM_2 = 2,
    H2024_MODE_ITEM_3 = 3,
    H2024_MODE_ITEM_4 = 4
} H2024Mode;

CarStatus H2024_BuildRoute(H2024Mode mode,
                           const CarConfig *config,
                           CarRoute *route);

#endif
