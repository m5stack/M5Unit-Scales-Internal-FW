#pragma once

#include "stm32f0xx_hal.h"

#define DB_TIME         10
#define LONG_PRESS_TIME 300

typedef enum {
    kNormal = 0x00,
    kReleased,
    kDoublePressed,
    kLongPressed,
} PressStatus_t;

PressStatus_t KeyUpdate();
