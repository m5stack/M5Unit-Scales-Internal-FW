#include "key.h"

#define GET_KEY_VALUE() ((GPIOA->IDR & 0x04) == 0x04)

PressStatus_t KeyUpdate(void) {
    static uint32_t last_press_time = 0;
    static uint8_t last_value       = 1;

    uint8_t value = GET_KEY_VALUE();
    uint32_t time = HAL_GetTick();

    // btn state not change
    if (last_value == value) {
        return kNormal;
    }

    last_value = value;

    if (value == 1 && time - last_press_time > 10) {
        if (time - last_press_time > LONG_PRESS_TIME) {
            return kLongPressed;
        }

        if (time - last_press_time > DB_TIME) {
            return kReleased;
        }
    }

    last_press_time = HAL_GetTick();
    return kNormal;
}
