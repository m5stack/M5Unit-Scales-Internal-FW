#include "sk6812.h"
#include <stdlib.h>
#include "spi.h"

uint8_t led_num;
uint8_t *color_buf = NULL;

void Sk6812Init(uint8_t num) {
    color_buf = (uint8_t *)calloc(num, sizeof(uint8_t) * 12);
    led_num   = num;
}

void Sk6812SetColor(uint8_t num, uint32_t color) {
    if (num >= led_num) {
        return;
    }
    color =
        ((color >> 8) & 0xff) | ((color << 8) & 0xff00) | (color & 0xff0000);
    uint8_t *ptr = &color_buf[num * 12];
    for (uint8_t j = 0; j < 3; j++) {
        for (uint8_t i = 0; i < 4; i++) {
            if (color & (1 << (2 * i + j * 8)))
                ptr[3 - i] = 0x03;
            else
                ptr[3 - i] = 0x01;
            if (color & (1 << (2 * i + 1 + j * 8)))
                ptr[3 - i] |= 0x30;
            else
                ptr[3 - i] |= 0x10;
        }
        ptr += 4;
    }
}

void Sk6812SetAllColor(uint32_t color) {
    for (uint8_t i = 0; i < led_num; i++) {
        Sk6812SetColor(i, color);
    }
}

void Sk6812Show(void) {
    uint8_t buf = 0x00;

    HAL_SPI_Transmit_DMA(&hspi1, &buf, 1);
    HAL_Delay(1);
    HAL_SPI_Transmit_DMA(&hspi1, color_buf, led_num * 12);
    // HAL_Delay(1);
}
