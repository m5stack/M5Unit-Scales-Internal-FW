#pragma once

#include "stdint.h"

void Sk6812Init(uint8_t num);
void Sk6812SetColor(uint8_t num, uint32_t color);
void Sk6812SetAllColor(uint32_t color);
void Sk6812Show(void);
