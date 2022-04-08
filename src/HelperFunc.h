#pragma once

#include <stdint.h>

bool delayCheck(uint32_t iOldTimer, uint32_t iDuration);

uint16_t getParBEM(uint16_t PAR, uint8_t CH);
uint16_t getComBEM(uint16_t COM, uint8_t CH);
uint16_t getParREL(uint16_t PAR, uint8_t CH);
uint16_t getComREL(uint16_t COM, uint8_t CH);
uint16_t getParADC(uint16_t PAR, uint8_t CH);
uint16_t getComADC(uint16_t COM, uint8_t CH);
