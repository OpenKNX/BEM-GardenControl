#pragma once

#include <stdint.h>
// #include "Helper.h"

// bool delayCheck(uint32_t iOldTimer, uint32_t iDuration);

uint16_t getParBEM(uint16_t PAR, uint8_t CH);
uint16_t getComBEM(uint16_t COM, uint8_t CH);
uint16_t getParREL(uint16_t PAR, uint8_t CH);
uint16_t getComREL(uint16_t COM, uint8_t CH);

#ifdef ADC_enable
uint16_t getParADC(uint16_t PAR, uint8_t CH);
uint16_t getComADC(uint16_t COM, uint8_t CH);
uint16_t getParCUR(uint16_t PAR, uint8_t CH);
uint16_t getComCUR(uint16_t COM, uint8_t CH);
#endif
#ifdef BinInputs
uint16_t getParBIN(uint16_t PAR, uint8_t CH);
uint16_t getComBIN(uint16_t COM, uint8_t CH);
#endif