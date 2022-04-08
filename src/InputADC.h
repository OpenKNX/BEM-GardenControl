#pragma once

#include <stdint.h>

#define ADC_Wert                1
#define SMT50_Bodenfeuchte      2
#define SMT50_BodenTemperatur   3


void processInputADC();
void initInputADC();

float getSensorValue(uint8_t channel);