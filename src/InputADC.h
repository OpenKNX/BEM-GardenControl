#pragma once

#include <stdint.h>




void processInput_ADC(bool readyFlag);
void initInputADC();

float getSensorValue(uint8_t channel);