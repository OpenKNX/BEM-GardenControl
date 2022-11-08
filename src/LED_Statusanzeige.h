#pragma once

#include <stdint.h>

#define LED24VAC   0
#define LEDRelais3 1
#define LEDRelais2 2
#define LEDRelais1 3

void initI2cStatusLeds();
void setLED_ON_ALL();
void setLED_OFF_ALL();
void setLED_OFF_ALL();
void set_State_LED(uint8_t ch, bool state);
void setLED_24VAC(bool state);
void setLED_Relais(uint8_t ch, bool state);
void setLED_Ventil(uint8_t ch, bool state);
