#ifndef IOExp_h
#define IOExp_h

#include <stdint.h>

#include "PCA9554.h"


void init_IOExpander_GPIOs_TOP();
void init_IOExpander_GPIOs_BOT();

void clearInitFlags_IOExp();

bool get_IOExpander_Input(uint8_t ch);
void set_IOExpander_Input(uint8_t ch, bool state);
bool get_IOExpander_BOT_Input(uint8_t ch);
//void set_IOExpander_BOT_Input(uint8_t ch, bool state);

void enable_5V(bool state);
void set_ADC1_VoltageDiff(bool state);
void set_ADC2_VoltageDiff(bool state);
void set_ADC3_VoltageDiff(bool state);

void control_Ventil(uint8_t ch, bool state);
void control_Relais(uint8_t nr, bool state);
bool getStatus_Ventil(uint8_t ch);
bool getStatus_Relais(uint8_t ch);

bool getInitFlag_PCA9555();
bool getInitFlag_PCA9554();



#endif