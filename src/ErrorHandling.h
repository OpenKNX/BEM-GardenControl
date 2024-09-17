#pragma once

#include <stdint.h>


uint8_t processErrorHandling();

uint8_t getError();

bool get_24V_AC_Error();
bool get_5V_Error();
bool get_12V_Error();
bool get_24V_Error();
bool check_24V_4_20mA_CH1();
bool check_24V_4_20mA_CH2();
bool get_12V_or_24V_Error();
bool get_ADC_Ready_Flag_TOP();
bool get_ADC_Ready_Flag_BOT();
void set_ADC_Ready_Flag_TOP();
void set_ADC_Ready_Flag_BOT();