#pragma once

#include <stdint.h>


uint8_t processErrorHandling();

uint8_t getError();
bool get_5V_Error();
bool get_5V_out_Error();
bool get_12V_Error();
bool get_24V_Error();