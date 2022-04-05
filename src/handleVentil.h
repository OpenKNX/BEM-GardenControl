#pragma once

#include <stdint.h>

void processVentil();
void processRelais();
void set_Ventil_State(uint8_t ch, bool state);
void set_Relais_State(uint8_t ch, bool state);

