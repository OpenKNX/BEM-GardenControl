#pragma once

#include <stdint.h>

void processVentil();
void processRelais();
void process_ventil_states();
void process_relais_states();
void process_5V_Relais();
void set_Ventil_State(uint8_t ch, bool state);
void set_Ventil_State_single(uint8_t ch);
void set_Ventil_Sperrobjekt(uint8_t ch, bool state);
void set_Relais_State(uint8_t ch, bool state);
bool get_Ventil_StateOld(uint8_t ch);
void set_Relais_Sperrobjekt(uint8_t ch, bool state);
bool get_Relais_StateOld(uint8_t ch);
void set_5V_Relais_State(bool state);
bool get_5V_Relais_State(bool state);
void control_5V_Relais(bool state);
void control_Ventil(uint8_t ch, bool state);
void control_Relais(uint8_t nr, bool state);
void clear_ALL_Ventil_states();
void clear_ALL_Relais_states();

