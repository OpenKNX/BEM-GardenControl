#pragma once

#include <Arduino.h>
//#include <knx.h>

#define zaehlerElek     1
#define zaehlerWasser   2
#define zaehlerGas      3
#define zaehlerSonstig  4

#define unit_l  0
#define unit_m3 1

void InitS0Input1();
void InitS0Input2();
void InitS0Input3();
void InitS0Input4();

void processReadS0Input();

uint16_t setZaehlerImpulse(uint8_t i, uint16_t impulse);


void sendZaehlerStand(int i, uint16_t S0_Zaehler[], uint16_t S0_Zaehler_old[]);
void sendZaehlerStand_2(int i, uint16_t S0_Zaehler[], uint16_t S0_Zaehler_old[]);
