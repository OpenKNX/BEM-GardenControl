#pragma once

#include <stdint.h>

#define ID1 0
#define ID2 1
#define ID3 3


#define HW_1_0  7 // ID1 = H ; ID2 = H ; ID3 = H  --> b00000111 --> 0x07 --> dez 7

void init_GPIOs();
uint8_t get_HW_ID_TOP();
uint8_t get_HW_ID_BOT();
void initHW();
void print_HW_ID_TOP(uint8_t id);
void print_HW_ID_BOT(uint8_t id);
uint8_t get_PROG_LED_PIN(uint8_t hwID);
uint8_t get_PROG_BUTTON_PIN(uint8_t hwID);
uint8_t get_SAVE_INTERRUPT_PIN(uint8_t hwID);