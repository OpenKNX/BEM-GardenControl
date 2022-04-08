#pragma once

#include <stdint.h>

#define ID1 0
#define ID2 1
#define ID3 3

// IDs TOP
#define HW_1_0      7 // ID1 = H ; ID2 = H ; ID3 = H  --> b00000111 --> 0x07 --> dez 7

// IDs BOT
#define HW_BOT_1_0  0 // ID1 = L ; ID2 = L ; ID3 = L  --> b00000000 --> 0x00 --> dez 0


//void init_GPIOs();
void read_HW_ID_TOP();
void read_HW_ID_BOT();
uint8_t get_HW_ID_TOP();
uint8_t get_HW_ID_BOT();
void initHW();
void initHW_Top();
void initHW_Bot();
void print_HW_ID_TOP(uint8_t id);
void print_HW_ID_BOT(uint8_t id);
uint8_t get_PROG_LED_PIN();
uint8_t get_PROG_BUTTON_PIN();
uint8_t get_SAVE_INTERRUPT_PIN();
uint8_t get_SSR_EN_PIN();
uint8_t get_5V_EN_PIN();
uint8_t get_5V_fault_PIN();
uint8_t get_5V_status_PIN();