#pragma once

#include <stdint.h>

#define ID1 0
#define ID2 1
#define ID3 3

// IDs TOP
#define HW_1_0      7 // ID1 = H ; ID2 = H ; ID3 = H  --> b00000111 --> 0x07 --> dez 7 (MCP3428)
#define HW_2_0      6 // ID1 = L ; ID2 = H ; ID3 = H  --> b00000110 --> 0x06 --> dez 6 (MCP3428)
#define HW_2_1      4 // ID1 = L ; ID2 = L ; ID3 = H  --> b00000100 --> 0x04 --> dez 4 (ADS1015)

// IDs BOT
#define HW_BOT_1_0  0 // ID1 = L ; ID2 = L ; ID3 = L  --> b00000000 --> 0x00 --> dez 0  (MCP3428)
#define HW_BOT_2_0  1 // ID1 = H ; ID2 = L ; ID3 = L  --> b00000001 --> 0x01 --> dez 1  (MCP3428)
#define HW_BOT_2_1  3 // ID1 = H ; ID2 = H ; ID3 = L  --> b00000011 --> 0x03 --> dez 3  (ADS1015)


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
uint8_t get_HW_ID();
uint8_t get_Status_PIN();
uint8_t get_PROG_LED_PIN();
uint8_t get_PROG_BUTTON_PIN();
uint8_t get_SAVE_INTERRUPT_PIN();
uint8_t get_SSR_EN_PIN();
uint8_t get_5V_EN_PIN();
uint8_t get_5V_fault_PIN();
uint8_t get_5V_status_PIN();