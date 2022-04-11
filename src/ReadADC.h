#pragma once

#define Resolution12Bit 12 // 240SPS max
#define Resolution14Bit 14 //  60SPS max
#define Resolution16Bit 16 //  15SPS max

// ADC TOP
#define ADC_Ch1    0
#define ADC_Ch2    1
#define ADC_Ch3    2
#define ADC_12V_CH 3
// ADC BOT
#define ADC_4_20mA_Ch1 0
#define ADC_4_20mA_Ch2 1
#define ADC_24V_CH     2

#define DIV_12V  1
#define DIV_5V   0
#define is4_20mA 1
#define DIV_24V  0



void processADConversation();

void initADC_TOP(uint8_t res_top);
void initADC_BOT(uint8_t res_bot);

void set_ADC_DIV(uint8_t ch, bool div);
void set_ADC_CorrFactor(uint8_t ch, float corrFactor);

bool get_CH1_DIV();
bool get_CH2_DIV();
bool get_CH3_DIV();

void clearInitFlags_ADC();

void StartAdcConversation(uint8_t ch);
void StartAdcConversation_BOT(uint8_t ch);

long ReadAdcValue();
long ReadAdcValue_BOT();

uint16_t getAdcValue(uint8_t ch);
uint16_t getAdcValue_BOT(uint8_t ch);

float getAdcVoltage(uint8_t ch, bool div);
float getAdcVoltage_BOT(uint8_t ch, bool isCurrent);

bool isADCready();


float getAdcVoltage_CH1();
float getAdcVoltage_CH2();
float getAdcVoltage_CH3();
float getAdcVoltage_12V();

float get4_20mA(uint8_t ch);
float get4_20mA_CH1();
float get4_20mA_CH2();

float getAdcVoltage_24V();

