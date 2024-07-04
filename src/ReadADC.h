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

/**************  ADS1015  ******************************/
void initADC_TOP_ADS1015(uint8_t res_top);
bool getAdcI2cRun_TOP();
bool isAdcI2cBusy_TOP();
void requestADC(uint8_t ch);
uint16_t getAdcI2cValue_TOP(uint8_t ch);
float getAdcI2cValueMillivolt_TOP(uint8_t ch);
void readAdcI2cValue(uint8_t ch);

void initADC_BOT_ADS1015(uint8_t res_bot);
bool getAdcI2cRun_BOT();
bool isAdcI2cBusy_BOT();
void requestADC_BOT(uint8_t ch);
uint16_t getAdcI2cValue_BOT(uint8_t ch);
void readAdcI2cValue_BOT(uint8_t ch);
/*******************************************************/
/**************  MCP3428  ******************************/
void initADC_TOP_MCP3428(uint8_t res_top);
void initADC_BOT_MCP3428(uint8_t res_bot);

void StartAdcConversation_TOP(uint8_t ch);
void StartAdcConversation_BOT(uint8_t ch);

void ReadAdcValue();
void ReadAdcValue_BOT();
bool isAdcBusy_TOP_MCP3428();
bool isAdcBusy_BOT_MCP3428();

/*******************************************************/

bool processADConversation_TOP();
bool processADConversation_BOT();

void set_ADC_DIV(uint8_t ch, bool div);
void set_ADC_CorrFactor(uint8_t ch, float corrFactor);

bool get_CH1_DIV();
bool get_CH2_DIV();
bool get_CH3_DIV();

void clearInitFlags_ADC();

uint16_t getAdcValue(uint8_t ch);
uint16_t getAdcValue_BOT(uint8_t ch);

float getAdcVoltage_TOP(uint8_t ch);
float getAdcVoltage_BOT(uint8_t ch);


float getAdcVoltage_CH1();
float getAdcVoltage_CH2();
float getAdcVoltage_CH3();




