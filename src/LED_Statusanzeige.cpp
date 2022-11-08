#include "LED_Statusanzeige.h"
#include "PCF8575.h"

#define i2cAddr_LED_CH1_16 0x24

#define I2C_Expander_CH 15
#define MAX_NUMBER_OF_I2C_Channels 16




PCF8575 pcf8575_LED_CH1_16(i2cAddr_LED_CH1_16, &Wire);

bool state_LED[MAX_NUMBER_OF_I2C_Channels];

bool status_Led_ON = false;
uint16_t state_LED_Out = 0xFFFF;




void initI2cStatusLeds()
{
    // LED I2C Inputs
    for (int i = 0; i < MAX_NUMBER_OF_I2C_Channels; i++)
    {
        pcf8575_LED_CH1_16.pinMode(i, OUTPUT);
    }
    pcf8575_LED_CH1_16.pcf8575_Clear();
}

void setLED_ON_ALL()
{
    pcf8575_LED_CH1_16.pcf8575_WriteALL(0x0000);
    status_Led_ON = true;
}

void setLED_OFF_ALL()
{
    pcf8575_LED_CH1_16.pcf8575_WriteALL(0xFFFF);
    status_Led_ON = false;
}

void set_State_LED(uint8_t ch, bool state)
{
    // Set VCC I2C Inputs
    if (ch <= MAX_NUMBER_OF_I2C_Channels)
    {
        //state_LED_Out ^= (-state_LED[I2C_Expander_CH - ch] ^ state_LED_Out) & (1 << ch);
        state_LED_Out ^= (-state ^ state_LED_Out) & (1 << ch);
        pcf8575_LED_CH1_16.pcf8575_WriteALL(state_LED_Out);
    }
}

void setLED_24VAC(bool state)
{
    set_State_LED(LED24VAC, state);
}

void setLED_Relais(uint8_t ch, bool state)
{
    set_State_LED(ch, state);
}

void setLED_Ventil(uint8_t ch, bool state)
{
    set_State_LED(15-ch, state);
}