#include "LED_Statusanzeige.h"

#define PCF_8575 1
#define TCA_9555 2

#define i2cAddr_LED_CH1_16 0x24

#define I2C_Expander_CH 15
#define MAX_NUMBER_OF_I2C_Channels 16

PCF8575 pcf8575_LED_CH1_16(i2cAddr_LED_CH1_16, &Wire);
TCA9555 tca9555_LED_CH1_16(0x22, &Wire);

bool state_LED[MAX_NUMBER_OF_I2C_Channels];

bool status_Led_ON = false;
uint16_t state_LED_Out = 0xFFFF;

uint8_t ioExp = 0;

void initI2cStatusLeds()
{
    // CHeck with Io-Expander is used
    if (pcf8575_LED_CH1_16.isConnected())
    {
        SERIAL_DEBUG.println("Status LED IO-EXP: PCF8575");
        // LED I2C Inputs
        for (int i = 0; i < MAX_NUMBER_OF_I2C_Channels; i++)
        {
            pcf8575_LED_CH1_16.pinMode(i, OUTPUT);
        }
        pcf8575_LED_CH1_16.pcf8575_Clear();
        ioExp = PCF_8575;
    }
    else if (tca9555_LED_CH1_16.isConnected())
    {
        SERIAL_DEBUG.println("Status LED IO-EXP: TCA95555");
        // LED I2C Inputs
        for (int i = 0; i < MAX_NUMBER_OF_I2C_Channels; i++)
        {
            tca9555_LED_CH1_16.pinMode1(i, OUTPUT);
        }
        tca9555_LED_CH1_16.write16(0xFFFF);
        ioExp = TCA_9555;
    }
    else
    {
        SERIAL_DEBUG.println("No Status LED PCB detected");
    }
}

void setLED_ON_ALL()
{
    status_Led_ON = true;
    switch (ioExp)
    {
        case PCF_8575:
            pcf8575_LED_CH1_16.pcf8575_WriteALL(0x0000);

            break;
        case TCA_9555:
            tca9555_LED_CH1_16.write16(0x0000);
            break;

        default:
            SERIAL_DEBUG.println("Status Anzeige. IO-Exp not defined");
            status_Led_ON = false;
            break;
    }
}

void setLED_OFF_ALL()
{
    switch (ioExp)
    {
        case PCF_8575:
            state_LED_Out = 0xFFFF;
            pcf8575_LED_CH1_16.pcf8575_WriteALL(0xFFFF);
            break;
        case TCA_9555:
            state_LED_Out = 0xFFFF;
            tca9555_LED_CH1_16.write16(0xFFFF);
            break;

        default:
            SERIAL_DEBUG.println("Status Anzeige. IO-Exp not defined");
            break;
    }
    status_Led_ON = false;
}

void setLED_OFF_Ventil()
{
    switch (ioExp)
    {
        case PCF_8575:
            state_LED_Out |= 0xFFF0;
            pcf8575_LED_CH1_16.pcf8575_WriteALL(state_LED_Out);
            break;
        case TCA_9555:
            state_LED_Out |= 0xFFF0;
            tca9555_LED_CH1_16.write16(state_LED_Out);
            break;

        default:
            SERIAL_DEBUG.println("Status Anzeige. IO-Exp not defined");
            break;
    }
}

void setLED_OFF_Relais()
{
    switch (ioExp)
    {
        case PCF_8575:
            state_LED_Out |= 0x000E;
            pcf8575_LED_CH1_16.pcf8575_WriteALL(state_LED_Out);
            break;
        case TCA_9555:
            state_LED_Out |= 0x000E;
            tca9555_LED_CH1_16.write16(state_LED_Out);
            break;

        default:
            SERIAL_DEBUG.println("Status Anzeige. IO-Exp not defined");
            break;
    }
}

void set_State_LED(uint8_t ch, bool state)
{
    switch (ioExp)
    {
        case PCF_8575:
            // Set VCC I2C Inputs
            if (ch <= MAX_NUMBER_OF_I2C_Channels)
            {
                // state_LED_Out ^= (-state_LED[I2C_Expander_CH - ch] ^ state_LED_Out) & (1 << ch);
                state_LED_Out ^= (-state ^ state_LED_Out) & (1 << ch);
                pcf8575_LED_CH1_16.pcf8575_WriteALL(state_LED_Out);
            }
            break;
        case TCA_9555:
            if (ch <= MAX_NUMBER_OF_I2C_Channels)
            {
                state_LED_Out ^= (-state ^ state_LED_Out) & (1 << ch);
                tca9555_LED_CH1_16.write1(ch, state);
            }
            break;

        default:
            SERIAL_DEBUG.println("Status Anzeige. IO-Exp not defined");
            break;
    }
}

void setLED_24VAC(bool state)
{
    set_State_LED(LED24VAC, !state);
}

void setLED_Relais(uint8_t ch, bool state)
{
    set_State_LED(ch, state);
}

void setLED_Ventil(uint8_t ch, bool state)
{
    set_State_LED(15 - ch, state);
}

void setLED_ERROR(bool state)
{
    set_State_LED(LEDERROR, !state);
}