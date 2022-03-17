#pragma once

#include <stdint.h>
#include "Sensor_Value_Input.h"

#include "hardware.h"
#include "MCP3428.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#include "ErrorHandling.h"

#define gain_1 1
#define gain_2 2
#define gain_4 4
#define gain_8 8

#define maxADC_CH 4
#define sampleRate_100SPS 10 // read each  10ms
#define sampleRate_20SPS 50  // read each  50ms
#define sampleRate_10SPS 100 // read each 100ms
#define sampleRate_5SPS 200  // read each 200ms

MCP3428 MCP3428_adc(i2cADC, &Wire1);
MCP3428 MCP3428_adc_BOT(i2cADC_BOT, &Wire1);

bool init_flag_PCP3428_Top = false;
bool init_flag_PCP3428_Bot = false;

uint32_t READ_Delay = 0;
uint8_t adc_CH = 1;
uint8_t adc_CH_BOT = 1;

long adc_Value[maxADC_CH] = {1};
long adc_Value_BOT[maxADC_CH] = {1};

uint8_t resolution_TOP = 0;
uint8_t resolution_BOT = 0;

bool Ch1_div = false;
bool Ch2_div = false;
bool Ch3_div = false;

enum State
{
    Set = 1,
    Set_BOT = 2,
    Read = 3,
    Read_BOT = 4,
    wait_Init = 5,
};

State ADC_State = wait_Init;

void initADC_TOP(uint8_t res_top)
{
    resolution_TOP = res_top; // set resolution TOP
    if (!get_5V_Error())
    {
        SERIAL_PORT.print("  MCP3428_TOP:");
        SERIAL_PORT.println(MCP3428_adc.testConnection());
        init_flag_PCP3428_Top = true;
    }
    else
    {
        SERIAL_PORT.print("  MCP3428_TOP ERROR: no +5V_Iso");
    }
}

void initADC_BOT(uint8_t res_bot)
{
    resolution_BOT = res_bot; // set resolution BOT

    if (!get_5V_Error())
    {
        SERIAL_PORT.print("  MCP3428_BOT:");
        SERIAL_PORT.println(MCP3428_adc_BOT.testConnection());
        init_flag_PCP3428_Bot = true;
    }
    else
    {
        SERIAL_PORT.print("  MCP3428_BOT ERROR: no +5V_Iso");
    }
}

void clearInitFlags_ADC()
{
    init_flag_PCP3428_Top = false;
    init_flag_PCP3428_Bot = false;

    // clear ADC values
    for (int i = 0; i < maxADC_CH; i++)
    {
        adc_Value[i] = 0;
        adc_Value_BOT[i] = 0;
    }
}

void set_CH1_DIV(bool div)
{
    Ch1_div = div;
    set_ADC1_VoltageDiff(div);
}

void set_CH2_DIV(bool div)
{
    Ch2_div = div;
    set_ADC2_VoltageDiff(div);
}

void set_CH3_DIV(bool div)
{
    Ch3_div = div;
    set_ADC3_VoltageDiff(div);
}

bool get_CH1_DIV()
{
    return Ch1_div;
}

bool get_CH2_DIV()
{
    return Ch2_div;
}

bool get_CH3_DIV()
{
    return Ch3_div;
}

void StartAdcConversation(uint8_t ch)
{
    if (!get_5V_Error() && init_flag_PCP3428_Top)
    {
        MCP3428_adc.SetConfiguration(ch, resolution_TOP, 1, gain_1);
    }
}

void StartAdcConversation_BOT(uint8_t ch)
{
    if (!get_5V_Error() && init_flag_PCP3428_Bot)
    {
        MCP3428_adc_BOT.SetConfiguration(ch, resolution_BOT, 1, gain_1);
    }
}

long ReadAdcValue()
{
    return MCP3428_adc.readADC();
}

long ReadAdcValue_BOT()
{
    return MCP3428_adc_BOT.readADC();
}

uint16_t getAdcValue(uint8_t ch)
{
    return adc_Value[ch - 1];
}

uint16_t getAdcValue_BOT(uint8_t ch)
{
    return adc_Value_BOT[ch - 1];
}

float getAdcVoltage(uint8_t ch, bool div)
{

    if (div) // 0-12V
    {
        switch (resolution_TOP)
        {
        case Resolution12Bit:
            return (float)adc_Value[ch - 1] * 0.006; // 2.047 / 2047.0 * 6.0;
            break;
        case Resolution14Bit:
            return (float)adc_Value[ch - 1] * 0.0015; // 2.047 / 8191.0 * 6.0;
            break;
        case Resolution16Bit:
            return (float)adc_Value[ch - 1] * 0.000375; // 2.047 / 32767.0 * 6.0;
            break;
        default:
            SERIAL_PORT.print("wrong RES");
            return 0;
            break;
        }
    }
    else // 0-5V
    {
        SERIAL_PORT.print("5V ");
        switch (resolution_TOP)
        {
        case Resolution12Bit:
            // Serial.print(adc_Value[ch - 1]);
            // Serial.print(" ");
            return adc_Value[ch - 1] * 0.003; // 2.047 / 2047.0 * 3.0;
            break;
        case Resolution14Bit:
            // Serial.print(adc_Value[ch - 1]);
            // Serial.print(" ");
            return adc_Value[ch - 1] * 0.00075; // 2.047 / 8191.0 * 3.0;
            break;
        case Resolution16Bit:
            // Serial.print(adc_Value[ch - 1]);
            // Serial.print(" ");
            return adc_Value[ch - 1] * 0.0001875; // 2.047 / 32767.0 * 3.0;
            break;
        default:
            SERIAL_PORT.print("wrong RES");
            return 0;
            break;
        }
    }
}

float getAdcVoltage_BOT(uint8_t ch, bool isCurrent)
{
    if (isCurrent) // 4-20mA
    {
        switch (resolution_BOT)
        {
        case Resolution12Bit:
            return adc_Value_BOT[ch - 1] * 0.01; // 2.047 / 2047.0 / 100;
            break;
        case Resolution14Bit:
            return adc_Value_BOT[ch - 1] * 0.0025; // 2.047 / 8191.0 / 100;
            break;
        case Resolution16Bit:
            return adc_Value_BOT[ch - 1] * 0.000625; // 2.047 / 32767.0 / 100;
            break;
        default:
            SERIAL_PORT.print("wrong RES");
            return 0;
            break;
        }
    }
    else // 0-24V
    {
        switch (resolution_BOT)
        {
        case Resolution12Bit:
            return adc_Value_BOT[ch - 1] * 0.016; // 2.047 / 2047.0 * 6.0;
            break;
        case Resolution14Bit:
            return adc_Value_BOT[ch - 1] * 0.004; // 2.047 / 8191.0 * 6.0;
            break;
        case Resolution16Bit:
            return adc_Value_BOT[ch - 1] * 0.001; // 2.047 / 32767.0 * 16;
            break;
        default:
            SERIAL_PORT.print("wrong RES");
            return 0;
            break;
        }
    }
}

bool isADCready()
{
    return MCP3428_adc.CheckConversion();
}

float checkZero(float value)
{
    if (value > 0.05)
        return value;
    else
        return 0;
}

float getAdcVoltage_CH1()
{
    return checkZero(getAdcVoltage(1, Ch1_div));
}

float getAdcVoltage_CH2()
{
    return checkZero(getAdcVoltage(2, Ch2_div));
}

float getAdcVoltage_CH3()
{
    return checkZero(getAdcVoltage(3, Ch3_div));
}

float getAdcVoltage_12V()
{
    return checkZero(getAdcVoltage(4, DIV_12V));
}

float get4_20mA_CH1()
{
    return checkZero(getAdcVoltage_BOT(1, is4_20mA));
}

float get4_20mA_CH2()
{
    return checkZero(getAdcVoltage_BOT(2, is4_20mA));
}

float getAdcVoltage_24V()
{
    return checkZero(getAdcVoltage_BOT(3, DIV_24V));
}

void processADConversation()
{
    switch (ADC_State)
    {
    case wait_Init:
        if (!get_5V_Error())
        {
            if (!init_flag_PCP3428_Top || !init_flag_PCP3428_Bot)
            {
                initADC_TOP(Resolution16Bit);
                initADC_BOT(Resolution16Bit);
            }
            ADC_State = Set;
        }
        break;
    case Set:
        StartAdcConversation(adc_CH);
        ADC_State = Set_BOT;
        break;
    case Set_BOT:
        StartAdcConversation_BOT(adc_CH);
        ADC_State = Read;
        break;

    case Read:
        if (delayCheck(READ_Delay, sampleRate_10SPS))
        {
            if (!get_5V_Error() && init_flag_PCP3428_Top)
            {
                adc_Value[adc_CH - 1] = ReadAdcValue();
            }
            else
            {
                adc_Value[adc_CH - 1] = 0;
            }

            READ_Delay = millis();
            ADC_State = Read_BOT;
        }
        break;
    case Read_BOT:
        if (!get_5V_Error() && init_flag_PCP3428_Bot)
        {
            adc_Value_BOT[adc_CH - 1] = ReadAdcValue_BOT();
        }
        else
        {
            adc_Value_BOT[adc_CH - 1] = 0;
        }

        adc_CH++;
        if (adc_CH > maxADC_CH)
            adc_CH = 1;

        READ_Delay = millis();

        if (!init_flag_PCP3428_Top || !init_flag_PCP3428_Bot)
        {
            ADC_State = wait_Init;
        }
        else
        {
            ADC_State = Set;
        }
        break;

    default:
        break;
    }
}