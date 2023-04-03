
#include <stdint.h>
#include "ReadADC.h"
#include "HelperFunc.h"

#include "MCP3428.h"
#include "GardenControl.h"
#include "SystemFailureHandling.h"
#include "I2C_IOExpander.h"
#include "ErrorHandling.h"
#include "Device_setup.h"

#define gain_1 1
#define gain_2 2
#define gain_4 4
#define gain_8 8

#define CH1 1
#define CH2 2
#define CH3 3
#define CH4 4

#define MaxInputChannel 4

#define sampleRate_100SPS 10 // read each  10ms
#define sampleRate_20SPS 50  // read each  50ms
#define sampleRate_10SPS 100 // read each 100ms
#define sampleRate_5SPS 200  // read each 200ms

MCP3428 MCP3428_adc(i2cADC, &Wire1);
MCP3428 MCP3428_adc_BOT(i2cADC_BOT, &Wire1);

uint8_t failureCounter_ADC_TOP = 0;
uint8_t failureCounter_ADC_BOT = 0;

bool init_flag_PCP3428_Top = false;
bool init_flag_PCP3428_Bot = false;

uint32_t READ_Delay = 0;
uint8_t adc_CH = 1;
uint8_t adc_CH_BOT = 0;

long adc_Value[MaxInputChannel] = {1};
long adc_Value_BOT[MaxInputChannel] = {1};

uint8_t resolution_TOP = 0;
uint8_t resolution_BOT = 0;

bool ADC_div[MaxInputChannel] = {false};
float corr_Factor[MaxInputChannel] = {1};

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
    corr_Factor[3] = 1;       // Correctionfactor 12V input always 1

    if (!get_5V_Error())
    {
        switch (get_HW_ID_TOP())
        {
        case HW_1_0:
        case HW_2_0:
            SERIAL_PORT.print("  MCP3428_TOP:");
            if (MCP3428_adc.testConnection())
            {
                SERIAL_PORT.println("OK");
                init_flag_PCP3428_Top = true;
            }
            else
            {
                init_flag_PCP3428_Top = false;
                SERIAL_PORT.println("NOK");
                failureCounter_ADC_TOP++;
                if (failureCounter_ADC_TOP > 10)
                {
                    // rebootExternalPWR();                     // *******************************************************************************************
                    failureCounter_ADC_TOP = 0;
                }
            }
            break;
        case HW_2_1:
            SERIAL_PORT.print("  ADS1015_TOP:");
            /*if (MCP3428_adc.testConnection())
            {
                SERIAL_PORT.println("OK");
                init_flag_PCP3428_Top = true;
            }
            else
            {
                init_flag_PCP3428_Top = false;
                SERIAL_PORT.println("NOK");
                failureCounter_ADC_TOP++;
                if (failureCounter_ADC_TOP > 10)
                {
                    // rebootExternalPWR();                     // *******************************************************************************************
                    failureCounter_ADC_TOP = 0;
                }
            }*/
            break;
        default:
            SERIAL_PORT.println("Wrong ID: ADC TOP Init");
            break;
        }
    }
    else
    {
        SERIAL_PORT.print("  ADC_TOP ERROR: no +5V_Iso");
    }
}

void initADC_BOT(uint8_t res_bot)
{
    resolution_BOT = res_bot; // set resolution BOT

    if (!get_5V_Error())
    {
        switch (get_HW_ID_BOT())
        {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
            SERIAL_PORT.print("  MCP3428_BOT:");
            if (MCP3428_adc_BOT.testConnection())
            {
                SERIAL_PORT.println("OK");
                init_flag_PCP3428_Bot = true;
            }
            else
            {
                init_flag_PCP3428_Bot = false;
                SERIAL_PORT.println("NOK");
                failureCounter_ADC_BOT++;
                if (failureCounter_ADC_BOT > 10)
                {
                    // rebootExternalPWR();  //************************************************************************************************
                    failureCounter_ADC_BOT = 0;
                }
            }
            break;
        case HW_BOT_2_1:
            SERIAL_PORT.print("  ADS1015_TOP:");
            /*if (MCP3428_adc.testConnection())
            {
                SERIAL_PORT.println("OK");
                init_flag_PCP3428_Top = true;
            }
            else
            {
                init_flag_PCP3428_Top = false;
                SERIAL_PORT.println("NOK");
                failureCounter_ADC_TOP++;
                if (failureCounter_ADC_TOP > 10)
                {
                    // rebootExternalPWR();                     // *******************************************************************************************
                    failureCounter_ADC_TOP = 0;
                }
            }*/
            break;
        default:
            SERIAL_PORT.println("Wrong ID: ADC BOT Init");
            break;
        }
    }
    else
    {
        SERIAL_PORT.print("  ADC_BOT ERROR: no +5V_Iso");
    }
}

void clearInitFlags_ADC()
{
    init_flag_PCP3428_Top = false;
    init_flag_PCP3428_Bot = false;

    // clear ADC values
    for (int i = 0; i < MaxInputChannel; i++)
    {
        adc_Value[i] = 0;
        adc_Value_BOT[i] = 0;
    }
}

void set_ADC_DIV(uint8_t ch, bool div)
{
    ADC_div[ch] = div;

    switch (ch)
    {
    case 0:
        set_IOExpander_Input(IO_Set_DIV_1, div);
        SERIAL_PORT.print("Ch1 set ADC Div: ");
        SERIAL_PORT.println(div);
        break;
    case 1:
        set_IOExpander_Input(IO_Set_DIV_2, div);
        SERIAL_PORT.print("Ch2 set ADC Div: ");
        SERIAL_PORT.println(div);
        break;
    case 2:
        set_IOExpander_Input(IO_Set_DIV_3, div);
        SERIAL_PORT.print("Ch3 set ADC Div: ");
        SERIAL_PORT.println(div);
        break;

    default:
        break;
    }
}

void set_ADC_CorrFactor(uint8_t ch, float corrFactor)
{
    corr_Factor[ch] = corrFactor;
}

void StartAdcConversation(uint8_t ch)
{
    if (!get_5V_Error() && init_flag_PCP3428_Top)
    {
        switch (get_HW_ID_TOP())
        {
        case HW_1_0:
        case HW_2_0:
            MCP3428_adc_BOT.SetConfiguration(ch, resolution_TOP, 1, gain_1);
            break;
        case HW_2_1:
            // ADS1015
            break;
        default:
            SERIAL_PORT.println("Wrong ID: ADC TOP Start ADC Con");
            break;
        }
    }
}

void StartAdcConversation_BOT(uint8_t ch)
{
    if (!get_5V_Error() && init_flag_PCP3428_Bot)
    {
        switch (get_HW_ID_BOT())
        {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
            MCP3428_adc_BOT.SetConfiguration(ch, resolution_BOT, 1, gain_1);
            break;
        case HW_BOT_2_1:
            // ADS1015
            break;
        default:
            SERIAL_PORT.println("Wrong ID: ADC BOT Start ADC Con");
            break;
        }
    }
}

long ReadAdcValue()
{
    switch (get_HW_ID_TOP())
    {
    case HW_1_0:
    case HW_2_0:
        return MCP3428_adc.readADC();
        break;
    case HW_2_1:
        // ADS1015
        return 0;
        break;
    default:
        SERIAL_PORT.println("Wrong ID: ADC TOP Read ADC");
        return 0;
        break;
    }
}

long ReadAdcValue_BOT()
{
    switch (get_HW_ID_BOT())
    {
    case HW_BOT_1_0:
    case HW_BOT_2_0:
        return MCP3428_adc_BOT.readADC();
        break;
    case HW_BOT_2_1:
        // ADS1015
        return 0;
        break;
    default:
        SERIAL_PORT.println("Wrong ID: ADC BOT Read ADC");
        return 0;
        break;
    }
}

uint16_t getAdcValue(uint8_t ch)
{
    return adc_Value[ch];
}

uint16_t getAdcValue_BOT(uint8_t ch)
{
    return adc_Value_BOT[ch];
}

float checkZero(float value)
{
    if (value > 0.05)
        return value;
    else
        return 0;
}

float getAdcVoltage(uint8_t ch, bool div)
{
    switch (resolution_TOP)
    {
    case Resolution12Bit:
        return checkZero((adc_Value[ch] * 0.006) / corr_Factor[ch]); // 2.047 / 2047.0 * 6.0;
        break;
    case Resolution14Bit:
        return checkZero((adc_Value[ch] * 0.0015) / corr_Factor[ch]); // 2.047 / 8191.0 * 6.0;
        break;
    case Resolution16Bit:
        return checkZero((adc_Value[ch] * 0.000375) / corr_Factor[ch]); // 2.047 / 32767.0 * 6.0;
        break;
    default:
#ifdef InputADC_Output
        SERIAL_PORT.println("wrong RES 0-12V");
        SERIAL_PORT.println(resolution_TOP);
#endif
        return 0;
        break;
    }
    // ALTE UMSETZUNG HW1.x
    /*
    if (div) // 0-12V
    {
        switch (resolution_TOP)
        {
        case Resolution12Bit:
            return checkZero((adc_Value[ch] * 0.006) / corr_Factor[ch]); // 2.047 / 2047.0 * 6.0;
            break;
        case Resolution14Bit:
            return checkZero((adc_Value[ch] * 0.0015) / corr_Factor[ch]); // 2.047 / 8191.0 * 6.0;
            break;
        case Resolution16Bit:
            return checkZero((adc_Value[ch] * 0.000375) / corr_Factor[ch]); // 2.047 / 32767.0 * 6.0;
            break;
        default:
#ifdef InputADC_Output
            SERIAL_PORT.println("wrong RES 0-12V");
            SERIAL_PORT.println(resolution_TOP);
#endif
            return 0;
            break;
        }
    }
    else // 0-5V
    {
#ifdef InputADC_Output
        SERIAL_PORT.print("5V ");
#endif
        switch (resolution_TOP)
        {
        case Resolution12Bit:
            return checkZero(adc_Value[ch] * 0.003); // 2.047 / 2047.0 * 3.0;
            break;
        case Resolution14Bit:
            return checkZero(adc_Value[ch] * 0.00075); // 2.047 / 8191.0 * 3.0;
            break;
        case Resolution16Bit:
            return checkZero(adc_Value[ch] * 0.0001875); // 2.047 / 32767.0 * 3.0;
            break;
        default:
#ifdef InputADC_Output
            SERIAL_PORT.println("wrong RES 0-5V");
#endif
            return 0;
            break;
        }
    }
    */
}

float getAdcVoltage_BOT(uint8_t ch, bool isCurrent)
{
    if (isCurrent) // 4-20mA
    {
        switch (resolution_BOT)
        {
        case Resolution12Bit:
            return checkZero(adc_Value_BOT[ch] * 0.01); // 2.047 / 2047.0 / 100;
            break;
        case Resolution14Bit:
            return checkZero(adc_Value_BOT[ch] * 0.0025); // 2.047 / 8191.0 / 100;
            break;
        case Resolution16Bit:
            return checkZero(adc_Value_BOT[ch] * 0.000625); // 2.047 / 32767.0 / 100;
            break;
        default:
#ifdef Input_4_20mA_Output
            SERIAL_PORT.println("wrong RES 4-20mA");
#endif
            return 0;
            break;
        }
    }
    else // 0-24V
    {
        switch (resolution_BOT)
        {
        case Resolution12Bit:
            return checkZero(adc_Value_BOT[ch] * 0.016); // 2.047 / 2047.0 * 6.0;
            break;
        case Resolution14Bit:
            return checkZero(adc_Value_BOT[ch] * 0.004); // 2.047 / 8191.0 * 6.0;
            break;
        case Resolution16Bit:
            return checkZero(adc_Value_BOT[ch] * 0.001); // 2.047 / 32767.0 * 16;
            break;
        default:
#ifdef InputADC_Output
            SERIAL_PORT.print("wrong RES 0-24V: Value: ");
            SERIAL_PORT.println(resolution_BOT);
#endif
            return 0;
            break;
        }
    }
}

/*bool isADCready()
{
    return MCP3428_adc.CheckConversion();
}*/

float getAdcVoltage(uint8_t ch)
{
    return getAdcVoltage(ch, ADC_div[ch]);
}

float getAdcVoltage_12V()
{
    return getAdcVoltage(ADC_12V_CH, DIV_12V);
}

float getAdcVoltage_24V()
{
    return getAdcVoltage_BOT(ADC_24V_CH, DIV_24V);
}

float get4_20mA(uint8_t ch)
{
    return getAdcVoltage_BOT(ch, is4_20mA);
}

bool processADConversation()
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
        return false;
        break;
    case Set:
        StartAdcConversation(adc_CH);
        ADC_State = Set_BOT;
        return false;
        break;
    case Set_BOT:
        StartAdcConversation_BOT(adc_CH);
        ADC_State = Read;
        return false;
        break;

    case Read:
        if (delayCheck(READ_Delay, sampleRate_10SPS))
        {
            if (!get_5V_Error() && init_flag_PCP3428_Top)
            {
                adc_Value[adc_CH - 1] = ReadAdcValue();
                /*
                switch (adc_CH)
                {
                case CH1:
                    adc_Value[2] = ReadAdcValue();
                    break;
                case CH2:
                    adc_Value[1] = ReadAdcValue();
                    break;
                case CH3:
                    adc_Value[0] = ReadAdcValue();
                    break;
                case CH4:
                    adc_Value[3] = ReadAdcValue();
                    break;

                default:
                    break;
                }
                */
            }
            else
            {
                adc_Value[adc_CH - 1] = 0;
                /*
                switch (adc_CH)
                {
                case CH1:
                    adc_Value[2] = 0;
                    break;
                case CH2:
                    adc_Value[1] = 0;
                    break;
                case CH3:
                    adc_Value[0] = 0;
                    break;
                case CH4:
                    adc_Value[3] = 0;
                    break;

                default:
                    break;
                }
                */
            }

            READ_Delay = millis();
            ADC_State = Read_BOT;
        }
        return false;
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
        if (adc_CH > MaxInputChannel)
            adc_CH = 1;

        READ_Delay = millis();

        if (!init_flag_PCP3428_Top || !init_flag_PCP3428_Bot)
        {
            ADC_State = wait_Init;
        }
        else
        {
            ADC_State = Set;
            return true;
        }
        return false;
        break;

    default:
        return false;
        break;
    }
}