#include <stdint.h>

#include "ADS1X15.h"
#include "Device_setup.h"
#include "ErrorHandling.h"
#include "HelperFunc.h"
#include "I2C_IOExpander.h"
#include "MCP3428.h"
#include "OpenKNX.h"
#include "ReadADC.h"
#include "SystemFailureHandling.h"

#define MaxInputChannel 4

#define CH1 1
#define CH2 2
#define CH3 3
#define CH4 4

#define gain_1 1
#define gain_2 2
#define gain_4 4
#define gain_8 8

uint8_t failureCounter_ADC_TOP = 0;
uint8_t failureCounter_ADC_BOT = 0;

uint32_t READ_Delay = 0;
uint8_t adc_CH = 1;
uint8_t adc_CH_BOT = 0;

long adc_Value_TOP[MaxInputChannel] = {1};
long adc_Value_BOT[MaxInputChannel] = {1};

uint8_t resolution_TOP = 0;
uint8_t resolution_BOT = 0;

bool ADC_div[MaxInputChannel] = {false};
float corr_Factor[MaxInputChannel] = {1};

bool Ch1_div = false;
bool Ch2_div = false;
bool Ch3_div = false;

bool i2c_ADC_run_TOP = false;
bool i2c_ADC_run_BOT = false;

bool init_flag_Top = false;
bool init_flag_Bot = false;

/**************  MCP3428  ******************************/
#define sampleRate_100SPS 10 // read each  10ms
#define sampleRate_20SPS 50  // read each  50ms
#define sampleRate_10SPS 100 // read each 100ms
#define sampleRate_5SPS 200  // read each 200ms

MCP3428 MCP3428_adc_TOP(i2cADC, &Wire1);
MCP3428 MCP3428_adc_BOT(i2cADC_BOT, &Wire1);
/*******************************************************/

/**************  ADS1015  ******************************/
ADS1015 ADS_TOP(i2cADC_ADS1015_TOP, &Wire1);
ADS1015 ADS_BOT(i2cADC_ADS1015_BOT, &Wire1);
ADS1115 ADS1115_TOP(i2cADC_ADS1015_TOP, &Wire1);
ADS1115 ADS1115_BOT(i2cADC_ADS1015_BOT, &Wire1);
/*******************************************************/

enum State
{
    Set_TOP = 1,
    Set_BOT = 2,
    Read_TOP = 3,
    Read_BOT = 4,
    wait_Init = 5,
};
State ADC_State_TOP = wait_Init;
State ADC_State_BOT = wait_Init;

enum ADC_enum_CH
{
    ADC_1,
    ADC_2,
    ADC_3,
    ADC_4,
};
ADC_enum_CH ADC_TOP_CH = ADC_1;
ADC_enum_CH ADC_BOT_CH = ADC_1;

/********************************************************************************************
      ADS1015
********************************************************************************************/

bool I2CisConnected(uint8_t addr)
{
    Wire1.beginTransmission(addr);
    return (Wire1.endTransmission() == 0);
}

void initADC_TOP_ADS1115(uint8_t res_top)
{
    resolution_TOP = res_top; // set resolution BOT

    Serial.print("  ADS1115 TOP:");
    ADS1115_TOP.begin();
    if (I2CisConnected(i2cADC_ADS1015_TOP))
    {
        // I2C_adc.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
        // I2C_adc.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV
        // I2C_adc.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV
        // I2C_adc.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV
        // I2C_adc.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV
        // I2C_adc.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV
        ADS1115_TOP.setGain(2);
        // ADC_to_voltage_factor = ADS1115_TOP.toVoltage(); //  voltage factor
        i2c_ADC_run_TOP = true;
        init_flag_Top = true;
        Serial.println(" run");
    }
    else
    {
        init_flag_Top = false;
        i2c_ADC_run_TOP = false;
        Serial.println(" ERROR");
    }
}

void initADC_TOP_ADS1015(uint8_t res_top)
{

    resolution_TOP = res_top; // set resolution BOT

    Serial.print("  ADS1015 TOP:");
    ADS_TOP.begin();
    if (I2CisConnected(i2cADC_ADS1015_TOP))
    {
        // I2C_adc.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
        // I2C_adc.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV
        // I2C_adc.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV
        // I2C_adc.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV
        // I2C_adc.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV
        // I2C_adc.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV
        ADS_TOP.setGain(2);
        // ADC_to_voltage_factor = ADS_TOP.toVoltage(); //  voltage factor
        i2c_ADC_run_TOP = true;
        init_flag_Top = true;
        Serial.println(" run");
    }
    else
    {
        init_flag_Top = false;
        i2c_ADC_run_TOP = false;
        Serial.println(" ERROR");
    }
}

void initADC_BOT_ADS1115(uint8_t res_bot)
{

    resolution_BOT = res_bot; // set resolution BOT

    Serial.print("  ADS1115 BOT:");
    ADS1115_BOT.begin();
    if (I2CisConnected(i2cADC_ADS1015_BOT))
    {
        // I2C_adc.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
        // I2C_adc.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV
        // I2C_adc.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV
        // I2C_adc.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV
        // I2C_adc.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV
        // I2C_adc.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV
        ADS1115_BOT.setGain(2);
        // ADC_to_voltage_factor = ADS1115_BOT.toVoltage(); //  voltage factor
        i2c_ADC_run_BOT = true;
        init_flag_Bot = true;
        Serial.println(" run");
    }
    else
    {
        i2c_ADC_run_BOT = false;
        init_flag_Bot = false;
        Serial.println(" ERROR");
    }
}

void initADC_BOT_ADS1015(uint8_t res_bot)
{

    resolution_BOT = res_bot; // set resolution BOT

    Serial.print("  ADS1015 BOT:");
    ADS_BOT.begin();
    if (I2CisConnected(i2cADC_ADS1015_BOT))
    {
        // I2C_adc.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
        // I2C_adc.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV
        // I2C_adc.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV
        // I2C_adc.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV
        // I2C_adc.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV
        // I2C_adc.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV
        ADS_BOT.setGain(2);
        // ADC_to_voltage_factor = ADS_BOT.toVoltage(); //  voltage factor
        i2c_ADC_run_BOT = true;
        init_flag_Bot = true;
        Serial.println(" run");
    }
    else
    {
        i2c_ADC_run_BOT = false;
        init_flag_Bot = false;
        Serial.println(" ERROR");
    }
}

bool getAdcI2cRun_TOP()
{
    return i2c_ADC_run_TOP;
}

bool getAdcI2cRun_BOT()
{
    return i2c_ADC_run_BOT;
}

bool isAdcI2cBusy_TOP()
{
    switch (get_HW_ID_TOP())
    {
        case HW_1_0:
        case HW_2_0:
        case HW_2_1:
            return ADS_TOP.isBusy();
            break;
        case HW_3_0:
            return ADS1115_TOP.isBusy();
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID isAdcI2cBusy_TOP");
            return 0;
            break;
    }
}

bool isAdcI2cBusy_BOT()
{
    switch (get_HW_ID_BOT())
    {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
        case HW_BOT_2_1:
            return ADS_BOT.isBusy();
            break;
        case HW_BOT_5_0:
            return ADS1115_BOT.isBusy();
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID isAdcI2cBusy_BOT");
            return 0;
            break;
    }
}

void requestADC_TOP(uint8_t ch)
{
    switch (get_HW_ID_TOP())
    {
        case HW_1_0:
        case HW_2_0:
        case HW_2_1:
            ADS_TOP.requestADC(ch);
            break;
        case HW_3_0:
            ADS1115_TOP.requestADC(ch);
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID requestADC_TOP");
            break;
    }
}

void requestADC_BOT(uint8_t ch)
{
    switch (get_HW_ID_BOT())
    {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
        case HW_BOT_2_1:
            ADS_BOT.requestADC(ch);
            break;
        case HW_BOT_5_0:
            ADS1115_BOT.requestADC(ch);
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID requestADC_BOT");
            break;
    }
}

void readAdcI2cValue_TOP(uint8_t ch)
{
    switch (get_HW_ID_TOP())
    {
        case HW_1_0:
        case HW_2_0:
        case HW_2_1:
            adc_Value_TOP[ch] = ADS_TOP.getValue();
            break;
        case HW_3_0:
            adc_Value_TOP[ch] = ADS1115_TOP.getValue();
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID readAdcI2cValue_TOP");
            break;
    }
}

void readAdcI2cValue_BOT(uint8_t ch)
{
    switch (get_HW_ID_BOT())
    {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
        case HW_BOT_2_1:
            adc_Value_BOT[ch] = ADS_BOT.getValue();
            break;
        case HW_BOT_5_0:
            adc_Value_BOT[ch] = ADS1115_BOT.getValue();
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID readAdcI2cValue_BOT");
            break;
    }
}

uint16_t getAdcI2cValue_TOP(uint8_t ch)
{
    return adc_Value_TOP[ch];
}

float getAdcI2cValueMillivolt_TOP(uint8_t ch)
{
    return (float)getAdcI2cValue_TOP(ch) / (2.0 / 12.0);
}

uint16_t getAdcI2cValue_BOT(uint8_t ch)
{
    return adc_Value_BOT[ch];
}

/********************************************************************************************
  ENDE    ADS1015
********************************************************************************************/

void initADC_TOP_MCP3428(uint8_t res_top)
{
    resolution_TOP = res_top; // set resolution TOP
    corr_Factor[3] = 1;       // Correctionfactor 12V input always 1

    if (!get_24V_AC_Error())
    {
        SERIAL_PORT.print("  MCP3428_TOP:");
        if (MCP3428_adc_TOP.testConnection())
        {
            SERIAL_PORT.println("OK");
            init_flag_Top = true;
        }
        else
        {
            init_flag_Top = false;
            SERIAL_PORT.println("NOK");
            failureCounter_ADC_TOP++;
            if (failureCounter_ADC_TOP > 10)
            {
                // rebootExternalPWR();                     // *******************************************************************************************
                failureCounter_ADC_TOP = 0;
            }
        }
    }
    else
    {
        SERIAL_PORT.print("  MCP3428_TOP ERROR: no +5V_Iso");
    }
}

void initADC_BOT_MCP3428(uint8_t res_bot)
{
    resolution_BOT = res_bot; // set resolution BOT

    if (!get_24V_AC_Error())
    {
        SERIAL_PORT.print("  MCP3428_BOT:");
        if (MCP3428_adc_BOT.testConnection())
        {
            SERIAL_PORT.println("OK");
            init_flag_Bot = true;
        }
        else
        {
            init_flag_Bot = false;
            SERIAL_PORT.println("NOK");
            failureCounter_ADC_BOT++;
            if (failureCounter_ADC_BOT > 10)
            {
                // rebootExternalPWR();  //************************************************************************************************
                failureCounter_ADC_BOT = 0;
            }
        }
    }
    else
    {
        SERIAL_PORT.print("  MCP3428_BOT ERROR: no +5V_Iso");
    }
}

bool isAdcI2cBusy_TOP_MCP3428()
{
    return MCP3428_adc_TOP.CheckConversion();
}

bool isAdcBusy_TOP_MCP3428()
{
    return MCP3428_adc_TOP.CheckforResult();
}

bool isAdcBusy_BOT_MCP3428()
{
    return MCP3428_adc_BOT.CheckforResult();
}

void clearInitFlags_ADC()
{
    if (init_flag_Top || init_flag_Bot)
    {
        init_flag_Top = false;
        init_flag_Bot = false;

        // clear ADC values
        for (int i = 0; i < MaxInputChannel; i++)
        {
            adc_Value_TOP[i] = 0;
            adc_Value_BOT[i] = 0;
        }
        SERIAL_PORT.println("ERROR --> ADC Cleared !!!");
    }
}

void set_ADC_DIV(uint8_t ch, bool div)
{
    ADC_div[ch] = div;

    switch (ch)
    {
        case 0:
            set_IOExpander_TOP_Input(IO_Set_DIV_1, div);
            SERIAL_PORT.print("Ch1 set ADC Div: ");
            SERIAL_PORT.println(div);
            break;
        case 1:
            set_IOExpander_TOP_Input(IO_Set_DIV_2, div);
            SERIAL_PORT.print("Ch2 set ADC Div: ");
            SERIAL_PORT.println(div);
            break;
        case 2:
            set_IOExpander_TOP_Input(IO_Set_DIV_3, div);
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

void StartAdcConversation_TOP(uint8_t ch)
{
    if (!get_24V_AC_Error() && init_flag_Top)
    {
        MCP3428_adc_TOP.SetConfiguration(ch + 1, resolution_TOP, 0, gain_1);
    }
}

void StartAdcConversation_BOT(uint8_t ch)
{
    if (!get_24V_AC_Error() && init_flag_Bot)
    {
        MCP3428_adc_BOT.SetConfiguration(ch + 1, resolution_BOT, 0, gain_1);
    }
}

void Readadc_Value_TOP(uint8_t ch)
{
    adc_Value_TOP[ch] = MCP3428_adc_TOP.readADC();
}

void Readadc_Value_BOT(uint8_t ch)
{
    adc_Value_BOT[ch] = MCP3428_adc_BOT.readADC();
}

uint16_t getadc_Value_TOP(uint8_t ch)
{
    return adc_Value_TOP[ch];
}

uint16_t getadc_Value_BOT(uint8_t ch)
{
    return adc_Value_BOT[ch];
}

float checkZero(float value)
{
    if (value > 0.07)
        return value;
    else
        return 0;
}

float getAdcVoltage_TOP(uint8_t ch)
{
    switch (resolution_TOP)
    {
        case Resolution12Bit:
            return checkZero((adc_Value_TOP[ch] * 0.006) / corr_Factor[ch]); // 2.047 / 2047.0 * 6.0;
            break;
        case Resolution14Bit:
            return checkZero((adc_Value_TOP[ch] * 0.0015) / corr_Factor[ch]); // 2.047 / 8191.0 * 6.0;
            break;
        case Resolution16Bit:
            return checkZero((adc_Value_TOP[ch] * 0.000375) / corr_Factor[ch]); // 2.047 / 32767.0 * 6.0;
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

float getAdcVoltage_BOT(uint8_t ch)
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

float getAdcVoltage(uint8_t ch)
{
    return getAdcVoltage_TOP(ch);
}

bool processADConversation_TOP()
{
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_TOP())
        {
            case HW_1_0:
            case HW_2_0:
                if (!get_24V_AC_Error())
                {
                    switch (ADC_State_TOP)
                    {
                        case wait_Init:
                            if (!get_24V_AC_Error())
                            {
                                if (!init_flag_Top)
                                {
                                    initADC_TOP_MCP3428(Resolution16Bit);
                                    ADC_TOP_CH = ADC_1; // Damit er immer bei einer Spannungswiederker mit ADC1 beginnt
                                }
                                ADC_State_TOP = Set_TOP;
                            }
                            break;
                        case Set_TOP:
                            switch (ADC_TOP_CH)
                            {
                                case ADC_1:
                                    StartAdcConversation_TOP(2); //  CH1 -> liegt auch ADC2
                                    break;
                                case ADC_2:
                                    StartAdcConversation_TOP(1); //  CH2 -> liegt auch ADC1
                                    break;
                                case ADC_3:
                                    StartAdcConversation_TOP(0); //  CH3 -> liegt auch ADC0
                                    break;
                                case ADC_4:
                                    StartAdcConversation_TOP(3); // CH4  > liegt auch ADC3
                                    break;
                                default:
                                    Serial.println("Wrong StateADC TOP CH");
                                    break;
                            }
                            ADC_State_TOP = Read_TOP;
                            break;
                        case Read_TOP:

                            if (isAdcBusy_TOP_MCP3428() == false)
                            {
                                switch (ADC_TOP_CH)
                                {
                                    case ADC_1: // CH1
                                        Readadc_Value_TOP(0);
                                        ADC_TOP_CH = ADC_2;
                                        break;
                                    case ADC_2: // CH2
                                        Readadc_Value_TOP(1);
                                        ADC_TOP_CH = ADC_3;
                                        break;
                                    case ADC_3: // CH3
                                        Readadc_Value_TOP(2);
                                        ADC_TOP_CH = ADC_4;
                                        break;
                                    case ADC_4: // HSS
                                        Readadc_Value_TOP(3);
                                        ADC_TOP_CH = ADC_1;
                                        ADC_State_TOP = wait_Init; // Needed because of the Return true
                                        return true;
                                        break;

                                    default:
                                        Serial.println("Wrong StateADC TOP CH");
                                        break;
                                }
                                ADC_State_TOP = wait_Init;
                            }

                            break;

                        default:
                            Serial.println("Wrong StateADC TOP");
                            break;
                    }
                    break;

                    case HW_2_1:
                        if (!get_24V_AC_Error())
                        {
                            switch (ADC_State_TOP)
                            {
                                case wait_Init:
                                    if (!get_24V_AC_Error())
                                    {
                                        if (!init_flag_Top)
                                        {
                                            initADC_TOP_ADS1015(Resolution12Bit);
                                            ADC_TOP_CH = ADC_1; // Damit er immer bei einer Spannungswiederker mit ADC1 beginnt
                                        }
                                        ADC_State_TOP = Set_TOP;
                                    }
                                    break;
                                case Set_TOP:
                                    switch (ADC_TOP_CH)
                                    {
                                        case ADC_1:
                                            requestADC_TOP(2); //  CH1 -> liegt auch ADC2
                                            break;
                                        case ADC_2:
                                            requestADC_TOP(1); //  CH2 -> liegt auch ADC1
                                            break;
                                        case ADC_3:
                                            requestADC_TOP(0); //  CH3 -> liegt auch ADC0
                                            break;
                                        case ADC_4:
                                            requestADC_TOP(3); // CH4  > liegt auch ADC3
                                            break;
                                        default:
                                            Serial.println("Wrong StateADC TOP CH");
                                            break;
                                    }
                                    ADC_State_TOP = Read_TOP;
                                    break;
                                case Read_TOP:

                                    if (isAdcI2cBusy_TOP() == false)
                                    {
                                        switch (ADC_TOP_CH)
                                        {
                                            case ADC_1: // CH1
                                                readAdcI2cValue_TOP(0);
                                                ADC_TOP_CH = ADC_2;
                                                break;
                                            case ADC_2: // CH2
                                                readAdcI2cValue_TOP(1);
                                                ADC_TOP_CH = ADC_3;
                                                break;
                                            case ADC_3: // CH3
                                                readAdcI2cValue_TOP(2);
                                                ADC_TOP_CH = ADC_4;
                                                break;
                                            case ADC_4: // HSS
                                                readAdcI2cValue_TOP(3);
                                                ADC_TOP_CH = ADC_1;
                                                ADC_State_TOP = wait_Init; // Needed because of the Return true
                                                return true;
                                                break;

                                            default:
                                                Serial.println("Wrong StateADC TOP CH");
                                                break;
                                        }
                                        ADC_State_TOP = wait_Init;
                                    }
                                    break;

                                default:
                                    Serial.println("Wrong StateADC TOP");
                                    break;
                            }
                        }
                        else
                        {
                            ADC_State_TOP = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                        }
                        break;
                    case HW_3_0:
                        if (!get_24V_AC_Error())
                        {
                            switch (ADC_State_TOP)
                            {
                                case wait_Init:
                                    if (!get_24V_AC_Error())
                                    {
                                        if (!init_flag_Top)
                                        {
                                            initADC_TOP_ADS1115(Resolution16Bit);
                                            ADC_TOP_CH = ADC_1; // Damit er immer bei einer Spannungswiederker mit ADC1 beginnt
                                        }
                                        ADC_State_TOP = Set_TOP;
                                    }
                                    break;
                                case Set_TOP:
                                    switch (ADC_TOP_CH)
                                    {
                                        case ADC_1:
                                            requestADC_TOP(2); //  CH1 -> liegt auch ADC2
                                            break;
                                        case ADC_2:
                                            requestADC_TOP(1); //  CH2 -> liegt auch ADC1
                                            break;
                                        case ADC_3:
                                            requestADC_TOP(0); //  CH3 -> liegt auch ADC0
                                            break;
                                        case ADC_4:
                                            requestADC_TOP(3); // CH4  > liegt auch ADC3
                                            break;
                                        default:
                                            Serial.println("Wrong StateADC TOP CH");
                                            break;
                                    }
                                    ADC_State_TOP = Read_TOP;
                                    break;
                                case Read_TOP:

                                    if (isAdcI2cBusy_TOP() == false)
                                    {
                                        switch (ADC_TOP_CH)
                                        {
                                            case ADC_1: // CH1
                                                readAdcI2cValue_TOP(0);
                                                ADC_TOP_CH = ADC_2;
                                                break;
                                            case ADC_2: // CH2
                                                readAdcI2cValue_TOP(1);
                                                ADC_TOP_CH = ADC_3;
                                                break;
                                            case ADC_3: // CH3
                                                readAdcI2cValue_TOP(2);
                                                ADC_TOP_CH = ADC_4;
                                                break;
                                            case ADC_4: // HSS
                                                readAdcI2cValue_TOP(3);
                                                ADC_TOP_CH = ADC_1;
                                                ADC_State_TOP = wait_Init; // Needed because of the Return true
                                                return true;
                                                break;

                                            default:
                                                Serial.println("Wrong StateADC TOP CH");
                                                break;
                                        }
                                        ADC_State_TOP = wait_Init;
                                    }
                                    break;

                                default:
                                    Serial.println("Wrong StateADC TOP");
                                    break;
                            }
                        }
                        else
                        {
                            ADC_State_TOP = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                        }
                        break;

                    default:
                        Serial.println("Wrong HW ID ADC TOP");
                        break;
                }
                else
                {
                    ADC_State_TOP = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                }
        }
    }
    else
    {
        ADC_State_TOP = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
    }
    return false;
}

bool processADConversation_BOT()
{
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_BOT())
        {
            case HW_BOT_1_0:
            case HW_BOT_2_0:
                if (!get_24V_AC_Error())
                {
                    switch (ADC_State_BOT)
                    {
                        case wait_Init:
                            if (!get_24V_AC_Error())
                            {
                                if (!init_flag_Bot)
                                {
                                    initADC_BOT_MCP3428(Resolution16Bit);
                                    ADC_BOT_CH = ADC_1; // Damit er immer bei einer Spannungswiederker mit ADC1 beginnt
                                }
                                ADC_State_BOT = Set_BOT;
                            }
                            break;
                        case Set_BOT:
                            switch (ADC_BOT_CH)
                            {
                                case ADC_1:
                                    StartAdcConversation_BOT(0);
                                    break;
                                case ADC_2:
                                    StartAdcConversation_BOT(1);
                                    break;
                                case ADC_3:
                                    StartAdcConversation_BOT(2);
                                    break;
                                case ADC_4:
                                    StartAdcConversation_BOT(3);
                                    break;
                                default:
                                    Serial.println("Wrong StateADC BOT CH");
                                    break;
                            }
                            ADC_State_BOT = Read_BOT;
                            break;
                        case Read_BOT:

                            if (isAdcBusy_BOT_MCP3428() == false)
                            {
                                switch (ADC_BOT_CH)
                                {
                                    case ADC_1: // CH1
                                        Readadc_Value_BOT(0);
                                        ADC_BOT_CH = ADC_2;
                                        break;
                                    case ADC_2: // CH2
                                        Readadc_Value_BOT(1);
                                        ADC_BOT_CH = ADC_3;
                                        break;
                                    case ADC_3: // CH3
                                        Readadc_Value_BOT(2);
                                        ADC_BOT_CH = ADC_4;
                                        break;
                                    case ADC_4: // HSS
                                        Readadc_Value_BOT(3);
                                        ADC_BOT_CH = ADC_1;
                                        ADC_State_BOT = wait_Init; // Needed because of the Return true
                                        return true;
                                        break;

                                    default:
                                        Serial.println("Wrong StateADC BOT CH");
                                        break;
                                }
                                ADC_State_BOT = wait_Init;
                            }
                            else
                            {
                                if (!init_flag_Bot)
                                {
                                    ADC_State_BOT = wait_Init;
                                }
                            }
                            break;

                        default:
                            Serial.println("Wrong StateADC BOT");
                            break;
                    }
                }
                else
                {
                    ADC_State_BOT = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                }

                break;
            case HW_BOT_2_1:
                if (!get_24V_AC_Error())
                {
                    switch (ADC_State_BOT)
                    {
                        case wait_Init:
                            if (!get_24V_AC_Error())
                            {
                                if (!init_flag_Bot)
                                {
                                    initADC_BOT_ADS1015(Resolution12Bit);
                                    ADC_BOT_CH = ADC_1; // Damit er immer bei einer Spannungswiederker mit ADC1 beginnt
                                }
                                ADC_State_BOT = Set_BOT;
                            }
                            break;
                        case Set_BOT:
                            switch (ADC_BOT_CH)
                            {
                                case ADC_1:
                                    requestADC_BOT(1); // 4-20mA_1 -> liegt auch ADC1
                                    // Cur_CH = ADC2;
                                    break;
                                case ADC_2:
                                    requestADC_BOT(0); // 4-20mA_2 -> liegt auch ADC0
                                    // Cur_CH = ADC3;
                                    break;
                                case ADC_3:
                                    requestADC_BOT(2);
                                    // Cur_CH = ADC4;
                                    break;
                                case ADC_4:
                                    requestADC_BOT(3);
                                    // Cur_CH = ADC1;
                                    break;
                                default:
                                    Serial.println("Wrong StateADC BOT CH");
                                    break;
                            }
                            ADC_State_BOT = Read_BOT;
                            break;
                        case Read_BOT:

                            if (isAdcI2cBusy_BOT() == false)
                            {
                                switch (ADC_BOT_CH)
                                {
                                    case ADC_1: // 4-20mA CH1
                                        readAdcI2cValue_BOT(0);
                                        ADC_BOT_CH = ADC_2;
                                        break;
                                    case ADC_2: // 4-20mA CH2
                                        readAdcI2cValue_BOT(1);
                                        ADC_BOT_CH = ADC_3;
                                        break;
                                    case ADC_3: // HSS CS1
                                        readAdcI2cValue_BOT(2);
                                        ADC_BOT_CH = ADC_4;
                                        break;
                                    case ADC_4: // HSS CS2
                                        readAdcI2cValue_BOT(3);
                                        ADC_BOT_CH = ADC_1;
                                        ADC_State_BOT = wait_Init; // Needed because of the Return true
                                        return true;
                                        break;

                                    default:
                                        Serial.println("Wrong StateADC BOT CH");
                                        break;
                                }
                                ADC_State_BOT = wait_Init;
                            }
                            else
                            {
                                ADC_State_BOT = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                            }
                            break;

                        default:
                            Serial.println("Wrong StateADC");
                            break;
                    }
                }
                else
                {
                    ADC_State_BOT = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                }
                break;
            case HW_BOT_5_0:
                if (!get_24V_AC_Error())
                {
                    switch (ADC_State_BOT)
                    {
                        case wait_Init:
                            if (!get_24V_AC_Error())
                            {
                                if (!init_flag_Bot)
                                {
                                    initADC_BOT_ADS1115(Resolution16Bit);
                                    ADC_BOT_CH = ADC_1; // Damit er immer bei einer Spannungswiederker mit ADC1 beginnt
                                }
                                ADC_State_BOT = Set_BOT;
                            }
                            break;
                        case Set_BOT:
                            switch (ADC_BOT_CH)
                            {
                                case ADC_1:
                                    requestADC_BOT(1); // 4-20mA_1 -> liegt auch ADC1
                                    // Cur_CH = ADC2;
                                    break;
                                case ADC_2:
                                    requestADC_BOT(0); // 4-20mA_2 -> liegt auch ADC0
                                    // Cur_CH = ADC3;
                                    break;
                                case ADC_3:
                                    requestADC_BOT(2);
                                    // Cur_CH = ADC4;
                                    break;
                                case ADC_4:
                                    requestADC_BOT(3);
                                    // Cur_CH = ADC1;
                                    break;
                                default:
                                    Serial.println("Wrong StateADC BOT CH");
                                    break;
                            }
                            ADC_State_BOT = Read_BOT;
                            break;
                        case Read_BOT:

                            if (isAdcI2cBusy_BOT() == false)
                            {
                                switch (ADC_BOT_CH)
                                {
                                    case ADC_1: // 4-20mA CH1
                                        readAdcI2cValue_BOT(0);
                                        ADC_BOT_CH = ADC_2;
                                        break;
                                    case ADC_2: // 4-20mA CH2
                                        readAdcI2cValue_BOT(1);
                                        ADC_BOT_CH = ADC_3;
                                        break;
                                    case ADC_3: // HSS CS1
                                        readAdcI2cValue_BOT(2);
                                        ADC_BOT_CH = ADC_4;
                                        break;
                                    case ADC_4: // HSS CS2
                                        readAdcI2cValue_BOT(3);
                                        ADC_BOT_CH = ADC_1;
                                        ADC_State_BOT = wait_Init; // Needed because of the Return true
                                        return true;
                                        break;

                                    default:
                                        Serial.println("Wrong StateADC BOT CH");
                                        break;
                                }
                                ADC_State_BOT = wait_Init;
                            }
                            else
                            {
                                ADC_State_BOT = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                            }
                            break;

                        default:
                            Serial.println("Wrong StateADC");
                            break;
                    }
                }
                else
                {
                    ADC_State_BOT = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
                }
                break;

            default:
                Serial.println("Wrong HW-ID ADC Typ BOT");
                break;
        }
    }
    else
    {
        ADC_State_BOT = wait_Init; // When ADC is not init after pwr cycle, go back to "wait_init" to init the ADC again
    }
    return false;
}
