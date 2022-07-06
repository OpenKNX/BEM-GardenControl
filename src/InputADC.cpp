#include <knx.h>
#include "InputADC.h"
#include "GardenControl.h"
#include "KnxHelper.h"
#include "BEM_hardware.h"
#include "HelperFunc.h"
#include "ReadADC.h"
#include "ErrorHandling.h"

#define Channel_inaktiv 0
#define ADC_Wert 1
#define SMT50_Bodenfeuchte 2
#define SMT50_BodenTemperatur 3

#define SensorType_voltage 0
#define SensorType_temperature 1
#define SensorType_humidity 2
#define SensorType_co2 3
#define SensorType_lux 4
#define SensorType_pressure 5
#define SensorType_flow 6
#define SensorType_rain 7
#define SensorType_windspeed 8
#define SensorType_percent 9

uint32_t processDelay[ADC_ChannelCount] = {0};
uint32_t sendDelay[ADC_ChannelCount] = {0};

uint8_t channel = 0;

union InputADCValuesOLD
{
    float ladcValue[ADC_ChannelCount];
    // uint16_t ladcValueU16[ADC_ChannelCount];
    // uint8_t lsoilmoistureU8[ADC_ChannelCount];
} valueOld;

float calculateSensorValueLinearFunction(uint8_t channel, float a, float b, bool Div)
{
    return ((getAdcVoltage(channel, Div)) - b) / a;
}

void processInput_ADC(bool readyFlag)
{
    bool lSend = false;
    float lAbsolute;
    // uint16_t lAbsoluteU16;
    // uint8_t lRelativU8;
    uint8_t Dpt = 0;

    uint32_t lCycle;

    union InputADCValues
    {
        float ladcValue;
        // uint16_t ladcValueU16;
        // uint8_t lsoilmoistureU8;
    } value;

    if (!get_5V_Error())
    {

        if (knx.paramByte(getParADC(ADC_CHSensorType, channel)) != Channel_inaktiv && readyFlag)
        {

            lCycle = knx.paramWord(getParADC(ADC_CHSendcycletime, channel)) * 1000;

            // we waited enough, let's send the value
            if (lCycle && delayCheck(sendDelay[channel], lCycle))
            {
                lSend = true;
            }

            if (delayCheck(processDelay[channel], 1000) || lSend)
            {
                switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
                {
                case ADC_Wert:
#ifdef InputADC_Output
                    SERIAL_PORT.print("ADC_");
                    SERIAL_PORT.print(channel + 1);
                    SERIAL_PORT.print(": ");
                    SERIAL_PORT.print(lCycle);
                    SERIAL_PORT.print(": ");
#endif

                    // STEP 1: read Parameter DPT Format
                    switch (knx.paramByte(getParADC(ADC_CHSensorTypes, channel)))
                    {
                    case SensorType_voltage: // DPT9.020 (mV)
                        // STEP 2: Get new Sensor value
                        value.ladcValue = getAdcVoltage(channel, knx.paramByte(getParADC(ADC_CHVoltageDiv, channel)));
                        // STEP 2a: Get Abs value
                        lAbsolute = (knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel))) / 1000.0; // Value in mV
                        break;

                    default:
                        // STEP 2: Get new Sensor value
                        value.ladcValue = calculateSensorValueLinearFunction(channel, knx.paramWord(getParADC(ADC_CHGeradeM, channel)) / 100.0, knx.paramWord(getParADC(ADC_CHGeradeB, channel)) / 100.0, knx.paramByte(getParADC(ADC_CHVoltageDiv, channel)));
#ifdef InputADC_Output
                        SERIAL_PORT.print(value.ladcValue);
                        SERIAL_PORT.print(" | ");
                        SERIAL_PORT.print(knx.paramWord(getParADC(ADC_CHGeradeM, channel)) / 100.0);
                        SERIAL_PORT.print(" | ");
                        SERIAL_PORT.print(knx.paramWord(getParADC(ADC_CHGeradeB, channel)) / 100.0);
                        SERIAL_PORT.print(" | ");
#endif
                        // STEP 3: Check value Change "Absolut"
                        lAbsolute = (knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel)));
                        break;
                    }
                    // STEP 3a: Check if Change detected
                    if (lAbsolute > 0 && (abs(value.ladcValue - valueOld.ladcValue[channel])) >= lAbsolute)
                    {
                        lSend = true;
#ifdef InputADC_Output
                        SERIAL_PORT.print(" Abs ");
#endif
                    }
                    // STEP 4: Check value Change "Releative"
                    // STEP 4a: read Parameter DPT Format
                    switch (knx.paramByte(getParADC(ADC_CHSensorTypes, channel)))
                    {
                    case SensorType_voltage:                                                           // DPT9.020 (mV)
                        lAbsolute = (knx.paramByte(getParADC(ADC_CHSendenRelativ, channel))) / 1000.0; // Value in mV
                        break;

                    case SensorType_humidity:
                        lAbsolute = 100;
                        break;

                    case SensorType_percent:
                        lAbsolute = 100;
                        break;

                    default:
                        lAbsolute = (knx.paramByte(getParADC(ADC_CHSendenRelativ, channel)));
                        break;
                    }
                    // STEP 3b: Check if Change detected
                    if (lAbsolute > 0 && value.ladcValue > 0.2 && (abs(value.ladcValue - valueOld.ladcValue[channel])) >= value.ladcValue / 100 * lAbsolute)
                    {
                        lSend = true;
#ifdef InputADC_Output
                        SERIAL_PORT.print(" Rel ");
#endif
                    }
                    // STEP 4: Preset KO
                    switch (knx.paramByte(getParADC(ADC_CHSensorTypes, channel)))
                    {
                    case SensorType_voltage: // DPT9.020 (mV)
#ifdef InputADC_Output
                        SERIAL_PORT.println(value.ladcValue);
#endif
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue * 1000, getDPT(VAL_DPT_9));
                        break;

                    case SensorType_percent:
#ifdef InputADC_Output
                        SERIAL_PORT.println(value.ladcValue);
#endif
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue * 2.55, getDPT(VAL_DPT_5));
                        break;

                    default:
#ifdef InputADC_Output
                        SERIAL_PORT.println(value.ladcValue);
#endif
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue, getDPT(VAL_DPT_9));
                        break;
                    }

                    break;

                    /**********************************************************************************************************
                     *              SMT50 Bodenfeuchte                                                                        *
                     *********************************************************************************************************/
                case SMT50_Bodenfeuchte:
#ifdef InputADC_Output
                    SERIAL_PORT.print("ADC_");
                    SERIAL_PORT.print(channel + 1);
                    SERIAL_PORT.print(" ");
#endif
                    value.ladcValue = calculateSensorValueLinearFunction(channel, 0.06, 0, DIV_5V);

                    // senden bei Wertänderung Absolut
                    lAbsolute = knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel)); // Value in %
                    if (lAbsolute > 0 && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= lAbsolute)
                    {
                        lSend = true;
#ifdef InputADC_Output
                        SERIAL_PORT.print("Abs ");
#endif
                    }
#ifdef InputADC_Output
                    SERIAL_PORT.println(value.ladcValue);
#endif
                    // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                    knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend((uint8_t)(value.ladcValue * 2.55), getDPT(VAL_DPT_5));

                    break;

                    /**********************************************************************************************************
                     *              SMT50 BodenTemperatur                                                                     *
                     *********************************************************************************************************/
                case SMT50_BodenTemperatur:
#ifdef InputADC_Output
                    SERIAL_PORT.print("ADC_");
                    SERIAL_PORT.print(channel + 1);
                    SERIAL_PORT.print(" ");
#endif
                    // value.ladcValue = calculateSensorValue(channel, 0, 450); // x1 = 0.  x2 = 450°C
                    value.ladcValue = calculateSensorValueLinearFunction(channel, 0.01, 0.5, DIV_5V);

                    // senden bei Wertänderung Absolut
                    lAbsolute = knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel)) / 10.0; // Value in 0.1°C
                    if (lAbsolute > 0 && (abs(value.ladcValue - valueOld.ladcValue[channel])) >= lAbsolute)
                    {
                        lSend = true;
#ifdef InputADC_Output
                        SERIAL_PORT.print(" Abs ");
#endif
                    }
                    // senden bei Wertänderung Relativ
                    lAbsolute = knx.paramByte(getParADC(ADC_CHSendenRelativ, channel));
                    if (lAbsolute > 0 && (value.ladcValue > 0.5 || value.ladcValue < -0.5) && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= value.ladcValue / 100 * lAbsolute)
                    {
                        lSend = true;
#ifdef InputADC_Output
                        SERIAL_PORT.print(" Rel ");
#endif
                    }

#ifdef InputADC_Output
                    SERIAL_PORT.println(value.ladcValue);
#endif
                    // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                    knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue, getDPT(VAL_DPT_9));
                    break;

                default:
#ifdef InputADC_Output
                    SERIAL_PORT.print("Wrong ADC SensorTyp_CH");
                    SERIAL_PORT.print(channel + 1);
                    SERIAL_PORT.print(" | PAR value: ");
                    SERIAL_PORT.println(knx.paramByte(getParADC(ADC_CHSensorType, channel)));
#endif
                    break;
                }

                processDelay[channel] = millis();
            }

            if (lSend)
            {
#ifdef InputADC_Output
                SERIAL_PORT.print("KNX_ADC_");
                SERIAL_PORT.print(channel);
                SERIAL_PORT.print(": senden: ");
                SERIAL_PORT.println(value.ladcValue);
#endif
                knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).objectWritten();
                valueOld.ladcValue[channel] = value.ladcValue;
                sendDelay[channel] = millis();
                lSend = false;
            }
        } // ENDE if Channel aktive

        channel++;
        if (channel >= ADC_ChannelCount)
        {
            channel = 0;
        }
    } // Ende 5V Fehler
}

void initInputADC()
{
    for (int channel = 0; channel < ADC_ChannelCount; channel++)
    {
#ifdef InputADC_Output
        SERIAL_PORT.print("init ADC_");
        SERIAL_PORT.print(channel + 1);
        SERIAL_PORT.print(": ");
#endif
        switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
        {

        case Channel_inaktiv:
#ifdef InputADC_Output
            SERIAL_PORT.println("INAKTVE");
#endif
            break;

        case ADC_Wert:
            set_ADC_CorrFactor(channel, knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
#ifdef InputADC_Output
            SERIAL_PORT.println("ADC-Wert: (0-12V)");
            SERIAL_PORT.print("Kor: ");
            SERIAL_PORT.println(knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
#endif
            // set_ADC_DIV(channel, knx.paramByte(getParADC(ADC_CHVoltageDiv, channel))); //old only for HW_ID1
            break;
        case SMT50_Bodenfeuchte:
            set_ADC_CorrFactor(channel, knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
#ifdef InputADC_Output
            SERIAL_PORT.print("SMT50-BF");
            SERIAL_PORT.print("Kor: ");
            SERIAL_PORT.println(knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
#endif
            // set_ADC_DIV(channel, DIV_5V);   //old only for HW_ID1
            break;
        case SMT50_BodenTemperatur:
            set_ADC_CorrFactor(channel, knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
#ifdef InputADC_Output
            SERIAL_PORT.print("SMT50-BT");
            SERIAL_PORT.print("Kor: ");
            SERIAL_PORT.println(knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
#endif
            // set_ADC_DIV(channel, DIV_5V);   //old only for HW_ID1
            break;

        default:
#ifdef InputADC_Output
            SERIAL_PORT.print("Wrong SensorTYP: ");
            SERIAL_PORT.println(knx.paramByte(getParADC(ADC_CHSensorType, channel)));
#endif
            break;
        }
    }
}

float getSensorValue(uint8_t channel)
{
    float value;

    switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
    {
    case Channel_inaktiv:
#ifdef InputADC_Output
        SERIAL_PORT.print(" INAKTIV:  ");
#endif
        value = 0;
        break;

    case ADC_Wert:
#ifdef InputADC_Output
        SERIAL_PORT.print(" ADC-WERT: ");
#endif
        value = getAdcVoltage(channel, (knx.paramByte(getParADC(ADC_CHVoltageDiv, channel))));
        break;

    case SMT50_Bodenfeuchte:
#ifdef InputADC_Output
        SERIAL_PORT.print(" SMT50-BF: ");
#endif
        value = getAdcVoltage(channel, DIV_5V);
        value = value / 3.0 * 50.0;
        break;

    case SMT50_BodenTemperatur:
#ifdef InputADC_Output
        SERIAL_PORT.print(" SMT50-BT: ");
#endif
        value = getAdcVoltage(channel, DIV_5V);
        value = (value - 0.5) / 0.01;
        break;

    default:
#ifdef InputADC_Output
        SERIAL_PORT.print(" ADC Wrong PAR: ");
#endif
        break;
    }
    return value;
}
