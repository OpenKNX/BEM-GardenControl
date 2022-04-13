
#include <knx.h>
#include "Input_4_20mA.h"
#include "GardenControl.h"
#include "KnxHelper.h"
#include "BEM_hardware.h"
#include "HelperFunc.h"
#include "ReadADC.h"

#define Input_x_20mA_CH_inaktiv 0
#define Input_4_20mA 1
#define Input_0_20mA 2

#define value_4mA 4
#define value_0mA 0

#define SensorType_current 0
#define SensorType_temperature 1
#define SensorType_humidity 2
#define SensorType_co2 3
#define SensorType_lux 4
#define SensorType_pressure 5
#define SensorType_flow 6
#define SensorType_rain 7
#define SensorType_windspeed 8
#define SensorType_percent 9

uint32_t processDelay2[CUR_ChannelCount] = {0};
uint32_t sendDelay2[CUR_ChannelCount] = {0};

uint8_t channel2 = 0;

union InputADCValuesOLD
{
    float ladcValue[CUR_ChannelCount];
    // uint16_t ladcValueU16[CUR_ChannelCount];
    // uint8_t lsoilmoistureU8[CUR_ChannelCount];
} valueOld2;

float calculateSensorValueLinearFunction(uint8_t channel2, float a, float b)
{
    return ((get4_20mA(channel2)) - b) / a;
}

float calculateSensorValueLinearFunction2(uint8_t channel2, uint8_t p1y, int16_t p1x, int16_t p2x)
{
    float store = p2x - p1x;
    float m = (20 - p1y) / store;
    float b = p1y - (m * p1x);
    return ((get4_20mA(channel2)) - b) / m;
}

float getFunctionM(int16_t p1x, int16_t p1y, int16_t p2x, int16_t p2y)
{
    float store = p2x - p1x;
    return (p2y - p1y) / store;
}

float getFunctionB(int16_t p1x, int16_t p1y, int16_t p2x, int16_t p2y)
{
    float store = p2x - p1x;
    float m = (p2y - p1y) / store;
    return p1y - (m * p1x);
}

void processInput_4_20mA(bool readyFlag)
{
    bool lSend = false;
    float lAbsolute;
    // uint16_t lAbsoluteU16;
    // uint8_t lRelativU8;

    uint32_t lCycle;

    union InputADCValues
    {
        float ladcValue;
        // uint16_t ladcValueU16;
        // uint8_t lsoilmoistureU8;
    } value2;

    if (knx.paramByte(getParCUR(CUR_CHSensorType2, channel2)) != Input_x_20mA_CH_inaktiv && readyFlag)
    {

        lCycle = knx.paramWord(getParCUR(CUR_CHSendcycletime2, channel2)) * 1000;

        // we waited enough, let's send the value
        if (lCycle && delayCheck(sendDelay2[channel2], lCycle))
        {
            lSend = true;
        }

        if (delayCheck(processDelay2[channel2], 1000))
        {
            // SERIAL_PORT.println(knx.paramByte(getParCUR(CUR_CHSensorType2, channel2)));

            switch (knx.paramByte(getParCUR(CUR_CHSensorType2, channel2)))
            {
            case Input_4_20mA:
#ifdef Input_4_20mA_Output
                SERIAL_PORT.print("4_20mA_CH");
                SERIAL_PORT.print(channel2);
                SERIAL_PORT.print(": ");
                SERIAL_PORT.print(lCycle);
                SERIAL_PORT.print(": ");
#endif
                break;
            case Input_0_20mA:
#ifdef Input_4_20mA_Output
                SERIAL_PORT.print("0_20mA_CH");
                SERIAL_PORT.print(channel2);
                SERIAL_PORT.print(": ");
#endif
                break;
            }

            // STEP 1: Get new Sensor value
            switch (knx.paramByte(getParCUR(CUR_CHSensorTypes2, channel2)))
            {
            case SensorType_current:
                value2.ladcValue = get4_20mA(channel2);
                break;
            case SensorType_percent:
                value2.ladcValue = get4_20mA(channel2);
                break;
            default:

                switch (knx.paramByte(getParCUR(CUR_CHSensorType2, channel2)))
                {
                case Input_4_20mA:
                    value2.ladcValue = calculateSensorValueLinearFunction2(channel2, value_4mA, knx.paramWord(getParCUR(CUR_CHPoint4mA, channel2)), knx.paramWord(getParCUR(CUR_CHPoint20mA, channel2)));
                    break;
                case Input_0_20mA:
                    value2.ladcValue = calculateSensorValueLinearFunction2(channel2, value_0mA, knx.paramWord(getParCUR(CUR_CHPoint4mA, channel2)), knx.paramWord(getParCUR(CUR_CHPoint20mA, channel2)));
                    break;
                default:
                    value2.ladcValue = 0;
#ifdef Input_4_20mA_Output
                    SERIAL_PORT.print("Wrong Par");
#endif
                    break;
                }
                break;
            }
#ifdef Input_4_20mA_Output
            SERIAL_PORT.print(value2.ladcValue);
            SERIAL_PORT.print(" | ");
            SERIAL_PORT.print(knx.paramWord(getParCUR(CUR_CHPoint4mA, channel2)));
            SERIAL_PORT.print(" | ");
            SERIAL_PORT.print(knx.paramWord(getParCUR(CUR_CHPoint20mA, channel2)));
            SERIAL_PORT.print(" | ");
#endif
            // STEP 2: Check value Change "Absolut"
            // STEP 2a: read Parameter DPT Format
            lAbsolute = (knx.paramWord(getParCUR(CUR_CHSendenAbsolut2, channel2)));

            // STEP 2b: Check if Change detected
            if (lAbsolute > 0 && (abs(value2.ladcValue - valueOld2.ladcValue[channel2])) >= lAbsolute)
            {
                lSend = true;
#ifdef Input_4_20mA_Output
                SERIAL_PORT.print(" Abs ");
#endif
            }
            // STEP 3: Check value Change "Releative"
            // STEP 3a: read Parameter DPT Format
            switch (knx.paramByte(getParCUR(CUR_CHSensorType2, channel2)))
            {
            case SensorType_humidity:
                lAbsolute = 100;
                break;

            case SensorType_percent:
                lAbsolute = 100;
                break;

            default:
                lAbsolute = (knx.paramByte(getParCUR(CUR_CHSendenRelativ2, channel2)));
                break;
            }
            // STEP 3b: Check if Change detected
            if (lAbsolute > 0 && value2.ladcValue > 0.2 && roundf(abs(value2.ladcValue - valueOld2.ladcValue[channel2])) >= value2.ladcValue / 100 * lAbsolute)
            {
                lSend = true;
#ifdef Input_4_20mA_Output
                SERIAL_PORT.print(" Rel ");
#endif
            }
            // STEP 4: Preset KO
            switch (knx.paramByte(getParCUR(CUR_CHSensorType2, channel2)))
            {
            case SensorType_percent:
#ifdef Input_4_20mA_Output
                SERIAL_PORT.println(value2.ladcValue);
#endif
                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                knx.getGroupObject(getComCUR(CUR_KoCUR_BASE__1, channel2)).valueNoSend(value2.ladcValue * 2.55, getDPT(VAL_DPT_5));
                break;

            default:
#ifdef Input_4_20mA_Output
                SERIAL_PORT.println(value2.ladcValue);
#endif
                // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                knx.getGroupObject(getComCUR(CUR_KoCUR_BASE__1, channel2)).valueNoSend(value2.ladcValue, getDPT(VAL_DPT_9));
                break;
            }

            processDelay2[channel2] = millis();
        } // ENDE DelayCheck

        if (lSend)
        {
#ifdef Input_4_20mA_Output
            SERIAL_PORT.print("KNX_4-20mA");
            SERIAL_PORT.print(channel2);
            SERIAL_PORT.print(": senden: ");
            SERIAL_PORT.println(value2.ladcValue);
#endif
            knx.getGroupObject(getComCUR(CUR_KoCUR_BASE__1, channel2)).objectWritten();
            valueOld2.ladcValue[channel2] = value2.ladcValue;
            sendDelay2[channel2] = millis();
            lSend = false;
        }
    } // if CH active

    channel2++;
    if (channel2 >= CUR_ChannelCount)
    {
        channel2 = 0;
    }
}