#include <knx.h>
#include "InputADC.h"
#include "GardenControl.h"
#include "KnxHelper.h"
#include "BEM_hardware.h"
#include "HelperFunc.h"
#include "ReadADC.h"

uint32_t processDelay[ADC_ChannelCount] = {0};
uint32_t sendDelay[ADC_ChannelCount] = {0};

uint8_t channel = 0;

union InputADCValuesOLD
{
    float ladcValue[ADC_ChannelCount];
    // uint16_t ladcValueU16[ADC_ChannelCount];
    // uint8_t lsoilmoistureU8[ADC_ChannelCount];
} valueOld;

void processInputADC()
{
    bool lSend = false;
    uint16_t lAbsoluteU16;
    uint8_t lRelativU8;

    uint16_t lCycle;

    union InputADCValues
    {
        float ladcValue;
        // uint16_t ladcValueU16;
        // uint8_t lsoilmoistureU8;
    } value;

    lCycle = knx.paramWord(getParADC(ADC_CHSendcycletime, channel)) * 1000;

    // we waited enough, let's send the value
    if (lCycle && delayCheck(sendDelay[channel], lCycle))
    {
        lSend = true;
    }

    if (delayCheck(processDelay[channel], 500))
    {
        switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
        {
        case ADC_Wert:
            SERIAL_PORT.print("ADC");
            SERIAL_PORT.print(channel);
            SERIAL_PORT.print(": ");
            value.ladcValue = (getAdcVoltage(channel, (knx.paramByte(getParADC(ADC_CHVoltageDiv, channel))))) * 1000;

            // senden bei Wertänderung Absolut
            lAbsoluteU16 = knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel)); // Value in mV
            if (lAbsoluteU16 > 0 && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= lAbsoluteU16)
            {
                lSend = true;
                SERIAL_PORT.print(" Abs ");
            }
            // senden bei Wertänderung Relativ
            lRelativU8 = knx.paramByte(getParADC(ADC_CHSendenRelativ, channel)); // Value in mV
            if (lRelativU8 > 0 && value.ladcValue > 0.2 && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= value.ladcValue / 100 * lRelativU8)
            {
                lSend = true;
                SERIAL_PORT.print(" Rel ");
            }

            SERIAL_PORT.println(value.ladcValue);
            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue, getDPT(VAL_DPT_9));

            break;

        case SMT50_Bodenfeuchte:
            SERIAL_PORT.print("ADC");
            SERIAL_PORT.print(channel);

            value.ladcValue = getSensorValue(channel); // 0 - 100%

            // senden bei Wertänderung Absolut
            lAbsoluteU16 = knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel)); // Value in %
            if (lAbsoluteU16 > 0 && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= lAbsoluteU16)
            {
                lSend = true;
                SERIAL_PORT.print(" Abs ");
            }

            SERIAL_PORT.println(value.ladcValue);
            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend((uint8_t)(value.ladcValue * 2.55), getDPT(VAL_DPT_5));

            break;

        case SMT50_BodenTemperatur:
            SERIAL_PORT.print("ADC");
            SERIAL_PORT.print(channel);

            //
            value.ladcValue = getSensorValue(channel) + 50;
            valueOld.ladcValue[channel] = valueOld.ladcValue[channel] + 50;

            // senden bei Wertänderung Absolut
            lAbsoluteU16 = knx.paramWord(getParADC(ADC_CHSendenAbsolut, channel)) / 10.0; // Value in 0.1°C
            if (lAbsoluteU16 > 0 && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= lAbsoluteU16)
            {
                lSend = true;
                SERIAL_PORT.print(" Abs ");
            }
            // senden bei Wertänderung Relativ
            lRelativU8 = knx.paramByte(getParADC(ADC_CHSendenRelativ, channel));
            if (lRelativU8 > 0 && (value.ladcValue > 0.5 || value.ladcValue < -0.5) && roundf(abs(value.ladcValue - valueOld.ladcValue[channel])) >= value.ladcValue / 100 * lRelativU8)
            {
                lSend = true;
                SERIAL_PORT.print(" Rel ");
            }
        
            value.ladcValue = value.ladcValue - 50;
            valueOld.ladcValue[channel] = valueOld.ladcValue[channel] - 50;
            SERIAL_PORT.println(value.ladcValue);

            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue, getDPT(VAL_DPT_9));
            break;

        default:
            SERIAL_PORT.println("Wrong ADC SensorTyp");
            break;
        }

        processDelay[channel] = millis();
    }

    if (lSend)
    {
        SERIAL_PORT.print("KNX_ADC");
        SERIAL_PORT.print(channel);
        SERIAL_PORT.print(": senden: ");
        SERIAL_PORT.println(value.ladcValue);
        knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).objectWritten();
        valueOld.ladcValue[channel] = value.ladcValue;
        sendDelay[channel] = millis();
        lSend = false;
    }

    channel++;
    if (channel >= ADC_ChannelCount)
    {
        channel = 0;
    }
}

void initInputADC()
{
    for (int channel = 0; channel < ADC_ChannelCount; channel++)
    {
        SERIAL_PORT.print("init ADC");
        SERIAL_PORT.print(channel + 1);
        SERIAL_PORT.print(": ");
        switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
        {
        case ADC_Wert:
            set_ADC_CorrFactor(channel, 1); // inital all Factors to 1
            if (knx.paramByte(getParADC(ADC_CHVoltageDiv, channel)))
            {
                set_ADC_CorrFactor(channel, knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
                SERIAL_PORT.println("ADC-Wert: (0-12V)");
                SERIAL_PORT.print("Kor: ");
                SERIAL_PORT.println(knx.paramWord(getParADC(ADC_CHVoltageCorrection, channel)) / 100.0);
            }
            else
                SERIAL_PORT.println("ADC-Wert: (0-5V)");
            set_ADC_DIV(channel, knx.paramByte(getParADC(ADC_CHVoltageDiv, channel)));
            break;
        case SMT50_Bodenfeuchte:
            SERIAL_PORT.println("SMT50-BF");
            set_ADC_DIV(channel, DIV_5V);
            break;
        case SMT50_BodenTemperatur:
            SERIAL_PORT.println("SMT50-BT");
            set_ADC_DIV(channel, DIV_5V);
            break;

        default:
            break;
        }
    }
}

float getSensorValue(uint8_t channel)
{
    float value;

    switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
    {
    case ADC_Wert:
        SERIAL_PORT.print(" ADC-WERT: ");
        value = getAdcVoltage(channel, (knx.paramByte(getParADC(ADC_CHVoltageDiv, channel))));
        break;

    case SMT50_Bodenfeuchte:
        SERIAL_PORT.print(" SMT50-BF: ");
        value = getAdcVoltage(channel, DIV_5V);
        value = value / 3.0 * 50.0;
        break;

    case SMT50_BodenTemperatur:
        SERIAL_PORT.print(" SMT50-BT: ");
        value = getAdcVoltage(channel, DIV_5V);
        value = (value - 0.5) / 0.01;
        break;

    default:
        break;
    }
    return value;
}
