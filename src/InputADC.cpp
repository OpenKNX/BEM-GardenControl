#include <knx.h>
#include "InputADC.h"
#include "GardenControl.h"
#include "KnxHelper.h"
#include "BEM_hardware.h"
#include "HelperFunc.h"
#include "ReadADC.h"

uint32_t processDelay = 0;
uint8_t channel = 0;

union InputADCValuesOLD
{
    float ladcValue[ADC_ChannelCount];
    uint8_t lsoilmoisture[ADC_ChannelCount];
} valueOld;

void processInputADC()
{
    bool lSend = false;
    bool lAbsoluteBool = false;

    union InputADCValues
    {
        float ladcValue;
        uint8_t lsoilmoisture;
    } value;

    if (delayCheck(processDelay, 1000))
    {
        switch (knx.paramByte(getParADC(ADC_CHSensorType, channel)))
        {
        case ADC_Wert:
            SERIAL_PORT.print("ADC");
            SERIAL_PORT.print(channel);
            SERIAL_PORT.println(": ADC Wert");
            value.ladcValue = getAdcVoltage(channel, (knx.paramByte(getParADC(ADC_CHVoltageDiv, channel))));

            // senden bei Wertänderung
            lAbsoluteBool = knx.paramInt(getParADC(ADC_CHSendenAbsolut, channel)); // Value in mV
            if (lAbsoluteBool > 0 && value.ladcValue != valueOld.ladcValue[channel])
                lSend = true;

            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).valueNoSend(value.ladcValue, getDPT(VAL_DPT_9));

            break;

        case SMT50_Bodenfeuchte:
            SERIAL_PORT.print("ADC");
            SERIAL_PORT.print(channel);
            SERIAL_PORT.println(": SMT50-BodenFeuchte");
            break;

        case SMT50_BodenTemperatur:
            SERIAL_PORT.print("ADC");
            SERIAL_PORT.print(channel);
            SERIAL_PORT.println(": SMT50-BodenTemp");
            break;

        default:
            SERIAL_PORT.println("Wrong ADC SensorTyp");
            break;
        }
    }

    if (lSend)
    {
        knx.getGroupObject(getComADC(ADC_KoGO_BASE__1, channel)).objectWritten();
        valueOld.ladcValue[channel] = value.ladcValue;
        // sendDelay[channel] = millis();
        lSend = false;
    }

    channel++;
    if (channel > ADC_ChannelCount)
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
            if (knx.paramByte(getParADC(ADC_CHVoltageDiv, channel)))
                SERIAL_PORT.println("ADC-Wert: (0-12V)");
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
