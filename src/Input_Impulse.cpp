#include <Arduino.h>
#include <Wire.h>
//#include <knx.h>
#include "KnxHelper.h"
#include "GardenControl.h"
#include "BEM_hardware.h"
#include "HelperFunc.h"
#include "Input_Impulse.h"

#define BIN_Input_Impuls 3

uint32_t counter[BIN_ChannelCount] = {0};
uint32_t counterOLD[BIN_ChannelCount] = {0};
uint32_t readTimer = 0;
uint32_t sendDelay_Impl[BIN_ChannelCount] = {0};
uint32_t processDelay_Impl[BIN_ChannelCount] = {0};

float lValueOLD[BIN_ChannelCount] = {0};
float waterflow[BIN_ChannelCount] = {0};

uint8_t channel_Impl = 0;
float valueQ = 6.6;

void interrupt_Impluse1()
{
    counter[0]++;
}

void interrupt_Impluse2()
{
    counter[1]++;
}

void interrupt_Impluse3()
{
    counter[2]++;
}

void interrupt_Impluse4()
{
    counter[3]++;
}

// Inputs Optocoupler
#define OptoIN_1 12
#define OptoIN_2 11
#define OptoIN_3 10
#define OptoIN_4 9


void InitImpulseInputs()
{
    for (int ch = 0; ch < BIN_ChannelCount; ch++)
    {
        if (knx.paramByte(getParBIN(BIN_CHInputTypes3, ch)) == BIN_Input_Impuls)
        {
#ifdef InputImpuls_Output
            SERIAL_PORT.print("  INIT: Impl_");
            SERIAL_PORT.print(ch);
#endif
            switch (ch)
            {
            case 0:
                pinMode(OptoIN_1, INPUT_PULLUP);
                attachInterrupt(digitalPinToInterrupt(OptoIN_1), interrupt_Impluse1, CHANGE);
#ifdef InputImpuls_Output
                SERIAL_PORT.println(": TRUE");
#endif
                break;
            case 1:
                pinMode(OptoIN_2, INPUT_PULLUP);
                attachInterrupt(digitalPinToInterrupt(OptoIN_2), interrupt_Impluse2, CHANGE);
#ifdef InputImpuls_Output
                SERIAL_PORT.println(": TRUE");
#endif
                break;
            case 2:
                pinMode(OptoIN_3, INPUT_PULLUP);
                attachInterrupt(digitalPinToInterrupt(OptoIN_3), interrupt_Impluse3, CHANGE);
#ifdef InputImpuls_Output
                SERIAL_PORT.println(": TRUE");
#endif
                break;
            case 3:
                pinMode(OptoIN_4, INPUT_PULLUP);
                attachInterrupt(digitalPinToInterrupt(OptoIN_4), interrupt_Impluse4, CHANGE);
#ifdef InputImpuls_Output
                SERIAL_PORT.println(": TRUE");
#endif
                break;

            default:
            #ifdef InputImpuls_Output
                SERIAL_PORT.println(": WRONG PAR");
#endif
                break;
            }
        }
    }
}

void processReadImpulseInput()
{
    if (delayCheck(readTimer, 1000))
    {

        for (int ch = 0; ch < BIN_ChannelCount; ch++)
        {
            if (knx.paramByte(getParBIN(BIN_CHInputTypes3, ch)) == BIN_Input_Impuls)
            {
                waterflow[ch] = counter[ch] / ((knx.paramWord(getParBIN(BIN_CHFrequenceEqu3, ch)) * 0.1));
                counterOLD[ch] = counter[ch];
                counter[ch] = 0;
#ifdef InputImpuls_Output
                SERIAL_PORT.print("->Impluse_");
                SERIAL_PORT.print(ch);
                SERIAL_PORT.print(": ");
                SERIAL_PORT.println(waterflow[ch]);
#endif
            }
        }

        readTimer = millis();
    }
}

void processInputImpulse()
{
    bool lSend = false;
    float lValue;
    float lAbsolute;
    uint16_t lCycle;

    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, channel_Impl)) == BIN_Input_Impuls)
    {

        lCycle = knx.paramWord(getParBIN(BIN_CHSendcycletime3, channel_Impl)) * 1000;

        // we waited enough, let's send the value
        if (lCycle && delayCheck(sendDelay_Impl[channel_Impl], lCycle))
        {
            lSend = true;
        }

        if (delayCheck(processDelay_Impl[channel_Impl], 500))
        {
#ifdef InputImpuls_Output2
            SERIAL_PORT.print("Impl_");
            SERIAL_PORT.print(channel_Impl);
            SERIAL_PORT.print(": ");
            SERIAL_PORT.print(counterOLD[channel_Impl]);
            SERIAL_PORT.print(" | ");

            
#endif
            // STEP 1: Get new Sensor value
            lValue = getFlowValue(channel_Impl);

            // STEP 2a: Get Abs value
            lAbsolute = (knx.paramWord(getParBIN(BIN_CHSendenAbsolut3, channel_Impl)));
            // STEP 2b: Check if Change detected
            if (lAbsolute > 0 && (abs(lValue - lValueOLD[channel_Impl])) >= lAbsolute)
            {
                lSend = true;
#ifdef InputImpuls_Output2
                SERIAL_PORT.print(" Abs ");
#endif
            }
            // STEP 4: Check value Change "Releative"
            lAbsolute = (knx.paramByte(getParBIN(BIN_CHSendenRelativ3, channel_Impl)));
            // STEP 4a: Check if Change detected
            if (lAbsolute > 0 && lValue > 0.2 && (abs(lValue - lValueOLD[channel_Impl])) >= lValue / 100 * lAbsolute)
            {
                lSend = true;
#ifdef InputImpuls_Output2
                SERIAL_PORT.print(" Rel ");
#endif
            }

#ifdef InputImpuls_Output2
            SERIAL_PORT.println(lValue);
#endif
            // STEP 5: Preset KO
            // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
            knx.getGroupObject(getComBIN(BIN_KoBIN_BASE__1, channel_Impl)).valueNoSend(lValue*60.0, getDPT(VAL_DPT_9)); // from l/min to l/h 

            processDelay_Impl[channel_Impl] = millis();
        }

        if (lSend)
        {
#ifdef InputImpuls_Output2
            SERIAL_PORT.print("KNX_Impl");
            SERIAL_PORT.print(channel_Impl);
            SERIAL_PORT.print(": senden: ");
            SERIAL_PORT.println(lValue);
#endif
            knx.getGroupObject(getComBIN(BIN_KoBIN_BASE__1, channel_Impl)).objectWritten();
            lValueOLD[channel_Impl] = lValue;
            sendDelay_Impl[channel_Impl] = millis();
            lSend = false;
        }

    } // ENDE IF is Impulseingang

    channel_Impl++;
    if (channel_Impl >= BIN_ChannelCount)
    {
        channel_Impl = 0;
    }
}

float getFlowValue(uint8_t ch)
{
    if (ch > BIN_ChannelCount)
        return 0;
    else
        return waterflow[ch];
}