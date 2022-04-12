#include "Input_BIN.h"
#include <knx.h>
#include "KnxHelper.h"
#include "GardenControl.h"
#include "HelperFunc.h"
#include "BEM_hardware.h"
#include "ReadBinary.h"

#define BIN_Input 1

#define BIN_Edge_fall_and_rise 0
#define BIN_Edge_fall 1
#define BIN_Edge_rise 2

#define BIN_ValueOPEN_FALSE 0
#define BIN_ValueOPEN_TRUE 1

uint8_t channel_BIN = 0;

uint32_t sendDelay_BIN[BIN_ChannelCount] = {0};

bool ladcValueOLD[BIN_ChannelCount] = {0};
bool linitFlag[BIN_ChannelCount] = {0};

void processInput_BIN()
{
    bool lSend = false;
    uint16_t lCycle;

    bool ladcValue;

    // Check if Channel = BIN-Input
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, channel_BIN)) == BIN_Input)
    {

        lCycle = knx.paramWord(getParBIN(BIN_CHSendcycletime3, channel_BIN)) * 1000;

        // we waited enough, let's send the value
        if (lCycle && delayCheck(sendDelay_BIN[channel_BIN], lCycle))
        {
            lSend = true;
        }

        // STEP 1: Get new Sensor value
        // NOTE HW-Implementation: TRUE = BIN-Input OPEN
        switch ((knx.paramByte(getParBIN(BIN_CHValueOpen3, channel_BIN)) >> BIN_CHValueOpen3Shift) & 1)
        {
        case BIN_ValueOPEN_TRUE:
            ladcValue = getStateInput(channel_BIN);
            break;

        case BIN_ValueOPEN_FALSE:
            ladcValue = !getStateInput(channel_BIN);
            break;

        default:
#ifdef Input_BIN_Output
            SERIAL_PORT.println("Wrong Par: ValueOpen");
#endif
            return;
            break;
        }
        // STEP 2: check if send Buswiederkehr
        if (linitFlag[channel_BIN] == false)
        {
            if ((knx.paramByte(getParBIN(BIN_CHSendenStart3, channel_BIN)) >> BIN_CHSendenStart3Shift) & 1)
            {
                knx.getGroupObject(getComBIN(BIN_Ko_Status_BinarInput, channel_BIN)).valueNoSend((ladcValue), getDPT(VAL_DPT_1));
                lSend = true;
            }
            else
            {
                ladcValueOLD[channel_BIN] = ladcValue;
            }

            linitFlag[channel_BIN] = true;
        }

        // STEP 3: check if change
        if (ladcValue != ladcValueOLD[channel_BIN])
        {
#ifdef Input_BIN_Output
            SERIAL_PORT.print("BIN_CH");
            SERIAL_PORT.print(channel_BIN);
            SERIAL_PORT.print(": ");
#endif
            // Par: send falling & rising
            if (knx.paramByte(getParBIN(BIN_CHSendFlanken3, channel_BIN)) == BIN_Edge_fall_and_rise)
            {
                lSend = true;
                knx.getGroupObject(getComBIN(BIN_Ko_Status_BinarInput, channel_BIN)).valueNoSend((ladcValue), getDPT(VAL_DPT_1));
#ifdef Input_BIN_Output
                SERIAL_PORT.print(ladcValue);
                SERIAL_PORT.println(" (R&F)");

#endif
            }
            // check falling edge
            else if (ladcValueOLD[channel_BIN] < ladcValue && knx.paramByte(getParBIN(BIN_CHSendFlanken3, channel_BIN)) == BIN_Edge_fall)
            {
                lSend = true;
                knx.getGroupObject(getComBIN(BIN_Ko_Status_BinarInput, channel_BIN)).valueNoSend((ladcValue), getDPT(VAL_DPT_1));
#ifdef Input_BIN_Output
                SERIAL_PORT.print(ladcValue);
                SERIAL_PORT.println(" (F)");

#endif
            }
            // check rising edge
            else if (ladcValue < ladcValueOLD[channel_BIN] && knx.paramByte(getParBIN(BIN_CHSendFlanken3, channel_BIN)) == BIN_Edge_rise)
            {
                lSend = true;
                knx.getGroupObject(getComBIN(BIN_Ko_Status_BinarInput, channel_BIN)).valueNoSend((ladcValue), getDPT(VAL_DPT_1));
#ifdef Input_BIN_Output
                SERIAL_PORT.print(ladcValue);
                SERIAL_PORT.println(" (R)");

#endif
            }
            else
            {
#ifdef Input_BIN_Output
                SERIAL_PORT.print(ladcValue);
                SERIAL_PORT.println(" (NO EDGE sending)");

#endif
            }
        }

        if (lSend)
        {
#ifdef Input_BIN_Output
            SERIAL_PORT.print("KNX_BIN");
            SERIAL_PORT.print(channel_BIN);
            SERIAL_PORT.print(": senden: ");
            SERIAL_PORT.println(ladcValue);
#endif
            knx.getGroupObject(getComBIN(BIN_Ko_Status_BinarInput, channel_BIN)).objectWritten();
            ladcValueOLD[channel_BIN] = ladcValue;
            sendDelay_BIN[channel_BIN] = millis();
            lSend = false;
        }

    } // ENDE IF

    channel_BIN++;
    if (channel_BIN >= BIN_ChannelCount)
    {
        channel_BIN = 0;
    }
}