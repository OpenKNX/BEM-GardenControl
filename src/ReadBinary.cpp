#include <Arduino.h>
#include <Wire.h>
#include "HelperFunc.h"
//#include <knx.h>

#include "GardenControl.h"
#include "KnxHelper.h"
#include "ReadBinary.h"

#define BIN_Input 1

bool flagBINinput[] = {false,false,false,false};

bool inputState[BIN_ChannelCount] = {0};

uint8_t interruptBinInputId = 0;

uint8_t optoIN_BinInput[] = {OptoIN_1,OptoIN_2,OptoIN_3,OptoIN_4};

void interrupt_BIN()
{
    flagBINinput[interruptBinInputId] = true;
}

void readBinInput(uint8_t id)
{
    inputState[id] = digitalRead(optoIN_BinInput[id]);
}

void InitBinInput(uint8_t id)
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, id)) == BIN_Input)
    {
        pinMode(optoIN_BinInput[id], INPUT_PULLUP);
        interruptBinInputId = id;
        attachInterrupt(digitalPinToInterrupt(optoIN_BinInput[id]), interrupt_BIN, CHANGE);
        readBinInput(id);
#ifdef Input_BIN_Output
        SERIAL_PORT.println("  init BIN_0: TRUE");
#endif
    }
}

void processReadInputs()
{
    for(uint8_t i=0;i<=sizeof(flagBINinput);i++)
    {
        if (flagBINinput[i] == true)
        {
            flagBINinput[i] = false;
            readBinInput(i);
        }
    }
}

bool getStateInput(uint8_t ch)
{
    return inputState[ch];
}
