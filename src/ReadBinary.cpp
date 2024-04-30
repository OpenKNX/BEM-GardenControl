#include <Arduino.h>

#include "HelperFunc.h"
#include "KnxHelper.h"
#include "OpenKNX.h"
#include "ReadBinary.h"
#include <Wire.h>

#define BIN_Input 1

#ifdef BinInputs

bool flagBINinput1 = false;
bool flagBINinput2 = false;
bool flagBINinput3 = false;
bool flagBINinput4 = false;

bool inputState[BIN_ChannelCount] = {0};

void interrupt_BIN_1()
{
    flagBINinput1 = true;
}

void interrupt_BIN_2()
{
    flagBINinput2 = true;
}

void interrupt_BIN_3()
{
    flagBINinput3 = true;
}

void interrupt_BIN_4()
{
    flagBINinput4 = true;
}

void readBinInput1()
{
    inputState[0] = digitalRead(OptoIN_1);
}

void readBinInput2()
{
    inputState[1] = digitalRead(OptoIN_2);
}

void readBinInput3()
{
    inputState[2] = digitalRead(OptoIN_3);
}

void readBinInput4()
{
    inputState[3] = digitalRead(OptoIN_4);
}

void InitBinInput1(uint8_t GPIO)
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 0)) == BIN_Input)
    {
        pinMode(GPIO, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_1, CHANGE);
        readBinInput1();
#ifdef Input_BIN_Output
        SERIAL_PORT.println("  init BIN_0: TRUE");
#endif
    }
}

void InitBinInput2(uint8_t GPIO)
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 1)) == BIN_Input)
    {
        pinMode(GPIO, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_2, CHANGE);
        readBinInput2();
#ifdef Input_BIN_Output
        SERIAL_PORT.println("  init BIN_1: TRUE");
#endif
    }
}

void InitBinInput3(uint8_t GPIO)
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 2)) == BIN_Input)
    {
        pinMode(GPIO, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_3, CHANGE);
        readBinInput3();
#ifdef Input_BIN_Output
        SERIAL_PORT.println("   init BIN_2: TRUE");
#endif
    }
}

void InitBinInput4(uint8_t GPIO)
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 3)) == BIN_Input)
    {
        pinMode(GPIO, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_4, CHANGE);
        readBinInput4();
#ifdef Input_BIN_Output
        SERIAL_PORT.println("  init BIN_3: TRUE");
#endif
    }
}

void processReadInputs()
{
    if (flagBINinput1 == true)
    {
        flagBINinput1 = false;
        readBinInput1();
    }
    else if (flagBINinput2 == true)
    {
        flagBINinput2 = false;
        readBinInput2();
    }
    else if (flagBINinput3 == true)
    {
        flagBINinput3 = false;
        readBinInput3();
    }
    else if (flagBINinput4 == true)
    {
        flagBINinput4 = false;
        readBinInput4();
    }
}

bool getStateInput(uint8_t ch)
{
    return inputState[ch];
}

#endif
