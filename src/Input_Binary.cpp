#include <Arduino.h>
#include <Wire.h>
//#include <knx.h>
#include "hardware.h"
//#include "KnxHelper.h"
#include "Input_Binary.h"

bool flagBINinput1 = false;
bool flagBINinput2 = false;
bool flagBINinput3 = false;
bool flagBINinput4 = false;

bool inputState[4] = {0};

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
   pinMode(GPIO, INPUT_PULLUP); 
   attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_1, CHANGE);
   readBinInput1();
}

void InitBinInput2(uint8_t GPIO)
{
   pinMode(GPIO, INPUT_PULLUP); 
   attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_2, CHANGE);
   readBinInput2();
}

void InitBinInput3(uint8_t GPIO)
{
   pinMode(GPIO, INPUT_PULLUP); 
   attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_3, CHANGE);
   readBinInput3();
}

void InitBinInput4(uint8_t GPIO)
{
   pinMode(GPIO, INPUT_PULLUP); 
   attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_BIN_4, CHANGE);
   readBinInput4();
}


void processBinInputs()
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

bool getStateInput1()
{
    return inputState[0];
}

bool getStateInput2()
{
    return inputState[1];
}

bool getStateInput3()
{
    return inputState[2];
}

bool getStateInput4()
{
    return inputState[3];
}



