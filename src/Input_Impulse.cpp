#include <Arduino.h>
#include <Wire.h>
//#include <knx.h>
#include "hardware.h"
//#include "KnxHelper.h"
#include "Input_Impulse.h"
#include "GardenControlDevice.h"

uint32_t counter = 0;
uint32_t readTimer = 0;
float  waterflow = 0;

float valueQ = 6.6;

void interrupt_Impluse()
{
    counter++;
}

void InitImpulseInput(uint8_t GPIO)
{
    pinMode(GPIO, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(GPIO), interrupt_Impluse, CHANGE);
}

void processImpulseInput()
{
    if (delayCheck(readTimer, 1000))
    {
       readTimer = millis();
       waterflow = counter / valueQ;
       counter = 0;
    }
}

float getFlowValue()
{
    return waterflow;
}