#include <Arduino.h>
#include <Wire.h>
//#include <knx.h>
#include "BEM_hardware.h"
#include "ErrorHandling.h"
#include "Sensor_Value_Input.h"
#include "I2C_IOExpander.h"
#include "GardenControlDevice.h"
#include "Device_setup.h"

#define Threshold_24V_min 22
#define Threshold_24V_max 26
#define Threshold_12V_min 11
#define Threshold_12V_max 12

#define ERROR_24V 0
#define ERROR_12V 1
#define ERROR_5V 2
#define ERROR_5V_output 3

#define DelayTime 1000

bool startDelay = false;
uint32_t delayTimer = 0;

uint8_t error = 0;

uint8_t processErrorHandling()
{
#ifdef ADC_enable
    // Check 24V
    float value = (float)getAdcVoltage_24V();
    if (value > Threshold_24V_max || value < Threshold_24V_min)
    {
        error |= 1 << ERROR_24V;
    }
    else
    {
        error &= ~(1 << ERROR_24V);
    }

    // Check 12V
    value = (float)getAdcVoltage_12V();
    if (value > Threshold_12V_max || value < Threshold_12V_min)
    {
        error |= 1 << ERROR_12V;
    }
    else
    {
        error &= ~(1 << ERROR_12V);
    }
#endif
    // Check 5V
    if (digitalRead(get_5V_status_PIN()))
    {
        error |= 1 << ERROR_5V;
        clearInitFlags_ADC();
        clearInitFlags_IOExp();
    }
    else
    {
        if (!startDelay)
        {
            startDelay = true;
            delayTimer = millis();
        }
        // error &= ~(1 << ERROR_5V);
    }

    // read +5V Sensor fault
    if (!get_IOExpander_Input(get_5V_fault_PIN()))
    {
        error |= 1 << ERROR_5V_output;
    }
    else
    {
        error &= ~(1 << ERROR_5V_output);
    }

    if (startDelay && delayCheck(delayTimer, DelayTime))
    {
        startDelay = false;
        error &= ~(1 << ERROR_5V);
    }

    return error;
}

uint8_t getError()
{
    return error;
}

bool get_5V_Error()
{
    return (error >> ERROR_5V) & 1;
}

bool get_12V_Error()
{
    return (error >> ERROR_12V) & 1;
}

bool get_24V_Error()
{
    return (error >> ERROR_24V) & 1;
}

bool get_5V_out_Error()
{
    return (error >> ERROR_5V_output) & 1;
}