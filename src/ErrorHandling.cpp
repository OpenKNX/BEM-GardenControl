#include <Arduino.h>
#include <Wire.h>
#include <knx.h>
#include "KnxHelper.h"
#include "GardenControl.h"

#include "HelperFunc.h"
#include "ErrorHandling.h"
#include "ReadADC.h"
#include "I2C_IOExpander.h"
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
#define DelayTime_DiagKO 1000

bool startDelay = false;
uint32_t delayTimer = 0;
uint32_t delayTimer_DiagKO = 0;

uint8_t error = 0;
uint8_t error_old = 0;

uint8_t processErrorHandling()
{
    if (startDelay && delayCheck(delayTimer, DelayTime))
    {
        startDelay = false;
        error &= ~(1 << ERROR_5V);
    }

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
    switch (get_HW_ID())
    {
    case HW_1_0:
        value = (float)getAdcVoltage_12V();
        if (value > Threshold_12V_max || value < Threshold_12V_min)
        {
            error |= 1 << ERROR_12V;
        }
        else
        {
            error &= ~(1 << ERROR_12V);
        }
        break;
    case HW_2_0:
        error &= ~(1 << ERROR_12V);
        break;
    default:
        SERIAL_PORT.println("ErrorHandling 12V HW ID not defined");
        break;
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

    if (error_old != error && delayCheck(delayTimer_DiagKO, DelayTime_DiagKO))
    {
        knx.getGroupObject(BEM_Ko_Diagnose_KO_PWR).value(error, getDPT(VAL_DPT_5));
        error_old = error;
        delayTimer_DiagKO = millis();
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