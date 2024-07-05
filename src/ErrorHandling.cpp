#include "KnxHelper.h"
#include "OpenKNX.h"
#include <Arduino.h>
#include <Wire.h>

#include "Device_setup.h"
#include "ErrorHandling.h"
#include "HelperFunc.h"
#include "I2C_IOExpander.h"
#include "LED_Statusanzeige.h"
#ifdef ADC_enable
    #include "ReadADC.h"
#endif

#define ERROR_24V_AC 0
#define ERROR_VCC_5V 1
#define ERROR_VCC_12V 2
#define ERROR_VCC_24V 3
#define ERROR_Relais_5V 4

#define DelayTime 1000
#define DelayTime_DiagKO 1000

bool startDelay = false;
bool restart_5V_Relais = false;

bool initADCFlag_TOP = false;
bool initADCFlag_BOT = false;

uint32_t delayTimer = 0;
uint32_t delayTimer_DiagKO = 0;
uint32_t timer1sek = 0;
uint32_t RestartTimer_5V_Relais = 0;

uint8_t error = 0;
uint8_t error_old = 0;

void restart_Relais_5V()
{
    if (delayCheck(RestartTimer_5V_Relais, 1000) && restart_5V_Relais)
    {
        digitalWrite(get_SSR_EN_PIN(), true);
        restart_5V_Relais = false;
        Serial.println("------> Restart");
    }
}

uint8_t processErrorHandling()
{
    if (delayCheck(timer1sek, 1000))
    {
        if (get_24V_AC_Error())
        {
            setLED_24VAC(true);
        }
        else
        {
            setLED_24VAC(false);
        }
        timer1sek = millis();
    }

    if (startDelay && delayCheck(delayTimer, DelayTime))
    {
        startDelay = false;
        error &= ~(1 << ERROR_24V_AC);
    }

    // Check 24V AC
    if (digitalRead(get_5V_status_PIN()))
    {
        error |= 1 << ERROR_24V_AC;
#ifdef ADC_enable
        clearInitFlags_ADC();
#endif
        clearInitFlags_IOExp();
        
        initADCFlag_TOP = false;
        initADCFlag_BOT = false;
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
        error |= 1 << ERROR_VCC_5V;
    }
    else
    {
        error &= ~(1 << ERROR_VCC_5V);
    }

    // Check ext Relais 5V
    if (!digitalRead(get_SSR_FAULT_PIN()))
    {
        error |= 1 << ERROR_Relais_5V;
       /* RestartTimer_5V_Relais = millis();
        digitalWrite(get_SSR_EN_PIN(), false);
        restart_5V_Relais = true;
        Serial.println("------> STOPP");
        */
    }
    else
    {
        error &= ~(1 << ERROR_Relais_5V);
    }

    if (error_old != error && delayCheck(delayTimer_DiagKO, DelayTime_DiagKO))
    {
        knx.getGroupObject(BEM_Ko_Diagnose_KO_PWR).value(error, getDPT(VAL_DPT_5));
        error_old = error;
        delayTimer_DiagKO = millis();
    }

    restart_Relais_5V();

    return error;
}

uint8_t getError()
{
    return error;
}

bool get_5V_Error()
{
    return (error >> ERROR_VCC_5V) & 1;
}

bool get_12V_Error()
{
    return (error >> ERROR_VCC_12V) & 1;
}

bool get_24V_Error()
{
    return (error >> ERROR_VCC_24V) & 1;
}

bool get_24V_AC_Error()
{
    return (error >> ERROR_24V_AC) & 1;
}

bool get_ADC_Ready_Flag_TOP()
{
    return initADCFlag_TOP;
}

bool get_ADC_Ready_Flag_BOT()
{
    return initADCFlag_BOT;
}

void set_ADC_Ready_Flag_TOP()
{
    initADCFlag_TOP = true;
}

void set_ADC_Ready_Flag_BOT()
{
    initADCFlag_BOT = true;
}