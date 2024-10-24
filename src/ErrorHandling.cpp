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

// ERROR Byte
#define ERROR_24V_AC 0
#define ERROR_VCC_5V 1
#define ERROR_VCC_12V 2
#define ERROR_VCC_24V 3
#define ERROR_Relais_5V 4
#define ERROR_24V_4_20mA_CH1 5
#define ERROR_24V_4_20mA_CH2 6
#define ERROR_VCC12_or_VCC24 7

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

    // read +5V Output  fault
    if (!get_IOExpander_TOP_Input(get_5V_Output_fault_PIN()))
    {
        error |= 1 << ERROR_VCC_5V;
    }
    else
    {
        error &= ~(1 << ERROR_VCC_5V);
    }

    // read +12V Output  fault
    if (get_12V_Output_fault_PIN() != 255 && get_IOExpander_TOP_Input(get_12V_Output_fault_PIN()))
    {
        error |= 1 << ERROR_VCC_12V;
    }
    else
    {
        error &= ~(1 << ERROR_VCC_12V);
    }

    // read +24V Output  fault
    if (get_24V_Output_fault_PIN() != 255 && get_IOExpander_TOP_Input(get_24V_Output_fault_PIN()))
    {
        error |= 1 << ERROR_VCC_24V;
    }
    else
    {
        error &= ~(1 << ERROR_VCC_24V);
    }

    // read +24V 4-20mA CH1
    if (check_24V_4_20mA_CH1())
    {
        error |= 1 << ERROR_24V_4_20mA_CH1;
    }
    else
    {
        error &= ~(1 << ERROR_24V_4_20mA_CH1);
    }

    // read +24V 4-20mA CH2
    if (check_24V_4_20mA_CH2())
    {
        error |= 1 << ERROR_24V_4_20mA_CH2;
    }
    else
    {
        error &= ~(1 << ERROR_24V_4_20mA_CH2);
    }

    // read ERROR_VCC12_or_VCC24 (only for special HW)
    if (get_12_or_24V_Output_fault_PIN() != 255 && !get_IOExpander_TOP_Input(get_12_or_24V_Output_fault_PIN()))
    {
        error |= 1 << ERROR_VCC12_or_VCC24;
    }
    else
    {
        error &= ~(1 << ERROR_VCC12_or_VCC24);
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

bool get_12V_or_24V_Error()
{
    return (error >> ERROR_VCC12_or_VCC24) & 1;
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

bool check_24V_4_20mA_CH1()
{
    switch (get_HW_ID_BOT())
    {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
        case HW_BOT_2_1:
            return false;
            break;
        case HW_BOT_5_0:
            if (getAdcVoltage_BOT(2) > 2.4)
            {
                return true;
            }
            else
            {
                return false;
            }
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID  check_24V_4_20mA()");
            return false;
            break;
    }
}

bool check_24V_4_20mA_CH2()
{
    switch (get_HW_ID_BOT())
    {
        case HW_BOT_1_0:
        case HW_BOT_2_0:
        case HW_BOT_2_1:
            return false;
            break;
        case HW_BOT_5_0:
            if (getAdcVoltage_BOT(3) > 2.4)
            {
                return true;
            }
            else
            {
                return false;
            }
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID  check_24V_4_20mA()");
            return false;
            break;
    }
}