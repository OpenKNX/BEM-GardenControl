#include <Arduino.h>

#include "SystemFailureHandling.h"
#include "Device_setup.h"
#include "HelperFunc.h"
#include "OpenKNX.h"
#include "handleVentilRelais.h"
#include <Wire.h>

bool flag_RebootExternalPWR = false;

uint32_t waitDelay;

enum State_Maschine
{
    PosIDLE = 1,
    PosDisablePWR = 2,
    PosWait = 3,
    PosEnablePWR = 4,
    PosWait2 = 5,
};

State_Maschine State_M = PosIDLE;

/*
Typische Fehler im Bewässerungssystem:

- Alle Ventile sind zu:
    - aber der Druck auf der Zuleitung ist kleiner als erwartet --> Leck
    - aber der Durchflussmengenzähler misst noch einen Durchfluß --> Leck



*/



void processSysFailure()
{
    switch (State_M)
    {
        case PosIDLE:
            if (flag_RebootExternalPWR)
            {
                State_M = PosDisablePWR;
                flag_RebootExternalPWR = false;
            }
            break;

        case PosDisablePWR:
            SERIAL_PORT.println("disable Main Relay & wait");
            control_5V_Relais(false);
            digitalWrite(get_5V_EN_PIN(), LOW);
            waitDelay = millis();
            State_M = PosWait;
            break;

        case PosWait:
            if (delayCheck(waitDelay, 2000))
            {
                SERIAL_PORT.println("enable Main Relay & wait");
                control_5V_Relais(true);
                waitDelay = millis();
                State_M = PosEnablePWR;
            }
            break;

        case PosEnablePWR:
            if (delayCheck(waitDelay, 1000))
            {
                SERIAL_PORT.println("enable +5V & Wait");
                digitalWrite(get_5V_EN_PIN(), HIGH);
                waitDelay = millis();
                State_M = PosWait2;
            }
            break;

        case PosWait2:
            if (delayCheck(waitDelay, 1000))
            {
                SERIAL_PORT.println("Done");
                State_M = PosIDLE;
            }
            break;

        default:
            break;
    }
}

void rebootExternalPWR()
{
    flag_RebootExternalPWR = true;
}