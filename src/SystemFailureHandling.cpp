#include <Arduino.h>
#include <Wire.h>
//#include <knx.h>
#include "BEM_hardware.h"
//#include "KnxHelper.h"
#include "SystemFailureHandling.h"


/*
Typische Fehler im Bewässerungssystem:

- Alle Ventile sind zu: 
    - aber der Druck auf der Zuleitung ist kleiner als erwartet --> Leck
    - aber der Durchflussmengenzähler misst noch einen Durchfluß --> Leck



*/

void processSysFailure()
{


}