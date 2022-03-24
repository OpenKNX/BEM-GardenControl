#include <Arduino.h>
#include "Device_setup.h"
#include "BEM_hardware.h"
#include "I2C_IOExpander.h"
#include "GardenControlDevice.h"


uint32_t LED_Delay = 0;

bool TestLEDstate = false;

void appSetup();
void appLoop();

void setup()
{
  Serial.begin(115200);
  delay(6000);
  Serial.println("Start Appication");
 
  appSetup();

  
}

void loop()
{
   appLoop();

  if (delayCheck(LED_Delay, 200))
  {
    TestLEDstate = !TestLEDstate;
    digitalWrite(get_PROG_LED_PIN(), TestLEDstate);   
    LED_Delay = millis();
  }
}
