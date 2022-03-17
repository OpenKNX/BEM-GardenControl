#include <Arduino.h>
#include "Device_setup.h"
#include "hardware.h"
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

  init_GPIOs();
  initHW();

  // Print HW-ID
  print_HW_ID_TOP(get_HW_ID_TOP());

  digitalWrite(SSR_EN, HIGH);
  delay(1000);
  digitalWrite(SSR_EN, LOW);
  delay(1000);
 
 appSetup();

  SERIAL_PORT.println("Setup Done");
}

void loop()
{
   appLoop();

  if (delayCheck(LED_Delay, 200))
  {
    TestLEDstate = !TestLEDstate;
    digitalWrite(PROG_LED_PIN, TestLEDstate);   
    LED_Delay = millis();
  }
}
