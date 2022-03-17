#include "Device_setup.h"
#include <stdint.h>
#include <Arduino.h>
#include "hardware.h"
#include "I2C_IOExpander.h"
#include "Input_Binary.h"
#include "Input_Impulse.h"

void init_GPIOs()
{
  // RP2040 GPIO Init
  pinMode(PROG_LED_PIN, OUTPUT);
  pinMode(SSR_EN, OUTPUT);
  pinMode(iso_5V, INPUT);
  pinMode(OptoIN_1, INPUT);
  pinMode(OptoIN_2, INPUT);
  pinMode(OptoIN_3, INPUT);
  pinMode(OptoIN_4, INPUT);

  digitalWrite(PROG_LED_PIN, LOW);
}

uint8_t get_PROG_LED_PIN(uint8_t hwID)
{
  switch (hwID)
  {
  case HW_1_0: // V1.x
    return 24;
    break;
  default:
    return 255;
    break;
  }
}

uint8_t get_PROG_BUTTON_PIN(uint8_t hwID)
{
  switch (hwID)
  {
  case HW_1_0: // V1.x
    return 25;
    break;

  default:
    return 250;
    break;
  }
}

uint8_t get_SAVE_INTERRUPT_PIN(uint8_t hwID)
{
  switch (hwID)
  {
  case HW_1_0: // V1.x
    return 23;
    break;

  default:
    return 252;
    break;
  }
}

void print_HW_ID_TOP(uint8_t id)
{
  SERIAL_PORT.print("HW-ID-TOP: ");
  switch (id)
  {
  case HW_1_0:
    SERIAL_PORT.println("V1.x");
    break;

  default:
    SERIAL_PORT.println("Not Defined");
    break;
    break;
  }
}

void print_HW_ID_BOT(uint8_t id)
{
  SERIAL_PORT.print("HW-ID-BOT: ");
  switch (id)
  {
  case HW_1_0:
    SERIAL_PORT.println("V1.x");
    break;

  default:
    SERIAL_PORT.println("Not Defined");
    break;
  }
}

uint8_t get_HW_ID_TOP()
{
  uint8_t hw_ID = 0;
  // read Inputs
  bitWrite(hw_ID, 0, digitalRead(ID1));
  bitWrite(hw_ID, 1, digitalRead(ID2));
  bitWrite(hw_ID, 2, digitalRead(ID3));
  return hw_ID;
}

uint8_t get_HW_ID_BOT()
{
  uint8_t hw_ID = 0;
  // read Inputs
  bitWrite(hw_ID, 0, get_IOExpander_BOT_Input(IO_HW_ID1));
  bitWrite(hw_ID, 1, get_IOExpander_BOT_Input(IO_HW_ID2));
  bitWrite(hw_ID, 2, get_IOExpander_BOT_Input(IO_HW_ID3));
  SERIAL_PORT.println(hw_ID, BIN);
  return hw_ID;
}

void initHW()
{
  uint8_t hw_ID = 0;
  // Set Inputs
  pinMode(ID1, INPUT_PULLUP);
  pinMode(ID2, INPUT_PULLUP);
  pinMode(ID3, INPUT_PULLUP);
  // read Inputs
  bitWrite(hw_ID, 0, digitalRead(ID1));
  bitWrite(hw_ID, 1, digitalRead(ID2));
  bitWrite(hw_ID, 2, digitalRead(ID3));

  // init Inputs
  InitBinInput1();    // Input 1
  InitBinInput2();    // Input 2
  InitBinInput3();    // Input 3
  InitImpulseInput(); // Input 4
}