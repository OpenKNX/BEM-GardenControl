#include <Arduino.h>
#include <Wire.h>
//#include <knx.h>
#include "hardware.h"
//#include "KnxHelper.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#include "Sensor_Value_Input.h"
#include "Device_setup.h"
#include "Input_Binary.h"
#include "Input_Impulse.h"
#include "ErrorHandling.h"

uint32_t heartbeatDelay = 0;
uint32_t startupDelay = 0;
uint32_t READ_ADC_Delay = 0;
uint32_t Output_Delay = 0;
uint32_t READ_PRINT = 0;
uint32_t TestDelay = 0;

bool TestState = false;

bool delayCheck(uint32_t iOldTimer, uint32_t iDuration)
{
  return millis() - iOldTimer >= iDuration;
}

void appSetup()
{
  SERIAL_PORT.println("Start init HW TOP");
  read_HW_ID_TOP();
  print_HW_ID_TOP(get_HW_ID_TOP());
  initHW();
  SERIAL_PORT.println("Done");

  SERIAL_PORT.println("Start init HW BOT");
#ifdef IOExp_enable
  read_HW_ID_BOT();
  print_HW_ID_BOT(get_HW_ID_BOT());
  initHW_Bot();
#endif
  SERIAL_PORT.println("Done");

  // load ETS parameters
  // load_ETS_par();

  digitalWrite(SSR_EN, HIGH);
  delay(1000);
  digitalWrite(SSR_EN, LOW);
  delay(1000);
}

void appLoop()
{
  processErrorHandling();

#ifdef ADC_enable
  processADConversation();
#endif
#ifdef BinInputs
  processBinInputs();
#endif

#ifdef S0Inputs
  processS0Input(0);
  processS0Input(1);
#endif
#ifdef ImplInput
  processImpulseInput();
#endif

  if (delayCheck(READ_ADC_Delay, 10000))
  {
    if (!getError())
    {
      TestState = !TestState;
      control_Relais(1, TestState);
      control_Relais(2, TestState);
      control_Relais(3, TestState);
      READ_ADC_Delay = millis();
    }
  }

  if (delayCheck(Output_Delay, 1000))
  {
    if (getError())
    {
      SERIAL_PORT.println(getError());
    }
#ifdef ADC_enable
      SERIAL_PORT.print("ADC CH1: ");
      SERIAL_PORT.println(getAdcVoltage_CH1());
      SERIAL_PORT.print("ADC CH2: ");
      SERIAL_PORT.println(getAdcVoltage_CH2());
      SERIAL_PORT.print("ADC CH3: ");
      SERIAL_PORT.println(getAdcVoltage_CH3());
      SERIAL_PORT.print("ADC CH4: ");
      SERIAL_PORT.println(getAdcVoltage_12V());

      SERIAL_PORT.print("ADC CH5: ");
      SERIAL_PORT.println(get4_20mA_CH1());
      SERIAL_PORT.print("ADC CH6: ");
      SERIAL_PORT.println(get4_20mA_CH2());
      SERIAL_PORT.print("ADC CH7: ");
      SERIAL_PORT.println(getAdcVoltage_24V());
#endif

#ifdef BinInputs
      SERIAL_PORT.print("BIN CH1: ");
      SERIAL_PORT.println(getStateInput1());
      SERIAL_PORT.print("BIN CH2: ");
      SERIAL_PORT.println(getStateInput2());
      SERIAL_PORT.print("BIN CH3: ");
      SERIAL_PORT.println(getStateInput3());
      SERIAL_PORT.print("BIN CH4: ");
      SERIAL_PORT.println(getStateInput4());
#endif

#ifdef ImplInput
      SERIAL_PORT.print("Impl: ");
      SERIAL_PORT.println(getFlowValue());
#endif

#ifdef IOExp_enable
      // enable/disable +5V BaseNoard
      enable_5V(LOW);

      control_Ventil(Ventil_1, HIGH);
      control_Ventil(Ventil_2, HIGH);
      control_Ventil(Ventil_3, HIGH);
      control_Ventil(Ventil_4, HIGH);
#endif
#ifdef Opto_IN
      SERIAL_PORT.print("Opto CH1: ");
      SERIAL_PORT.println(digitalRead(OptoIN_1));
      SERIAL_PORT.print("Opto CH2: ");
      SERIAL_PORT.println(digitalRead(OptoIN_2));
      SERIAL_PORT.print("Opto CH3: ");
      SERIAL_PORT.println(digitalRead(OptoIN_3));
      SERIAL_PORT.print("Opto CH4: ");
      SERIAL_PORT.println(digitalRead(OptoIN_4));
#endif

    Output_Delay = millis();
  }
}