#include <stdint.h>

#include "BEM_hardware.h"
#include "PCA9554.h"
#include "PCA9555.h"
#include "ErrorHandling.h"
#include "GardenControl.h"
#include "SystemFailureHandling.h"

PCA9554 pca9554(i2cAddr_IO, &Wire1);     // Create an object at this address
PCA9555 pca9555(i2cAddr_IO_Bot, &Wire1); // Create an object at this address

bool init_flag_PCA9555 = false;
bool init_flag_PCA9554 = false;

uint8_t failureCounter = 0;
uint8_t failureCounter2 = 0;

void init_IOExpander_GPIOs_TOP()
{
  // check if +5V iso is available
  if (!get_5V_Error())
  {
    SERIAL_PORT.print("  PCA9554: ");
    if (pca9554.isConnected())
    {
      pca9554.begin();

      pca9554.pinMode(0, OUTPUT);
      pca9554.pinMode(1, INPUT);
      pca9554.pinMode(2, INPUT);
      pca9554.pinMode(3, INPUT);
      pca9554.pinMode(4, INPUT);
      pca9554.pinMode(5, OUTPUT);
      pca9554.pinMode(6, OUTPUT);
      pca9554.pinMode(7, OUTPUT);

      SERIAL_PORT.print(pca9554.readRegister(Reg_input_Ports), BIN);

      SERIAL_PORT.println(" OK");

      init_flag_PCA9554 = true;
    }
    else
    {
      SERIAL_PORT.println("NOK");
      failureCounter++;
      if (failureCounter > 10)
      {
        rebootExternalPWR();
        failureCounter = 0;
      }
    }
  }
  else
  {
    SERIAL_PORT.println("  PCA9554 ERROR: no +5V_Iso");
  }
}

void init_IOExpander_GPIOs_BOT()
{
  // check if +5V iso is available
  if (!get_5V_Error())
  {
    SERIAL_PORT.print("  PCA9555: ");
    if (pca9555.isConnected())
    {
      SERIAL_PORT.println("OK");
      pca9555.begin();
      
      for (uint8_t i = 0; i < 16; i++)
      {
        pca9555.pinMode(i, OUTPUT);
        pca9555.digitalWrite(i, LOW);
        //SERIAL_PORT.println(pca9555.getConfigReg());
        delay(1);
      }
      
      init_flag_PCA9555 = true;
    }
    else
    {
      SERIAL_PORT.println("NOK");
      failureCounter2++;
      if (failureCounter2 > 10)
      {
        rebootExternalPWR();
        failureCounter2 = 0;
      }
    }
  }
  else
  {
    SERIAL_PORT.println("  PCA9555 ERROR: no +5V_Iso");
  }
}

void clearInitFlags_IOExp()
{
  init_flag_PCA9554 = false;
  init_flag_PCA9555 = false;
}

void set_IOExpander_Input(uint8_t ch, bool state)
{
  // check if +5V iso is available
  if (!get_5V_Error())
  {
    if (init_flag_PCA9554)
    {
      pca9554.digitalWrite(ch, state);
    }
    else
    {
      init_IOExpander_GPIOs_TOP();
      pca9554.digitalWrite(ch, state);
    }
  }
}

bool get_IOExpander_Input(uint8_t ch)
{
  // check if +5V iso is available
  if (!get_5V_Error())
  {
    if (init_flag_PCA9554)
    {
      return pca9554.digitalRead(ch);
    }
    else
    {
      init_IOExpander_GPIOs_TOP();
      return pca9554.digitalRead(ch);
    }
  }
  else
  {
    SERIAL_PORT.println("5V ERROR");
    return 0;
  }
}

bool get_IOExpander_BOT_Input(uint8_t ch)
{
  // check if +5V iso is available
  if (!get_5V_Error())
  {
    if (init_flag_PCA9555)
    {
      return pca9555.digitalRead(ch);
    }
    else
    {
      init_IOExpander_GPIOs_BOT();
      return pca9555.digitalRead(ch);
    }
  }
  else
  {
    return 0;
  }
}

void set_IOExpander_BOT_Input(uint8_t ch, bool state)
{
  // check if +5V iso is available
  if (!get_5V_Error())
  {
    if (init_flag_PCA9555)
    {
      pca9555.digitalWrite(ch, state);
    }
    else
    {
      init_IOExpander_GPIOs_BOT();
      pca9555.digitalWrite(ch, state);
    }
  }
  else
  {
    SERIAL_PORT.println("EE");
  }
}

bool getStatus_Ventil(uint8_t ch)
{
  if (ch <= BEM_ChannelCount && ch > 0)
  {
    return get_IOExpander_BOT_Input(ch);
  }
  {
    return 0;
  }
}

bool getStatus_Relais(uint8_t nr)
{
  if (nr <= REL_ChannelCount && nr > 0)
  {
    nr = nr + RelaisOffset;
    return get_IOExpander_BOT_Input(nr);
  }
  else
  {
    return 0;
  }
}

void enable_5V(bool state)
{
  set_IOExpander_Input(IO_5V_EN, state);
}

void set_ADC1_VoltageDiff(bool state)
{
  set_IOExpander_Input(IO_Set_DIV_1, state);
}

void set_ADC2_VoltageDiff(bool state)
{
  set_IOExpander_Input(IO_Set_DIV_2, state);
}

void set_ADC3_VoltageDiff(bool state)
{
  set_IOExpander_Input(IO_Set_DIV_3, state);
}

bool getInitFlag_PCA9555()
{
  return init_flag_PCA9555;
}

bool getInitFlag_PCA9554()
{
  return init_flag_PCA9554;
}