#include <stdint.h>

#include "hardware.h"
#include "PCA9554.h"
#include "PCA9555.h"

PCA9554 pca9554(i2cAddr_IO, &Wire1);     // Create an object at this address
PCA9555 pca9555(i2cAddr_IO_Bot, &Wire1); // Create an object at this address

void init_IOExpander_GPIOs_TOP()
{
  SERIAL_PORT.print("  PCA9554: ");
  if (pca9554.isConnected())
  {
    SERIAL_PORT.println("OK");
    pca9554.begin();

    pca9554.pinMode(0, OUTPUT);
    pca9554.pinMode(1, INPUT);
    pca9554.pinMode(2, INPUT);
    pca9554.pinMode(3, INPUT);
    pca9554.pinMode(4, INPUT);
    pca9554.pinMode(5, OUTPUT);
    pca9554.pinMode(6, OUTPUT);
    pca9554.pinMode(7, OUTPUT);
  }
  else
  {
    SERIAL_PORT.println("NOK");
  }
}

void init_IOExpander_GPIOs_BOT()
{
  // check if +5V iso is available
  if (digitalRead(iso_5V) == 0)
  {
    SERIAL_PORT.print("  PCA9555: ");
    if (pca9555.isConnected())
    {
      SERIAL_PORT.println("OK");
      pca9555.begin();

      for (uint8_t i = 0; i < 15; i++)
      {
        pca9555.pinMode(i, OUTPUT);
        pca9555.digitalWrite(i, LOW);
      }
    }
    else
    {
      SERIAL_PORT.println("NOK");
    }
  }
  else
  {
    SERIAL_PORT.print("  PCA9555 ERROR: no +5V_Iso");
  }
}

bool init_IOExpander_GPIOs(uint8_t ch)
{
  return pca9554.digitalRead(ch);
}

void set_IOExpander_Input(uint8_t ch, bool state)
{
  pca9554.digitalWrite(ch, state);
}

bool get_IOExpander_Input(uint8_t ch)
{
  return pca9554.digitalRead(ch);
}

bool get_IOExpander_BOT_Input(uint8_t ch)
{
  return pca9555.digitalRead(ch);
}

void set_IOExpander_BOT_Input(uint8_t ch, bool state)
{
  // check if +5V iso is available
  if (digitalRead(iso_5V) == 0)
  {
    pca9555.digitalWrite(ch, state);
  }
  else
  {
    SERIAL_PORT.print("PCA9555 ERROR: no +5V_Iso");
  }
}

void control_Ventil(uint8_t ch, bool state)
{

  if (ch <= maxCountVentil && ch > 0)
  {
    ch = ch - 1;
    set_IOExpander_BOT_Input(ch, state);
  }
}

void control_Relais(uint8_t nr, bool state)
{
  if (nr <= maxCountRelais && nr > 0)
  {
    nr = nr + RelaisOffset;
    set_IOExpander_BOT_Input(nr, state);
  }
}

bool getStatus_Ventil(uint8_t ch)
{
  if (ch <= maxCountVentil && ch > 0)
  {
    ch = ch - 1;
    return get_IOExpander_BOT_Input(ch);
  }
  {
    return 0;
  }
}

bool getStatus_Relais(uint8_t nr)
{
  if (nr <= maxCountRelais && nr > 0)
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