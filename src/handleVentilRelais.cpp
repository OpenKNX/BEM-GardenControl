#include <stdint.h>
#include "handleVentilRelais.h"
#include "GardenControl.h"
#include "I2C_IOExpander.h"
#include "KnxHelper.h"
#include "BEM_hardware.h"
#include "Device_setup.h"

bool ventil_State[BEM_ChannelCount] = {0};
bool ventil_State_old[BEM_ChannelCount] = {true};
bool ventil_Sperrobjekt[BEM_ChannelCount] = {0};

bool relais_State[REL_ChannelCount] = {0};
bool relais_State_old[REL_ChannelCount] = {true};
bool relais_Sperrobjekt[REL_ChannelCount] = {0};

bool relais_5V_State = false;
bool relais_5V_State_old = false;
bool relais_5V_startup_flag = true;

/*****************************************************************************
 * Ventils
 ****************************************************************************/
void processVentil()
{
  // check of state chnage and Send new Status
  for (int ch = 0; ch < BEM_ChannelCount; ch++)
  {
    if (ventil_State_old[ch] != ventil_State[ch] && ventil_Sperrobjekt[ch] == false)
    {
      control_Ventil(ch, ventil_State[ch]);
      SERIAL_PORT.print("Ventil_");
      SERIAL_PORT.print(ch+1);
      SERIAL_PORT.print(": ");
      SERIAL_PORT.println(ventil_State[ch]);
      knx.getGroupObject(BEM_KoOffset + (ch * BEM_KoBlockSize + BEM_Ko_Status_ventil)).value(ventil_State[ch], getDPT(VAL_DPT_1));
      ventil_State_old[ch] = ventil_State[ch];
    }
  }
}

void set_Ventil_State(uint8_t ch, bool state)
{
  ventil_State[ch] = state;
}

void set_Ventil_Sperrobjekt(uint8_t ch, bool state)
{
  SERIAL_PORT.print("Sperrobjekt Ventil CH");
  SERIAL_PORT.print(ch);
  SERIAL_PORT.print(": ");
  SERIAL_PORT.println(state);
  ventil_Sperrobjekt[ch] = state;
}

/*****************************************************************************
 * Relais
 ****************************************************************************/
void processRelais()
{
  // check of state chnage and Send new Status
  for (int ch = 0; ch < REL_ChannelCount; ch++)
  {
    if (relais_State_old[ch] != relais_State[ch] && relais_Sperrobjekt[ch] == false)
    {
      control_Relais(ch, relais_State[ch]);
      SERIAL_PORT.print("Relais_");
      SERIAL_PORT.print(ch+1);
      SERIAL_PORT.print(": ");
      SERIAL_PORT.println(relais_State[ch]);
      knx.getGroupObject(REL_KoOffset + (ch * REL_KoBlockSize + REL_Ko_Status_relais)).value(relais_State[ch], getDPT(VAL_DPT_1));
      relais_State_old[ch] = relais_State[ch];
    }
  }
}

void control_Ventil(uint8_t ch, bool state)
{
  if (ch <= BEM_ChannelCount && ch > 0)
  {
    set_IOExpander_BOT_Input(ch, state);
  }
}

void control_Relais(uint8_t nr, bool state)
{
  switch (nr)
  {
  case 0:
    set_IOExpander_BOT_Input(14, state);
    break;
  case 1:
    set_IOExpander_BOT_Input(13, state);
    break;
  case 2:
    set_IOExpander_BOT_Input(12, state);
    break;
  default:
    break;
  }
}

void set_Relais_State(uint8_t ch, bool state)
{
  relais_State[ch] = state;
}

void set_Relais_Sperrobjekt(uint8_t ch, bool state)
{
  SERIAL_PORT.print("Sperrobjekt Relais CH");
  SERIAL_PORT.print(ch);
  SERIAL_PORT.print(": ");
  SERIAL_PORT.println(state);
  relais_Sperrobjekt[ch] = state;
}

/*****************************************************************************
 * externes 5V Relais
 ****************************************************************************/
void control_5V_Relais(bool state)
{
  digitalWrite(get_SSR_EN_PIN(), state);
  relais_5V_State = state;
}

void process_5V_Relais()
{
  if (relais_5V_startup_flag)
  {
    if ((knx.paramByte(BEM_ext5VRelaisStartState) >> BEM_ext5VRelaisStartStateShift) & 1) // Senden bei Startup "AN"
    {
      SERIAL_PORT.println("E");
      knx.getGroupObject(BEM_Ko_Status_5V_relais).value(relais_5V_State, getDPT(VAL_DPT_1));
    }
    relais_5V_startup_flag = false;
  }

  if (relais_5V_State_old != relais_5V_State)
  {
    control_5V_Relais(relais_5V_State);
    knx.getGroupObject(BEM_Ko_Status_5V_relais).value(relais_5V_State, getDPT(VAL_DPT_1));
    relais_5V_State_old = relais_5V_State;
  }
}



void set_5V_Relais_State(bool state)
{
  relais_5V_State = state;
}

bool get_5V_Relais_State(bool state)
{
  return relais_5V_State;
}