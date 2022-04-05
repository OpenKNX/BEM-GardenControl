#include <stdint.h>
#include "GardenControl.h"
#include "I2C_IOExpander.h"
#include "KnxHelper.h"
#include "BEM_hardware.h"

bool ventil_State[BEM_ChannelCount] = {0};
bool ventil_State_old[BEM_ChannelCount]  = {true}; 
bool relais_State[BEM_ChannelCount] = {0};
bool relais_State_old[BEM_ChannelCount]  = {true}; 

/*****************************************************************************
 * Ventils
 ****************************************************************************/
void processVentil()
{
  //check of state chnage and Send new Status
  for(int ch=0; ch<BEM_ChannelCount ;ch++)
  {
    if(ventil_State_old[ch] != ventil_State[ch])
    {
        control_Ventil(ch, ventil_State[ch]);
        //SERIAL_PORT.print("senden ");
        //SERIAL_PORT.println(BEM_KoOffset + (ch * BEM_KoBlockSize + BEM_Ko_Status_ventil));
        knx.getGroupObject(BEM_KoOffset + (ch * BEM_KoBlockSize + BEM_Ko_Status_ventil)).value(ventil_State[ch], getDPT(VAL_DPT_1));
        ventil_State_old[ch] = ventil_State[ch];
    }
  } 
}

void set_Ventil_State(uint8_t ch, bool state)
{
  ventil_State[ch] = state;
}


/*****************************************************************************
 * Relais
 ****************************************************************************/
void processRelais()
{
  //check of state chnage and Send new Status
  for(int ch=0; ch<REL_ChannelCount ;ch++)
  {
    if(relais_State_old[ch] != relais_State[ch])
    {
        control_Ventil(ch, relais_State[ch]);
        //SERIAL_PORT.print("senden ");
        //SERIAL_PORT.println(BEM_KoOffset + (ch * BEM_KoBlockSize + BEM_Ko_Status_ventil));
        knx.getGroupObject(REL_KoOffset + (ch * REL_KoBlockSize + REL_Ko_Status_relais)).value(relais_State[ch], getDPT(VAL_DPT_1));
        relais_State_old[ch] = relais_State[ch];
    }
  } 
}

void set_Relais_State(uint8_t ch, bool state)
{
  relais_State[ch] = state;
}