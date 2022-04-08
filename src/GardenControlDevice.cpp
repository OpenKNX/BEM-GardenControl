#include <Arduino.h>
#include <Wire.h>
#include <knx.h>
#include "GardenControl.h"
#include "HelperFunc.h"
#include "BEM_hardware.h"
#include "KnxHelper.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#include "Sensor_Value_Input.h"
#include "S0Function.h"
#include "Device_setup.h"
#include "Input_Binary.h"
#include "Input_Impulse.h"
#include "ErrorHandling.h"
#include "handleVentilRelais.h"
#include "InputADC.h"


//#include "Logic.h"

// Logic gLogic;

const uint8_t cFirmwareMajor = 1;    // 0-31
const uint8_t cFirmwareMinor = 0;    // 0-31
const uint8_t cFirmwareRevision = 0; // 0-63

//uint32_t heartbeatDelay = 0;
//uint32_t startupDelay = 0;
uint32_t READ_ADC_Delay = 0;
uint32_t Output_Delay = 0;
uint32_t READ_PRINT = 0;
uint32_t TestDelay = 0;
uint32_t LED_Delay2 = 0;
uint32_t LED_Delay = 0;

bool TestState = false;
bool TestLEDstate = false;
bool TestLEDstate2 = false;

void waitStartupLoop()
{
   //KNX.loop();

  if (delayCheck(LED_Delay2, 700))
  {
    TestLEDstate2 = !TestLEDstate2;
    digitalWrite(get_PROG_LED_PIN(), TestLEDstate2);
    LED_Delay2 = millis();
  }
}

void ProcessReadRequests()
{
  // this method is called after startup delay and executes read requests, which should just happen once after startup
  static bool sCalledProcessReadRequests = false;
  if (!sCalledProcessReadRequests)
  {
    // we go through all IO devices defined as outputs and check for initial read requests
    sCalledProcessReadRequests = true;
  }
  // gLogic.processReadRequests();    // ******************************************************************************  ändern !!!!!!!!!!!!!
}

/*
bool processDiagnoseCommand()
{
  char *lBuffer = gLogic.getDiagnoseBuffer();
  bool lOutput = false;
  if (lBuffer[0] == 'v')
  {
    // Command v: retrun fimware version, do not forward this to logic,
    // because it means firmware version of the outermost module
    sprintf(lBuffer, "VER [%d] %d.%d", cFirmwareMajor, cFirmwareMinor, cFirmwareRevision);
    lOutput = true;
  }
  else
  {
    // let's check other modules for this command
    lOutput = gLogic.processDiagnoseCommand();
  }
  return lOutput;
}

void ProcessDiagnoseCommand(GroupObject &iKo)
{
  // this method is called as soon as iKo is changed
  // an external change is expected
  // because this iKo also is changed within this method,
  // the method is called again. This might result in
  // an endless loop. This is prevented by the isCalled pattern.
  static bool sIsCalled = false;
  if (!sIsCalled)
  {
    sIsCalled = true;
    //diagnose is interactive and reacts on commands
    gLogic.initDiagnose(iKo);
    if (processDiagnoseCommand())
      gLogic.outputDiagnose(iKo);
    sIsCalled = false;
  }
};
*/

void ProcessKoCallback(GroupObject &iKo)
{
  // check if we evaluate own KO
  if (iKo.asap() == LOG_KoDiagnose)
  {
    // ProcessDiagnoseCommand(iKo);       // ******************************************************************************  ändern !!!!!!!!!!!!!
  }
  else if(iKo.asap() == BEM_Ko_Set_5V_relais)
  {
    set_5V_Relais_State(iKo.value(getDPT(VAL_DPT_1)));
  }
  else
  {
    bool callLogic = true;
    for (int koIndex = 0; koIndex < BEM_ChannelCount + REL_ChannelCount; koIndex++)
    {
      if (iKo.asap() == BEM_KoOffset + (BEM_Ko_Set_ventil + (koIndex * BEM_KoBlockSize))) // KO Abfrage für Ventile 
      {
        uint8_t ventil_Nr = ((iKo.asap() - BEM_KoOffset) / BEM_KoBlockSize);
        set_Ventil_State(ventil_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
      else if(iKo.asap() == BEM_KoOffset + (BEM_Ko_Sperr_ventil + (koIndex * BEM_KoBlockSize))) // KO Abfrage für Sperrobjekte Ventile 
      {
        uint8_t ventil_Nr = ((iKo.asap() - BEM_KoOffset) / BEM_KoBlockSize);
        set_Ventil_Sperrobjekt(ventil_Nr, iKo.value(getDPT(VAL_DPT_1)));
      }
      else if (iKo.asap() == REL_KoOffset + (REL_Ko_Set_relais + (koIndex * REL_KoBlockSize))) // KO Abfrage für Relais 
      {
        uint8_t relais_Nr = ((iKo.asap() - REL_KoOffset) / REL_KoBlockSize);
        set_Relais_State(relais_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
      else if(iKo.asap() == REL_KoOffset + (REL_Ko_Sperr_relais + (koIndex * REL_KoBlockSize))) // KO Abfrage für Sperrobjekte Relais 
      {
        uint8_t relais_Nr = ((iKo.asap() - REL_KoOffset) / REL_KoBlockSize);
        set_Relais_Sperrobjekt(relais_Nr, iKo.value(getDPT(VAL_DPT_1)));
      }
    }

    // for handling external inputs, logik always to be called
    // gLogic.processInputKo(iKo);          // ******************************************************************************  ändern !!!!!!!!!!!!!
  }
}

void appSetup()
{
  if (knx.configured())
  {
    if (GroupObject::classCallback() == 0)
      GroupObject::classCallback(ProcessKoCallback);
    // Setup Logik
    // Logic::addLoopCallback(EnOcean::taskCallback, &enOcean);        // ************************************************************
    // gLogic.setup(false);                                            // ************************************************************
  }

  SERIAL_PORT.println("Start init HW TOP");
  read_HW_ID_TOP();
  print_HW_ID_TOP(get_HW_ID_TOP());
  initHW();
  SERIAL_PORT.println("Done");
  // enable Main Relay
  SERIAL_PORT.print("enable Main Relay: ");
  SERIAL_PORT.println((knx.paramByte(BEM_ext5VRelaisStateBegin) >> BEM_ext5VRelaisStateBeginShift) & 1);
  SERIAL_PORT.println(knx.paramByte(BEM_ext5VRelaisStateBegin),BIN);
  control_5V_Relais((knx.paramByte(BEM_ext5VRelaisStateBegin) >> BEM_ext5VRelaisStateBeginShift) & 1);
  // wait so start Relay and Power Supply
  SERIAL_PORT.println("wait");
  delay(1000);
  // disable internal 5V to clear Errors
  SERIAL_PORT.println("stop 5V");
  digitalWrite(get_5V_EN_PIN(), LOW);
  // wait until internal 5V voltage go to 0V
  SERIAL_PORT.println("wait");
  delay(1000);
  // restart 5V
  SERIAL_PORT.println("restart 5V");
  digitalWrite(get_5V_EN_PIN(), HIGH);
  // wait until internal 5V powered up
  SERIAL_PORT.println("wait");
  // wait
  delay(1000);
  /*while (digitalRead(get_5V_status_PIN()))   // ******************************************************************************  ändern !!!!!!!!!!!!!
  {
    waitStartupLoop();
  }*/

  SERIAL_PORT.println("Start init HW TOP + BOT");
  initHW_Top();
  read_HW_ID_BOT();
  print_HW_ID_BOT(get_HW_ID_BOT());
  initHW_Bot();
  

  
  // load ETS parameters
  SERIAL_PORT.println("Load Parameters");
  initInputADC();
  // load_ETS_par();
  SERIAL_PORT.println("Done");
  delay(3000);
}

void appLoop()
{
  processErrorHandling();
  processVentil();
  processRelais();
  process_5V_Relais();

#ifdef ADC_enable
  //processADConversation();
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
/*
  if (delayCheck(READ_ADC_Delay, 10000))
  {
    if (!getError())
    {
      TestState = !TestState;
      control_Relais(1, TestState);
      control_Relais(2, TestState);
      control_Relais(3, TestState);
      READ_ADC_Delay = millis();
      SERIAL_PORT.print("--> Relais: ");
      SERIAL_PORT.println(TestState);
    }
  }
*/
  if (delayCheck(Output_Delay, 1000))
  {
    if (getError())
    {
#ifdef SerialError
      SERIAL_PORT.println(getError());
      SERIAL_PORT.println(get_5V_Error());
      SERIAL_PORT.println(get_12V_Error());
      SERIAL_PORT.println(get_24V_Error());
      SERIAL_PORT.println(get_5V_out_Error());
#endif
    }
#ifdef ADC_enable_Output

    SERIAL_PORT.print("ADC CH1: ");
    SERIAL_PORT.println(getSensorValue(0));
    SERIAL_PORT.println(getAdcVoltage_CH1());
    SERIAL_PORT.println(getAdcValue(0));
    SERIAL_PORT.print("ADC CH2: ");
    SERIAL_PORT.println(getSensorValue(1));
    SERIAL_PORT.println(getAdcVoltage_CH2());
    SERIAL_PORT.println(getAdcValue(1));
    SERIAL_PORT.print("ADC CH3: ");
    SERIAL_PORT.println(getSensorValue(2));
    SERIAL_PORT.println(getAdcVoltage_CH3());
    SERIAL_PORT.println(getAdcValue(2));
    SERIAL_PORT.print("ADC CH4: ");
    SERIAL_PORT.println(getAdcVoltage_12V());
    SERIAL_PORT.print("ADC CH5: ");
    SERIAL_PORT.println(get4_20mA_CH1());
    SERIAL_PORT.print("ADC CH6: ");
    SERIAL_PORT.println(get4_20mA_CH2());
    SERIAL_PORT.print("ADC CH7: ");
    SERIAL_PORT.println(getAdcVoltage_24V());
#endif

#ifdef BinInputs_Output
    SERIAL_PORT.print("BIN CH1: ");
    SERIAL_PORT.println(getStateInput1());
    SERIAL_PORT.print("BIN CH2: ");
    SERIAL_PORT.println(getStateInput2());
    SERIAL_PORT.print("BIN CH3: ");
    SERIAL_PORT.println(getStateInput3());
    SERIAL_PORT.print("BIN CH4: ");
    SERIAL_PORT.println(getStateInput4());
#endif

#ifdef ImplInput_Output
    SERIAL_PORT.print("Impl: ");
    SERIAL_PORT.println(getFlowValue());
#endif

#ifdef Opto_IN_Output
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

  if (delayCheck(LED_Delay, 200))
  {
    TestLEDstate = !TestLEDstate;
    digitalWrite(get_PROG_LED_PIN(), TestLEDstate);
    LED_Delay = millis();
  }

  ProcessReadRequests();
}