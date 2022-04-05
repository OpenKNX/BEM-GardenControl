#include <Arduino.h>
#include <Wire.h>
#include <knx.h>
#include "GardenControl.h"
#include "BEM_hardware.h"
#include "KnxHelper.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#include "Sensor_Value_Input.h"
#include "Device_setup.h"
#include "Input_Binary.h"
#include "Input_Impulse.h"
#include "ErrorHandling.h"
#include "handleVentil.h"

//#include "Logic.h"

// Logic gLogic;

const uint8_t cFirmwareMajor = 1;    // 0-31
const uint8_t cFirmwareMinor = 0;    // 0-31
const uint8_t cFirmwareRevision = 0; // 0-63

uint32_t heartbeatDelay = 0;
uint32_t startupDelay = 0;
uint32_t READ_ADC_Delay = 0;
uint32_t Output_Delay = 0;
uint32_t READ_PRINT = 0;
uint32_t TestDelay = 0;
uint32_t LED_Delay2 = 0;
uint32_t LED_Delay = 0;

bool TestState = false;
bool TestLEDstate = false;
bool TestLEDstate2 = false;

bool delayCheck(uint32_t iOldTimer, uint32_t iDuration)
{
  return millis() - iOldTimer >= iDuration;
}

void waitStartupLoop()
{
  // KNX.loop();

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
  // SERIAL_PORT.print("KO: ");
  // SERIAL_PORT.println(iKo.asap());

  // check if we evaluate own KO
  if (iKo.asap() == LOG_KoDiagnose)
  {
    // ProcessDiagnoseCommand(iKo);       // ******************************************************************************  ändern !!!!!!!!!!!!!
  }
  else
  {
    bool callLogic = true;
    for (int koIndex = 0; koIndex < BEM_ChannelCount + REL_ChannelCount; koIndex++)
    {
      // KO Abfrage für Ventile 
      if (iKo.asap() == BEM_KoOffset + (BEM_Ko_Set_ventil + (koIndex * BEM_KoBlockSize)))
      {
        uint8_t ventil_Nr = ((iKo.asap() - BEM_KoOffset) / BEM_KoBlockSize);
        set_Ventil_State(ventil_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
      // KO Abfrage für Relais 
      else if (iKo.asap() == REL_KoOffset + (REL_Ko_Set_relais + (koIndex * REL_KoBlockSize)))
      {
#ifdef KNXcallback
        SERIAL_PORT.print("KO: ");
        SERIAL_PORT.println(iKo.asap());
#endif
        uint8_t relais_Nr = ((iKo.asap() - REL_KoOffset) / REL_KoBlockSize);
#ifdef KNXcallback
        SERIAL_PORT.print("Relais: ");
        SERIAL_PORT.println(relais_Nr);
#endif
        set_Relais_State(relais_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
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
  SERIAL_PORT.println("enable Main Relay");
  digitalWrite(get_SSR_EN_PIN(), HIGH);
  // wait so start Relay and Power Supply
  SERIAL_PORT.println("wait");
  delay(1000);
  // disable 5V to clear Errors
  SERIAL_PORT.println("stop 5V");
  digitalWrite(get_5V_EN_PIN(), LOW);
  // wait until 5V voltage go to 0V
  SERIAL_PORT.println("wait");
  delay(1000);
  // restart 5V
  SERIAL_PORT.println("restart 5V");
  digitalWrite(get_5V_EN_PIN(), HIGH);
  // wait until 5V powered up
  SERIAL_PORT.println("wait");
  delay(1000);
  // wait
  /*while (digitalRead(get_5V_status_PIN()))   // ******************************************************************************  ändern !!!!!!!!!!!!!
  {
    waitStartupLoop();
  }*/

  SERIAL_PORT.println("Start init HW TOP + BOT");
  initHW_Top();
  read_HW_ID_BOT();
  print_HW_ID_BOT(get_HW_ID_BOT());
  initHW_Bot();
  SERIAL_PORT.println("Done");

  // load ETS parameters
  // load_ETS_par();
}

void appLoop()
{
  processErrorHandling();
  processVentil();

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
      SERIAL_PORT.print("--> Relais: ");
      SERIAL_PORT.println(TestState);
    }
  }

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

  if (delayCheck(LED_Delay, 200))
  {
    TestLEDstate = !TestLEDstate;
    digitalWrite(get_PROG_LED_PIN(), TestLEDstate);
    LED_Delay = millis();
  }

  ProcessReadRequests();
}