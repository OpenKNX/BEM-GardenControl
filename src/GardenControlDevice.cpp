#include <Arduino.h>
#include <Wire.h>
#include <knx.h>
#include "GardenControl.h"
#include "HelperFunc.h"

#include "KnxHelper.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#include "ReadADC.h"
#include "Device_setup.h"
#include "ReadBinary.h"
#include "Input_Impulse.h"
#include "ErrorHandling.h"
#include "handleVentilRelais.h"
#include "InputADC.h"
#include "Input_4_20mA.h"
#include "Input_BIN.h"
#include "Input_S0.h"
#include "SystemFailureHandling.h"
#include "LED_Statusanzeige.h"

#include "Logic.h"

#define KNXcallback

#ifndef DEBUG
#define DEBUG 0
#endif

Logic gLogic;

uint32_t READ_ADC_Delay = 0;
uint32_t Output_Delay = 0;
uint32_t READ_PRINT = 0;
uint32_t TestDelay = 0;
uint32_t LED_Delay[] = {0,0};

bool initADCFlag = false;
bool TestState = false;
bool TestLEDstate[] = {false, false};

enum StateMaschine
{
  Pos1 = 1,
  Pos2 = 2,
  Pos3 = 3,
  Pos4 = 4,
  Pos5 = 5,
};

StateMaschine StateM = Pos1;

uint32_t heartbeatDelay = 0;
uint32_t startupDelay = 0;

// true solgange der Start des gesamten Moduls verzögert werden soll
bool startupDelayfunc()
{
  return !delayCheck(startupDelay, getDelayPattern(LOG_StartupDelayBase));
}

void ProcessHeartbeat()
{
  // the first heartbeat is send directly after startup delay of the device
  if (heartbeatDelay == 0 || delayCheck(heartbeatDelay, getDelayPattern(LOG_HeartbeatDelayBase)))
  {
    // we waited enough, let's send a heartbeat signal
    knx.getGroupObject(LOG_KoHeartbeat).value(true, getDPT(VAL_DPT_1));

    heartbeatDelay = millis();
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
  gLogic.processReadRequests(); // ******************************************************************************  ändern !!!!!!!!!!!!!
}

void waitStartupLoop()
{
  knx.loop();
  ProcessReadRequests();
  processVentil();     // PRIO 3
  processRelais();     // PRIO 3
  process_5V_Relais(); // PRIO 3

  if (delayCheck(LED_Delay[1], 700))
  {
    TestLEDstate[1] = !TestLEDstate[1];
    digitalWrite(get_PROG_LED_PIN(), TestLEDstate[1]);
    LED_Delay[1] = millis();
    SERIAL_PORT.println("Wait for 5V");
  }
}

bool processDiagnoseCommand()
{
  char *lBuffer = gLogic.getDiagnoseBuffer();
  bool lOutput = false;
  if (lBuffer[0] == 'v')
  {
    // Command v: retrun fimware version, do not forward this to logic,
    // because it means firmware version of the outermost module
    sprintf(lBuffer, "VER [%d] %d.%d", 0, MAIN_ApplicationVersion / 16, MAIN_ApplicationVersion % 16);
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
    // diagnose is interactive and reacts on commands
    gLogic.initDiagnose(iKo);
    if (processDiagnoseCommand())
      gLogic.outputDiagnose(iKo);
    sIsCalled = false;
  }
};

void ProcessKoCallback(GroupObject &iKo)
{
#ifdef KNXcallback
  SERIAL_PORT.println(iKo.asap());
#endif
  // check if we evaluate own KO
  if (iKo.asap() == LOG_KoDiagnose)
  {
    ProcessDiagnoseCommand(iKo); // ******************************************************************************  ändern !!!!!!!!!!!!!
  }
  else if (iKo.asap() == BEM_Ko_Set_5V_relais)
  {
    set_5V_Relais_State(iKo.value(getDPT(VAL_DPT_1)));
  }
  else
  {
    bool callLogic = true;
    for (int koIndex = 0; koIndex < BEM_ChannelCount; koIndex++)
    {
      if (iKo.asap() == BEM_KoOffset + (BEM_Ko_Set_ventil + (koIndex * BEM_KoBlockSize))) // KO Abfrage für Ventile
      {
        uint8_t ventil_Nr = ((iKo.asap() - BEM_KoOffset) / BEM_KoBlockSize);
#ifdef KNXcallback
        SERIAL_PORT.print("KO_Ventil_");
        SERIAL_PORT.print(ventil_Nr + 1);
        SERIAL_PORT.print(": ");
        SERIAL_PORT.println((bool)iKo.value(getDPT(VAL_DPT_1)));
#endif
        set_Ventil_State(ventil_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
      else if (iKo.asap() == BEM_KoOffset + (BEM_Ko_Sperr_ventil + (koIndex * BEM_KoBlockSize))) // KO Abfrage für Sperrobjekte Ventile
      {
        uint8_t ventil_Nr = ((iKo.asap() - BEM_KoOffset) / BEM_KoBlockSize);
        set_Ventil_Sperrobjekt(ventil_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
    }
    for (int koIndex = 0; koIndex < REL_ChannelCount; koIndex++)
    {
      if (iKo.asap() == REL_KoOffset + (REL_Ko_Set_relais + (koIndex * REL_KoBlockSize))) // KO Abfrage für Relais
      {
        uint8_t relais_Nr = ((iKo.asap() - REL_KoOffset) / REL_KoBlockSize);
#ifdef KNXcallback
        SERIAL_PORT.print("KO_Relais_");
        SERIAL_PORT.print(relais_Nr + 1);
        SERIAL_PORT.print(": ");
        SERIAL_PORT.println((bool)iKo.value(getDPT(VAL_DPT_1)));
#endif
        set_Relais_State(relais_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
      else if (iKo.asap() == REL_KoOffset + (REL_Ko_Sperr_relais + (koIndex * REL_KoBlockSize))) // KO Abfrage für Sperrobjekte Relais
      {
        uint8_t relais_Nr = ((iKo.asap() - REL_KoOffset) / REL_KoBlockSize);
        set_Relais_Sperrobjekt(relais_Nr, iKo.value(getDPT(VAL_DPT_1)));
        callLogic = false;
      }
    }

    // for handling external inputs, logik always to be called
    gLogic.processInputKo(iKo); // ******************************************************************************  ändern !!!!!!!!!!!!!
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
    gLogic.setup(false); // ************************************************************
  }

  // enable Main Relay
  SERIAL_PORT.print("enable Main Relay: ");
  SERIAL_PORT.println((knx.paramByte(BEM_ext5VRelaisStateBegin) >> BEM_ext5VRelaisStateBeginShift) & 1);
  SERIAL_PORT.println(knx.paramByte(BEM_ext5VRelaisStateBegin), BIN);
  control_5V_Relais((knx.paramByte(BEM_ext5VRelaisStateBegin) >> BEM_ext5VRelaisStateBeginShift) & 1);
  // wait so start Relay and Power Supply
  SERIAL_PORT.println("wait");
  delay(1000);
  // disable internal 5V to clear Errors
  SERIAL_PORT.println("stop 5V");
  digitalWrite(get_5V_EN_PIN(), HIGH);
  // wait until internal 5V voltage go to 0V
  SERIAL_PORT.println("wait");
  delay(1000);
  // restart 5V
  SERIAL_PORT.println("restart 5V");
  digitalWrite(get_5V_EN_PIN(), LOW);
  // wait until internal 5V powered up
  SERIAL_PORT.println("wait");
  // wait
  delay(500);

  // start the framework
  knx.start();

  // I2C Init
  Wire.setSDA(20);
  Wire.setSCL(21);

  Wire.begin();

  initI2cStatusLeds();
  setLED_ON_ALL();
  delay(500);
  setLED_OFF_ALL();

  while (digitalRead(get_5V_status_PIN())) // ******************************************************************************  ändern !!!!!!!!!!!!!
  {
    waitStartupLoop();
  }

  if (!digitalRead(get_5V_status_PIN()))
  {
    SERIAL_PORT.println("Start init HW TOP + BOT");
    initHW_Top();
    read_HW_ID_BOT();
    print_HW_ID_BOT(get_HW_ID_BOT());
    initHW_Bot();

    // load ETS parameters
    SERIAL_PORT.println("Load Parameters");
    
    // init Inputs: Binäereingänge
    for(uint8_t i=0;i<4;i++)
    {
      InitBinInput(i);
    }

    for(uint8_t i=0;i<4;i++)
    {
      InitS0Input(i);
    }

    initInputADC();

    InitImpulseInputs();
    // load_ETS_par();
    SERIAL_PORT.println("Done");
    delay(3000);
  }
}

void appLoop()
{
  if (startupDelayfunc())
    return;

  // LED Anzeige

  if (get_5V_Error())
  {
    initADCFlag = false;
  }

  processErrorHandling(); // PRIO 1
  processSysFailure();    // PRIO 1

#ifdef BinInputs
  processReadInputs(); // PRIO 1
#endif
#ifdef S0Inputs
  processReadS0Input();
#endif
#ifdef ImplInput
  processReadImpulseInput(); // PRIO 1
#endif

  switch (StateM)
  {
  case Pos1:
#ifdef ADC_enable
    if (processADConversation() && initADCFlag == false)
    {
      SERIAL_PORT.println("--> ADC ready <--"); // PRIO 3
      initADCFlag = true;
    }
#endif
    processInput_ADC(initADCFlag);
    StateM = Pos2;
    break;
  case Pos2:
    processInput_4_20mA(initADCFlag);
    StateM = Pos3;
    break;
  case Pos3:
    processInput_BIN();
    StateM = Pos4;
    break;
  case Pos4:
    processInputImpulse();
    StateM = Pos5;
    break;
  case Pos5:
    processVentil();     // PRIO 3
    processRelais();     // PRIO 3
    process_5V_Relais(); // PRIO 3
    StateM = Pos1;
    break;
  default:
    break;
  }

  // Hier warten bis die Funktionen davor schon ein paar mal ausgeführt wurden !!!!

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
    if (get_5V_Error())
    {
      setLED_24VAC(true);
    }
    else
    {
      setLED_24VAC(false);
    }
    /*
    SERIAL_PORT.print("Ventil1: ");
    SERIAL_PORT.println(get_Ventil_StateOld(0));
    SERIAL_PORT.print("Relais1: ");
    SERIAL_PORT.println(get_Relais_StateOld(0));
*/
    if (getError())
    {

#ifdef SerialError
      SERIAL_PORT.println(getError());
      SERIAL_PORT.print("Ventil1: ");
      SERIAL_PORT.println(get_Ventil_StateOld(0));
      SERIAL_PORT.println(get_5V_Error());
      SERIAL_PORT.println(get_12V_Error());
      SERIAL_PORT.println(get_24V_Error());
      SERIAL_PORT.println(get_5V_out_Error());
#endif
    }
#ifdef ADC_enable_Output
    SERIAL_PORT.print("ADC CH1: ");
    SERIAL_PORT.println(getSensorValue(0));
    SERIAL_PORT.print("ADC CH2: ");
    SERIAL_PORT.println(getSensorValue(1));
    SERIAL_PORT.print("ADC CH3: ");
    SERIAL_PORT.println(getSensorValue(2));
    SERIAL_PORT.print("VCC_12V: ");
    switch (get_HW_ID())
    {
    case HW_1_0:
      SERIAL_PORT.println(getAdcVoltage_12V());
      break;
    case HW_2_0:
    case HW_2_1:
      SERIAL_PORT.println("NA");
      break;
    }
    SERIAL_PORT.print("VCC_24V: ");
    SERIAL_PORT.println(getAdcVoltage_24V());
    /*
    SERIAL_PORT.print("ADC CH5: ");
    SERIAL_PORT.println(get4_20mA_CH1());
    SERIAL_PORT.print("ADC CH6: ");
    SERIAL_PORT.println(get4_20mA_CH2());
    */
    SERIAL_PORT.println(" ");
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

  if (DEBUG && delayCheck(LED_Delay[0], 200))
  {
    TestLEDstate[0] = !TestLEDstate[0];
    digitalWrite(get_PROG_LED_PIN(), TestLEDstate[0]);
    LED_Delay[0] = millis();
  }

  ProcessHeartbeat();
  ProcessReadRequests();
  gLogic.loop();
}