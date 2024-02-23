#include "HelperFunc.h"
#include "knxprod.h"
#include <Arduino.h>
#include <Wire.h>
// #include <knx.h>

#include "Device_setup.h"
#include "ErrorHandling.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#include "InputADC.h"
#include "Input_4_20mA.h"
#include "Input_BIN.h"
#include "Input_Impulse.h"
#include "Input_S0.h"
#include "KnxHelper.h"
#include "LED_Statusanzeige.h"
#include "ReadADC.h"
#include "ReadBinary.h"
#include "SystemFailureHandling.h"
#include "handleVentilRelais.h"

uint32_t READ_ADC_Delay = 0;
uint32_t Output_Delay = 0;
uint32_t READ_PRINT = 0;
uint32_t TestDelay = 0;
uint32_t LED_Delay2 = 0;
uint32_t LED_Delay = 0;

bool initADCFlag = false;
bool TestState = false;
bool TestLEDstate = false;
bool TestLEDstate2 = false;

enum StateMaschine
{
    Pos1 = 1,
    Pos2 = 2,
    Pos3 = 3,
    Pos4 = 4,
    Pos5 = 5,
};

StateMaschine StateM = Pos1;

GardenControlDevice openknxGardenControlModule;

const std::string GardenControlDevice::name()
{
    return "GardenControl";
}

const std::string GardenControlDevice::version()
{
    return "0.0"; // MODULE_GardenControlModule_Version;
}

GardenControlDevice::GardenControlDevice()
{
}

GardenControlDevice::~GardenControlDevice()
{
}

void GardenControlDevice::waitStartupLoop()
{
    knx.loop();
    ProcessReadRequests();
    processVentil();     // PRIO 3
    processRelais();     // PRIO 3
    process_5V_Relais(); // PRIO 3

    if (delayCheck(LED_Delay2, 700))
    {
        TestLEDstate2 = !TestLEDstate2;
        digitalWrite(get_PROG_LED_PIN(), TestLEDstate2);
        LED_Delay2 = millis();
        SERIAL_DEBUG.println("Wait for 5V");
    }
}

void GardenControlDevice::processInputKo(GroupObject &iKo)
{
#ifdef KNXcallback
    SERIAL_DEBUG.println(iKo.asap());
#endif

    if (iKo.asap() == BEM_Ko_Set_5V_relais)
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
                SERIAL_DEBUG.print("KO_Ventil_");
                SERIAL_DEBUG.print(ventil_Nr + 1);
                SERIAL_DEBUG.print(": ");
                SERIAL_DEBUG.println((bool)iKo.value(getDPT(VAL_DPT_1)));
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
                SERIAL_DEBUG.print("KO_Relais_");
                SERIAL_DEBUG.print(relais_Nr + 1);
                SERIAL_DEBUG.print(": ");
                SERIAL_DEBUG.println((bool)iKo.value(getDPT(VAL_DPT_1)));
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
    }
}

void GardenControlDevice::setup()
{
    // enable Main Relay
    SERIAL_DEBUG.print("enable Main Relay: ");
    SERIAL_DEBUG.println((knx.paramByte(BEM_ext5VRelaisStateBegin) >> BEM_ext5VRelaisStateBeginShift) & 1);
    SERIAL_DEBUG.println(knx.paramByte(BEM_ext5VRelaisStateBegin), BIN);
    control_5V_Relais((knx.paramByte(BEM_ext5VRelaisStateBegin) >> BEM_ext5VRelaisStateBeginShift) & 1);
    // wait so start Relay and Power Supply
    SERIAL_DEBUG.println("wait");
    delay(1000);
    // disable internal 5V to clear Errors
    SERIAL_DEBUG.println("stop 5V");
    digitalWrite(get_5V_EN_PIN(), HIGH);
    // wait until internal 5V voltage go to 0V
    SERIAL_DEBUG.println("wait");
    delay(1000);
    // restart 5V
    SERIAL_DEBUG.println("restart 5V");
    digitalWrite(get_5V_EN_PIN(), LOW);
    // wait until internal 5V powered up
    SERIAL_DEBUG.println("wait");
    // wait
    delay(500);

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
        SERIAL_DEBUG.println("Start init HW TOP + BOT");
        initHW_Top();
        read_HW_ID_BOT();
        print_HW_ID_BOT(get_HW_ID_BOT());
        initHW_Bot();

        // load ETS parameters
        SERIAL_DEBUG.println("Load Parameters");
        // init Inputs: Binäereingänge
        InitBinInput1(OptoIN_1); // Input 1
        InitBinInput2(OptoIN_2); // Input 2
        InitBinInput3(OptoIN_3); // Input 3
        InitBinInput4(OptoIN_4); // Input 4

        InitS0Input1();
        InitS0Input2();
        InitS0Input3();
        InitS0Input4();

        initInputADC();

        InitImpulseInputs();
        // load_ETS_par();
        SERIAL_DEBUG.println("Done");
        delay(3000);
    }
}

void GardenControlDevice::loop()
{
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
                SERIAL_DEBUG.println("--> ADC ready <--"); // PRIO 3
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
          SERIAL_DEBUG.print("--> Relais: ");
          SERIAL_DEBUG.println(TestState);
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
        SERIAL_DEBUG.print("Ventil1: ");
        SERIAL_DEBUG.println(get_Ventil_StateOld(0));
        SERIAL_DEBUG.print("Relais1: ");
        SERIAL_DEBUG.println(get_Relais_StateOld(0));
    */
        if (getError())
        {

#ifdef SerialError
            SERIAL_DEBUG.println(getError());
            SERIAL_DEBUG.print("Ventil1: ");
            SERIAL_DEBUG.println(get_Ventil_StateOld(0));
            SERIAL_DEBUG.println(get_5V_Error());
            SERIAL_DEBUG.println(get_12V_Error());
            SERIAL_DEBUG.println(get_24V_Error());
            SERIAL_DEBUG.println(get_5V_out_Error());
#endif
        }
#ifdef ADC_enable_Output
        SERIAL_DEBUG.print("ADC CH1: ");
        SERIAL_DEBUG.println(getSensorValue(0));
        SERIAL_DEBUG.print("ADC CH2: ");
        SERIAL_DEBUG.println(getSensorValue(1));
        SERIAL_DEBUG.print("ADC CH3: ");
        SERIAL_DEBUG.println(getSensorValue(2));
        SERIAL_DEBUG.print("VCC_12V: ");
        switch (get_HW_ID())
        {
            case HW_1_0:
                SERIAL_DEBUG.println(getAdcVoltage_12V());
                break;
            case HW_2_0:
                SERIAL_DEBUG.println("NA");
                break;
        }
        SERIAL_DEBUG.print("VCC_24V: ");
        SERIAL_DEBUG.println(getAdcVoltage_24V());
        /*
        SERIAL_DEBUG.print("ADC CH5: ");
        SERIAL_DEBUG.println(get4_20mA_CH1());
        SERIAL_DEBUG.print("ADC CH6: ");
        SERIAL_DEBUG.println(get4_20mA_CH2());
        */
        SERIAL_DEBUG.println(" ");
#endif

#ifdef BinInputs_Output
        SERIAL_DEBUG.print("BIN CH1: ");
        SERIAL_DEBUG.println(getStateInput1());
        SERIAL_DEBUG.print("BIN CH2: ");
        SERIAL_DEBUG.println(getStateInput2());
        SERIAL_DEBUG.print("BIN CH3: ");
        SERIAL_DEBUG.println(getStateInput3());
        SERIAL_DEBUG.print("BIN CH4: ");
        SERIAL_DEBUG.println(getStateInput4());
#endif

#ifdef ImplInput_Output
        SERIAL_DEBUG.print("Impl: ");
        SERIAL_DEBUG.println(getFlowValue());
#endif

#ifdef Opto_IN_Output
        SERIAL_DEBUG.print("Opto CH1: ");
        SERIAL_DEBUG.println(digitalRead(OptoIN_1));
        SERIAL_DEBUG.print("Opto CH2: ");
        SERIAL_DEBUG.println(digitalRead(OptoIN_2));
        SERIAL_DEBUG.print("Opto CH3: ");
        SERIAL_DEBUG.println(digitalRead(OptoIN_3));
        SERIAL_DEBUG.print("Opto CH4: ");
        SERIAL_DEBUG.println(digitalRead(OptoIN_4));
#endif

        Output_Delay = millis();
    }

    if (delayCheck(LED_Delay, 200))
    {
        TestLEDstate = !TestLEDstate;
        digitalWrite(get_PROG_LED_PIN(), TestLEDstate);
        LED_Delay = millis();
    }
}