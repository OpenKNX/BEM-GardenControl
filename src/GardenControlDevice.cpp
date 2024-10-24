#include <Arduino.h>

#include "HelperFunc.h"
#include "OpenKNX.h"
#include <Wire.h>

#include "Device_setup.h"
#include "ErrorHandling.h"
#include "GardenControlDevice.h"
#include "I2C_IOExpander.h"
#ifdef ADC_enable
    #include "InputADC.h"
    #include "Input_4_20mA.h"
    #include "ReadADC.h"
#endif
#ifdef BinInputs
    #include "Input_BIN.h"
    #include "ReadBinary.h"
#endif
#ifdef ImplInput
    #include "Input_Impulse.h"
#endif
#ifdef S0Inputs
    #include "Input_S0.h"
#endif
#include "KnxHelper.h"
#include "LED_Statusanzeige.h"
#include "SystemFailureHandling.h"
#include "handleVentilRelais.h"

uint32_t READ_ADC_Delay = 0;
uint32_t Output_Delay = 0;
uint32_t READ_PRINT = 0;
uint32_t TestDelay = 0;
uint32_t LED_Delay2 = 0;
uint32_t LED_Delay = 0;

bool HWinit_Done = false;
bool TestState = false;
bool TestLEDstate = false;
bool TestLEDstate2 = false;

uint8_t adc_TOP_cycle_count = 0;
uint8_t adc_BOT_cycle_count = 0;

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

// TODO *****************************************************************************************************************************
void GardenControlDevice::waitStartupLoop()
{
    //  knx.loop();
    //  ProcessReadRequests();
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

void GardenControlDevice::initialHWinit()
{
    SERIAL_DEBUG.println("Start init HW TOP + BOT");
    initHW_Top();
    read_HW_ID_BOT();
    print_HW_ID_BOT(get_HW_ID_BOT());
    initHW_Bot();

    // load ETS parameters
    SERIAL_DEBUG.println("Load Parameters");
// init Inputs: Binäereingänge
#ifdef BinInputs
    InitBinInput1(OptoIN_1); // Input 1
    InitBinInput2(OptoIN_2); // Input 2
    InitBinInput3(OptoIN_3); // Input 3
    InitBinInput4(OptoIN_4); // Input 4
#endif
#ifdef S0Inputs
    InitS0Input1();
    InitS0Input2();
    InitS0Input3();
    InitS0Input4();
#endif
#ifdef ADC_enable
    initInputADC();
#endif
#ifdef ImplInput
    InitImpulseInputs();
#endif
    // load_ETS_par();
    SERIAL_DEBUG.println("Done");
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
    delay(200);
    setLED_OFF_ALL();

    // set KOs initial Ventil
    for (int i = 0; i < BEM_ChannelCount; i++)
    {
        knx.getGroupObject(BEM_KoOffset + (i * BEM_KoBlockSize + BEM_Ko_Status_ventil)).valueNoSend(false, getDPT(VAL_DPT_1));
    }
    // set KOs initial Relays
    for (int i = 0; i < REL_ChannelCount; i++)
    {
        knx.getGroupObject(REL_KoOffset + (i * REL_KoBlockSize + REL_Ko_Status_relais)).valueNoSend(false, getDPT(VAL_DPT_1));
    }
}

void GardenControlDevice::loop()
{
    // Abfrage ob die HW schon komplett initialisiert wurde. Das ist nur möglich, wenn auch die 24VAC(5V) anliegen
    if (!HWinit_Done && !digitalRead(get_5V_status_PIN()))
    {
        initialHWinit();
        HWinit_Done = true;
        digitalWrite(get_PROG_LED_PIN(), false);

        // Enable HW TOP
        init_IOExpander_GPIOs_TOP();
        set_IOExpander_TOP_Output(IO_5V_EN_V3, HIGH);
        set_IOExpander_TOP_Output(IO_12V_EN_V3, HIGH);
        set_IOExpander_TOP_Output(IO_24V_EN_V3, HIGH);
    }
    // HW ist komplett initialisiert --> ab hier beginnt die eigentliche Loop()
    else if (HWinit_Done)
    {

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
                if (processADConversation_TOP() && get_ADC_Ready_Flag_TOP() == false)
                {
                    adc_TOP_cycle_count++;
                    if (adc_TOP_cycle_count >= 5) // wait 5 times to set the flag -> to be sure that the ADC value is good
                    {
                        SERIAL_DEBUG.println("--> ADC TOP ready <--"); // PRIO 3
                        set_ADC_Ready_Flag_TOP();                      // Now all ADC CH have a new Value sampled
                        adc_TOP_cycle_count = 0;
                    }
                }
#endif
#ifdef ADC_enable
                if (processADConversation_BOT() && get_ADC_Ready_Flag_BOT() == false)
                {
                    adc_BOT_cycle_count++;
                    if (adc_BOT_cycle_count >= 5) // wait 5 times to set the flag -> to be sure that the ADC value is good
                    {
                        SERIAL_DEBUG.println("--> ADC BOT ready <--"); // PRIO 3
                        set_ADC_Ready_Flag_BOT();                      // Now all ADC CH have a new Value sampled
                        adc_BOT_cycle_count = 0;
                    }
                }
#endif
#ifdef ADC_enable
                processInput_ADC(get_ADC_Ready_Flag_TOP());
#endif
                StateM = Pos2;
                break;
            case Pos2:
#ifdef ADC_enable
                processInput_4_20mA(get_ADC_Ready_Flag_BOT());
#endif
                StateM = Pos3;
                break;
            case Pos3:
#ifdef BinInputs
                processInput_BIN();
#endif
                StateM = Pos4;
                break;
            case Pos4:
#ifdef ImplInput
                processInputImpulse();
#endif
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

        if (delayCheck(Output_Delay, 2007000000))
        {

            // only TEST enable 24V outputs for 4-20mA
            // set_IOExpander_BOT_Output_PCA9555(14, HIGH);
            // set_IOExpander_BOT_Output_PCA9555(15, HIGH);

#ifdef ErrorBits_Output
            SERIAL_DEBUG.println("------------------");
            SERIAL_DEBUG.print("--> Error 5V: ");
            SERIAL_DEBUG.println(get_5V_Error());
            SERIAL_DEBUG.print("--> Error 12V: ");
            SERIAL_DEBUG.println(get_12V_Error());
            SERIAL_DEBUG.print("--> Error 24V: ");
            SERIAL_DEBUG.println(get_24V_Error());
            SERIAL_DEBUG.print("--> Error 12/24V: ");
            SERIAL_DEBUG.println(get_12V_or_24V_Error());
            SERIAL_DEBUG.println("------------------");
#endif

#ifdef ADC_enable_Output
            SERIAL_DEBUG.print("ADC CH1: ");
            SERIAL_DEBUG.print(getAdcI2cValue_TOP(0));
            SERIAL_DEBUG.print(" Volt: ");
            SERIAL_DEBUG.println(getAdcVoltage_TOP(0), 3);
            SERIAL_DEBUG.print("ADC CH2: ");
            SERIAL_DEBUG.print(getAdcI2cValue_TOP(1));
            SERIAL_DEBUG.print(" Volt: ");
            SERIAL_DEBUG.println(getAdcVoltage_TOP(1), 3);
            SERIAL_DEBUG.print("ADC CH3: ");
            SERIAL_DEBUG.print(getAdcI2cValue_TOP(2));
            SERIAL_DEBUG.print(" Volt: ");
            SERIAL_DEBUG.println(getAdcVoltage_TOP(2), 3);
            SERIAL_DEBUG.print("ADC CH4: ");
            SERIAL_DEBUG.print(getAdcI2cValue_TOP(3));
            SERIAL_DEBUG.print(" Volt: ");
            SERIAL_DEBUG.println(getAdcVoltage_TOP(3), 3);
            SERIAL_DEBUG.println("------------------");
            SERIAL_DEBUG.print("4-20mA CH1: ");
            SERIAL_DEBUG.print(getAdcI2cValue_BOT(0));
            SERIAL_DEBUG.print(" Curr: ");
            SERIAL_DEBUG.println(getAdcVoltage_BOT(0));
            SERIAL_DEBUG.print("4-20mA CH2: ");
            SERIAL_DEBUG.print(getAdcI2cValue_BOT(1));
            SERIAL_DEBUG.print(" Curr: ");
            SERIAL_DEBUG.println(getAdcVoltage_BOT(1));
            SERIAL_DEBUG.println("------------------");
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
    } // END IF HW is init
    // IN der ELSE-Schleife kann man Funktionen aufrufen, solange die HW noch nicht komplettt initialisiert wurde und noch keine 24VAC(5V) anliegen
    else
    {
#ifdef ProgLedblinking1sek
        if (delayCheck(LED_Delay, 200))
        {
            TestLEDstate = !TestLEDstate;
            digitalWrite(get_PROG_LED_PIN(), TestLEDstate);
            LED_Delay = millis();
        }
#endif
    }
}