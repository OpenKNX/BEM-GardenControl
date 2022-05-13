#include <Arduino.h>
#include "Input_S0.h"
#include "BEM_hardware.h"
#include "KnxHelper.h"
#include "Helper.h"
#include "HelperFunc.h"
#include "GardenControl.h"
#include "ErrorHandling.h"

#define BIN_Input_S0 2

//-------- Test only --------------------
//#define Input_S0_Output_Int
unsigned long time_S0_LED_Blink[BIN_ChannelCount] = {0};
typedef enum
{
    cycle = 1,
    calc_Zaehler = 2,
    set_Mom_0 = 3,
    set_KO_value_Zaehler = 4,
    set_KO_value_Verbrauch = 5,
    send_KO = 6,
} stateMaschine;

stateMaschine State[2] = {cycle, cycle};

uint8_t channel_S0 = 0;

//-------- S0 variable -------------------
bool lSendZaehlerWert_S0[BIN_ChannelCount] = {false};
bool lSendMomLeistung_S0[BIN_ChannelCount] = {false};
bool impuls_S0[BIN_ChannelCount] = {0};
bool startinit = false;
uint32_t sendDelay_S0[BIN_ChannelCount] = {0};
uint32_t minSendDelay_S0[BIN_ChannelCount] = {0};
uint32_t sendDelayCon_S0[BIN_ChannelCount] = {0};
uint32_t minSendDelayCon_S0[BIN_ChannelCount] = {0};
float mom_S0[BIN_ChannelCount] = {1};
float mom_S0_old[BIN_ChannelCount] = {10};

uint16_t S0_Zaehler[BIN_ChannelCount] = {1, 1};
uint16_t S0_Zaehler_old[BIN_ChannelCount] = {0, 0};
uint16_t S0_impuls[BIN_ChannelCount];
unsigned long time_S0_start[BIN_ChannelCount] = {0};
unsigned long time_S0_stopp[BIN_ChannelCount] = {0};
uint16_t zaehler_Impulse[BIN_ChannelCount] = {5, 5};
uint32_t abs_S0[BIN_ChannelCount] = {0};

// bool S01_impuls = false;
// bool S02_impuls = false;

void interrupt_S0_1()
{
    time_S0_stopp[0] = millis();
    impuls_S0[0] = true;

#ifdef Input_S0_Output_Int
    SERIAL_PORT.print("S0_1: ");
    SERIAL_PORT.println(S0_impuls[0]);
#endif
}

void interrupt_S0_2()
{
    time_S0_stopp[1] = millis();
    impuls_S0[1] = true;
#ifdef Input_S0_Output_Int
    SERIAL_PORT.print("S0_2: ");
    SERIAL_PORT.println(S0_impuls[1]);
#endif
}

void interrupt_S0_3()
{
    time_S0_stopp[2] = millis();
    impuls_S0[2] = true;
#ifdef Input_S0_Output_Int
    SERIAL_PORT.print("S0_3: ");
    SERIAL_PORT.println(S0_impuls[2]);
#endif
}

void interrupt_S0_4()
{
    time_S0_stopp[3] = millis();
    impuls_S0[3] = true;
#ifdef Input_S0_Output_Int
    SERIAL_PORT.print("S0_4: ");
    SERIAL_PORT.println(S0_impuls[3]);
#endif
}

void InitS0Input1()
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 0)) == BIN_Input_S0)
    {
        pinMode(OptoIN_1, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(OptoIN_1), interrupt_S0_1, FALLING);
#ifdef Input_S0_Output
        SERIAL_PORT.println("   init S0_0: TRUE");
#endif
    }
}

void InitS0Input2()
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 1)) == BIN_Input_S0)
    {
        pinMode(OptoIN_2, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(OptoIN_2), interrupt_S0_2, FALLING);
#ifdef Input_S0_Output
        SERIAL_PORT.println("   init S0_1: TRUE");
#endif
    }
}

void InitS0Input3()
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 2)) == BIN_Input_S0)
    {
        pinMode(OptoIN_3, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(OptoIN_3), interrupt_S0_3, FALLING);
#ifdef Input_S0_Output
        SERIAL_PORT.println("   init S0_2: TRUE");
#endif
    }
}

void InitS0Input4()
{
    if (knx.paramByte(getParBIN(BIN_CHInputTypes3, 3)) == BIN_Input_S0)
    {
        pinMode(OptoIN_4, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(OptoIN_4), interrupt_S0_4, FALLING);
#ifdef Input_S0_Output
        SERIAL_PORT.println("   init S0_3: TRUE");
#endif
    }
}

void processReadS0Input()
{
    float mom_S0[BIN_ChannelCount];
    uint32_t maxPulsLength;

    uint32_t lCycle;
    uint8_t lsendMode;
    uint8_t lsendModeCon;
    bool det_maxPuls = false;

    if (!get_5V_Error())
    {

        if (knx.paramByte(getParBIN(BIN_CHInputTypes3, channel_S0)) == BIN_Input_S0)
        {

            //----------------- Mom Verbrauch: Mindestleistung/durchfluss - Berechnung = 0(W/l/m3) -------------------
            // Berechnung max Pulsdauer für Mindestleistung/durchfluss
            // Dauer = 3600sek * Impulse / Mindestleistung

            maxPulsLength = 3600 * zaehler_Impulse[channel_S0] / knx.paramWord(getParBIN(BIN_CHDefineMinValueS0, channel_S0));
            // prüfen ob aktueller puls länger ist als Pulslänge min Leistung oder Durchfluss

            if (delayCheck(time_S0_start[channel_S0], maxPulsLength))
            {
                det_maxPuls = true;
                mom_S0[channel_S0] = 0;
#ifdef Input_S0_Output
                // SERIAL_PORT.println("max Puls");
#endif
            }
            else
            {
                det_maxPuls = false;
            }

            lsendMode = knx.paramByte(getParBIN(BIN_CHS0SendModeCounter, channel_S0));
            lsendModeCon = knx.paramByte(getParBIN(BIN_CHS0SendModeCon, channel_S0));

            // we waited enough, let's send the value
            if ((lsendMode == 2 || lsendMode == 3) && delayCheck(sendDelay_S0[channel_S0], knx.paramWord(getParBIN(BIN_CHSendDelayS0, channel_S0)) * 1000))
            {
                lSendZaehlerWert_S0[channel_S0] = true;
#ifdef Input_S0_Output
                SERIAL_PORT.println("Zyklisch Zähler");
#endif
            }
            // we waited enough, let's send the value
            else if ((lsendModeCon == 2 || lsendModeCon == 3) && delayCheck(sendDelayCon_S0[channel_S0], knx.paramWord(getParBIN(BIN_CHSendDelayConS0, channel_S0)) * 1000))
            {
                lSendMomLeistung_S0[channel_S0] = true;
#ifdef Input_S0_Output
                SERIAL_PORT.println("Zyklisch Verb");
#endif
            }

            if (impuls_S0[channel_S0] == true)
            {
#ifdef Debug_S0_LED
                digitalWrite(Diag_LED, true);
                time_S0_LED_Blink[1] = millis();
#endif

#ifdef Input_S0_Output
                SERIAL_PORT.print("S0_");
                SERIAL_PORT.print(channel_S0 + 1);
#endif

                //-------------------------------------- Zähler ----------------------------------------------------------
                S0_impuls[channel_S0]++;
                if (S0_impuls[channel_S0] >= zaehler_Impulse[channel_S0])
                {
                    S0_Zaehler[channel_S0]++;
                    // senden bei Wertänderung
                    if ((lsendMode == 1 || lsendMode == 3) && S0_Zaehler[channel_S0] - S0_Zaehler_old[channel_S0] >= knx.paramWord(getParBIN(BIN_CHSendminValuechangeS0, channel_S0)))
                    {
                        if (delayCheck(minSendDelay_S0[channel_S0], knx.paramWord(getParBIN(BIN_CHSendminValueDelayS0, channel_S0)) * 1000))
                        {
                            minSendDelay_S0[channel_S0] = millis();
                            lSendZaehlerWert_S0[channel_S0] = true;
                        }
                    }
                    // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                    knx.getGroupObject(getComBIN(BIN_KoS0_Ges_Verbrauch, channel_S0)).valueNoSend(S0_Zaehler[channel_S0], Dpt(13, 1)); // getDPT(VAL_DPT_13)); // from l/min to l/h
#ifdef Input_S0_Output
                    SERIAL_PORT.print(" Zähler: ");
                    SERIAL_PORT.print(S0_Zaehler[channel_S0]);
#endif
                    S0_impuls[channel_S0] = 0;
                }
                //-------------------------------------- Zähler ENDE -----------------------------------------------------

                //-------------------------------------- Mom Verbrauch ---------------------------------------------------
                if (!det_maxPuls) // nur berechnen wenn Pulslänge kleiner max Pulslänge Mindestleistung/durchfluss
                {
                    ;
                    switch (knx.paramByte(getParBIN(BIN_CHDefineS0zaehler, channel_S0)))
                    {
                    case zaehlerElek:
                        // calculation mom Verbrauch (W)
                        mom_S0[channel_S0] = 3600.0 / ((time_S0_stopp[channel_S0] - time_S0_start[channel_S0]) / (zaehler_Impulse[channel_S0] * 1.0));

#ifdef Input_S0_Output
                        SERIAL_PORT.print(" Mom (W): ");
#endif
                        break;

                    case zaehlerWasser:

                        switch (knx.paramByte(getParBIN(BIN_CHDefineUnitS0, channel_S0)))
                        {
                        case unit_l:
                            // calculation mom Verbrauch (l/h)
                            mom_S0[channel_S0] = 3600.0 / ((time_S0_stopp[channel_S0] - time_S0_start[channel_S0]) / (zaehler_Impulse[channel_S0] * 1.0));
#ifdef Input_S0_Output
                            SERIAL_PORT.print(" Mom W (l/h): ");
#endif
                            break;
                        case unit_m3:
                            // calculation mom Verbrauch (m3/s)
                            mom_S0[channel_S0] = 1 / ((time_S0_stopp[channel_S0] - time_S0_start[channel_S0]) / (zaehler_Impulse[channel_S0] * 1000.0));
#ifdef Input_S0_Output
                            SERIAL_PORT.print(" Mom W (m3/s): ");
#endif
                            break;
                        default:
                            break;
                        }
                        break;

                    case zaehlerGas:
                        switch (knx.paramByte(getParBIN(BIN_CHDefineUnitS0, channel_S0)))
                        {
                        case unit_l:
                            // calculation mom Verbrauch (l/h)
                            mom_S0[channel_S0] = 3600.0 / ((time_S0_stopp[channel_S0] - time_S0_start[channel_S0]) / (zaehler_Impulse[channel_S0] * 1.0));
#ifdef Input_S0_Output
                            SERIAL_PORT.print(" Mom Gas (l/h): ");
#endif
                            break;
                        case unit_m3:
                            // calculation mom Verbrauch (m3/s)
                            mom_S0[channel_S0] = 1 / ((time_S0_stopp[channel_S0] - time_S0_start[channel_S0]) / (zaehler_Impulse[channel_S0] * 1000.0));
#ifdef Input_S0_Output
                            SERIAL_PORT.print(" Mom Gas (m3/s): ");
#endif
                            break;
                        default:
                            break;
                        }
                        break;

                    default:
                        break;
                    }
                } // ENDE IF (processCalc)
                else
                {
#ifdef Input_S0_Output
                    SERIAL_PORT.print(" Mom: ");
#endif
                    mom_S0[channel_S0] = 0;
                }
#ifdef Input_S0_Output
                SERIAL_PORT.print(mom_S0[channel_S0]);
                SERIAL_PORT.print(" ");
                SERIAL_PORT.print(mom_S0_old[channel_S0]);
                SERIAL_PORT.print(" ");
                SERIAL_PORT.print(abs(mom_S0[channel_S0] - mom_S0_old[channel_S0]));
                SERIAL_PORT.print(" ");
                SERIAL_PORT.println((time_S0_stopp[channel_S0] - time_S0_start[channel_S0]));
#endif
                time_S0_start[channel_S0] = time_S0_stopp[channel_S0];

                // senden bei Wertänderung
                if ((lsendModeCon == 1 || lsendModeCon == 3) && abs(mom_S0[channel_S0] - mom_S0_old[channel_S0]) >= knx.paramWord(getParBIN(BIN_CHSendminValuechangeConS0, channel_S0)))
                {
                    if (delayCheck(minSendDelayCon_S0[channel_S0], knx.paramWord(getParBIN(BIN_CHSendminValueDelayConS0, channel_S0)) * 1000))
                    {
                        minSendDelayCon_S0[channel_S0] = millis();
                        lSendMomLeistung_S0[channel_S0] = true;
#ifdef Input_S0_Output
                        SERIAL_PORT.println("Wert-Än Verb");
#endif
                    }
                }
                mom_S0_old[channel_S0] = mom_S0[channel_S0];

                switch (knx.paramByte(getParBIN(BIN_CHDefineS0zaehler, channel_S0)))
                {
                case zaehlerElek:
                    // we always store the new value in KO, even it it is not sent (to satisfy potential read request)

                    knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], Dpt(14, 1));        // getDPT(VAL_DPT_14)); // MOD_KoS01_ZaehlerWert+channel_S0 da KO nur 1 Byte auseinander liegen
                    mom_S0[channel_S0] = mom_S0[channel_S0] / 1000.0;                                                                      // Umrechnung in KW
                    knx.getGroupObject(getComBIN(BIN_KoS0_Akt2_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], getDPT(VAL_DPT_9)); // MOD_KoS01_ZaehlerWert+channel_S0 da KO nur 1 Byte auseinander liegen
                    break;
                case zaehlerWasser:
                    switch (knx.paramByte(getParBIN(BIN_CHDefineUnitS0, channel_S0)))
                    {
                    case unit_l:
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt2_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], getDPT(VAL_DPT_9)); // l/h
                        mom_S0[channel_S0] = mom_S0[channel_S0] / 60000.0;                                                                     // umrechnung in m3/s
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], Dpt(14, 1));        // getDPT(VAL_DPT_14)); // m3/s
                        break;
                    case unit_m3:
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], Dpt(14, 1));        // getDPT(VAL_DPT_14)); // m3/s
                        mom_S0[channel_S0] = mom_S0[channel_S0] * 60000.0;                                                                     // umrechnung in l/h
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], getDPT(VAL_DPT_9)); // l/h
                        break;
                        break;
                    default:
                        break;
                    }
                    break;

                case zaehlerGas:
                    switch (knx.paramByte(getParBIN(BIN_CHDefineUnitS0, channel_S0)))
                    {
                    case unit_l:
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt2_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], getDPT(VAL_DPT_9)); // l/h
                        mom_S0[channel_S0] = mom_S0[channel_S0] / 60000.0;                                                                     // umrechnung in m3/s
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], Dpt(14, 1));        // getDPT(VAL_DPT_14)); // m3/s
                        break;
                    case unit_m3:
                        // we always store the new value in KO, even it it is not sent (to satisfy potential read request)
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], Dpt(14, 1));        // getDPT(VAL_DPT_14)); // m3/s
                        mom_S0[channel_S0] = mom_S0[channel_S0] * 60000.0;                                                                     // umrechnung in l/h
                        knx.getGroupObject(getComBIN(BIN_KoS0_Akt2_Verbrauch, channel_S0)).valueNoSend(mom_S0[channel_S0], getDPT(VAL_DPT_9)); // l/h
                        break;
                        break;
                    default:
                        break;
                    }
                    break;

                default:
                    break;
                }

                //-------------------------------------- Mom Verbrauch ENDE -----------------------------------------------
                impuls_S0[channel_S0] = false;
            }

            // Send Zählerwerte
            if (lSendZaehlerWert_S0[channel_S0])
            {
#ifdef Input_S0_Output
                SERIAL_PORT.println("Send Zähler");
#endif

                knx.getGroupObject(getComBIN(BIN_KoS0_Ges_Verbrauch, channel_S0)).objectWritten();

                S0_Zaehler_old[channel_S0] = S0_Zaehler[channel_S0];
                sendDelay_S0[channel_S0] = millis();
                minSendDelay_S0[channel_S0] = millis();
                lSendZaehlerWert_S0[channel_S0] = false;
            }
            else if (lSendMomLeistung_S0[channel_S0])
            {
#ifdef Input_S0_Output
                SERIAL_PORT.println("Send Leistung");
#endif
                knx.getGroupObject(getComBIN(BIN_KoS0_Akt1_Verbrauch, channel_S0)).objectWritten(); // KW oder m3/s
                knx.getGroupObject(getComBIN(BIN_KoS0_Akt2_Verbrauch, channel_S0)).objectWritten(); // W oder l/h
                sendDelayCon_S0[channel_S0] = millis();
                minSendDelayCon_S0[channel_S0] = millis();
                lSendMomLeistung_S0[channel_S0] = false;
            }
        } // ENDI IF  CH = aktive

        channel_S0++;
        if (channel_S0 >= BIN_ChannelCount)
        {
            channel_S0 = 0;
        }
    } // ENDE 5V Fehler
}

uint16_t setZaehlerImpulse(uint8_t i, uint16_t impulse)
{
    zaehler_Impulse[i] = impulse;
    return zaehler_Impulse[i];
}

/*
void sendZaehlerStand(int i, uint16_t S0_Zaehler[], uint16_t S0_Zaehler_old[])
{
    if (S0_Zaehler[i] != S0_Zaehler_old[i])
    {
        S0_Zaehler_old[i] = S0_Zaehler[i];

        if (i == 0)
        {
            switch (knx.paramByte(MOD_DefineS0zaehler1))
            {
            case 1: // Elektrischer Zähler
                knx.getGroupObject(MOD_KoS01_Ges_Verbrauch).valueNoSend(S0_Zaehler[i], getDPT(VAL_DPT_14));
                break;
            case 2: // Wasser Zähler
                knx.getGroupObject(MOD_KoS01_Ges_Verbrauch).valueNoSend(S0_Zaehler[i], getDPT(VAL_DPT_13));
                break;
            case 3: // Gas Zähler
                knx.getGroupObject(MOD_KoS01_Ges_Verbrauch).valueNoSend(S0_Zaehler[i], getDPT(VAL_DPT_13));
                break;

            default:
                break;
            }
        }
        else if (i == 1)
        {
            switch (knx.paramByte(MOD_DefineS0zaehler2))
            {
            case 1: // Elektrischer Zähler
                knx.getGroupObject(MOD_KoS02_Ges_Verbrauch).valueNoSend(S0_Zaehler[i], getDPT(VAL_DPT_14));
                break;
            case 2: // Wasser Zähler
                knx.getGroupObject(MOD_KoS02_Ges_Verbrauch).valueNoSend(S0_Zaehler[i], getDPT(VAL_DPT_13));
                break;
            case 3: // Gas Zähler
                knx.getGroupObject(MOD_KoS02_Ges_Verbrauch).valueNoSend(S0_Zaehler[i], getDPT(VAL_DPT_13));
                break;

            default:
                break;
            }
        }
    }

#ifdef KDEBUG
    SERIAL_PORT.print("Zaehler");
    SERIAL_PORT.print(i);
    SERIAL_PORT.print(": ");
    SERIAL_PORT.println(S0_Zaehler[i]);
#endif
}
*/

void sendZaehlerStand_2(int i, uint16_t S0_Zaehler[], uint16_t S0_Zaehler_old[])
{
    // Knx.write(32 + (i * 3), S0_Zaehler[i]);
    // Knx.task();
    S0_Zaehler_old[i] = S0_Zaehler[i];
#ifdef KDEBUG
    SERIAL_PORT.print("Zaehler");
    SERIAL_PORT.print(i);
    SERIAL_PORT.print(": ");
    SERIAL_PORT.println(S0_Zaehler[i]);
#endif
}
