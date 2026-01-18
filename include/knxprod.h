#pragma once


#define paramDelay(time) (uint32_t)( \
            (time & 0xC000) == 0xC000 ? (time & 0x3FFF) * 100 : \
            (time & 0xC000) == 0x0000 ? (time & 0x3FFF) * 1000 : \
            (time & 0xC000) == 0x4000 ? (time & 0x3FFF) * 60000 : \
            (time & 0xC000) == 0x8000 ? ((time & 0x3FFF) > 1000 ? 3600000 : \
                                         (time & 0x3FFF) * 3600000 ) : 0 )
                                             
#define ETS_ModuleId_NONE 0
#define ETS_ModuleId_BASE 1
#define ETS_ModuleId_UCT 2
#define ETS_ModuleId_BEM 3
#define ETS_ModuleId_REL 4
#define ETS_ModuleId_ADC 5
#define ETS_ModuleId_CUR 6
#define ETS_ModuleId_BI 7
#define ETS_ModuleId_MTR 8
#define ETS_ModuleId_LOG 9
#define ETS_ModuleId_DFA 10
#define MAIN_FirmwareName "GardenControl"
#define MAIN_OpenKnxId 0xA2
#define MAIN_ApplicationNumber 16
#define MAIN_ApplicationVersion 16
#define MAIN_ApplicationEncoding iso-8859-15
#define MAIN_ParameterSize 15569
#define MAIN_MaxKoNumber 839
#define MAIN_OrderNumber "SmartMF-GardenControl"
#define BASE_ModuleVersion 21
#define UCT_ModuleVersion 4
#define BI_ModuleVersion 2
#define MTR_ModuleVersion 5
#define LOG_ModuleVersion 56
#define DFA_ModuleVersion 7
// Parameter with single occurrence


#define BASE_StartupDelayBase                     0      // 2 Bits, Bit 7-6
#define     BASE_StartupDelayBaseMask 0xC0
#define     BASE_StartupDelayBaseShift 6
#define BASE_StartupDelayTime                     0      // 14 Bits, Bit 13-0
#define     BASE_StartupDelayTimeMask 0x3FFF
#define     BASE_StartupDelayTimeShift 0
#define BASE_HeartbeatDelayBase                   2      // 2 Bits, Bit 7-6
#define     BASE_HeartbeatDelayBaseMask 0xC0
#define     BASE_HeartbeatDelayBaseShift 6
#define BASE_HeartbeatDelayTime                   2      // 14 Bits, Bit 13-0
#define     BASE_HeartbeatDelayTimeMask 0x3FFF
#define     BASE_HeartbeatDelayTimeShift 0
#define BASE_Timezone                             4      // 5 Bits, Bit 7-3
#define     BASE_TimezoneMask 0xF8
#define     BASE_TimezoneShift 3
#define BASE_CombinedTimeDate                     4      // 1 Bit, Bit 2
#define     BASE_CombinedTimeDateMask 0x04
#define     BASE_CombinedTimeDateShift 2
#define BASE_SummertimeAll                        4      // 2 Bits, Bit 1-0
#define     BASE_SummertimeAllMask 0x03
#define     BASE_SummertimeAllShift 0
#define BASE_SummertimeDE                         4      // 2 Bits, Bit 1-0
#define     BASE_SummertimeDEMask 0x03
#define     BASE_SummertimeDEShift 0
#define BASE_SummertimeWorld                      4      // 2 Bits, Bit 1-0
#define     BASE_SummertimeWorldMask 0x03
#define     BASE_SummertimeWorldShift 0
#define BASE_SummertimeKO                         4      // 2 Bits, Bit 1-0
#define     BASE_SummertimeKOMask 0x03
#define     BASE_SummertimeKOShift 0
#define BASE_TimezoneCustom                       5      // char*, 63 Byte
#define BASE_Latitude                            69      // float
#define BASE_Longitude                           73      // float
#define BASE_Diagnose                            78      // 1 Bit, Bit 7
#define     BASE_DiagnoseMask 0x80
#define     BASE_DiagnoseShift 7
#define BASE_Watchdog                            78      // 1 Bit, Bit 6
#define     BASE_WatchdogMask 0x40
#define     BASE_WatchdogShift 6
#define BASE_ReadTimeDate                        78      // 1 Bit, Bit 5
#define     BASE_ReadTimeDateMask 0x20
#define     BASE_ReadTimeDateShift 5
#define BASE_HeartbeatExtended                   78      // 1 Bit, Bit 4
#define     BASE_HeartbeatExtendedMask 0x10
#define     BASE_HeartbeatExtendedShift 4
#define BASE_InternalTime                        78      // 1 Bit, Bit 3
#define     BASE_InternalTimeMask 0x08
#define     BASE_InternalTimeShift 3
#define BASE_ManualSave                          78      // 3 Bits, Bit 2-0
#define     BASE_ManualSaveMask 0x07
#define     BASE_ManualSaveShift 0
#define BASE_PeriodicSave                        79      // 8 Bits, Bit 7-0
#define BASE_Dummy                               109      // uint8_t
#define BASE_ModuleEnabled_UCT                   110      // 1 Bit, Bit 6
#define     BASE_ModuleEnabled_UCTMask 0x40
#define     BASE_ModuleEnabled_UCTShift 6
#define BASE_ModuleEnabled_BEM                   110      // 1 Bit, Bit 5
#define     BASE_ModuleEnabled_BEMMask 0x20
#define     BASE_ModuleEnabled_BEMShift 5
#define BASE_ModuleEnabled_REL                   110      // 1 Bit, Bit 4
#define     BASE_ModuleEnabled_RELMask 0x10
#define     BASE_ModuleEnabled_RELShift 4
#define BASE_ModuleEnabled_ADC                   110      // 1 Bit, Bit 3
#define     BASE_ModuleEnabled_ADCMask 0x08
#define     BASE_ModuleEnabled_ADCShift 3
#define BASE_ModuleEnabled_CUR                   110      // 1 Bit, Bit 2
#define     BASE_ModuleEnabled_CURMask 0x04
#define     BASE_ModuleEnabled_CURShift 2
#define BASE_ModuleEnabled_BI                    110      // 1 Bit, Bit 1
#define     BASE_ModuleEnabled_BIMask 0x02
#define     BASE_ModuleEnabled_BIShift 1
#define BASE_ModuleEnabled_MTR                   110      // 1 Bit, Bit 0
#define     BASE_ModuleEnabled_MTRMask 0x01
#define     BASE_ModuleEnabled_MTRShift 0
#define BASE_ModuleEnabled_LOG                   111      // 1 Bit, Bit 7
#define     BASE_ModuleEnabled_LOGMask 0x80
#define     BASE_ModuleEnabled_LOGShift 7
#define BASE_ModuleEnabled_DFA                   111      // 1 Bit, Bit 6
#define     BASE_ModuleEnabled_DFAMask 0x40
#define     BASE_ModuleEnabled_DFAShift 6

// Zeitbasis
#define ParamBASE_StartupDelayBase                    ((knx.paramByte(BASE_StartupDelayBase) & BASE_StartupDelayBaseMask) >> BASE_StartupDelayBaseShift)
// Zeit
#define ParamBASE_StartupDelayTime                    (knx.paramWord(BASE_StartupDelayTime) & BASE_StartupDelayTimeMask)
// Zeit (in Millisekunden)
#define ParamBASE_StartupDelayTimeMS                  (paramDelay(knx.paramWord(BASE_StartupDelayTime)))
// Zeitbasis
#define ParamBASE_HeartbeatDelayBase                  ((knx.paramByte(BASE_HeartbeatDelayBase) & BASE_HeartbeatDelayBaseMask) >> BASE_HeartbeatDelayBaseShift)
// Zeit
#define ParamBASE_HeartbeatDelayTime                  (knx.paramWord(BASE_HeartbeatDelayTime) & BASE_HeartbeatDelayTimeMask)
// Zeit (in Millisekunden)
#define ParamBASE_HeartbeatDelayTimeMS                (paramDelay(knx.paramWord(BASE_HeartbeatDelayTime)))
// Zeitzone
#define ParamBASE_Timezone                            ((knx.paramByte(BASE_Timezone) & BASE_TimezoneMask) >> BASE_TimezoneShift)
// Empfangen über
#define ParamBASE_CombinedTimeDate                    ((bool)(knx.paramByte(BASE_CombinedTimeDate) & BASE_CombinedTimeDateMask))
// Sommerzeit ermitteln durch
#define ParamBASE_SummertimeAll                       (knx.paramByte(BASE_SummertimeAll) & BASE_SummertimeAllMask)
// Sommerzeit ermitteln durch
#define ParamBASE_SummertimeDE                        (knx.paramByte(BASE_SummertimeDE) & BASE_SummertimeDEMask)
// Sommerzeit ermitteln durch
#define ParamBASE_SummertimeWorld                     (knx.paramByte(BASE_SummertimeWorld) & BASE_SummertimeWorldMask)
// Sommerzeit ermitteln durch
#define ParamBASE_SummertimeKO                        (knx.paramByte(BASE_SummertimeKO) & BASE_SummertimeKOMask)
// POSIX TZ-String
#define ParamBASE_TimezoneCustom                      (knx.paramData(BASE_TimezoneCustom))
// Breitengrad
#define ParamBASE_Latitude                            (knx.paramFloat(BASE_Latitude, Float_Enc_IEEE754Single))
// Längengrad
#define ParamBASE_Longitude                           (knx.paramFloat(BASE_Longitude, Float_Enc_IEEE754Single))
// Diagnoseobjekt anzeigen
#define ParamBASE_Diagnose                            ((bool)(knx.paramByte(BASE_Diagnose) & BASE_DiagnoseMask))
// Watchdog aktivieren
#define ParamBASE_Watchdog                            ((bool)(knx.paramByte(BASE_Watchdog) & BASE_WatchdogMask))
// Bei Neustart vom Bus lesen
#define ParamBASE_ReadTimeDate                        ((bool)(knx.paramByte(BASE_ReadTimeDate) & BASE_ReadTimeDateMask))
// Erweitertes "In Betrieb"
#define ParamBASE_HeartbeatExtended                   ((bool)(knx.paramByte(BASE_HeartbeatExtended) & BASE_HeartbeatExtendedMask))
// InternalTime
#define ParamBASE_InternalTime                        ((bool)(knx.paramByte(BASE_InternalTime) & BASE_InternalTimeMask))
// Manuelles speichern
#define ParamBASE_ManualSave                          (knx.paramByte(BASE_ManualSave) & BASE_ManualSaveMask)
// Zyklisches speichern
#define ParamBASE_PeriodicSave                        (knx.paramByte(BASE_PeriodicSave))
// 
#define ParamBASE_Dummy                               (knx.paramByte(BASE_Dummy))
// UCT
#define ParamBASE_ModuleEnabled_UCT                   ((bool)(knx.paramByte(BASE_ModuleEnabled_UCT) & BASE_ModuleEnabled_UCTMask))
// BEM
#define ParamBASE_ModuleEnabled_BEM                   ((bool)(knx.paramByte(BASE_ModuleEnabled_BEM) & BASE_ModuleEnabled_BEMMask))
// REL
#define ParamBASE_ModuleEnabled_REL                   ((bool)(knx.paramByte(BASE_ModuleEnabled_REL) & BASE_ModuleEnabled_RELMask))
// ADC
#define ParamBASE_ModuleEnabled_ADC                   ((bool)(knx.paramByte(BASE_ModuleEnabled_ADC) & BASE_ModuleEnabled_ADCMask))
// CUR
#define ParamBASE_ModuleEnabled_CUR                   ((bool)(knx.paramByte(BASE_ModuleEnabled_CUR) & BASE_ModuleEnabled_CURMask))
// BI
#define ParamBASE_ModuleEnabled_BI                    ((bool)(knx.paramByte(BASE_ModuleEnabled_BI) & BASE_ModuleEnabled_BIMask))
// MTR
#define ParamBASE_ModuleEnabled_MTR                   ((bool)(knx.paramByte(BASE_ModuleEnabled_MTR) & BASE_ModuleEnabled_MTRMask))
// LOG
#define ParamBASE_ModuleEnabled_LOG                   ((bool)(knx.paramByte(BASE_ModuleEnabled_LOG) & BASE_ModuleEnabled_LOGMask))
// DFA
#define ParamBASE_ModuleEnabled_DFA                   ((bool)(knx.paramByte(BASE_ModuleEnabled_DFA) & BASE_ModuleEnabled_DFAMask))

#define BASE_KoHeartbeat 1
#define BASE_KoTime 2
#define BASE_KoDate 3
#define BASE_KoDateTime 4
#define BASE_KoIsSummertime 5
#define BASE_KoManualSave 6
#define BASE_KoDiagnose 7

// In Betrieb
#define KoBASE_Heartbeat                           (knx.getGroupObject(BASE_KoHeartbeat))
// Uhrzeit
#define KoBASE_Time                                (knx.getGroupObject(BASE_KoTime))
// Datum
#define KoBASE_Date                                (knx.getGroupObject(BASE_KoDate))
// Uhrzeit/Datum
#define KoBASE_DateTime                            (knx.getGroupObject(BASE_KoDateTime))
// Sommerzeit aktiv
#define KoBASE_IsSummertime                        (knx.getGroupObject(BASE_KoIsSummertime))
// Speichern
#define KoBASE_ManualSave                          (knx.getGroupObject(BASE_KoManualSave))
// Diagnose
#define KoBASE_Diagnose                            (knx.getGroupObject(BASE_KoDiagnose))



#define BEM_ext5VRelais                         114      // 1 Bit, Bit 7
#define     BEM_ext5VRelaisMask 0x80
#define     BEM_ext5VRelaisShift 7
#define BEM_ext5VRelaisStateBegin               114      // 1 Bit, Bit 6
#define     BEM_ext5VRelaisStateBeginMask 0x40
#define     BEM_ext5VRelaisStateBeginShift 6
#define BEM_ext5VRelaisStartState               114      // 1 Bit, Bit 5
#define     BEM_ext5VRelaisStartStateMask 0x20
#define     BEM_ext5VRelaisStartStateShift 5
#define BEM_Diag_KO_PWT_enable                  114      // 1 Bit, Bit 4
#define     BEM_Diag_KO_PWT_enableMask 0x10
#define     BEM_Diag_KO_PWT_enableShift 4
#define BEM_KOsStateSendStartup                 114      // 1 Bit, Bit 4
#define     BEM_KOsStateSendStartupMask 0x10
#define     BEM_KOsStateSendStartupShift 4

// externes +5V Relais vorhanden?
#define ParamBEM_ext5VRelais                         ((bool)(knx.paramByte(BEM_ext5VRelais) & BEM_ext5VRelaisMask))
// externes +5V Relais Zustand nach Startup
#define ParamBEM_ext5VRelaisStateBegin               ((bool)(knx.paramByte(BEM_ext5VRelaisStateBegin) & BEM_ext5VRelaisStateBeginMask))
// externes +5V Relais Status KO senden nach Startup
#define ParamBEM_ext5VRelaisStartState               ((bool)(knx.paramByte(BEM_ext5VRelaisStartState) & BEM_ext5VRelaisStartStateMask))
// Diagnose KO (Spannungen) aktivieren
#define ParamBEM_Diag_KO_PWT_enable                  ((bool)(knx.paramByte(BEM_Diag_KO_PWT_enable) & BEM_Diag_KO_PWT_enableMask))
// Status KOs senden nach Startup
#define ParamBEM_KOsStateSendStartup                 ((bool)(knx.paramByte(BEM_KOsStateSendStartup) & BEM_KOsStateSendStartupMask))

#define BEM_Ko_Set_5V_relais 21
#define BEM_Ko_Status_5V_relais 22
#define BEM_Ko_Diagnose_KO_PWR 23
#define BEM_Ko_Set_Magnetventil_Nr 24

// ext +5V Relais
#define KoBEM__Set_5V_relais                      (knx.getGroupObject(BEM_Ko_Set_5V_relais))
// ext +5V Relais
#define KoBEM__Status_5V_relais                   (knx.getGroupObject(BEM_Ko_Status_5V_relais))
// Power Rails
#define KoBEM__Diagnose_KO_PWR                    (knx.getGroupObject(BEM_Ko_Diagnose_KO_PWR))
// Magnetventil-Nr
#define KoBEM__Set_Magnetventil_Nr                (knx.getGroupObject(BEM_Ko_Set_Magnetventil_Nr))

#define BEM_ChannelCount 12

// Parameter per channel
#define BEM_ParamBlockOffset 115
#define BEM_ParamBlockSize 1
#define BEM_ParamCalcIndex(index) (index + BEM_ParamBlockOffset + _channelIndex * BEM_ParamBlockSize)

#define BEM_CHAktive                             0      // 1 Bit, Bit 7
#define     BEM_CHAktiveMask 0x80
#define     BEM_CHAktiveShift 7
#define BEM_CHSperr                              0      // 1 Bit, Bit 6
#define     BEM_CHSperrMask 0x40
#define     BEM_CHSperrShift 6

// Ventil %C%
#define ParamBEM_CHAktive                            ((bool)(knx.paramByte(BEM_ParamCalcIndex(BEM_CHAktive)) & BEM_CHAktiveMask))
// Sperrobjekt
#define ParamBEM_CHSperr                             ((bool)(knx.paramByte(BEM_ParamCalcIndex(BEM_CHSperr)) & BEM_CHSperrMask))

// deprecated
#define BEM_KoOffset 30

// Communication objects per channel (multiple occurrence)
#define BEM_KoBlockOffset 30
#define BEM_KoBlockSize 3

#define BEM_KoCalcNumber(index) (index + BEM_KoBlockOffset + _channelIndex * BEM_KoBlockSize)
#define BEM_KoCalcIndex(number) ((number >= BEM_KoCalcNumber(0) && number < BEM_KoCalcNumber(BEM_KoBlockSize)) ? (number - BEM_KoBlockOffset) % BEM_KoBlockSize : -1)
#define BEM_KoCalcChannel(number) ((number >= BEM_KoBlockOffset && number < BEM_KoBlockOffset + BEM_ChannelCount * BEM_KoBlockSize) ? (number - BEM_KoBlockOffset) / BEM_KoBlockSize : -1)

#define BEM_Ko_Set_ventil 0
#define BEM_Ko_Status_ventil 1
#define BEM_Ko_Sperr_ventil 2

// Magnetventil %C%:
#define KoBEM__Set_ventil                         (knx.getGroupObject(BEM_KoCalcNumber(BEM_Ko_Set_ventil)))
// Magnetventil %C%:
#define KoBEM__Status_ventil                      (knx.getGroupObject(BEM_KoCalcNumber(BEM_Ko_Status_ventil)))
// Magnetventil %C%:
#define KoBEM__Sperr_ventil                       (knx.getGroupObject(BEM_KoCalcNumber(BEM_Ko_Sperr_ventil)))

#define REL_ChannelCount 2

// Parameter per channel
#define REL_ParamBlockOffset 127
#define REL_ParamBlockSize 1
#define REL_ParamCalcIndex(index) (index + REL_ParamBlockOffset + _channelIndex * REL_ParamBlockSize)

#define REL_CHaktive                             0      // 1 Bit, Bit 7
#define     REL_CHaktiveMask 0x80
#define     REL_CHaktiveShift 7
#define REL_CHsperr                              0      // 1 Bit, Bit 6
#define     REL_CHsperrMask 0x40
#define     REL_CHsperrShift 6

// Kanal
#define ParamREL_CHaktive                            ((bool)(knx.paramByte(REL_ParamCalcIndex(REL_CHaktive)) & REL_CHaktiveMask))
// Sperrobjekt
#define ParamREL_CHsperr                             ((bool)(knx.paramByte(REL_ParamCalcIndex(REL_CHsperr)) & REL_CHsperrMask))

// deprecated
#define REL_KoOffset 66

// Communication objects per channel (multiple occurrence)
#define REL_KoBlockOffset 66
#define REL_KoBlockSize 3

#define REL_KoCalcNumber(index) (index + REL_KoBlockOffset + _channelIndex * REL_KoBlockSize)
#define REL_KoCalcIndex(number) ((number >= REL_KoCalcNumber(0) && number < REL_KoCalcNumber(REL_KoBlockSize)) ? (number - REL_KoBlockOffset) % REL_KoBlockSize : -1)
#define REL_KoCalcChannel(number) ((number >= REL_KoBlockOffset && number < REL_KoBlockOffset + REL_ChannelCount * REL_KoBlockSize) ? (number - REL_KoBlockOffset) / REL_KoBlockSize : -1)

#define REL_Ko_Set_relais 0
#define REL_Ko_Status_relais 1
#define REL_Ko_Sperr_relais 2

// Relais %C%:
#define KoREL__Set_relais                         (knx.getGroupObject(REL_KoCalcNumber(REL_Ko_Set_relais)))
// Relais %C%:
#define KoREL__Status_relais                      (knx.getGroupObject(REL_KoCalcNumber(REL_Ko_Status_relais)))
// Relais %C%:
#define KoREL__Sperr_relais                       (knx.getGroupObject(REL_KoCalcNumber(REL_Ko_Sperr_relais)))

#define ADC_ChannelCount 4

// Parameter per channel
#define ADC_ParamBlockOffset 129
#define ADC_ParamBlockSize 17
#define ADC_ParamCalcIndex(index) (index + ADC_ParamBlockOffset + _channelIndex * ADC_ParamBlockSize)

#define ADC_CHSensorType                         0      // 8 Bits, Bit 7-0
#define ADC_CHSendcycletime                      1      // int16_t
#define ADC_CHSendenAbsolut                      3      // int16_t
#define ADC_CHSendenRelativ                      5      // int8_t
#define ADC_CHValueFilter                        6      // int8_t
#define ADC_CHVoltageDiv                         7      // 1 Bit, Bit 7
#define     ADC_CHVoltageDivMask 0x80
#define     ADC_CHVoltageDivShift 7
#define ADC_CHVoltageCorrection                  8      // int16_t
#define ADC_CHSensorTypes                       10      // 8 Bits, Bit 7-0
#define ADC_CHGeradeM                           11      // int16_t
#define ADC_CHGeradeB                           13      // int16_t
#define ADC_CHSMT50DPTType                      15      // 8 Bits, Bit 7-0

// Sensortyp
#define ParamADC_CHSensorType                        (knx.paramByte(ADC_ParamCalcIndex(ADC_CHSensorType)))
// zyklisch senden(0 = nicht zyklisch senden)
#define ParamADC_CHSendcycletime                     ((int16_t)knx.paramWord(ADC_ParamCalcIndex(ADC_CHSendcycletime)))
// senden bei absoluter Abweichung(0 = nicht senden)
#define ParamADC_CHSendenAbsolut                     ((int16_t)knx.paramWord(ADC_ParamCalcIndex(ADC_CHSendenAbsolut)))
// senden bei relativer Abweichung(0 = nicht senden)
#define ParamADC_CHSendenRelativ                     ((int8_t)knx.paramByte(ADC_ParamCalcIndex(ADC_CHSendenRelativ)))
// Wert glätten: P =
#define ParamADC_CHValueFilter                       ((int8_t)knx.paramByte(ADC_ParamCalcIndex(ADC_CHValueFilter)))
// Eingangsspannungsbereich
#define ParamADC_CHVoltageDiv                        ((bool)(knx.paramByte(ADC_ParamCalcIndex(ADC_CHVoltageDiv)) & ADC_CHVoltageDivMask))
// Korrekturfaktor
#define ParamADC_CHVoltageCorrection                 ((int16_t)knx.paramWord(ADC_ParamCalcIndex(ADC_CHVoltageCorrection)))
// Meßwerteinheit (KO)
#define ParamADC_CHSensorTypes                       (knx.paramByte(ADC_ParamCalcIndex(ADC_CHSensorTypes)))
// Wert m
#define ParamADC_CHGeradeM                           ((int16_t)knx.paramWord(ADC_ParamCalcIndex(ADC_CHGeradeM)))
// Wert b
#define ParamADC_CHGeradeB                           ((int16_t)knx.paramWord(ADC_ParamCalcIndex(ADC_CHGeradeB)))
// Wähle DPT
#define ParamADC_CHSMT50DPTType                      (knx.paramByte(ADC_ParamCalcIndex(ADC_CHSMT50DPTType)))

// deprecated
#define ADC_KoOffset 75

// Communication objects per channel (multiple occurrence)
#define ADC_KoBlockOffset 75
#define ADC_KoBlockSize 1

#define ADC_KoCalcNumber(index) (index + ADC_KoBlockOffset + _channelIndex * ADC_KoBlockSize)
#define ADC_KoCalcIndex(number) ((number >= ADC_KoCalcNumber(0) && number < ADC_KoCalcNumber(ADC_KoBlockSize)) ? (number - ADC_KoBlockOffset) % ADC_KoBlockSize : -1)
#define ADC_KoCalcChannel(number) ((number >= ADC_KoBlockOffset && number < ADC_KoBlockOffset + ADC_ChannelCount * ADC_KoBlockSize) ? (number - ADC_KoBlockOffset) / ADC_KoBlockSize : -1)

#define ADC_KoGO_BASE__1 0

// GO_BASE_%C%_1
#define KoADC_GO_BASE__1                          (knx.getGroupObject(ADC_KoCalcNumber(ADC_KoGO_BASE__1)))

#define CUR_ChannelCount 2

// Parameter per channel
#define CUR_ParamBlockOffset 197
#define CUR_ParamBlockSize 12
#define CUR_ParamCalcIndex(index) (index + CUR_ParamBlockOffset + _channelIndex * CUR_ParamBlockSize)

#define CUR_CHSensorType2                        0      // 8 Bits, Bit 7-0
#define CUR_CHSendcycletime2                     1      // int16_t
#define CUR_CHSendenAbsolut2                     3      // int16_t
#define CUR_CHSendenRelativ2                     5      // int8_t
#define CUR_CHValueFilter2                       6      // int8_t
#define CUR_CHSensorTypes2                       7      // 8 Bits, Bit 7-0
#define CUR_CHPoint4mA                           8      // int16_t
#define CUR_CHPoint20mA                         10      // int16_t

// Sensortyp
#define ParamCUR_CHSensorType2                       (knx.paramByte(CUR_ParamCalcIndex(CUR_CHSensorType2)))
// zyklisch senden(0 = nicht zyklisch senden)
#define ParamCUR_CHSendcycletime2                    ((int16_t)knx.paramWord(CUR_ParamCalcIndex(CUR_CHSendcycletime2)))
// senden bei absoluter Abweichung(0 = nicht senden)
#define ParamCUR_CHSendenAbsolut2                    ((int16_t)knx.paramWord(CUR_ParamCalcIndex(CUR_CHSendenAbsolut2)))
// senden bei relativer Abweichung(0 = nicht senden)
#define ParamCUR_CHSendenRelativ2                    ((int8_t)knx.paramByte(CUR_ParamCalcIndex(CUR_CHSendenRelativ2)))
// Wert glätten: P =
#define ParamCUR_CHValueFilter2                      ((int8_t)knx.paramByte(CUR_ParamCalcIndex(CUR_CHValueFilter2)))
// Meßwerteinheit (KO)
#define ParamCUR_CHSensorTypes2                      (knx.paramByte(CUR_ParamCalcIndex(CUR_CHSensorTypes2)))
// Wert bei 4mA
#define ParamCUR_CHPoint4mA                          ((int16_t)knx.paramWord(CUR_ParamCalcIndex(CUR_CHPoint4mA)))
// Wert bei 20mA
#define ParamCUR_CHPoint20mA                         ((int16_t)knx.paramWord(CUR_ParamCalcIndex(CUR_CHPoint20mA)))

// deprecated
#define CUR_KoOffset 81

// Communication objects per channel (multiple occurrence)
#define CUR_KoBlockOffset 81
#define CUR_KoBlockSize 1

#define CUR_KoCalcNumber(index) (index + CUR_KoBlockOffset + _channelIndex * CUR_KoBlockSize)
#define CUR_KoCalcIndex(number) ((number >= CUR_KoCalcNumber(0) && number < CUR_KoCalcNumber(CUR_KoBlockSize)) ? (number - CUR_KoBlockOffset) % CUR_KoBlockSize : -1)
#define CUR_KoCalcChannel(number) ((number >= CUR_KoBlockOffset && number < CUR_KoBlockOffset + CUR_ChannelCount * CUR_KoBlockSize) ? (number - CUR_KoBlockOffset) / CUR_KoBlockSize : -1)

#define CUR_KoCUR_BASE__1 0

// CUR_BASE_%C%_1
#define KoCUR_CUR_BASE__1                         (knx.getGroupObject(CUR_KoCalcNumber(CUR_KoCUR_BASE__1)))



#define BI_ChannelCount 3

// Parameter per channel
#define BI_ParamBlockOffset 221
#define BI_ParamBlockSize 4
#define BI_ParamCalcIndex(index) (index + BI_ParamBlockOffset + _channelIndex * BI_ParamBlockSize)

#define BI_ChannelActive                        0      // 1 Bit, Bit 7
#define     BI_ChannelActiveMask 0x80
#define     BI_ChannelActiveShift 7
#define BI_ChannelOpen                          0      // 2 Bits, Bit 5-4
#define     BI_ChannelOpenMask 0x30
#define     BI_ChannelOpenShift 4
#define BI_ChannelClose                         0      // 2 Bits, Bit 3-2
#define     BI_ChannelCloseMask 0x0C
#define     BI_ChannelCloseShift 2
#define BI_ChannelPeriodic                      0      // 1 Bit, Bit 2
#define     BI_ChannelPeriodicMask 0x04
#define     BI_ChannelPeriodicShift 2
#define BI_ChannelDebouncing                    1      // 8 Bits, Bit 7-0
#define BI_ChannelPeriodicBase                  2      // 2 Bits, Bit 7-6
#define     BI_ChannelPeriodicBaseMask 0xC0
#define     BI_ChannelPeriodicBaseShift 6
#define BI_ChannelPeriodicTime                  2      // 14 Bits, Bit 13-0
#define     BI_ChannelPeriodicTimeMask 0x3FFF
#define     BI_ChannelPeriodicTimeShift 0

// Aktiv
#define ParamBI_ChannelActive                       ((bool)(knx.paramByte(BI_ParamCalcIndex(BI_ChannelActive)) & BI_ChannelActiveMask))
// Geöffnet
#define ParamBI_ChannelOpen                         ((knx.paramByte(BI_ParamCalcIndex(BI_ChannelOpen)) & BI_ChannelOpenMask) >> BI_ChannelOpenShift)
// Geschlossen
#define ParamBI_ChannelClose                        ((knx.paramByte(BI_ParamCalcIndex(BI_ChannelClose)) & BI_ChannelCloseMask) >> BI_ChannelCloseShift)
// Zyklisch senden
#define ParamBI_ChannelPeriodic                     ((bool)(knx.paramByte(BI_ParamCalcIndex(BI_ChannelPeriodic)) & BI_ChannelPeriodicMask))
// Entprellung
#define ParamBI_ChannelDebouncing                   (knx.paramByte(BI_ParamCalcIndex(BI_ChannelDebouncing)))
// Zeitbasis
#define ParamBI_ChannelPeriodicBase                 ((knx.paramByte(BI_ParamCalcIndex(BI_ChannelPeriodicBase)) & BI_ChannelPeriodicBaseMask) >> BI_ChannelPeriodicBaseShift)
// Zeit
#define ParamBI_ChannelPeriodicTime                 (knx.paramWord(BI_ParamCalcIndex(BI_ChannelPeriodicTime)) & BI_ChannelPeriodicTimeMask)
// Zeit (in Millisekunden)
#define ParamBI_ChannelPeriodicTimeMS               (paramDelay(knx.paramWord(BI_ParamCalcIndex(BI_ChannelPeriodicTime))))

// deprecated
#define BI_KoOffset 100

// Communication objects per channel (multiple occurrence)
#define BI_KoBlockOffset 100
#define BI_KoBlockSize 1

#define BI_KoCalcNumber(index) (index + BI_KoBlockOffset + _channelIndex * BI_KoBlockSize)
#define BI_KoCalcIndex(number) ((number >= BI_KoCalcNumber(0) && number < BI_KoCalcNumber(BI_KoBlockSize)) ? (number - BI_KoBlockOffset) % BI_KoBlockSize : -1)
#define BI_KoCalcChannel(number) ((number >= BI_KoBlockOffset && number < BI_KoBlockOffset + BI_ChannelCount * BI_KoBlockSize) ? (number - BI_KoBlockOffset) / BI_KoBlockSize : -1)

#define BI_KoChannelOutput 0

// 
#define KoBI_ChannelOutput                       (knx.getGroupObject(BI_KoCalcNumber(BI_KoChannelOutput)))

#define MTR_VisibleChannels                     233      // uint8_t

// Verfügbare Kanäle
#define ParamMTR_VisibleChannels                     (knx.paramByte(MTR_VisibleChannels))

#define MTR_ChannelCount 12

// Parameter per channel
#define MTR_ParamBlockOffset 234
#define MTR_ParamBlockSize 62
#define MTR_ParamCalcIndex(index) (index + MTR_ParamBlockOffset + _channelIndex * MTR_ParamBlockSize)

#define MTR_ChannelInModifier                    0      // float
#define MTR_ChannelOutModifier                   4      // float
#define MTR_ChannelInType                       24      // 4 Bits, Bit 7-4
#define     MTR_ChannelInTypeMask 0xF0
#define     MTR_ChannelInTypeShift 4
#define MTR_ChannelPulseType                    24      // 2 Bits, Bit 7-6
#define     MTR_ChannelPulseTypeMask 0xC0
#define     MTR_ChannelPulseTypeShift 6
#define MTR_ChannelOutType                      24      // 4 Bits, Bit 3-0
#define     MTR_ChannelOutTypeMask 0x0F
#define     MTR_ChannelOutTypeShift 0
#define MTR_ChannelDurationType                 24      // 4 Bits, Bit 3-0
#define     MTR_ChannelDurationTypeMask 0x0F
#define     MTR_ChannelDurationTypeShift 0
#define MTR_ChannelMode                         32      // 4 Bits, Bit 7-4
#define     MTR_ChannelModeMask 0xF0
#define     MTR_ChannelModeShift 4
#define MTR_ChannelLock                         32      // 2 Bits, Bit 3-2
#define     MTR_ChannelLockMask 0x0C
#define     MTR_ChannelLockShift 2
#define MTR_ChannelBackstop                     33      // 1 Bit, Bit 7
#define     MTR_ChannelBackstopMask 0x80
#define     MTR_ChannelBackstopShift 7
#define MTR_ChannelIgnoreZero                   33      // 1 Bit, Bit 6
#define     MTR_ChannelIgnoreZeroMask 0x40
#define     MTR_ChannelIgnoreZeroShift 6
#define MTR_ChannelInDistance                   48      // uint16_t
#define MTR_ChannelInPulses                     50      // uint16_t
#define MTR_ChannelInFallback                   52      // uint16_t
#define MTR_ChannelCalcWaitTime                 54      // uint16_t
#define MTR_ChannelCalcAbortTime                56      // uint16_t
#define MTR_ChannelSendOnChange                 58      // uint16_t
#define MTR_ChannelInSourceKo                   60      // 15 Bits, Bit 15-1
#define     MTR_ChannelInSourceKoMask 0xFFFE
#define     MTR_ChannelInSourceKoShift 1

// Multiplikator
#define ParamMTR_ChannelInModifier                   (knx.paramFloat(MTR_ParamCalcIndex(MTR_ChannelInModifier), Float_Enc_IEEE754Single))
// Multiplikator
#define ParamMTR_ChannelOutModifier                  (knx.paramFloat(MTR_ParamCalcIndex(MTR_ChannelOutModifier), Float_Enc_IEEE754Single))
// Datentyp
#define ParamMTR_ChannelInType                       ((knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelInType)) & MTR_ChannelInTypeMask) >> MTR_ChannelInTypeShift)
// Formel
#define ParamMTR_ChannelPulseType                    ((knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelPulseType)) & MTR_ChannelPulseTypeMask) >> MTR_ChannelPulseTypeShift)
// Datentyp
#define ParamMTR_ChannelOutType                      (knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelOutType)) & MTR_ChannelOutTypeMask)
// Datentyp
#define ParamMTR_ChannelDurationType                 (knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelDurationType)) & MTR_ChannelDurationTypeMask)
// Zählertyp
#define ParamMTR_ChannelMode                         ((knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelMode)) & MTR_ChannelModeMask) >> MTR_ChannelModeShift)
// Sperrobjekt
#define ParamMTR_ChannelLock                         ((knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelLock)) & MTR_ChannelLockMask) >> MTR_ChannelLockShift)
// Rücklaufsperre
#define ParamMTR_ChannelBackstop                     ((bool)(knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelBackstop)) & MTR_ChannelBackstopMask))
// Ignoriere 0
#define ParamMTR_ChannelIgnoreZero                   ((bool)(knx.paramByte(MTR_ParamCalcIndex(MTR_ChannelIgnoreZero)) & MTR_ChannelIgnoreZeroMask))
// Maximale  Differenz
#define ParamMTR_ChannelInDistance                   (knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelInDistance)))
// Impulse je Einheit
#define ParamMTR_ChannelInPulses                     (knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelInPulses)))
// Rückfallzeit
#define ParamMTR_ChannelInFallback                   (knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelInFallback)))
// Mindestwartezeit
#define ParamMTR_ChannelCalcWaitTime                 (knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelCalcWaitTime)))
// Abbruchzeit
#define ParamMTR_ChannelCalcAbortTime                (knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelCalcAbortTime)))
// Senden bei Änderung
#define ParamMTR_ChannelSendOnChange                 (knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelSendOnChange)))
// Bestehendes KO
#define ParamMTR_ChannelInSourceKo                   ((knx.paramWord(MTR_ParamCalcIndex(MTR_ChannelInSourceKo)) & MTR_ChannelInSourceKoMask) >> MTR_ChannelInSourceKoShift)

// deprecated
#define MTR_KoOffset 150

// Communication objects per channel (multiple occurrence)
#define MTR_KoBlockOffset 150
#define MTR_KoBlockSize 4

#define MTR_KoCalcNumber(index) (index + MTR_KoBlockOffset + _channelIndex * MTR_KoBlockSize)
#define MTR_KoCalcIndex(number) ((number >= MTR_KoCalcNumber(0) && number < MTR_KoCalcNumber(MTR_KoBlockSize)) ? (number - MTR_KoBlockOffset) % MTR_KoBlockSize : -1)
#define MTR_KoCalcChannel(number) ((number >= MTR_KoBlockOffset && number < MTR_KoBlockOffset + MTR_ChannelCount * MTR_KoBlockSize) ? (number - MTR_KoBlockOffset) / MTR_KoBlockSize : -1)

#define MTR_KoChannelInput 0
#define MTR_KoChannelOutput 1
#define MTR_KoChannelOptional 2
#define MTR_KoChannelReset 3

// 
#define KoMTR_ChannelInput                        (knx.getGroupObject(MTR_KoCalcNumber(MTR_KoChannelInput)))
// 
#define KoMTR_ChannelOutput                       (knx.getGroupObject(MTR_KoCalcNumber(MTR_KoChannelOutput)))
// 
#define KoMTR_ChannelOptional                     (knx.getGroupObject(MTR_KoCalcNumber(MTR_KoChannelOptional)))
// 
#define KoMTR_ChannelReset                        (knx.getGroupObject(MTR_KoCalcNumber(MTR_KoChannelReset)))

#define LOG_BuzzerInstalled                     978      // 1 Bit, Bit 7
#define     LOG_BuzzerInstalledMask 0x80
#define     LOG_BuzzerInstalledShift 7
#define LOG_LedInstalled                        978      // 1 Bit, Bit 6
#define     LOG_LedInstalledMask 0x40
#define     LOG_LedInstalledShift 6
#define LOG_VacationKo                          978      // 1 Bit, Bit 5
#define     LOG_VacationKoMask 0x20
#define     LOG_VacationKoShift 5
#define LOG_HolidayKo                           978      // 1 Bit, Bit 4
#define     LOG_HolidayKoMask 0x10
#define     LOG_HolidayKoShift 4
#define LOG_VacationRead                        978      // 1 Bit, Bit 3
#define     LOG_VacationReadMask 0x08
#define     LOG_VacationReadShift 3
#define LOG_HolidaySend                         978      // 1 Bit, Bit 2
#define     LOG_HolidaySendMask 0x04
#define     LOG_HolidaySendShift 2
#define LOG_Neujahr                             979      // 1 Bit, Bit 7
#define     LOG_NeujahrMask 0x80
#define     LOG_NeujahrShift 7
#define LOG_DreiKoenige                         979      // 1 Bit, Bit 6
#define     LOG_DreiKoenigeMask 0x40
#define     LOG_DreiKoenigeShift 6
#define LOG_Weiberfastnacht                     979      // 1 Bit, Bit 5
#define     LOG_WeiberfastnachtMask 0x20
#define     LOG_WeiberfastnachtShift 5
#define LOG_Rosenmontag                         979      // 1 Bit, Bit 4
#define     LOG_RosenmontagMask 0x10
#define     LOG_RosenmontagShift 4
#define LOG_Fastnachtsdienstag                  979      // 1 Bit, Bit 3
#define     LOG_FastnachtsdienstagMask 0x08
#define     LOG_FastnachtsdienstagShift 3
#define LOG_Aschermittwoch                      979      // 1 Bit, Bit 2
#define     LOG_AschermittwochMask 0x04
#define     LOG_AschermittwochShift 2
#define LOG_Frauentag                           979      // 1 Bit, Bit 1
#define     LOG_FrauentagMask 0x02
#define     LOG_FrauentagShift 1
#define LOG_Gruendonnerstag                     979      // 1 Bit, Bit 0
#define     LOG_GruendonnerstagMask 0x01
#define     LOG_GruendonnerstagShift 0
#define LOG_Karfreitag                          980      // 1 Bit, Bit 7
#define     LOG_KarfreitagMask 0x80
#define     LOG_KarfreitagShift 7
#define LOG_Ostersonntag                        980      // 1 Bit, Bit 6
#define     LOG_OstersonntagMask 0x40
#define     LOG_OstersonntagShift 6
#define LOG_Ostermontag                         980      // 1 Bit, Bit 5
#define     LOG_OstermontagMask 0x20
#define     LOG_OstermontagShift 5
#define LOG_TagDerArbeit                        980      // 1 Bit, Bit 4
#define     LOG_TagDerArbeitMask 0x10
#define     LOG_TagDerArbeitShift 4
#define LOG_Himmelfahrt                         980      // 1 Bit, Bit 3
#define     LOG_HimmelfahrtMask 0x08
#define     LOG_HimmelfahrtShift 3
#define LOG_Pfingstsonntag                      980      // 1 Bit, Bit 2
#define     LOG_PfingstsonntagMask 0x04
#define     LOG_PfingstsonntagShift 2
#define LOG_Pfingstmontag                       980      // 1 Bit, Bit 1
#define     LOG_PfingstmontagMask 0x02
#define     LOG_PfingstmontagShift 1
#define LOG_Fronleichnam                        980      // 1 Bit, Bit 0
#define     LOG_FronleichnamMask 0x01
#define     LOG_FronleichnamShift 0
#define LOG_Friedensfest                        981      // 1 Bit, Bit 7
#define     LOG_FriedensfestMask 0x80
#define     LOG_FriedensfestShift 7
#define LOG_MariaHimmelfahrt                    981      // 1 Bit, Bit 6
#define     LOG_MariaHimmelfahrtMask 0x40
#define     LOG_MariaHimmelfahrtShift 6
#define LOG_DeutscheEinheit                     981      // 1 Bit, Bit 5
#define     LOG_DeutscheEinheitMask 0x20
#define     LOG_DeutscheEinheitShift 5
#define LOG_Reformationstag                     981      // 1 Bit, Bit 4
#define     LOG_ReformationstagMask 0x10
#define     LOG_ReformationstagShift 4
#define LOG_Allerheiligen                       981      // 1 Bit, Bit 3
#define     LOG_AllerheiligenMask 0x08
#define     LOG_AllerheiligenShift 3
#define LOG_BussBettag                          981      // 1 Bit, Bit 2
#define     LOG_BussBettagMask 0x04
#define     LOG_BussBettagShift 2
#define LOG_Advent1                             981      // 1 Bit, Bit 1
#define     LOG_Advent1Mask 0x02
#define     LOG_Advent1Shift 1
#define LOG_Advent2                             981      // 1 Bit, Bit 0
#define     LOG_Advent2Mask 0x01
#define     LOG_Advent2Shift 0
#define LOG_Advent3                             982      // 1 Bit, Bit 7
#define     LOG_Advent3Mask 0x80
#define     LOG_Advent3Shift 7
#define LOG_Advent4                             982      // 1 Bit, Bit 6
#define     LOG_Advent4Mask 0x40
#define     LOG_Advent4Shift 6
#define LOG_Heiligabend                         982      // 1 Bit, Bit 5
#define     LOG_HeiligabendMask 0x20
#define     LOG_HeiligabendShift 5
#define LOG_Weihnachtstag1                      982      // 1 Bit, Bit 4
#define     LOG_Weihnachtstag1Mask 0x10
#define     LOG_Weihnachtstag1Shift 4
#define LOG_Weihnachtstag2                      982      // 1 Bit, Bit 3
#define     LOG_Weihnachtstag2Mask 0x08
#define     LOG_Weihnachtstag2Shift 3
#define LOG_Silvester                           982      // 1 Bit, Bit 2
#define     LOG_SilvesterMask 0x04
#define     LOG_SilvesterShift 2
#define LOG_Nationalfeiertag                    982      // 1 Bit, Bit 1
#define     LOG_NationalfeiertagMask 0x02
#define     LOG_NationalfeiertagShift 1
#define LOG_MariaEmpfaengnis                    982      // 1 Bit, Bit 0
#define     LOG_MariaEmpfaengnisMask 0x01
#define     LOG_MariaEmpfaengnisShift 0
#define LOG_NationalfeiertagSchweiz             983      // 1 Bit, Bit 7
#define     LOG_NationalfeiertagSchweizMask 0x80
#define     LOG_NationalfeiertagSchweizShift 7
#define LOG_Totensonntag                        983      // 1 Bit, Bit 6
#define     LOG_TotensonntagMask 0x40
#define     LOG_TotensonntagShift 6
#define LOG_Weltkindertag                       983      // 1 Bit, Bit 5
#define     LOG_WeltkindertagMask 0x20
#define     LOG_WeltkindertagShift 5
#define LOG_BuzzerSilent                        984      // uint16_t
#define LOG_BuzzerNormal                        986      // uint16_t
#define LOG_BuzzerLoud                          988      // uint16_t
#define LOG_VisibleChannels                     990      // uint8_t
#define LOG_LedMapping                          991      // 3 Bits, Bit 7-5
#define     LOG_LedMappingMask 0xE0
#define     LOG_LedMappingShift 5
#define LOG_UserFormula1                        992      // char*, 99 Byte
#define LOG_UserFormula1Active                  1091      // 1 Bit, Bit 7
#define     LOG_UserFormula1ActiveMask 0x80
#define     LOG_UserFormula1ActiveShift 7
#define LOG_UserFormula2                        1092      // char*, 99 Byte
#define LOG_UserFormula2Active                  1191      // 1 Bit, Bit 7
#define     LOG_UserFormula2ActiveMask 0x80
#define     LOG_UserFormula2ActiveShift 7
#define LOG_UserFormula3                        1192      // char*, 99 Byte
#define LOG_UserFormula3Active                  1291      // 1 Bit, Bit 7
#define     LOG_UserFormula3ActiveMask 0x80
#define     LOG_UserFormula3ActiveShift 7
#define LOG_UserFormula4                        1292      // char*, 99 Byte
#define LOG_UserFormula4Active                  1391      // 1 Bit, Bit 7
#define     LOG_UserFormula4ActiveMask 0x80
#define     LOG_UserFormula4ActiveShift 7
#define LOG_UserFormula5                        1392      // char*, 99 Byte
#define LOG_UserFormula5Active                  1491      // 1 Bit, Bit 7
#define     LOG_UserFormula5ActiveMask 0x80
#define     LOG_UserFormula5ActiveShift 7
#define LOG_UserFormula6                        1492      // char*, 99 Byte
#define LOG_UserFormula6Active                  1591      // 1 Bit, Bit 7
#define     LOG_UserFormula6ActiveMask 0x80
#define     LOG_UserFormula6ActiveShift 7
#define LOG_UserFormula7                        1592      // char*, 99 Byte
#define LOG_UserFormula7Active                  1691      // 1 Bit, Bit 7
#define     LOG_UserFormula7ActiveMask 0x80
#define     LOG_UserFormula7ActiveShift 7
#define LOG_UserFormula8                        1692      // char*, 99 Byte
#define LOG_UserFormula8Active                  1791      // 1 Bit, Bit 7
#define     LOG_UserFormula8ActiveMask 0x80
#define     LOG_UserFormula8ActiveShift 7
#define LOG_UserFormula9                        1792      // char*, 99 Byte
#define LOG_UserFormula9Active                  1891      // 1 Bit, Bit 7
#define     LOG_UserFormula9ActiveMask 0x80
#define     LOG_UserFormula9ActiveShift 7
#define LOG_UserFormula10                       1892      // char*, 99 Byte
#define LOG_UserFormula10Active                 1991      // 1 Bit, Bit 7
#define     LOG_UserFormula10ActiveMask 0x80
#define     LOG_UserFormula10ActiveShift 7
#define LOG_UserFormula11                       1992      // char*, 99 Byte
#define LOG_UserFormula11Active                 2091      // 1 Bit, Bit 7
#define     LOG_UserFormula11ActiveMask 0x80
#define     LOG_UserFormula11ActiveShift 7
#define LOG_UserFormula12                       2092      // char*, 99 Byte
#define LOG_UserFormula12Active                 2191      // 1 Bit, Bit 7
#define     LOG_UserFormula12ActiveMask 0x80
#define     LOG_UserFormula12ActiveShift 7
#define LOG_UserFormula13                       2192      // char*, 99 Byte
#define LOG_UserFormula13Active                 2291      // 1 Bit, Bit 7
#define     LOG_UserFormula13ActiveMask 0x80
#define     LOG_UserFormula13ActiveShift 7
#define LOG_UserFormula14                       2292      // char*, 99 Byte
#define LOG_UserFormula14Active                 2391      // 1 Bit, Bit 7
#define     LOG_UserFormula14ActiveMask 0x80
#define     LOG_UserFormula14ActiveShift 7
#define LOG_UserFormula15                       2392      // char*, 99 Byte
#define LOG_UserFormula15Active                 2491      // 1 Bit, Bit 7
#define     LOG_UserFormula15ActiveMask 0x80
#define     LOG_UserFormula15ActiveShift 7
#define LOG_UserFormula16                       2492      // char*, 99 Byte
#define LOG_UserFormula16Active                 2591      // 1 Bit, Bit 7
#define     LOG_UserFormula16ActiveMask 0x80
#define     LOG_UserFormula16ActiveShift 7
#define LOG_UserFormula17                       2592      // char*, 99 Byte
#define LOG_UserFormula17Active                 2691      // 1 Bit, Bit 7
#define     LOG_UserFormula17ActiveMask 0x80
#define     LOG_UserFormula17ActiveShift 7
#define LOG_UserFormula18                       2692      // char*, 99 Byte
#define LOG_UserFormula18Active                 2791      // 1 Bit, Bit 7
#define     LOG_UserFormula18ActiveMask 0x80
#define     LOG_UserFormula18ActiveShift 7
#define LOG_UserFormula19                       2792      // char*, 99 Byte
#define LOG_UserFormula19Active                 2891      // 1 Bit, Bit 7
#define     LOG_UserFormula19ActiveMask 0x80
#define     LOG_UserFormula19ActiveShift 7
#define LOG_UserFormula20                       2892      // char*, 99 Byte
#define LOG_UserFormula20Active                 2991      // 1 Bit, Bit 7
#define     LOG_UserFormula20ActiveMask 0x80
#define     LOG_UserFormula20ActiveShift 7
#define LOG_UserFormula21                       2992      // char*, 99 Byte
#define LOG_UserFormula21Active                 3091      // 1 Bit, Bit 7
#define     LOG_UserFormula21ActiveMask 0x80
#define     LOG_UserFormula21ActiveShift 7
#define LOG_UserFormula22                       3092      // char*, 99 Byte
#define LOG_UserFormula22Active                 3191      // 1 Bit, Bit 7
#define     LOG_UserFormula22ActiveMask 0x80
#define     LOG_UserFormula22ActiveShift 7
#define LOG_UserFormula23                       3192      // char*, 99 Byte
#define LOG_UserFormula23Active                 3291      // 1 Bit, Bit 7
#define     LOG_UserFormula23ActiveMask 0x80
#define     LOG_UserFormula23ActiveShift 7
#define LOG_UserFormula24                       3292      // char*, 99 Byte
#define LOG_UserFormula24Active                 3391      // 1 Bit, Bit 7
#define     LOG_UserFormula24ActiveMask 0x80
#define     LOG_UserFormula24ActiveShift 7
#define LOG_UserFormula25                       3392      // char*, 99 Byte
#define LOG_UserFormula25Active                 3491      // 1 Bit, Bit 7
#define     LOG_UserFormula25ActiveMask 0x80
#define     LOG_UserFormula25ActiveShift 7
#define LOG_UserFormula26                       3492      // char*, 99 Byte
#define LOG_UserFormula26Active                 3591      // 1 Bit, Bit 7
#define     LOG_UserFormula26ActiveMask 0x80
#define     LOG_UserFormula26ActiveShift 7
#define LOG_UserFormula27                       3592      // char*, 99 Byte
#define LOG_UserFormula27Active                 3691      // 1 Bit, Bit 7
#define     LOG_UserFormula27ActiveMask 0x80
#define     LOG_UserFormula27ActiveShift 7
#define LOG_UserFormula28                       3692      // char*, 99 Byte
#define LOG_UserFormula28Active                 3791      // 1 Bit, Bit 7
#define     LOG_UserFormula28ActiveMask 0x80
#define     LOG_UserFormula28ActiveShift 7
#define LOG_UserFormula29                       3792      // char*, 99 Byte
#define LOG_UserFormula29Active                 3891      // 1 Bit, Bit 7
#define     LOG_UserFormula29ActiveMask 0x80
#define     LOG_UserFormula29ActiveShift 7
#define LOG_UserFormula30                       3892      // char*, 99 Byte
#define LOG_UserFormula30Active                 3991      // 1 Bit, Bit 7
#define     LOG_UserFormula30ActiveMask 0x80
#define     LOG_UserFormula30ActiveShift 7

// Akustischer Signalgeber vorhanden (Buzzer)?
#define ParamLOG_BuzzerInstalled                     ((bool)(knx.paramByte(LOG_BuzzerInstalled) & LOG_BuzzerInstalledMask))
// Optischer Signalgeber vorhanden (RGB-LED)?
#define ParamLOG_LedInstalled                        ((bool)(knx.paramByte(LOG_LedInstalled) & LOG_LedInstalledMask))
// Urlaubsbehandlung aktivieren?
#define ParamLOG_VacationKo                          ((bool)(knx.paramByte(LOG_VacationKo) & LOG_VacationKoMask))
// Feiertage auf dem Bus verfügbar machen?
#define ParamLOG_HolidayKo                           ((bool)(knx.paramByte(LOG_HolidayKo) & LOG_HolidayKoMask))
// Nach Neustart Urlaubsinfo lesen?
#define ParamLOG_VacationRead                        ((bool)(knx.paramByte(LOG_VacationRead) & LOG_VacationReadMask))
// Nach Neuberechnung Feiertagsinfo senden?
#define ParamLOG_HolidaySend                         ((bool)(knx.paramByte(LOG_HolidaySend) & LOG_HolidaySendMask))
// 1. Neujahr
#define ParamLOG_Neujahr                             ((bool)(knx.paramByte(LOG_Neujahr) & LOG_NeujahrMask))
// 2. Heilige Drei Könige
#define ParamLOG_DreiKoenige                         ((bool)(knx.paramByte(LOG_DreiKoenige) & LOG_DreiKoenigeMask))
// 3. Weiberfastnacht
#define ParamLOG_Weiberfastnacht                     ((bool)(knx.paramByte(LOG_Weiberfastnacht) & LOG_WeiberfastnachtMask))
// 4. Rosenmontag
#define ParamLOG_Rosenmontag                         ((bool)(knx.paramByte(LOG_Rosenmontag) & LOG_RosenmontagMask))
// 5. Fastnachtsdienstag
#define ParamLOG_Fastnachtsdienstag                  ((bool)(knx.paramByte(LOG_Fastnachtsdienstag) & LOG_FastnachtsdienstagMask))
// 6. Aschermittwoch
#define ParamLOG_Aschermittwoch                      ((bool)(knx.paramByte(LOG_Aschermittwoch) & LOG_AschermittwochMask))
// 7. Frauentag
#define ParamLOG_Frauentag                           ((bool)(knx.paramByte(LOG_Frauentag) & LOG_FrauentagMask))
// 8. Gründonnerstag
#define ParamLOG_Gruendonnerstag                     ((bool)(knx.paramByte(LOG_Gruendonnerstag) & LOG_GruendonnerstagMask))
// 9. Karfreitag
#define ParamLOG_Karfreitag                          ((bool)(knx.paramByte(LOG_Karfreitag) & LOG_KarfreitagMask))
// 10. Ostersonntag
#define ParamLOG_Ostersonntag                        ((bool)(knx.paramByte(LOG_Ostersonntag) & LOG_OstersonntagMask))
// 11. Ostermontag
#define ParamLOG_Ostermontag                         ((bool)(knx.paramByte(LOG_Ostermontag) & LOG_OstermontagMask))
// 12. Tag der Arbeit
#define ParamLOG_TagDerArbeit                        ((bool)(knx.paramByte(LOG_TagDerArbeit) & LOG_TagDerArbeitMask))
// 13. Christi Himmelfahrt
#define ParamLOG_Himmelfahrt                         ((bool)(knx.paramByte(LOG_Himmelfahrt) & LOG_HimmelfahrtMask))
// 14. Pfingstsonntag
#define ParamLOG_Pfingstsonntag                      ((bool)(knx.paramByte(LOG_Pfingstsonntag) & LOG_PfingstsonntagMask))
// 15. Pfingstmontag
#define ParamLOG_Pfingstmontag                       ((bool)(knx.paramByte(LOG_Pfingstmontag) & LOG_PfingstmontagMask))
// 16. Fronleichnam
#define ParamLOG_Fronleichnam                        ((bool)(knx.paramByte(LOG_Fronleichnam) & LOG_FronleichnamMask))
// 17. Hohes Friedensfest
#define ParamLOG_Friedensfest                        ((bool)(knx.paramByte(LOG_Friedensfest) & LOG_FriedensfestMask))
// 18. Mariä Himmelfahrt
#define ParamLOG_MariaHimmelfahrt                    ((bool)(knx.paramByte(LOG_MariaHimmelfahrt) & LOG_MariaHimmelfahrtMask))
// 19. Tag der Deutschen Einheit
#define ParamLOG_DeutscheEinheit                     ((bool)(knx.paramByte(LOG_DeutscheEinheit) & LOG_DeutscheEinheitMask))
// 20. Reformationstag
#define ParamLOG_Reformationstag                     ((bool)(knx.paramByte(LOG_Reformationstag) & LOG_ReformationstagMask))
// 21. Allerheiligen
#define ParamLOG_Allerheiligen                       ((bool)(knx.paramByte(LOG_Allerheiligen) & LOG_AllerheiligenMask))
// 22. Buß- und Bettag
#define ParamLOG_BussBettag                          ((bool)(knx.paramByte(LOG_BussBettag) & LOG_BussBettagMask))
// 23. Erster Advent
#define ParamLOG_Advent1                             ((bool)(knx.paramByte(LOG_Advent1) & LOG_Advent1Mask))
// 24. Zweiter Advent
#define ParamLOG_Advent2                             ((bool)(knx.paramByte(LOG_Advent2) & LOG_Advent2Mask))
// 25. Dritter Advent
#define ParamLOG_Advent3                             ((bool)(knx.paramByte(LOG_Advent3) & LOG_Advent3Mask))
// 26. Vierter Advent
#define ParamLOG_Advent4                             ((bool)(knx.paramByte(LOG_Advent4) & LOG_Advent4Mask))
// 27. Heiligabend
#define ParamLOG_Heiligabend                         ((bool)(knx.paramByte(LOG_Heiligabend) & LOG_HeiligabendMask))
// 28. Erster Weihnachtstag
#define ParamLOG_Weihnachtstag1                      ((bool)(knx.paramByte(LOG_Weihnachtstag1) & LOG_Weihnachtstag1Mask))
// 29. Zweiter Weihnachtstag
#define ParamLOG_Weihnachtstag2                      ((bool)(knx.paramByte(LOG_Weihnachtstag2) & LOG_Weihnachtstag2Mask))
// 30. Silvester
#define ParamLOG_Silvester                           ((bool)(knx.paramByte(LOG_Silvester) & LOG_SilvesterMask))
// 31. Nationalfeiertag (AT)
#define ParamLOG_Nationalfeiertag                    ((bool)(knx.paramByte(LOG_Nationalfeiertag) & LOG_NationalfeiertagMask))
// 32. Maria Empfängnis (AT)
#define ParamLOG_MariaEmpfaengnis                    ((bool)(knx.paramByte(LOG_MariaEmpfaengnis) & LOG_MariaEmpfaengnisMask))
// 33. Nationalfeiertag (CH)
#define ParamLOG_NationalfeiertagSchweiz             ((bool)(knx.paramByte(LOG_NationalfeiertagSchweiz) & LOG_NationalfeiertagSchweizMask))
// 34. Totensonntag
#define ParamLOG_Totensonntag                        ((bool)(knx.paramByte(LOG_Totensonntag) & LOG_TotensonntagMask))
// 35. Weltkindertag
#define ParamLOG_Weltkindertag                       ((bool)(knx.paramByte(LOG_Weltkindertag) & LOG_WeltkindertagMask))
// Frequenz für Buzzer (leise)
#define ParamLOG_BuzzerSilent                        (knx.paramWord(LOG_BuzzerSilent))
// Frequenz für Buzzer (normal)
#define ParamLOG_BuzzerNormal                        (knx.paramWord(LOG_BuzzerNormal))
// Frequenz für Buzzer (laut)
#define ParamLOG_BuzzerLoud                          (knx.paramWord(LOG_BuzzerLoud))
// Verfügbare Kanäle
#define ParamLOG_VisibleChannels                     (knx.paramByte(LOG_VisibleChannels))
// Lötpad A / B / C entspricht
#define ParamLOG_LedMapping                          ((knx.paramByte(LOG_LedMapping) & LOG_LedMappingMask) >> LOG_LedMappingShift)
// Formeldefinition
#define ParamLOG_UserFormula1                        (knx.paramData(LOG_UserFormula1))
// Benutzerformel 1 aktiv
#define ParamLOG_UserFormula1Active                  ((bool)(knx.paramByte(LOG_UserFormula1Active) & LOG_UserFormula1ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula2                        (knx.paramData(LOG_UserFormula2))
// Benutzerformel 2 aktiv
#define ParamLOG_UserFormula2Active                  ((bool)(knx.paramByte(LOG_UserFormula2Active) & LOG_UserFormula2ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula3                        (knx.paramData(LOG_UserFormula3))
// Benutzerformel 3 aktiv
#define ParamLOG_UserFormula3Active                  ((bool)(knx.paramByte(LOG_UserFormula3Active) & LOG_UserFormula3ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula4                        (knx.paramData(LOG_UserFormula4))
// Benutzerformel 4 aktiv
#define ParamLOG_UserFormula4Active                  ((bool)(knx.paramByte(LOG_UserFormula4Active) & LOG_UserFormula4ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula5                        (knx.paramData(LOG_UserFormula5))
// Benutzerformel 5 aktiv
#define ParamLOG_UserFormula5Active                  ((bool)(knx.paramByte(LOG_UserFormula5Active) & LOG_UserFormula5ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula6                        (knx.paramData(LOG_UserFormula6))
// Benutzerformel 6 aktiv
#define ParamLOG_UserFormula6Active                  ((bool)(knx.paramByte(LOG_UserFormula6Active) & LOG_UserFormula6ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula7                        (knx.paramData(LOG_UserFormula7))
// Benutzerformel 7 aktiv
#define ParamLOG_UserFormula7Active                  ((bool)(knx.paramByte(LOG_UserFormula7Active) & LOG_UserFormula7ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula8                        (knx.paramData(LOG_UserFormula8))
// Benutzerformel 8 aktiv
#define ParamLOG_UserFormula8Active                  ((bool)(knx.paramByte(LOG_UserFormula8Active) & LOG_UserFormula8ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula9                        (knx.paramData(LOG_UserFormula9))
// Benutzerformel 9 aktiv
#define ParamLOG_UserFormula9Active                  ((bool)(knx.paramByte(LOG_UserFormula9Active) & LOG_UserFormula9ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula10                       (knx.paramData(LOG_UserFormula10))
// Benutzerformel 10 aktiv
#define ParamLOG_UserFormula10Active                 ((bool)(knx.paramByte(LOG_UserFormula10Active) & LOG_UserFormula10ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula11                       (knx.paramData(LOG_UserFormula11))
// Benutzerformel 11 aktiv
#define ParamLOG_UserFormula11Active                 ((bool)(knx.paramByte(LOG_UserFormula11Active) & LOG_UserFormula11ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula12                       (knx.paramData(LOG_UserFormula12))
// Benutzerformel 12 aktiv
#define ParamLOG_UserFormula12Active                 ((bool)(knx.paramByte(LOG_UserFormula12Active) & LOG_UserFormula12ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula13                       (knx.paramData(LOG_UserFormula13))
// Benutzerformel 13 aktiv
#define ParamLOG_UserFormula13Active                 ((bool)(knx.paramByte(LOG_UserFormula13Active) & LOG_UserFormula13ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula14                       (knx.paramData(LOG_UserFormula14))
// Benutzerformel 14 aktiv
#define ParamLOG_UserFormula14Active                 ((bool)(knx.paramByte(LOG_UserFormula14Active) & LOG_UserFormula14ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula15                       (knx.paramData(LOG_UserFormula15))
// Benutzerformel 15 aktiv
#define ParamLOG_UserFormula15Active                 ((bool)(knx.paramByte(LOG_UserFormula15Active) & LOG_UserFormula15ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula16                       (knx.paramData(LOG_UserFormula16))
// Benutzerformel 16 aktiv
#define ParamLOG_UserFormula16Active                 ((bool)(knx.paramByte(LOG_UserFormula16Active) & LOG_UserFormula16ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula17                       (knx.paramData(LOG_UserFormula17))
// Benutzerformel 17 aktiv
#define ParamLOG_UserFormula17Active                 ((bool)(knx.paramByte(LOG_UserFormula17Active) & LOG_UserFormula17ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula18                       (knx.paramData(LOG_UserFormula18))
// Benutzerformel 18 aktiv
#define ParamLOG_UserFormula18Active                 ((bool)(knx.paramByte(LOG_UserFormula18Active) & LOG_UserFormula18ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula19                       (knx.paramData(LOG_UserFormula19))
// Benutzerformel 19 aktiv
#define ParamLOG_UserFormula19Active                 ((bool)(knx.paramByte(LOG_UserFormula19Active) & LOG_UserFormula19ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula20                       (knx.paramData(LOG_UserFormula20))
// Benutzerformel 20 aktiv
#define ParamLOG_UserFormula20Active                 ((bool)(knx.paramByte(LOG_UserFormula20Active) & LOG_UserFormula20ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula21                       (knx.paramData(LOG_UserFormula21))
// Benutzerformel 21 aktiv
#define ParamLOG_UserFormula21Active                 ((bool)(knx.paramByte(LOG_UserFormula21Active) & LOG_UserFormula21ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula22                       (knx.paramData(LOG_UserFormula22))
// Benutzerformel 22 aktiv
#define ParamLOG_UserFormula22Active                 ((bool)(knx.paramByte(LOG_UserFormula22Active) & LOG_UserFormula22ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula23                       (knx.paramData(LOG_UserFormula23))
// Benutzerformel 23 aktiv
#define ParamLOG_UserFormula23Active                 ((bool)(knx.paramByte(LOG_UserFormula23Active) & LOG_UserFormula23ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula24                       (knx.paramData(LOG_UserFormula24))
// Benutzerformel 24 aktiv
#define ParamLOG_UserFormula24Active                 ((bool)(knx.paramByte(LOG_UserFormula24Active) & LOG_UserFormula24ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula25                       (knx.paramData(LOG_UserFormula25))
// Benutzerformel 25 aktiv
#define ParamLOG_UserFormula25Active                 ((bool)(knx.paramByte(LOG_UserFormula25Active) & LOG_UserFormula25ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula26                       (knx.paramData(LOG_UserFormula26))
// Benutzerformel 26 aktiv
#define ParamLOG_UserFormula26Active                 ((bool)(knx.paramByte(LOG_UserFormula26Active) & LOG_UserFormula26ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula27                       (knx.paramData(LOG_UserFormula27))
// Benutzerformel 27 aktiv
#define ParamLOG_UserFormula27Active                 ((bool)(knx.paramByte(LOG_UserFormula27Active) & LOG_UserFormula27ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula28                       (knx.paramData(LOG_UserFormula28))
// Benutzerformel 28 aktiv
#define ParamLOG_UserFormula28Active                 ((bool)(knx.paramByte(LOG_UserFormula28Active) & LOG_UserFormula28ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula29                       (knx.paramData(LOG_UserFormula29))
// Benutzerformel 29 aktiv
#define ParamLOG_UserFormula29Active                 ((bool)(knx.paramByte(LOG_UserFormula29Active) & LOG_UserFormula29ActiveMask))
// Formeldefinition
#define ParamLOG_UserFormula30                       (knx.paramData(LOG_UserFormula30))
// Benutzerformel 30 aktiv
#define ParamLOG_UserFormula30Active                 ((bool)(knx.paramByte(LOG_UserFormula30Active) & LOG_UserFormula30ActiveMask))

#define LOG_KoVacation 15
#define LOG_KoHoliday1 16
#define LOG_KoHoliday2 17
#define LOG_KoLedLock 18
#define LOG_KoBuzzerLock 19

// Urlaub
#define KoLOG_Vacation                            (knx.getGroupObject(LOG_KoVacation))
// Welcher Feiertag ist heute?
#define KoLOG_Holiday1                            (knx.getGroupObject(LOG_KoHoliday1))
// Welcher Feiertag ist morgen?
#define KoLOG_Holiday2                            (knx.getGroupObject(LOG_KoHoliday2))
// LED sperren
#define KoLOG_LedLock                             (knx.getGroupObject(LOG_KoLedLock))
// Buzzer sperren
#define KoLOG_BuzzerLock                          (knx.getGroupObject(LOG_KoBuzzerLock))

#define LOG_ChannelCount 99

// Parameter per channel
#define LOG_ParamBlockOffset 3992
#define LOG_ParamBlockSize 85
#define LOG_ParamCalcIndex(index) (index + LOG_ParamBlockOffset + _channelIndex * LOG_ParamBlockSize)

#define LOG_fChannelDelayBase                    0      // 2 Bits, Bit 7-6
#define     LOG_fChannelDelayBaseMask 0xC0
#define     LOG_fChannelDelayBaseShift 6
#define LOG_fChannelDelayTime                    0      // 14 Bits, Bit 13-0
#define     LOG_fChannelDelayTimeMask 0x3FFF
#define     LOG_fChannelDelayTimeShift 0
#define LOG_fLogic                               2      // 8 Bits, Bit 7-0
#define LOG_fCalculate                           3      // 2 Bits, Bit 1-0
#define     LOG_fCalculateMask 0x03
#define     LOG_fCalculateShift 0
#define LOG_fDisable                             3      // 1 Bit, Bit 2
#define     LOG_fDisableMask 0x04
#define     LOG_fDisableShift 2
#define LOG_fAlarm                               3      // 1 Bit, Bit 3
#define     LOG_fAlarmMask 0x08
#define     LOG_fAlarmShift 3
#define LOG_fTGate                               3      // 1 Bit, Bit 4
#define     LOG_fTGateMask 0x10
#define     LOG_fTGateShift 4
#define LOG_fOInternalOn                         3      // 1 Bit, Bit 5
#define     LOG_fOInternalOnMask 0x20
#define     LOG_fOInternalOnShift 5
#define LOG_fOInternalOff                        3      // 1 Bit, Bit 6
#define     LOG_fOInternalOffMask 0x40
#define     LOG_fOInternalOffShift 6
#define LOG_fTrigger                             4      // 8 Bits, Bit 7-0
#define LOG_fTriggerE1                           4      // 1 Bit, Bit 0
#define     LOG_fTriggerE1Mask 0x01
#define     LOG_fTriggerE1Shift 0
#define LOG_fTriggerE2                           4      // 1 Bit, Bit 1
#define     LOG_fTriggerE2Mask 0x02
#define     LOG_fTriggerE2Shift 1
#define LOG_fTriggerI1                           4      // 1 Bit, Bit 2
#define     LOG_fTriggerI1Mask 0x04
#define     LOG_fTriggerI1Shift 2
#define LOG_fTriggerI2                           4      // 1 Bit, Bit 3
#define     LOG_fTriggerI2Mask 0x08
#define     LOG_fTriggerI2Shift 3
#define LOG_fTriggerTime                         4      // 8 Bits, Bit 7-0
#define LOG_fTriggerGateClose                    5      // 2 Bits, Bit 7-6
#define     LOG_fTriggerGateCloseMask 0xC0
#define     LOG_fTriggerGateCloseShift 6
#define LOG_fTriggerGateOpen                     5      // 2 Bits, Bit 5-4
#define     LOG_fTriggerGateOpenMask 0x30
#define     LOG_fTriggerGateOpenShift 4
#define LOG_fE1ConvertInt                        6      // 4 Bits, Bit 7-4
#define     LOG_fE1ConvertIntMask 0xF0
#define     LOG_fE1ConvertIntShift 4
#define LOG_fE1Convert                           6      // 4 Bits, Bit 7-4
#define     LOG_fE1ConvertMask 0xF0
#define     LOG_fE1ConvertShift 4
#define LOG_fE1ConvertFloat                      6      // 4 Bits, Bit 7-4
#define     LOG_fE1ConvertFloatMask 0xF0
#define     LOG_fE1ConvertFloatShift 4
#define LOG_fE1ConvertSpecial                    6      // 4 Bits, Bit 7-4
#define     LOG_fE1ConvertSpecialMask 0xF0
#define     LOG_fE1ConvertSpecialShift 4
#define LOG_fE1ConvertBool                       6      // 4 Bits, Bit 7-4
#define     LOG_fE1ConvertBoolMask 0xF0
#define     LOG_fE1ConvertBoolShift 4
#define LOG_fE1                                  6      // 2 Bits, Bit 1-0
#define     LOG_fE1Mask 0x03
#define     LOG_fE1Shift 0
#define LOG_fE1Dpt                               7      // 8 Bits, Bit 7-0
#define LOG_fE1RepeatBase                        8      // 2 Bits, Bit 7-6
#define     LOG_fE1RepeatBaseMask 0xC0
#define     LOG_fE1RepeatBaseShift 6
#define LOG_fE1RepeatTime                        8      // 14 Bits, Bit 13-0
#define     LOG_fE1RepeatTimeMask 0x3FFF
#define     LOG_fE1RepeatTimeShift 0
#define LOG_fE1OtherKO                          10      // uint16_t
#define LOG_fE1OtherKORel                       10      // int16_t
#define LOG_fE1Default                          12      // 2 Bits, Bit 1-0
#define     LOG_fE1DefaultMask 0x03
#define     LOG_fE1DefaultShift 0
#define LOG_fE1DefaultExt                       12      // 2 Bits, Bit 1-0
#define     LOG_fE1DefaultExtMask 0x03
#define     LOG_fE1DefaultExtShift 0
#define LOG_fE1DefaultEEPROM                    12      // 1 Bit, Bit 2
#define     LOG_fE1DefaultEEPROMMask 0x04
#define     LOG_fE1DefaultEEPROMShift 2
#define LOG_fE1DefaultRepeat                    12      // 1 Bit, Bit 3
#define     LOG_fE1DefaultRepeatMask 0x08
#define     LOG_fE1DefaultRepeatShift 3
#define LOG_fE1UseOtherKO                       12      // 2 Bits, Bit 5-4
#define     LOG_fE1UseOtherKOMask 0x30
#define     LOG_fE1UseOtherKOShift 4
#define LOG_fE1LowDelta                         13      // int32_t
#define LOG_fE1HighDelta                        17      // int32_t
#define LOG_fE1LowDeltaFloat                    13      // float
#define LOG_fE1HighDeltaFloat                   17      // float
#define LOG_fE1LowDeltaDouble                   13      // float
#define LOG_fE1HighDeltaDouble                  17      // float
#define LOG_fE1Low0Valid                        20      // 1 Bit, Bit 7
#define     LOG_fE1Low0ValidMask 0x80
#define     LOG_fE1Low0ValidShift 7
#define LOG_fE1Low1Valid                        20      // 1 Bit, Bit 6
#define     LOG_fE1Low1ValidMask 0x40
#define     LOG_fE1Low1ValidShift 6
#define LOG_fE1Low2Valid                        20      // 1 Bit, Bit 5
#define     LOG_fE1Low2ValidMask 0x20
#define     LOG_fE1Low2ValidShift 5
#define LOG_fE1Low3Valid                        20      // 1 Bit, Bit 4
#define     LOG_fE1Low3ValidMask 0x10
#define     LOG_fE1Low3ValidShift 4
#define LOG_fE1Low4Valid                        20      // 1 Bit, Bit 3
#define     LOG_fE1Low4ValidMask 0x08
#define     LOG_fE1Low4ValidShift 3
#define LOG_fE1Low5Valid                        20      // 1 Bit, Bit 2
#define     LOG_fE1Low5ValidMask 0x04
#define     LOG_fE1Low5ValidShift 2
#define LOG_fE1Low6Valid                        20      // 1 Bit, Bit 1
#define     LOG_fE1Low6ValidMask 0x02
#define     LOG_fE1Low6ValidShift 1
#define LOG_fE1Low0Dpt2                         13      // 8 Bits, Bit 7-0
#define LOG_fE1Low1Dpt2                         14      // 8 Bits, Bit 7-0
#define LOG_fE1Low2Dpt2                         15      // 8 Bits, Bit 7-0
#define LOG_fE1Low3Dpt2                         16      // 8 Bits, Bit 7-0
#define LOG_fE1LowDpt2Fix                       13      // 8 Bits, Bit 7-0
#define LOG_fE1Low0Dpt3Dir                      13      // 5 Bits, Bit 7-3
#define     LOG_fE1Low0Dpt3DirMask 0xF8
#define     LOG_fE1Low0Dpt3DirShift 3
#define LOG_fE1Low0Dpt3Dim                      13      // 3 Bits, Bit 2-0
#define     LOG_fE1Low0Dpt3DimMask 0x07
#define     LOG_fE1Low0Dpt3DimShift 0
#define LOG_fE1Low1Dpt3Dir                      14      // 5 Bits, Bit 7-3
#define     LOG_fE1Low1Dpt3DirMask 0xF8
#define     LOG_fE1Low1Dpt3DirShift 3
#define LOG_fE1Low1Dpt3Dim                      14      // 3 Bits, Bit 2-0
#define     LOG_fE1Low1Dpt3DimMask 0x07
#define     LOG_fE1Low1Dpt3DimShift 0
#define LOG_fE1Low2Dpt3Dir                      15      // 5 Bits, Bit 7-3
#define     LOG_fE1Low2Dpt3DirMask 0xF8
#define     LOG_fE1Low2Dpt3DirShift 3
#define LOG_fE1Low2Dpt3Dim                      15      // 3 Bits, Bit 2-0
#define     LOG_fE1Low2Dpt3DimMask 0x07
#define     LOG_fE1Low2Dpt3DimShift 0
#define LOG_fE1Low3Dpt3Dir                      16      // 5 Bits, Bit 7-3
#define     LOG_fE1Low3Dpt3DirMask 0xF8
#define     LOG_fE1Low3Dpt3DirShift 3
#define LOG_fE1Low3Dpt3Dim                      16      // 3 Bits, Bit 2-0
#define     LOG_fE1Low3Dpt3DimMask 0x07
#define     LOG_fE1Low3Dpt3DimShift 0
#define LOG_fE1LowDpt3FixDir                    13      // 5 Bits, Bit 7-3
#define     LOG_fE1LowDpt3FixDirMask 0xF8
#define     LOG_fE1LowDpt3FixDirShift 3
#define LOG_fE1LowDpt3FixDim                    13      // 3 Bits, Bit 2-0
#define     LOG_fE1LowDpt3FixDimMask 0x07
#define     LOG_fE1LowDpt3FixDimShift 0
#define LOG_fE1LowDpt5                          13      // uint8_t
#define LOG_fE1HighDpt5                         17      // uint8_t
#define LOG_fE1Low0Dpt5In                       13      // uint8_t
#define LOG_fE1Low1Dpt5In                       14      // uint8_t
#define LOG_fE1Low2Dpt5In                       15      // uint8_t
#define LOG_fE1Low3Dpt5In                       16      // uint8_t
#define LOG_fE1Low4Dpt5In                       17      // uint8_t
#define LOG_fE1Low5Dpt5In                       18      // uint8_t
#define LOG_fE1Low6Dpt5In                       19      // uint8_t
#define LOG_fE1LowDpt5Fix                       13      // uint8_t
#define LOG_fE1LowDpt5001                       13      // uint8_t
#define LOG_fE1HighDpt5001                      17      // uint8_t
#define LOG_fE1Low0Dpt5xIn                      13      // uint8_t
#define LOG_fE1Low1Dpt5xIn                      14      // uint8_t
#define LOG_fE1Low2Dpt5xIn                      15      // uint8_t
#define LOG_fE1Low3Dpt5xIn                      16      // uint8_t
#define LOG_fE1Low4Dpt5xIn                      17      // uint8_t
#define LOG_fE1Low5Dpt5xIn                      18      // uint8_t
#define LOG_fE1Low6Dpt5xIn                      19      // uint8_t
#define LOG_fE1LowDpt5xFix                      13      // uint8_t
#define LOG_fE1LowDpt6                          13      // int8_t
#define LOG_fE1HighDpt6                         17      // int8_t
#define LOG_fE1Low0Dpt6In                       13      // int8_t
#define LOG_fE1Low1Dpt6In                       14      // int8_t
#define LOG_fE1Low2Dpt6In                       15      // int8_t
#define LOG_fE1Low3Dpt6In                       16      // int8_t
#define LOG_fE1Low4Dpt6In                       17      // int8_t
#define LOG_fE1Low5Dpt6In                       18      // int8_t
#define LOG_fE1Low6Dpt6In                       19      // int8_t
#define LOG_fE1LowDpt6Fix                       13      // int8_t
#define LOG_fE1LowDpt7                          13      // uint16_t
#define LOG_fE1HighDpt7                         17      // uint16_t
#define LOG_fE1Low0Dpt7In                       13      // uint16_t
#define LOG_fE1Low1Dpt7In                       15      // uint16_t
#define LOG_fE1Low2Dpt7In                       17      // uint16_t
#define LOG_fE1LowDpt7Fix                       13      // uint16_t
#define LOG_fE1LowDpt8                          13      // int16_t
#define LOG_fE1HighDpt8                         17      // int16_t
#define LOG_fE1Low0Dpt8In                       13      // int16_t
#define LOG_fE1Low1Dpt8In                       15      // int16_t
#define LOG_fE1Low2Dpt8In                       17      // int16_t
#define LOG_fE1LowDpt8Fix                       13      // int16_t
#define LOG_fE1LowDpt9                          13      // float
#define LOG_fE1HighDpt9                         17      // float
#define LOG_fE1LowDpt9Fix                       13      // float
#define LOG_fE1LowDpt12                         13      // uint32_t
#define LOG_fE1HighDpt12                        17      // uint32_t
#define LOG_fE1LowDpt12Fix                      13      // uint32_t
#define LOG_fE1LowDpt13                         13      // int32_t
#define LOG_fE1HighDpt13                        17      // int32_t
#define LOG_fE1LowDpt13Fix                      13      // int32_t
#define LOG_fE1LowDpt14                         13      // float
#define LOG_fE1HighDpt14                        17      // float
#define LOG_fE1LowDpt14Fix                      13      // float
#define LOG_fE1Low0Dpt17                        13      // 8 Bits, Bit 7-0
#define LOG_fE1Low1Dpt17                        14      // 8 Bits, Bit 7-0
#define LOG_fE1Low2Dpt17                        15      // 8 Bits, Bit 7-0
#define LOG_fE1Low3Dpt17                        16      // 8 Bits, Bit 7-0
#define LOG_fE1Low4Dpt17                        17      // 8 Bits, Bit 7-0
#define LOG_fE1Low5Dpt17                        18      // 8 Bits, Bit 7-0
#define LOG_fE1Low6Dpt17                        19      // 8 Bits, Bit 7-0
#define LOG_fE1Low7Dpt17                        20      // 8 Bits, Bit 7-0
#define LOG_fE1LowDpt17Fix                      13      // 8 Bits, Bit 7-0
#define LOG_fE1LowDptRGB                        13      // int32_t
#define LOG_fE1HighDptRGB                       17      // int32_t
#define LOG_fE1LowDptRGBFix                     13      // int32_t
#define LOG_fE2ConvertInt                       21      // 4 Bits, Bit 7-4
#define     LOG_fE2ConvertIntMask 0xF0
#define     LOG_fE2ConvertIntShift 4
#define LOG_fE2Convert                          21      // 4 Bits, Bit 7-4
#define     LOG_fE2ConvertMask 0xF0
#define     LOG_fE2ConvertShift 4
#define LOG_fE2ConvertFloat                     21      // 4 Bits, Bit 7-4
#define     LOG_fE2ConvertFloatMask 0xF0
#define     LOG_fE2ConvertFloatShift 4
#define LOG_fE2ConvertSpecial                   21      // 4 Bits, Bit 7-4
#define     LOG_fE2ConvertSpecialMask 0xF0
#define     LOG_fE2ConvertSpecialShift 4
#define LOG_fE2ConvertBool                      21      // 4 Bits, Bit 7-4
#define     LOG_fE2ConvertBoolMask 0xF0
#define     LOG_fE2ConvertBoolShift 4
#define LOG_fE2                                 21      // 2 Bits, Bit 1-0
#define     LOG_fE2Mask 0x03
#define     LOG_fE2Shift 0
#define LOG_fE2Dpt                              22      // 8 Bits, Bit 7-0
#define LOG_fE2RepeatBase                       23      // 2 Bits, Bit 7-6
#define     LOG_fE2RepeatBaseMask 0xC0
#define     LOG_fE2RepeatBaseShift 6
#define LOG_fE2RepeatTime                       23      // 14 Bits, Bit 13-0
#define     LOG_fE2RepeatTimeMask 0x3FFF
#define     LOG_fE2RepeatTimeShift 0
#define LOG_fE2OtherKO                          25      // uint16_t
#define LOG_fE2OtherKORel                       25      // int16_t
#define LOG_fE2Default                          27      // 2 Bits, Bit 1-0
#define     LOG_fE2DefaultMask 0x03
#define     LOG_fE2DefaultShift 0
#define LOG_fE2DefaultExt                       27      // 2 Bits, Bit 1-0
#define     LOG_fE2DefaultExtMask 0x03
#define     LOG_fE2DefaultExtShift 0
#define LOG_fE2DefaultEEPROM                    27      // 1 Bit, Bit 2
#define     LOG_fE2DefaultEEPROMMask 0x04
#define     LOG_fE2DefaultEEPROMShift 2
#define LOG_fE2DefaultRepeat                    27      // 1 Bit, Bit 3
#define     LOG_fE2DefaultRepeatMask 0x08
#define     LOG_fE2DefaultRepeatShift 3
#define LOG_fE2UseOtherKO                       27      // 2 Bits, Bit 5-4
#define     LOG_fE2UseOtherKOMask 0x30
#define     LOG_fE2UseOtherKOShift 4
#define LOG_fE2LowDelta                         28      // int32_t
#define LOG_fE2HighDelta                        32      // int32_t
#define LOG_fE2LowDeltaFloat                    28      // float
#define LOG_fE2HighDeltaFloat                   32      // float
#define LOG_fE2LowDeltaDouble                   28      // float
#define LOG_fE2HighDeltaDouble                  32      // float
#define LOG_fE2Low0Valid                        35      // 1 Bit, Bit 7
#define     LOG_fE2Low0ValidMask 0x80
#define     LOG_fE2Low0ValidShift 7
#define LOG_fE2Low1Valid                        35      // 1 Bit, Bit 6
#define     LOG_fE2Low1ValidMask 0x40
#define     LOG_fE2Low1ValidShift 6
#define LOG_fE2Low2Valid                        35      // 1 Bit, Bit 5
#define     LOG_fE2Low2ValidMask 0x20
#define     LOG_fE2Low2ValidShift 5
#define LOG_fE2Low3Valid                        35      // 1 Bit, Bit 4
#define     LOG_fE2Low3ValidMask 0x10
#define     LOG_fE2Low3ValidShift 4
#define LOG_fE2Low4Valid                        35      // 1 Bit, Bit 3
#define     LOG_fE2Low4ValidMask 0x08
#define     LOG_fE2Low4ValidShift 3
#define LOG_fE2Low5Valid                        35      // 1 Bit, Bit 2
#define     LOG_fE2Low5ValidMask 0x04
#define     LOG_fE2Low5ValidShift 2
#define LOG_fE2Low6Valid                        35      // 1 Bit, Bit 1
#define     LOG_fE2Low6ValidMask 0x02
#define     LOG_fE2Low6ValidShift 1
#define LOG_fE2Low0Dpt2                         28      // 8 Bits, Bit 7-0
#define LOG_fE2Low1Dpt2                         29      // 8 Bits, Bit 7-0
#define LOG_fE2Low2Dpt2                         30      // 8 Bits, Bit 7-0
#define LOG_fE2Low3Dpt2                         31      // 8 Bits, Bit 7-0
#define LOG_fE2LowDpt2Fix                       28      // 8 Bits, Bit 7-0
#define LOG_fE2Low0Dpt3Dir                      28      // 5 Bits, Bit 7-3
#define     LOG_fE2Low0Dpt3DirMask 0xF8
#define     LOG_fE2Low0Dpt3DirShift 3
#define LOG_fE2Low0Dpt3Dim                      28      // 3 Bits, Bit 2-0
#define     LOG_fE2Low0Dpt3DimMask 0x07
#define     LOG_fE2Low0Dpt3DimShift 0
#define LOG_fE2Low1Dpt3Dir                      29      // 5 Bits, Bit 7-3
#define     LOG_fE2Low1Dpt3DirMask 0xF8
#define     LOG_fE2Low1Dpt3DirShift 3
#define LOG_fE2Low1Dpt3Dim                      29      // 3 Bits, Bit 2-0
#define     LOG_fE2Low1Dpt3DimMask 0x07
#define     LOG_fE2Low1Dpt3DimShift 0
#define LOG_fE2Low2Dpt3Dir                      30      // 5 Bits, Bit 7-3
#define     LOG_fE2Low2Dpt3DirMask 0xF8
#define     LOG_fE2Low2Dpt3DirShift 3
#define LOG_fE2Low2Dpt3Dim                      30      // 3 Bits, Bit 2-0
#define     LOG_fE2Low2Dpt3DimMask 0x07
#define     LOG_fE2Low2Dpt3DimShift 0
#define LOG_fE2Low3Dpt3Dir                      31      // 5 Bits, Bit 7-3
#define     LOG_fE2Low3Dpt3DirMask 0xF8
#define     LOG_fE2Low3Dpt3DirShift 3
#define LOG_fE2Low3Dpt3Dim                      31      // 3 Bits, Bit 2-0
#define     LOG_fE2Low3Dpt3DimMask 0x07
#define     LOG_fE2Low3Dpt3DimShift 0
#define LOG_fE2LowDpt3FixDir                    28      // 5 Bits, Bit 7-3
#define     LOG_fE2LowDpt3FixDirMask 0xF8
#define     LOG_fE2LowDpt3FixDirShift 3
#define LOG_fE2LowDpt3FixDim                    28      // 3 Bits, Bit 2-0
#define     LOG_fE2LowDpt3FixDimMask 0x07
#define     LOG_fE2LowDpt3FixDimShift 0
#define LOG_fE2LowDpt5                          28      // uint8_t
#define LOG_fE2HighDpt5                         32      // uint8_t
#define LOG_fE2Low0Dpt5In                       28      // uint8_t
#define LOG_fE2Low1Dpt5In                       29      // uint8_t
#define LOG_fE2Low2Dpt5In                       30      // uint8_t
#define LOG_fE2Low3Dpt5In                       31      // uint8_t
#define LOG_fE2Low4Dpt5In                       32      // uint8_t
#define LOG_fE2Low5Dpt5In                       33      // uint8_t
#define LOG_fE2Low6Dpt5In                       34      // uint8_t
#define LOG_fE2LowDpt5Fix                       28      // uint8_t
#define LOG_fE2LowDpt5001                       28      // uint8_t
#define LOG_fE2HighDpt5001                      32      // uint8_t
#define LOG_fE2Low0Dpt5xIn                      28      // uint8_t
#define LOG_fE2Low1Dpt5xIn                      29      // uint8_t
#define LOG_fE2Low2Dpt5xIn                      30      // uint8_t
#define LOG_fE2Low3Dpt5xIn                      31      // uint8_t
#define LOG_fE2Low4Dpt5xIn                      32      // uint8_t
#define LOG_fE2Low5Dpt5xIn                      33      // uint8_t
#define LOG_fE2Low6Dpt5xIn                      34      // uint8_t
#define LOG_fE2LowDpt5xFix                      28      // uint8_t
#define LOG_fE2LowDpt6                          28      // int8_t
#define LOG_fE2HighDpt6                         32      // int8_t
#define LOG_fE2Low0Dpt6In                       28      // int8_t
#define LOG_fE2Low1Dpt6In                       29      // int8_t
#define LOG_fE2Low2Dpt6In                       30      // int8_t
#define LOG_fE2Low3Dpt6In                       31      // int8_t
#define LOG_fE2Low4Dpt6In                       32      // int8_t
#define LOG_fE2Low5Dpt6In                       33      // int8_t
#define LOG_fE2Low6Dpt6In                       34      // int8_t
#define LOG_fE2LowDpt6Fix                       28      // int8_t
#define LOG_fE2LowDpt7                          28      // uint16_t
#define LOG_fE2HighDpt7                         32      // uint16_t
#define LOG_fE2Low0Dpt7In                       28      // uint16_t
#define LOG_fE2Low1Dpt7In                       30      // uint16_t
#define LOG_fE2Low2Dpt7In                       32      // uint16_t
#define LOG_fE2LowDpt7Fix                       28      // uint16_t
#define LOG_fE2LowDpt8                          28      // int16_t
#define LOG_fE2HighDpt8                         32      // int16_t
#define LOG_fE2Low0Dpt8In                       28      // int16_t
#define LOG_fE2Low1Dpt8In                       30      // int16_t
#define LOG_fE2Low2Dpt8In                       32      // int16_t
#define LOG_fE2LowDpt8Fix                       28      // int16_t
#define LOG_fE2LowDpt9                          28      // float
#define LOG_fE2HighDpt9                         32      // float
#define LOG_fE2LowDpt9Fix                       28      // float
#define LOG_fE2LowDpt12                         28      // uint32_t
#define LOG_fE2HighDpt12                        32      // uint32_t
#define LOG_fE2LowDpt12Fix                      28      // uint32_t
#define LOG_fE2LowDpt13                         28      // int32_t
#define LOG_fE2HighDpt13                        32      // int32_t
#define LOG_fE2LowDpt13Fix                      28      // int32_t
#define LOG_fE2LowDpt14                         28      // float
#define LOG_fE2HighDpt14                        32      // float
#define LOG_fE2LowDpt14Fix                      28      // float
#define LOG_fE2Low0Dpt17                        28      // 8 Bits, Bit 7-0
#define LOG_fE2Low1Dpt17                        29      // 8 Bits, Bit 7-0
#define LOG_fE2Low2Dpt17                        30      // 8 Bits, Bit 7-0
#define LOG_fE2Low3Dpt17                        31      // 8 Bits, Bit 7-0
#define LOG_fE2Low4Dpt17                        32      // 8 Bits, Bit 7-0
#define LOG_fE2Low5Dpt17                        33      // 8 Bits, Bit 7-0
#define LOG_fE2Low6Dpt17                        34      // 8 Bits, Bit 7-0
#define LOG_fE2Low7Dpt17                        35      // 8 Bits, Bit 7-0
#define LOG_fE2LowDpt17Fix                      28      // 8 Bits, Bit 7-0
#define LOG_fE2LowDptRGB                        28      // int32_t
#define LOG_fE2HighDptRGB                       32      // int32_t
#define LOG_fE2LowDptRGBFix                     28      // int32_t
#define LOG_fTd1DuskDawn                         6      // 4 Bits, Bit 7-4
#define     LOG_fTd1DuskDawnMask 0xF0
#define     LOG_fTd1DuskDawnShift 4
#define LOG_fTd2DuskDawn                         6      // 4 Bits, Bit 3-0
#define     LOG_fTd2DuskDawnMask 0x0F
#define     LOG_fTd2DuskDawnShift 0
#define LOG_fTd3DuskDawn                         7      // 4 Bits, Bit 7-4
#define     LOG_fTd3DuskDawnMask 0xF0
#define     LOG_fTd3DuskDawnShift 4
#define LOG_fTd4DuskDawn                         7      // 4 Bits, Bit 3-0
#define     LOG_fTd4DuskDawnMask 0x0F
#define     LOG_fTd4DuskDawnShift 0
#define LOG_fTd5DuskDawn                         8      // 4 Bits, Bit 7-4
#define     LOG_fTd5DuskDawnMask 0xF0
#define     LOG_fTd5DuskDawnShift 4
#define LOG_fTd6DuskDawn                         8      // 4 Bits, Bit 3-0
#define     LOG_fTd6DuskDawnMask 0x0F
#define     LOG_fTd6DuskDawnShift 0
#define LOG_fTd7DuskDawn                         9      // 4 Bits, Bit 7-4
#define     LOG_fTd7DuskDawnMask 0xF0
#define     LOG_fTd7DuskDawnShift 4
#define LOG_fTd8DuskDawn                         9      // 4 Bits, Bit 3-0
#define     LOG_fTd8DuskDawnMask 0x0F
#define     LOG_fTd8DuskDawnShift 0
#define LOG_fTYearDay                           10      // 2 Bits, Bit 7-6
#define     LOG_fTYearDayMask 0xC0
#define     LOG_fTYearDayShift 6
#define LOG_fTHoliday                           10      // 2 Bits, Bit 5-4
#define     LOG_fTHolidayMask 0x30
#define     LOG_fTHolidayShift 4
#define LOG_fTRestoreState                      10      // 2 Bits, Bit 3-2
#define     LOG_fTRestoreStateMask 0x0C
#define     LOG_fTRestoreStateShift 2
#define LOG_fTVacation                          10      // 2 Bits, Bit 1-0
#define     LOG_fTVacationMask 0x03
#define     LOG_fTVacationShift 0
#define LOG_fTd1ValueNum                        11      // uint8_t
#define LOG_fTd2ValueNum                        12      // uint8_t
#define LOG_fTd3ValueNum                        13      // uint8_t
#define LOG_fTd4ValueNum                        14      // uint8_t
#define LOG_fTd5ValueNum                        15      // uint8_t
#define LOG_fTd6ValueNum                        16      // uint8_t
#define LOG_fTd7ValueNum                        17      // uint8_t
#define LOG_fTd8ValueNum                        18      // uint8_t
#define LOG_fTd1Value                           20      // 1 Bit, Bit 7
#define     LOG_fTd1ValueMask 0x80
#define     LOG_fTd1ValueShift 7
#define LOG_fTd1Degree                          20      // 6 Bits, Bit 6-1
#define     LOG_fTd1DegreeMask 0x7E
#define     LOG_fTd1DegreeShift 1
#define LOG_fTd1HourAbs                         20      // 5 Bits, Bit 5-1
#define     LOG_fTd1HourAbsMask 0x3E
#define     LOG_fTd1HourAbsShift 1
#define LOG_fTd1HourRel                         20      // 5 Bits, Bit 5-1
#define     LOG_fTd1HourRelMask 0x3E
#define     LOG_fTd1HourRelShift 1
#define LOG_fTd1HourRelShort                    20      // 5 Bits, Bit 5-1
#define     LOG_fTd1HourRelShortMask 0x3E
#define     LOG_fTd1HourRelShortShift 1
#define LOG_fTd1MinuteAbs                       20      // 6 Bits, Bit 0--5
#define LOG_fTd1MinuteRel                       20      // 6 Bits, Bit 0--5
#define LOG_fTd1Weekday                         21      // 3 Bits, Bit 2-0
#define     LOG_fTd1WeekdayMask 0x07
#define     LOG_fTd1WeekdayShift 0
#define LOG_fTd2Value                           22      // 1 Bit, Bit 7
#define     LOG_fTd2ValueMask 0x80
#define     LOG_fTd2ValueShift 7
#define LOG_fTd2Degree                          22      // 6 Bits, Bit 6-1
#define     LOG_fTd2DegreeMask 0x7E
#define     LOG_fTd2DegreeShift 1
#define LOG_fTd2HourAbs                         22      // 5 Bits, Bit 5-1
#define     LOG_fTd2HourAbsMask 0x3E
#define     LOG_fTd2HourAbsShift 1
#define LOG_fTd2HourRel                         22      // 5 Bits, Bit 5-1
#define     LOG_fTd2HourRelMask 0x3E
#define     LOG_fTd2HourRelShift 1
#define LOG_fTd2HourRelShort                    22      // 5 Bits, Bit 5-1
#define     LOG_fTd2HourRelShortMask 0x3E
#define     LOG_fTd2HourRelShortShift 1
#define LOG_fTd2MinuteAbs                       22      // 6 Bits, Bit 0--5
#define LOG_fTd2MinuteRel                       22      // 6 Bits, Bit 0--5
#define LOG_fTd2Weekday                         23      // 3 Bits, Bit 2-0
#define     LOG_fTd2WeekdayMask 0x07
#define     LOG_fTd2WeekdayShift 0
#define LOG_fTd3Value                           24      // 1 Bit, Bit 7
#define     LOG_fTd3ValueMask 0x80
#define     LOG_fTd3ValueShift 7
#define LOG_fTd3Degree                          24      // 6 Bits, Bit 6-1
#define     LOG_fTd3DegreeMask 0x7E
#define     LOG_fTd3DegreeShift 1
#define LOG_fTd3HourAbs                         24      // 5 Bits, Bit 5-1
#define     LOG_fTd3HourAbsMask 0x3E
#define     LOG_fTd3HourAbsShift 1
#define LOG_fTd3HourRel                         24      // 5 Bits, Bit 5-1
#define     LOG_fTd3HourRelMask 0x3E
#define     LOG_fTd3HourRelShift 1
#define LOG_fTd3HourRelShort                    24      // 5 Bits, Bit 5-1
#define     LOG_fTd3HourRelShortMask 0x3E
#define     LOG_fTd3HourRelShortShift 1
#define LOG_fTd3MinuteAbs                       24      // 6 Bits, Bit 0--5
#define LOG_fTd3MinuteRel                       24      // 6 Bits, Bit 0--5
#define LOG_fTd3Weekday                         25      // 3 Bits, Bit 2-0
#define     LOG_fTd3WeekdayMask 0x07
#define     LOG_fTd3WeekdayShift 0
#define LOG_fTd4Value                           26      // 1 Bit, Bit 7
#define     LOG_fTd4ValueMask 0x80
#define     LOG_fTd4ValueShift 7
#define LOG_fTd4Degree                          26      // 6 Bits, Bit 6-1
#define     LOG_fTd4DegreeMask 0x7E
#define     LOG_fTd4DegreeShift 1
#define LOG_fTd4HourAbs                         26      // 5 Bits, Bit 5-1
#define     LOG_fTd4HourAbsMask 0x3E
#define     LOG_fTd4HourAbsShift 1
#define LOG_fTd4HourRel                         26      // 5 Bits, Bit 5-1
#define     LOG_fTd4HourRelMask 0x3E
#define     LOG_fTd4HourRelShift 1
#define LOG_fTd4HourRelShort                    26      // 5 Bits, Bit 5-1
#define     LOG_fTd4HourRelShortMask 0x3E
#define     LOG_fTd4HourRelShortShift 1
#define LOG_fTd4MinuteAbs                       26      // 6 Bits, Bit 0--5
#define LOG_fTd4MinuteRel                       26      // 6 Bits, Bit 0--5
#define LOG_fTd4Weekday                         27      // 3 Bits, Bit 2-0
#define     LOG_fTd4WeekdayMask 0x07
#define     LOG_fTd4WeekdayShift 0
#define LOG_fTd5Value                           28      // 1 Bit, Bit 7
#define     LOG_fTd5ValueMask 0x80
#define     LOG_fTd5ValueShift 7
#define LOG_fTd5Degree                          28      // 6 Bits, Bit 6-1
#define     LOG_fTd5DegreeMask 0x7E
#define     LOG_fTd5DegreeShift 1
#define LOG_fTd5HourAbs                         28      // 5 Bits, Bit 5-1
#define     LOG_fTd5HourAbsMask 0x3E
#define     LOG_fTd5HourAbsShift 1
#define LOG_fTd5HourRel                         28      // 5 Bits, Bit 5-1
#define     LOG_fTd5HourRelMask 0x3E
#define     LOG_fTd5HourRelShift 1
#define LOG_fTd5HourRelShort                    28      // 5 Bits, Bit 5-1
#define     LOG_fTd5HourRelShortMask 0x3E
#define     LOG_fTd5HourRelShortShift 1
#define LOG_fTd5MinuteAbs                       28      // 6 Bits, Bit 0--5
#define LOG_fTd5MinuteRel                       28      // 6 Bits, Bit 0--5
#define LOG_fTd5Weekday                         29      // 3 Bits, Bit 2-0
#define     LOG_fTd5WeekdayMask 0x07
#define     LOG_fTd5WeekdayShift 0
#define LOG_fTd6Value                           30      // 1 Bit, Bit 7
#define     LOG_fTd6ValueMask 0x80
#define     LOG_fTd6ValueShift 7
#define LOG_fTd6Degree                          30      // 6 Bits, Bit 6-1
#define     LOG_fTd6DegreeMask 0x7E
#define     LOG_fTd6DegreeShift 1
#define LOG_fTd6HourAbs                         30      // 5 Bits, Bit 5-1
#define     LOG_fTd6HourAbsMask 0x3E
#define     LOG_fTd6HourAbsShift 1
#define LOG_fTd6HourRel                         30      // 5 Bits, Bit 5-1
#define     LOG_fTd6HourRelMask 0x3E
#define     LOG_fTd6HourRelShift 1
#define LOG_fTd6HourRelShort                    30      // 5 Bits, Bit 5-1
#define     LOG_fTd6HourRelShortMask 0x3E
#define     LOG_fTd6HourRelShortShift 1
#define LOG_fTd6MinuteAbs                       30      // 6 Bits, Bit 0--5
#define LOG_fTd6MinuteRel                       30      // 6 Bits, Bit 0--5
#define LOG_fTd6Weekday                         31      // 3 Bits, Bit 2-0
#define     LOG_fTd6WeekdayMask 0x07
#define     LOG_fTd6WeekdayShift 0
#define LOG_fTd7Value                           32      // 1 Bit, Bit 7
#define     LOG_fTd7ValueMask 0x80
#define     LOG_fTd7ValueShift 7
#define LOG_fTd7Degree                          32      // 6 Bits, Bit 6-1
#define     LOG_fTd7DegreeMask 0x7E
#define     LOG_fTd7DegreeShift 1
#define LOG_fTd7HourAbs                         32      // 5 Bits, Bit 5-1
#define     LOG_fTd7HourAbsMask 0x3E
#define     LOG_fTd7HourAbsShift 1
#define LOG_fTd7HourRel                         32      // 5 Bits, Bit 5-1
#define     LOG_fTd7HourRelMask 0x3E
#define     LOG_fTd7HourRelShift 1
#define LOG_fTd7HourRelShort                    32      // 5 Bits, Bit 5-1
#define     LOG_fTd7HourRelShortMask 0x3E
#define     LOG_fTd7HourRelShortShift 1
#define LOG_fTd7MinuteAbs                       32      // 6 Bits, Bit 0--5
#define LOG_fTd7MinuteRel                       32      // 6 Bits, Bit 0--5
#define LOG_fTd7Weekday                         33      // 3 Bits, Bit 2-0
#define     LOG_fTd7WeekdayMask 0x07
#define     LOG_fTd7WeekdayShift 0
#define LOG_fTd8Value                           34      // 1 Bit, Bit 7
#define     LOG_fTd8ValueMask 0x80
#define     LOG_fTd8ValueShift 7
#define LOG_fTd8Degree                          34      // 6 Bits, Bit 6-1
#define     LOG_fTd8DegreeMask 0x7E
#define     LOG_fTd8DegreeShift 1
#define LOG_fTd8HourAbs                         34      // 5 Bits, Bit 5-1
#define     LOG_fTd8HourAbsMask 0x3E
#define     LOG_fTd8HourAbsShift 1
#define LOG_fTd8HourRel                         34      // 5 Bits, Bit 5-1
#define     LOG_fTd8HourRelMask 0x3E
#define     LOG_fTd8HourRelShift 1
#define LOG_fTd8HourRelShort                    34      // 5 Bits, Bit 5-1
#define     LOG_fTd8HourRelShortMask 0x3E
#define     LOG_fTd8HourRelShortShift 1
#define LOG_fTd8MinuteAbs                       34      // 6 Bits, Bit 0--5
#define LOG_fTd8MinuteRel                       34      // 6 Bits, Bit 0--5
#define LOG_fTd8Weekday                         35      // 3 Bits, Bit 2-0
#define     LOG_fTd8WeekdayMask 0x07
#define     LOG_fTd8WeekdayShift 0
#define LOG_fTy1Weekday1                        28      // 1 Bit, Bit 7
#define     LOG_fTy1Weekday1Mask 0x80
#define     LOG_fTy1Weekday1Shift 7
#define LOG_fTy1Weekday2                        28      // 1 Bit, Bit 6
#define     LOG_fTy1Weekday2Mask 0x40
#define     LOG_fTy1Weekday2Shift 6
#define LOG_fTy1Weekday3                        28      // 1 Bit, Bit 5
#define     LOG_fTy1Weekday3Mask 0x20
#define     LOG_fTy1Weekday3Shift 5
#define LOG_fTy1Weekday4                        28      // 1 Bit, Bit 4
#define     LOG_fTy1Weekday4Mask 0x10
#define     LOG_fTy1Weekday4Shift 4
#define LOG_fTy1Weekday5                        28      // 1 Bit, Bit 3
#define     LOG_fTy1Weekday5Mask 0x08
#define     LOG_fTy1Weekday5Shift 3
#define LOG_fTy1Weekday6                        28      // 1 Bit, Bit 2
#define     LOG_fTy1Weekday6Mask 0x04
#define     LOG_fTy1Weekday6Shift 2
#define LOG_fTy1Weekday7                        28      // 1 Bit, Bit 1
#define     LOG_fTy1Weekday7Mask 0x02
#define     LOG_fTy1Weekday7Shift 1
#define LOG_fTy1Day                             28      // 7 Bits, Bit 7-1
#define     LOG_fTy1DayMask 0xFE
#define     LOG_fTy1DayShift 1
#define LOG_fTy1IsWeekday                       28      // 1 Bit, Bit 0
#define     LOG_fTy1IsWeekdayMask 0x01
#define     LOG_fTy1IsWeekdayShift 0
#define LOG_fTy1Month                           29      // 4 Bits, Bit 7-4
#define     LOG_fTy1MonthMask 0xF0
#define     LOG_fTy1MonthShift 4
#define LOG_fTy2Weekday1                        30      // 1 Bit, Bit 7
#define     LOG_fTy2Weekday1Mask 0x80
#define     LOG_fTy2Weekday1Shift 7
#define LOG_fTy2Weekday2                        30      // 1 Bit, Bit 6
#define     LOG_fTy2Weekday2Mask 0x40
#define     LOG_fTy2Weekday2Shift 6
#define LOG_fTy2Weekday3                        30      // 1 Bit, Bit 5
#define     LOG_fTy2Weekday3Mask 0x20
#define     LOG_fTy2Weekday3Shift 5
#define LOG_fTy2Weekday4                        30      // 1 Bit, Bit 4
#define     LOG_fTy2Weekday4Mask 0x10
#define     LOG_fTy2Weekday4Shift 4
#define LOG_fTy2Weekday5                        30      // 1 Bit, Bit 3
#define     LOG_fTy2Weekday5Mask 0x08
#define     LOG_fTy2Weekday5Shift 3
#define LOG_fTy2Weekday6                        30      // 1 Bit, Bit 2
#define     LOG_fTy2Weekday6Mask 0x04
#define     LOG_fTy2Weekday6Shift 2
#define LOG_fTy2Weekday7                        30      // 1 Bit, Bit 1
#define     LOG_fTy2Weekday7Mask 0x02
#define     LOG_fTy2Weekday7Shift 1
#define LOG_fTy2Day                             30      // 7 Bits, Bit 7-1
#define     LOG_fTy2DayMask 0xFE
#define     LOG_fTy2DayShift 1
#define LOG_fTy2IsWeekday                       30      // 1 Bit, Bit 0
#define     LOG_fTy2IsWeekdayMask 0x01
#define     LOG_fTy2IsWeekdayShift 0
#define LOG_fTy2Month                           31      // 4 Bits, Bit 7-4
#define     LOG_fTy2MonthMask 0xF0
#define     LOG_fTy2MonthShift 4
#define LOG_fTy3Weekday1                        32      // 1 Bit, Bit 7
#define     LOG_fTy3Weekday1Mask 0x80
#define     LOG_fTy3Weekday1Shift 7
#define LOG_fTy3Weekday2                        32      // 1 Bit, Bit 6
#define     LOG_fTy3Weekday2Mask 0x40
#define     LOG_fTy3Weekday2Shift 6
#define LOG_fTy3Weekday3                        32      // 1 Bit, Bit 5
#define     LOG_fTy3Weekday3Mask 0x20
#define     LOG_fTy3Weekday3Shift 5
#define LOG_fTy3Weekday4                        32      // 1 Bit, Bit 4
#define     LOG_fTy3Weekday4Mask 0x10
#define     LOG_fTy3Weekday4Shift 4
#define LOG_fTy3Weekday5                        32      // 1 Bit, Bit 3
#define     LOG_fTy3Weekday5Mask 0x08
#define     LOG_fTy3Weekday5Shift 3
#define LOG_fTy3Weekday6                        32      // 1 Bit, Bit 2
#define     LOG_fTy3Weekday6Mask 0x04
#define     LOG_fTy3Weekday6Shift 2
#define LOG_fTy3Weekday7                        32      // 1 Bit, Bit 1
#define     LOG_fTy3Weekday7Mask 0x02
#define     LOG_fTy3Weekday7Shift 1
#define LOG_fTy3Day                             32      // 7 Bits, Bit 7-1
#define     LOG_fTy3DayMask 0xFE
#define     LOG_fTy3DayShift 1
#define LOG_fTy3IsWeekday                       32      // 1 Bit, Bit 0
#define     LOG_fTy3IsWeekdayMask 0x01
#define     LOG_fTy3IsWeekdayShift 0
#define LOG_fTy3Month                           33      // 4 Bits, Bit 7-4
#define     LOG_fTy3MonthMask 0xF0
#define     LOG_fTy3MonthShift 4
#define LOG_fTy4Weekday1                        34      // 1 Bit, Bit 7
#define     LOG_fTy4Weekday1Mask 0x80
#define     LOG_fTy4Weekday1Shift 7
#define LOG_fTy4Weekday2                        34      // 1 Bit, Bit 6
#define     LOG_fTy4Weekday2Mask 0x40
#define     LOG_fTy4Weekday2Shift 6
#define LOG_fTy4Weekday3                        34      // 1 Bit, Bit 5
#define     LOG_fTy4Weekday3Mask 0x20
#define     LOG_fTy4Weekday3Shift 5
#define LOG_fTy4Weekday4                        34      // 1 Bit, Bit 4
#define     LOG_fTy4Weekday4Mask 0x10
#define     LOG_fTy4Weekday4Shift 4
#define LOG_fTy4Weekday5                        34      // 1 Bit, Bit 3
#define     LOG_fTy4Weekday5Mask 0x08
#define     LOG_fTy4Weekday5Shift 3
#define LOG_fTy4Weekday6                        34      // 1 Bit, Bit 2
#define     LOG_fTy4Weekday6Mask 0x04
#define     LOG_fTy4Weekday6Shift 2
#define LOG_fTy4Weekday7                        34      // 1 Bit, Bit 1
#define     LOG_fTy4Weekday7Mask 0x02
#define     LOG_fTy4Weekday7Shift 1
#define LOG_fTy4Day                             34      // 7 Bits, Bit 7-1
#define     LOG_fTy4DayMask 0xFE
#define     LOG_fTy4DayShift 1
#define LOG_fTy4IsWeekday                       34      // 1 Bit, Bit 0
#define     LOG_fTy4IsWeekdayMask 0x01
#define     LOG_fTy4IsWeekdayShift 0
#define LOG_fTy4Month                           35      // 4 Bits, Bit 7-4
#define     LOG_fTy4MonthMask 0xF0
#define     LOG_fTy4MonthShift 4
#define LOG_fI1                                 36      // 2 Bits, Bit 7-6
#define     LOG_fI1Mask 0xC0
#define     LOG_fI1Shift 6
#define LOG_fI1Kind                             36      // 2 Bits, Bit 5-4
#define     LOG_fI1KindMask 0x30
#define     LOG_fI1KindShift 4
#define LOG_fI1AsTrigger                        36      // 1 Bit, Bit 3
#define     LOG_fI1AsTriggerMask 0x08
#define     LOG_fI1AsTriggerShift 3
#define LOG_fI1Function                         37      // uint8_t
#define LOG_fI1FunctionRel                      37      // int8_t
#define LOG_fI2                                 38      // 2 Bits, Bit 7-6
#define     LOG_fI2Mask 0xC0
#define     LOG_fI2Shift 6
#define LOG_fI2Kind                             38      // 2 Bits, Bit 5-4
#define     LOG_fI2KindMask 0x30
#define     LOG_fI2KindShift 4
#define LOG_fI2AsTrigger                        38      // 1 Bit, Bit 3
#define     LOG_fI2AsTriggerMask 0x08
#define     LOG_fI2AsTriggerShift 3
#define LOG_fI2Function                         39      // uint8_t
#define LOG_fI2FunctionRel                      39      // int8_t
#define LOG_fOStairtimeBase                     40      // 2 Bits, Bit 7-6
#define     LOG_fOStairtimeBaseMask 0xC0
#define     LOG_fOStairtimeBaseShift 6
#define LOG_fOStairtimeTime                     40      // 14 Bits, Bit 13-0
#define     LOG_fOStairtimeTimeMask 0x3FFF
#define     LOG_fOStairtimeTimeShift 0
#define LOG_fOBlinkBase                         42      // 2 Bits, Bit 7-6
#define     LOG_fOBlinkBaseMask 0xC0
#define     LOG_fOBlinkBaseShift 6
#define LOG_fOBlinkTime                         42      // 14 Bits, Bit 13-0
#define     LOG_fOBlinkTimeMask 0x3FFF
#define     LOG_fOBlinkTimeShift 0
#define LOG_fODelayOnBase                       44      // 2 Bits, Bit 7-6
#define     LOG_fODelayOnBaseMask 0xC0
#define     LOG_fODelayOnBaseShift 6
#define LOG_fODelayOnTime                       44      // 14 Bits, Bit 13-0
#define     LOG_fODelayOnTimeMask 0x3FFF
#define     LOG_fODelayOnTimeShift 0
#define LOG_fODelayOffBase                      46      // 2 Bits, Bit 7-6
#define     LOG_fODelayOffBaseMask 0xC0
#define     LOG_fODelayOffBaseShift 6
#define LOG_fODelayOffTime                      46      // 14 Bits, Bit 13-0
#define     LOG_fODelayOffTimeMask 0x3FFF
#define     LOG_fODelayOffTimeShift 0
#define LOG_fORepeatOnBase                      48      // 2 Bits, Bit 7-6
#define     LOG_fORepeatOnBaseMask 0xC0
#define     LOG_fORepeatOnBaseShift 6
#define LOG_fORepeatOnTime                      48      // 14 Bits, Bit 13-0
#define     LOG_fORepeatOnTimeMask 0x3FFF
#define     LOG_fORepeatOnTimeShift 0
#define LOG_fORepeatOffBase                     50      // 2 Bits, Bit 7-6
#define     LOG_fORepeatOffBaseMask 0xC0
#define     LOG_fORepeatOffBaseShift 6
#define LOG_fORepeatOffTime                     50      // 14 Bits, Bit 13-0
#define     LOG_fORepeatOffTimeMask 0x3FFF
#define     LOG_fORepeatOffTimeShift 0
#define LOG_fODelay                             52      // 1 Bit, Bit 7
#define     LOG_fODelayMask 0x80
#define     LOG_fODelayShift 7
#define LOG_fODelayOnRepeat                     52      // 2 Bits, Bit 6-5
#define     LOG_fODelayOnRepeatMask 0x60
#define     LOG_fODelayOnRepeatShift 5
#define LOG_fODelayOnReset                      52      // 1 Bit, Bit 4
#define     LOG_fODelayOnResetMask 0x10
#define     LOG_fODelayOnResetShift 4
#define LOG_fODelayOffRepeat                    52      // 2 Bits, Bit 3-2
#define     LOG_fODelayOffRepeatMask 0x0C
#define     LOG_fODelayOffRepeatShift 2
#define LOG_fODelayOffReset                     52      // 1 Bit, Bit 1
#define     LOG_fODelayOffResetMask 0x02
#define     LOG_fODelayOffResetShift 1
#define LOG_fOStair                             52      // 1 Bit, Bit 0
#define     LOG_fOStairMask 0x01
#define     LOG_fOStairShift 0
#define LOG_fORetrigger                         53      // 1 Bit, Bit 7
#define     LOG_fORetriggerMask 0x80
#define     LOG_fORetriggerShift 7
#define LOG_fOStairOff                          53      // 1 Bit, Bit 6
#define     LOG_fOStairOffMask 0x40
#define     LOG_fOStairOffShift 6
#define LOG_fORepeat                            53      // 1 Bit, Bit 5
#define     LOG_fORepeatMask 0x20
#define     LOG_fORepeatShift 5
#define LOG_fOOutputFilter                      53      // 2 Bits, Bit 4-3
#define     LOG_fOOutputFilterMask 0x18
#define     LOG_fOOutputFilterShift 3
#define LOG_fOSendOnChange                      53      // 1 Bit, Bit 2
#define     LOG_fOSendOnChangeMask 0x04
#define     LOG_fOSendOnChangeShift 2
#define LOG_fODpt                               54      // 8 Bits, Bit 7-0
#define LOG_fOOn                                55      // 8 Bits, Bit 7-0
#define LOG_fOOnBuzzer                          55      // 8 Bits, Bit 7-0
#define LOG_fOOnLed                             55      // 8 Bits, Bit 7-0
#define LOG_fOOnAll                             55      // 8 Bits, Bit 7-0
#define LOG_fOOnTone                            56      // 8 Bits, Bit 7-0
#define LOG_fOOnDpt1                            56      // 8 Bits, Bit 7-0
#define LOG_fOOnDpt2                            56      // 8 Bits, Bit 7-0
#define LOG_fOOnDpt3Dir                         56      // 5 Bits, Bit 7-3
#define     LOG_fOOnDpt3DirMask 0xF8
#define     LOG_fOOnDpt3DirShift 3
#define LOG_fOOnDpt3Dim                         56      // 3 Bits, Bit 2-0
#define     LOG_fOOnDpt3DimMask 0x07
#define     LOG_fOOnDpt3DimShift 0
#define LOG_fOOnDpt5                            56      // uint8_t
#define LOG_fOOnDpt5001                         56      // uint8_t
#define LOG_fOOnDpt6                            56      // int8_t
#define LOG_fOOnDpt7                            56      // uint16_t
#define LOG_fOOnDpt8                            56      // int16_t
#define LOG_fOOnDpt9                            56      // float
#define LOG_fOOnDpt12                           56      // uint32_t
#define LOG_fOOnDpt13                           56      // int32_t
#define LOG_fOOnDpt14                           56      // float
#define LOG_fOOnDpt16                           56      // char*, 14 Byte
#define LOG_fOOnDpt17                           56      // 8 Bits, Bit 7-0
#define LOG_fOOnRGB                             56      // 24 Bits, Bit 31-8
#define     LOG_fOOnRGBMask 0xFFFFFF00
#define     LOG_fOOnRGBShift 8
#define LOG_fOOnPAArea                          56      // 4 Bits, Bit 7-4
#define     LOG_fOOnPAAreaMask 0xF0
#define     LOG_fOOnPAAreaShift 4
#define LOG_fOOnPALine                          56      // 4 Bits, Bit 3-0
#define     LOG_fOOnPALineMask 0x0F
#define     LOG_fOOnPALineShift 0
#define LOG_fOOnPADevice                        57      // uint8_t
#define LOG_fOOnFunction                        56      // 8 Bits, Bit 7-0
#define LOG_fOOnKOKind                          61      // 2 Bits, Bit 7-6
#define     LOG_fOOnKOKindMask 0xC0
#define     LOG_fOOnKOKindShift 6
#define LOG_fOOnKONumber                        56      // uint16_t
#define LOG_fOOnKONumberRel                     56      // int16_t
#define LOG_fOOnKODpt                           58      // 8 Bits, Bit 7-0
#define LOG_fOOnKOSend                          61      // 2 Bits, Bit 5-4
#define     LOG_fOOnKOSendMask 0x30
#define     LOG_fOOnKOSendShift 4
#define LOG_fOOnKOSendNumber                    62      // uint16_t
#define LOG_fOOnKOSendNumberRel                 62      // int16_t
#define LOG_fOOff                               70      // 8 Bits, Bit 7-0
#define LOG_fOOffBuzzer                         70      // 8 Bits, Bit 7-0
#define LOG_fOOffLed                            70      // 8 Bits, Bit 7-0
#define LOG_fOOffAll                            70      // 8 Bits, Bit 7-0
#define LOG_fOOffTone                           71      // 8 Bits, Bit 7-0
#define LOG_fOOffDpt1                           71      // 8 Bits, Bit 7-0
#define LOG_fOOffDpt2                           71      // 8 Bits, Bit 7-0
#define LOG_fOOffDpt3Dir                        71      // 5 Bits, Bit 7-3
#define     LOG_fOOffDpt3DirMask 0xF8
#define     LOG_fOOffDpt3DirShift 3
#define LOG_fOOffDpt3Dim                        71      // 3 Bits, Bit 2-0
#define     LOG_fOOffDpt3DimMask 0x07
#define     LOG_fOOffDpt3DimShift 0
#define LOG_fOOffDpt5                           71      // uint8_t
#define LOG_fOOffDpt5001                        71      // uint8_t
#define LOG_fOOffDpt6                           71      // int8_t
#define LOG_fOOffDpt7                           71      // uint16_t
#define LOG_fOOffDpt8                           71      // int16_t
#define LOG_fOOffDpt9                           71      // float
#define LOG_fOOffDpt12                          71      // uint32_t
#define LOG_fOOffDpt13                          71      // int32_t
#define LOG_fOOffDpt14                          71      // float
#define LOG_fOOffDpt16                          71      // char*, 14 Byte
#define LOG_fOOffDpt17                          71      // 8 Bits, Bit 7-0
#define LOG_fOOffRGB                            71      // 24 Bits, Bit 31-8
#define     LOG_fOOffRGBMask 0xFFFFFF00
#define     LOG_fOOffRGBShift 8
#define LOG_fOOffPAArea                         71      // 4 Bits, Bit 7-4
#define     LOG_fOOffPAAreaMask 0xF0
#define     LOG_fOOffPAAreaShift 4
#define LOG_fOOffPALine                         71      // 4 Bits, Bit 3-0
#define     LOG_fOOffPALineMask 0x0F
#define     LOG_fOOffPALineShift 0
#define LOG_fOOffPADevice                       72      // uint8_t
#define LOG_fOOffFunction                       71      // 8 Bits, Bit 7-0
#define LOG_fOOffKOKind                         76      // 2 Bits, Bit 7-6
#define     LOG_fOOffKOKindMask 0xC0
#define     LOG_fOOffKOKindShift 6
#define LOG_fOOffKONumber                       71      // uint16_t
#define LOG_fOOffKONumberRel                    71      // int16_t
#define LOG_fOOffKODpt                          73      // 8 Bits, Bit 7-0
#define LOG_fOOffKOSend                         76      // 2 Bits, Bit 5-4
#define     LOG_fOOffKOSendMask 0x30
#define     LOG_fOOffKOSendShift 4
#define LOG_fOOffKOSendNumber                   77      // uint16_t
#define LOG_fOOffKOSendNumberRel                77      // int16_t

// Zeit bis der Kanal nach einem Neustart aktiv wird
#define ParamLOG_fChannelDelayBase                   ((knx.paramByte(LOG_ParamCalcIndex(LOG_fChannelDelayBase)) & LOG_fChannelDelayBaseMask) >> LOG_fChannelDelayBaseShift)
// Zeit bis der Kanal nach einem Neustart aktiv wird
#define ParamLOG_fChannelDelayTime                   (knx.paramWord(LOG_ParamCalcIndex(LOG_fChannelDelayTime)) & LOG_fChannelDelayTimeMask)
// Zeit bis der Kanal nach einem Neustart aktiv wird (in Millisekunden)
#define ParamLOG_fChannelDelayTimeMS                 (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fChannelDelayTime))))
// Logik-Operation
#define ParamLOG_fLogic                              (knx.paramByte(LOG_ParamCalcIndex(LOG_fLogic)))
// Logik auswerten
#define ParamLOG_fCalculate                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fCalculate)) & LOG_fCalculateMask)
// Kanal deaktivieren (zu Testzwecken)
#define ParamLOG_fDisable                            ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fDisable)) & LOG_fDisableMask))
// Alarmausgabe (Buzzer oder LED trotz Sperre schalten)?
#define ParamLOG_fAlarm                              ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fAlarm)) & LOG_fAlarmMask))
// Tor geht sofort wieder zu
#define ParamLOG_fTGate                              ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTGate)) & LOG_fTGateMask))
// Wert EIN intern weiterleiten
#define ParamLOG_fOInternalOn                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fOInternalOn)) & LOG_fOInternalOnMask))
// Wert AUS intern weiterleiten
#define ParamLOG_fOInternalOff                       ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fOInternalOff)) & LOG_fOInternalOffMask))
// Logik sendet ihren Wert weiter
#define ParamLOG_fTrigger                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fTrigger)))
//           Eingang 1
#define ParamLOG_fTriggerE1                          ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerE1)) & LOG_fTriggerE1Mask))
//           Eingang 2
#define ParamLOG_fTriggerE2                          ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerE2)) & LOG_fTriggerE2Mask))
//           Interner Eingang 3
#define ParamLOG_fTriggerI1                          ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerI1)) & LOG_fTriggerI1Mask))
//           Interner Eingang 4
#define ParamLOG_fTriggerI2                          ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerI2)) & LOG_fTriggerI2Mask))
// Logik sendet ihren Wert weiter
#define ParamLOG_fTriggerTime                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerTime)))
// Beim schließen vom Tor wird
#define ParamLOG_fTriggerGateClose                   ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerGateClose)) & LOG_fTriggerGateCloseMask) >> LOG_fTriggerGateCloseShift)
// Beim öffnen vom Tor wird
#define ParamLOG_fTriggerGateOpen                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTriggerGateOpen)) & LOG_fTriggerGateOpenMask) >> LOG_fTriggerGateOpenShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE1ConvertInt                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1ConvertInt)) & LOG_fE1ConvertIntMask) >> LOG_fE1ConvertIntShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE1Convert                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Convert)) & LOG_fE1ConvertMask) >> LOG_fE1ConvertShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE1ConvertFloat                     ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1ConvertFloat)) & LOG_fE1ConvertFloatMask) >> LOG_fE1ConvertFloatShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE1ConvertSpecial                   ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1ConvertSpecial)) & LOG_fE1ConvertSpecialMask) >> LOG_fE1ConvertSpecialShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE1ConvertBool                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1ConvertBool)) & LOG_fE1ConvertBoolMask) >> LOG_fE1ConvertBoolShift)
// Eingang 1
#define ParamLOG_fE1                                 (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1)) & LOG_fE1Mask)
// DPT für Eingang
#define ParamLOG_fE1Dpt                              (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Dpt)))
// Eingang wird gelesen alle
#define ParamLOG_fE1RepeatBase                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1RepeatBase)) & LOG_fE1RepeatBaseMask) >> LOG_fE1RepeatBaseShift)
// Eingang wird gelesen alle
#define ParamLOG_fE1RepeatTime                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1RepeatTime)) & LOG_fE1RepeatTimeMask)
// Eingang wird gelesen alle (in Millisekunden)
#define ParamLOG_fE1RepeatTimeMS                     (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fE1RepeatTime))))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fE1OtherKO                          (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1OtherKO)))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fE1OtherKORel                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1OtherKORel)))
// Falls Vorbelegung aus dem Speicher nicht möglich oder nicht gewünscht, dann vorbelegen mit
#define ParamLOG_fE1Default                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Default)) & LOG_fE1DefaultMask)
// Eingang vorbelegen mit
#define ParamLOG_fE1DefaultExt                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1DefaultExt)) & LOG_fE1DefaultExtMask)
// Eingangswert speichern und beim nächsten Neustart als Vorbelegung nutzen?
#define ParamLOG_fE1DefaultEEPROM                    ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1DefaultEEPROM)) & LOG_fE1DefaultEEPROMMask))
// Nur so lange zyklisch lesen, bis erstes Telegramm eingeht
#define ParamLOG_fE1DefaultRepeat                    ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1DefaultRepeat)) & LOG_fE1DefaultRepeatMask))
// Kommunikationsobjekt für Eingang
#define ParamLOG_fE1UseOtherKO                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1UseOtherKO)) & LOG_fE1UseOtherKOMask) >> LOG_fE1UseOtherKOShift)
// Von-Wert
#define ParamLOG_fE1LowDelta                         ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDelta)))
// Bis-Wert
#define ParamLOG_fE1HighDelta                        ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1HighDelta)))
// Von-Wert
#define ParamLOG_fE1LowDeltaFloat                    (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1LowDeltaFloat), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE1HighDeltaFloat                   (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1HighDeltaFloat), Float_Enc_IEEE754Single))
// Von-Wert
#define ParamLOG_fE1LowDeltaDouble                   (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1LowDeltaDouble), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE1HighDeltaDouble                  (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1HighDeltaDouble), Float_Enc_IEEE754Single))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low0Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Valid)) & LOG_fE1Low0ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low1Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Valid)) & LOG_fE1Low1ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low2Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Valid)) & LOG_fE1Low2ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low3Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Valid)) & LOG_fE1Low3ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low4Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low4Valid)) & LOG_fE1Low4ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low5Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low5Valid)) & LOG_fE1Low5ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE1Low6Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low6Valid)) & LOG_fE1Low6ValidMask))
// Eingang ist EIN, wenn Wert gleich
#define ParamLOG_fE1Low0Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt2)))
// ... oder wenn Wert gleich 
#define ParamLOG_fE1Low1Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt2)))
// ... oder wenn Wert gleich 
#define ParamLOG_fE1Low2Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt2)))
// ... oder wenn Wert gleich 
#define ParamLOG_fE1Low3Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt2)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt2Fix                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt2Fix)))
// Eingang ist EIN, wenn Wert gleich
#define ParamLOG_fE1Low0Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt3Dir)) & LOG_fE1Low0Dpt3DirMask) >> LOG_fE1Low0Dpt3DirShift)
// 
#define ParamLOG_fE1Low0Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt3Dim)) & LOG_fE1Low0Dpt3DimMask)
// ... oder wenn Wert gleich 
#define ParamLOG_fE1Low1Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt3Dir)) & LOG_fE1Low1Dpt3DirMask) >> LOG_fE1Low1Dpt3DirShift)
// 
#define ParamLOG_fE1Low1Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt3Dim)) & LOG_fE1Low1Dpt3DimMask)
// ... oder wenn Wert gleich 
#define ParamLOG_fE1Low2Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt3Dir)) & LOG_fE1Low2Dpt3DirMask) >> LOG_fE1Low2Dpt3DirShift)
// 
#define ParamLOG_fE1Low2Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt3Dim)) & LOG_fE1Low2Dpt3DimMask)
// ... oder wenn Wert gleich 
#define ParamLOG_fE1Low3Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt3Dir)) & LOG_fE1Low3Dpt3DirMask) >> LOG_fE1Low3Dpt3DirShift)
// 
#define ParamLOG_fE1Low3Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt3Dim)) & LOG_fE1Low3Dpt3DimMask)
// Eingang ist konstant
#define ParamLOG_fE1LowDpt3FixDir                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt3FixDir)) & LOG_fE1LowDpt3FixDirMask) >> LOG_fE1LowDpt3FixDirShift)
// 
#define ParamLOG_fE1LowDpt3FixDim                    (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt3FixDim)) & LOG_fE1LowDpt3FixDimMask)
// Von-Wert
#define ParamLOG_fE1LowDpt5                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt5)))
// Bis-Wert
#define ParamLOG_fE1HighDpt5                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1HighDpt5)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE1Low0Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE1Low1Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE1Low2Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE1Low3Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE1Low4Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low4Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE1Low5Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low5Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE1Low6Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low6Dpt5In)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt5Fix                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt5Fix)))
// Von-Wert
#define ParamLOG_fE1LowDpt5001                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt5001)))
// Bis-Wert
#define ParamLOG_fE1HighDpt5001                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1HighDpt5001)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE1Low0Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE1Low1Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE1Low2Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE1Low3Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE1Low4Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low4Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE1Low5Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low5Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE1Low6Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low6Dpt5xIn)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt5xFix                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt5xFix)))
// Von-Wert
#define ParamLOG_fE1LowDpt6                          ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt6)))
// Bis-Wert
#define ParamLOG_fE1HighDpt6                         ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1HighDpt6)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE1Low0Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE1Low1Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE1Low2Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE1Low3Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE1Low4Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low4Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE1Low5Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low5Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE1Low6Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low6Dpt6In)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt6Fix                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt6Fix)))
// Von-Wert
#define ParamLOG_fE1LowDpt7                          (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1LowDpt7)))
// Bis-Wert
#define ParamLOG_fE1HighDpt7                         (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1HighDpt7)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE1Low0Dpt7In                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1Low0Dpt7In)))
// ... oder bei Wert
#define ParamLOG_fE1Low1Dpt7In                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1Low1Dpt7In)))
// ... oder bei Wert
#define ParamLOG_fE1Low2Dpt7In                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1Low2Dpt7In)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt7Fix                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE1LowDpt7Fix)))
// Von-Wert
#define ParamLOG_fE1LowDpt8                          ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1LowDpt8)))
// Bis-Wert
#define ParamLOG_fE1HighDpt8                         ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1HighDpt8)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE1Low0Dpt8In                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1Low0Dpt8In)))
// ... oder bei Wert
#define ParamLOG_fE1Low1Dpt8In                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1Low1Dpt8In)))
// ... oder bei Wert
#define ParamLOG_fE1Low2Dpt8In                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1Low2Dpt8In)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt8Fix                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE1LowDpt8Fix)))
// Von-Wert
#define ParamLOG_fE1LowDpt9                          (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1LowDpt9), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE1HighDpt9                         (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1HighDpt9), Float_Enc_IEEE754Single))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt9Fix                       (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1LowDpt9Fix), Float_Enc_IEEE754Single))
// Von-Wert
#define ParamLOG_fE1LowDpt12                         (knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDpt12)))
// Bis-Wert
#define ParamLOG_fE1HighDpt12                        (knx.paramInt(LOG_ParamCalcIndex(LOG_fE1HighDpt12)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt12Fix                      (knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDpt12Fix)))
// Von-Wert
#define ParamLOG_fE1LowDpt13                         ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDpt13)))
// Bis-Wert
#define ParamLOG_fE1HighDpt13                        ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1HighDpt13)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt13Fix                      ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDpt13Fix)))
// Von-Wert
#define ParamLOG_fE1LowDpt14                         (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1LowDpt14), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE1HighDpt14                        (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1HighDpt14), Float_Enc_IEEE754Single))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt14Fix                      (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE1LowDpt14Fix), Float_Enc_IEEE754Single))
// Eingang ist EIN bei Szene
#define ParamLOG_fE1Low0Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low0Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low1Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low1Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low2Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low2Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low3Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low3Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low4Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low4Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low5Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low5Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low6Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low6Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE1Low7Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1Low7Dpt17)))
// Eingang ist konstant
#define ParamLOG_fE1LowDpt17Fix                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE1LowDpt17Fix)))
// Von-Wert
#define ParamLOG_fE1LowDptRGB                        ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDptRGB)))
// Bis-Wert
#define ParamLOG_fE1HighDptRGB                       ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1HighDptRGB)))
// Eingang ist konstant
#define ParamLOG_fE1LowDptRGBFix                     ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE1LowDptRGBFix)))
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE2ConvertInt                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2ConvertInt)) & LOG_fE2ConvertIntMask) >> LOG_fE2ConvertIntShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE2Convert                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Convert)) & LOG_fE2ConvertMask) >> LOG_fE2ConvertShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE2ConvertFloat                     ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2ConvertFloat)) & LOG_fE2ConvertFloatMask) >> LOG_fE2ConvertFloatShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE2ConvertSpecial                   ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2ConvertSpecial)) & LOG_fE2ConvertSpecialMask) >> LOG_fE2ConvertSpecialShift)
// Wert für Eingang wird ermittelt durch
#define ParamLOG_fE2ConvertBool                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2ConvertBool)) & LOG_fE2ConvertBoolMask) >> LOG_fE2ConvertBoolShift)
// Eingang 2
#define ParamLOG_fE2                                 (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2)) & LOG_fE2Mask)
// DPT für Eingang
#define ParamLOG_fE2Dpt                              (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Dpt)))
// Eingang wird gelesen alle
#define ParamLOG_fE2RepeatBase                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2RepeatBase)) & LOG_fE2RepeatBaseMask) >> LOG_fE2RepeatBaseShift)
// Eingang wird gelesen alle
#define ParamLOG_fE2RepeatTime                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2RepeatTime)) & LOG_fE2RepeatTimeMask)
// Eingang wird gelesen alle (in Millisekunden)
#define ParamLOG_fE2RepeatTimeMS                     (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fE2RepeatTime))))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fE2OtherKO                          (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2OtherKO)))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fE2OtherKORel                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2OtherKORel)))
// Falls Vorbelegung aus dem Speicher nicht möglich oder nicht gewünscht, dann vorbelegen mit
#define ParamLOG_fE2Default                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Default)) & LOG_fE2DefaultMask)
// Eingang vorbelegen mit
#define ParamLOG_fE2DefaultExt                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2DefaultExt)) & LOG_fE2DefaultExtMask)
// Eingangswert speichern und beim nächsten Neustart als Vorbelegung nutzen?
#define ParamLOG_fE2DefaultEEPROM                    ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2DefaultEEPROM)) & LOG_fE2DefaultEEPROMMask))
// Nur so lange zyklisch lesen, bis erstes Telegramm eingeht
#define ParamLOG_fE2DefaultRepeat                    ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2DefaultRepeat)) & LOG_fE2DefaultRepeatMask))
// Kommunikationsobjekt für Eingang
#define ParamLOG_fE2UseOtherKO                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2UseOtherKO)) & LOG_fE2UseOtherKOMask) >> LOG_fE2UseOtherKOShift)
// Von-Wert
#define ParamLOG_fE2LowDelta                         ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDelta)))
// Bis-Wert
#define ParamLOG_fE2HighDelta                        ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2HighDelta)))
// Von-Wert
#define ParamLOG_fE2LowDeltaFloat                    (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2LowDeltaFloat), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE2HighDeltaFloat                   (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2HighDeltaFloat), Float_Enc_IEEE754Single))
// Von-Wert
#define ParamLOG_fE2LowDeltaDouble                   (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2LowDeltaDouble), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE2HighDeltaDouble                  (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2HighDeltaDouble), Float_Enc_IEEE754Single))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low0Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Valid)) & LOG_fE2Low0ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low1Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Valid)) & LOG_fE2Low1ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low2Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Valid)) & LOG_fE2Low2ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low3Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Valid)) & LOG_fE2Low3ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low4Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low4Valid)) & LOG_fE2Low4ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low5Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low5Valid)) & LOG_fE2Low5ValidMask))
// Nächste Zeile auswerten?
#define ParamLOG_fE2Low6Valid                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low6Valid)) & LOG_fE2Low6ValidMask))
// Eingang ist EIN, wenn Wert gleich
#define ParamLOG_fE2Low0Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt2)))
// ... oder wenn Wert gleich 
#define ParamLOG_fE2Low1Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt2)))
// ... oder wenn Wert gleich 
#define ParamLOG_fE2Low2Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt2)))
// ... oder wenn Wert gleich 
#define ParamLOG_fE2Low3Dpt2                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt2)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt2Fix                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt2Fix)))
// Eingang ist EIN, wenn Wert gleich
#define ParamLOG_fE2Low0Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt3Dir)) & LOG_fE2Low0Dpt3DirMask) >> LOG_fE2Low0Dpt3DirShift)
// 
#define ParamLOG_fE2Low0Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt3Dim)) & LOG_fE2Low0Dpt3DimMask)
// ... oder wenn Wert gleich 
#define ParamLOG_fE2Low1Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt3Dir)) & LOG_fE2Low1Dpt3DirMask) >> LOG_fE2Low1Dpt3DirShift)
// 
#define ParamLOG_fE2Low1Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt3Dim)) & LOG_fE2Low1Dpt3DimMask)
// ... oder wenn Wert gleich 
#define ParamLOG_fE2Low2Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt3Dir)) & LOG_fE2Low2Dpt3DirMask) >> LOG_fE2Low2Dpt3DirShift)
// 
#define ParamLOG_fE2Low2Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt3Dim)) & LOG_fE2Low2Dpt3DimMask)
// ... oder wenn Wert gleich 
#define ParamLOG_fE2Low3Dpt3Dir                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt3Dir)) & LOG_fE2Low3Dpt3DirMask) >> LOG_fE2Low3Dpt3DirShift)
// 
#define ParamLOG_fE2Low3Dpt3Dim                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt3Dim)) & LOG_fE2Low3Dpt3DimMask)
// Eingang ist konstant
#define ParamLOG_fE2LowDpt3FixDir                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt3FixDir)) & LOG_fE2LowDpt3FixDirMask) >> LOG_fE2LowDpt3FixDirShift)
// 
#define ParamLOG_fE2LowDpt3FixDim                    (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt3FixDim)) & LOG_fE2LowDpt3FixDimMask)
// Von-Wert
#define ParamLOG_fE2LowDpt5                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt5)))
// Bis-Wert
#define ParamLOG_fE2HighDpt5                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2HighDpt5)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE2Low0Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE2Low1Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE2Low2Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE2Low3Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE2Low4Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low4Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE2Low5Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low5Dpt5In)))
// ... oder bei Wert
#define ParamLOG_fE2Low6Dpt5In                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low6Dpt5In)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt5Fix                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt5Fix)))
// Von-Wert
#define ParamLOG_fE2LowDpt5001                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt5001)))
// Bis-Wert
#define ParamLOG_fE2HighDpt5001                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2HighDpt5001)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE2Low0Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE2Low1Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE2Low2Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE2Low3Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE2Low4Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low4Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE2Low5Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low5Dpt5xIn)))
// ... oder bei Wert
#define ParamLOG_fE2Low6Dpt5xIn                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low6Dpt5xIn)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt5xFix                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt5xFix)))
// Von-Wert
#define ParamLOG_fE2LowDpt6                          ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt6)))
// Bis-Wert
#define ParamLOG_fE2HighDpt6                         ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2HighDpt6)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE2Low0Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE2Low1Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE2Low2Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE2Low3Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE2Low4Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low4Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE2Low5Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low5Dpt6In)))
// ... oder bei Wert
#define ParamLOG_fE2Low6Dpt6In                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low6Dpt6In)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt6Fix                       ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt6Fix)))
// Von-Wert
#define ParamLOG_fE2LowDpt7                          (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2LowDpt7)))
// Bis-Wert
#define ParamLOG_fE2HighDpt7                         (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2HighDpt7)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE2Low0Dpt7In                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2Low0Dpt7In)))
// ... oder bei Wert
#define ParamLOG_fE2Low1Dpt7In                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2Low1Dpt7In)))
// ... oder bei Wert
#define ParamLOG_fE2Low2Dpt7In                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2Low2Dpt7In)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt7Fix                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fE2LowDpt7Fix)))
// Von-Wert
#define ParamLOG_fE2LowDpt8                          ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2LowDpt8)))
// Bis-Wert
#define ParamLOG_fE2HighDpt8                         ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2HighDpt8)))
// Eingang ist EIN bei Wert
#define ParamLOG_fE2Low0Dpt8In                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2Low0Dpt8In)))
// ... oder bei Wert
#define ParamLOG_fE2Low1Dpt8In                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2Low1Dpt8In)))
// ... oder bei Wert
#define ParamLOG_fE2Low2Dpt8In                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2Low2Dpt8In)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt8Fix                       ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fE2LowDpt8Fix)))
// Von-Wert
#define ParamLOG_fE2LowDpt9                          (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2LowDpt9), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE2HighDpt9                         (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2HighDpt9), Float_Enc_IEEE754Single))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt9Fix                       (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2LowDpt9Fix), Float_Enc_IEEE754Single))
// Von-Wert
#define ParamLOG_fE2LowDpt12                         (knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDpt12)))
// Bis-Wert
#define ParamLOG_fE2HighDpt12                        (knx.paramInt(LOG_ParamCalcIndex(LOG_fE2HighDpt12)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt12Fix                      (knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDpt12Fix)))
// Von-Wert
#define ParamLOG_fE2LowDpt13                         ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDpt13)))
// Bis-Wert
#define ParamLOG_fE2HighDpt13                        ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2HighDpt13)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt13Fix                      ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDpt13Fix)))
// Von-Wert
#define ParamLOG_fE2LowDpt14                         (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2LowDpt14), Float_Enc_IEEE754Single))
// Bis-Wert
#define ParamLOG_fE2HighDpt14                        (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2HighDpt14), Float_Enc_IEEE754Single))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt14Fix                      (knx.paramFloat(LOG_ParamCalcIndex(LOG_fE2LowDpt14Fix), Float_Enc_IEEE754Single))
// Eingang ist EIN bei Szene
#define ParamLOG_fE2Low0Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low0Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low1Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low1Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low2Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low2Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low3Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low3Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low4Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low4Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low5Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low5Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low6Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low6Dpt17)))
// ... oder bei Szene
#define ParamLOG_fE2Low7Dpt17                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2Low7Dpt17)))
// Eingang ist konstant
#define ParamLOG_fE2LowDpt17Fix                      (knx.paramByte(LOG_ParamCalcIndex(LOG_fE2LowDpt17Fix)))
// Von-Wert
#define ParamLOG_fE2LowDptRGB                        ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDptRGB)))
// Bis-Wert
#define ParamLOG_fE2HighDptRGB                       ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2HighDptRGB)))
// Eingang ist konstant
#define ParamLOG_fE2LowDptRGBFix                     ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fE2LowDptRGBFix)))
// Zeitbezug
#define ParamLOG_fTd1DuskDawn                        ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1DuskDawn)) & LOG_fTd1DuskDawnMask) >> LOG_fTd1DuskDawnShift)
// Zeitbezug
#define ParamLOG_fTd2DuskDawn                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2DuskDawn)) & LOG_fTd2DuskDawnMask)
// Zeitbezug
#define ParamLOG_fTd3DuskDawn                        ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3DuskDawn)) & LOG_fTd3DuskDawnMask) >> LOG_fTd3DuskDawnShift)
// Zeitbezug
#define ParamLOG_fTd4DuskDawn                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4DuskDawn)) & LOG_fTd4DuskDawnMask)
// Zeitbezug
#define ParamLOG_fTd5DuskDawn                        ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5DuskDawn)) & LOG_fTd5DuskDawnMask) >> LOG_fTd5DuskDawnShift)
// Zeitbezug
#define ParamLOG_fTd6DuskDawn                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6DuskDawn)) & LOG_fTd6DuskDawnMask)
// Zeitbezug
#define ParamLOG_fTd7DuskDawn                        ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7DuskDawn)) & LOG_fTd7DuskDawnMask) >> LOG_fTd7DuskDawnShift)
// Zeitbezug
#define ParamLOG_fTd8DuskDawn                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8DuskDawn)) & LOG_fTd8DuskDawnMask)
// Typ der Zeitschaltuhr
#define ParamLOG_fTYearDay                           ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTYearDay)) & LOG_fTYearDayMask) >> LOG_fTYearDayShift)
// Feiertagsbehandlung
#define ParamLOG_fTHoliday                           ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTHoliday)) & LOG_fTHolidayMask) >> LOG_fTHolidayShift)
// Bei Neustart letzte Schaltzeit nachholen
#define ParamLOG_fTRestoreState                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTRestoreState)) & LOG_fTRestoreStateMask) >> LOG_fTRestoreStateShift)
// Urlaubsbehandlung
#define ParamLOG_fTVacation                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fTVacation)) & LOG_fTVacationMask)
// Zahlenwert
#define ParamLOG_fTd1ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1ValueNum)))
// Zahlenwert
#define ParamLOG_fTd2ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2ValueNum)))
// Zahlenwert
#define ParamLOG_fTd3ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3ValueNum)))
// Zahlenwert
#define ParamLOG_fTd4ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4ValueNum)))
// Zahlenwert
#define ParamLOG_fTd5ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5ValueNum)))
// Zahlenwert
#define ParamLOG_fTd6ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6ValueNum)))
// Zahlenwert
#define ParamLOG_fTd7ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7ValueNum)))
// Zahlenwert
#define ParamLOG_fTd8ValueNum                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8ValueNum)))
// Schaltwert
#define ParamLOG_fTd1Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1Value)) & LOG_fTd1ValueMask))
// Grad
#define ParamLOG_fTd1Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1Degree)) & LOG_fTd1DegreeMask) >> LOG_fTd1DegreeShift)
// Stunde
#define ParamLOG_fTd1HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1HourAbs)) & LOG_fTd1HourAbsMask) >> LOG_fTd1HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd1HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1HourRel)) & LOG_fTd1HourRelMask) >> LOG_fTd1HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd1HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1HourRelShort)) & LOG_fTd1HourRelShortMask) >> LOG_fTd1HourRelShortShift)
// Minute
#define ParamLOG_fTd1MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1MinuteAbs)))
// Minute
#define ParamLOG_fTd1MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1MinuteRel)))
// Wochentag
#define ParamLOG_fTd1Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd1Weekday)) & LOG_fTd1WeekdayMask)
// Schaltwert
#define ParamLOG_fTd2Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2Value)) & LOG_fTd2ValueMask))
// Grad
#define ParamLOG_fTd2Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2Degree)) & LOG_fTd2DegreeMask) >> LOG_fTd2DegreeShift)
// Stunde
#define ParamLOG_fTd2HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2HourAbs)) & LOG_fTd2HourAbsMask) >> LOG_fTd2HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd2HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2HourRel)) & LOG_fTd2HourRelMask) >> LOG_fTd2HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd2HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2HourRelShort)) & LOG_fTd2HourRelShortMask) >> LOG_fTd2HourRelShortShift)
// Minute
#define ParamLOG_fTd2MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2MinuteAbs)))
// Minute
#define ParamLOG_fTd2MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2MinuteRel)))
// Wochentag
#define ParamLOG_fTd2Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd2Weekday)) & LOG_fTd2WeekdayMask)
// Schaltwert
#define ParamLOG_fTd3Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3Value)) & LOG_fTd3ValueMask))
// Grad
#define ParamLOG_fTd3Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3Degree)) & LOG_fTd3DegreeMask) >> LOG_fTd3DegreeShift)
// Stunde
#define ParamLOG_fTd3HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3HourAbs)) & LOG_fTd3HourAbsMask) >> LOG_fTd3HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd3HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3HourRel)) & LOG_fTd3HourRelMask) >> LOG_fTd3HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd3HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3HourRelShort)) & LOG_fTd3HourRelShortMask) >> LOG_fTd3HourRelShortShift)
// Minute
#define ParamLOG_fTd3MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3MinuteAbs)))
// Minute
#define ParamLOG_fTd3MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3MinuteRel)))
// Wochentag
#define ParamLOG_fTd3Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd3Weekday)) & LOG_fTd3WeekdayMask)
// Schaltwert
#define ParamLOG_fTd4Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4Value)) & LOG_fTd4ValueMask))
// Grad
#define ParamLOG_fTd4Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4Degree)) & LOG_fTd4DegreeMask) >> LOG_fTd4DegreeShift)
// Stunde
#define ParamLOG_fTd4HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4HourAbs)) & LOG_fTd4HourAbsMask) >> LOG_fTd4HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd4HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4HourRel)) & LOG_fTd4HourRelMask) >> LOG_fTd4HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd4HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4HourRelShort)) & LOG_fTd4HourRelShortMask) >> LOG_fTd4HourRelShortShift)
// Minute
#define ParamLOG_fTd4MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4MinuteAbs)))
// Minute
#define ParamLOG_fTd4MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4MinuteRel)))
// Wochentag
#define ParamLOG_fTd4Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd4Weekday)) & LOG_fTd4WeekdayMask)
// Schaltwert
#define ParamLOG_fTd5Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5Value)) & LOG_fTd5ValueMask))
// Grad
#define ParamLOG_fTd5Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5Degree)) & LOG_fTd5DegreeMask) >> LOG_fTd5DegreeShift)
// Stunde
#define ParamLOG_fTd5HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5HourAbs)) & LOG_fTd5HourAbsMask) >> LOG_fTd5HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd5HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5HourRel)) & LOG_fTd5HourRelMask) >> LOG_fTd5HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd5HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5HourRelShort)) & LOG_fTd5HourRelShortMask) >> LOG_fTd5HourRelShortShift)
// Minute
#define ParamLOG_fTd5MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5MinuteAbs)))
// Minute
#define ParamLOG_fTd5MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5MinuteRel)))
// Wochentag
#define ParamLOG_fTd5Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd5Weekday)) & LOG_fTd5WeekdayMask)
// Schaltwert
#define ParamLOG_fTd6Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6Value)) & LOG_fTd6ValueMask))
// Grad
#define ParamLOG_fTd6Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6Degree)) & LOG_fTd6DegreeMask) >> LOG_fTd6DegreeShift)
// Stunde
#define ParamLOG_fTd6HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6HourAbs)) & LOG_fTd6HourAbsMask) >> LOG_fTd6HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd6HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6HourRel)) & LOG_fTd6HourRelMask) >> LOG_fTd6HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd6HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6HourRelShort)) & LOG_fTd6HourRelShortMask) >> LOG_fTd6HourRelShortShift)
// Minute
#define ParamLOG_fTd6MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6MinuteAbs)))
// Minute
#define ParamLOG_fTd6MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6MinuteRel)))
// Wochentag
#define ParamLOG_fTd6Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd6Weekday)) & LOG_fTd6WeekdayMask)
// Schaltwert
#define ParamLOG_fTd7Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7Value)) & LOG_fTd7ValueMask))
// Grad
#define ParamLOG_fTd7Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7Degree)) & LOG_fTd7DegreeMask) >> LOG_fTd7DegreeShift)
// Stunde
#define ParamLOG_fTd7HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7HourAbs)) & LOG_fTd7HourAbsMask) >> LOG_fTd7HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd7HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7HourRel)) & LOG_fTd7HourRelMask) >> LOG_fTd7HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd7HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7HourRelShort)) & LOG_fTd7HourRelShortMask) >> LOG_fTd7HourRelShortShift)
// Minute
#define ParamLOG_fTd7MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7MinuteAbs)))
// Minute
#define ParamLOG_fTd7MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7MinuteRel)))
// Wochentag
#define ParamLOG_fTd7Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd7Weekday)) & LOG_fTd7WeekdayMask)
// Schaltwert
#define ParamLOG_fTd8Value                           ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8Value)) & LOG_fTd8ValueMask))
// Grad
#define ParamLOG_fTd8Degree                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8Degree)) & LOG_fTd8DegreeMask) >> LOG_fTd8DegreeShift)
// Stunde
#define ParamLOG_fTd8HourAbs                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8HourAbs)) & LOG_fTd8HourAbsMask) >> LOG_fTd8HourAbsShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd8HourRel                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8HourRel)) & LOG_fTd8HourRelMask) >> LOG_fTd8HourRelShift)
// Sonnen auf-/untergang
#define ParamLOG_fTd8HourRelShort                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8HourRelShort)) & LOG_fTd8HourRelShortMask) >> LOG_fTd8HourRelShortShift)
// Minute
#define ParamLOG_fTd8MinuteAbs                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8MinuteAbs)))
// Minute
#define ParamLOG_fTd8MinuteRel                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8MinuteRel)))
// Wochentag
#define ParamLOG_fTd8Weekday                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fTd8Weekday)) & LOG_fTd8WeekdayMask)
// Mo
#define ParamLOG_fTy1Weekday1                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday1)) & LOG_fTy1Weekday1Mask))
// Di
#define ParamLOG_fTy1Weekday2                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday2)) & LOG_fTy1Weekday2Mask))
// Mi
#define ParamLOG_fTy1Weekday3                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday3)) & LOG_fTy1Weekday3Mask))
// Do
#define ParamLOG_fTy1Weekday4                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday4)) & LOG_fTy1Weekday4Mask))
// Fr
#define ParamLOG_fTy1Weekday5                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday5)) & LOG_fTy1Weekday5Mask))
// Sa
#define ParamLOG_fTy1Weekday6                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday6)) & LOG_fTy1Weekday6Mask))
// So
#define ParamLOG_fTy1Weekday7                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Weekday7)) & LOG_fTy1Weekday7Mask))
// Tag
#define ParamLOG_fTy1Day                             ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Day)) & LOG_fTy1DayMask) >> LOG_fTy1DayShift)
// Wochentag
#define ParamLOG_fTy1IsWeekday                       ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1IsWeekday)) & LOG_fTy1IsWeekdayMask))
// Monat
#define ParamLOG_fTy1Month                           ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy1Month)) & LOG_fTy1MonthMask) >> LOG_fTy1MonthShift)
// Mo
#define ParamLOG_fTy2Weekday1                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday1)) & LOG_fTy2Weekday1Mask))
// Di
#define ParamLOG_fTy2Weekday2                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday2)) & LOG_fTy2Weekday2Mask))
// Mi
#define ParamLOG_fTy2Weekday3                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday3)) & LOG_fTy2Weekday3Mask))
// Do
#define ParamLOG_fTy2Weekday4                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday4)) & LOG_fTy2Weekday4Mask))
// Fr
#define ParamLOG_fTy2Weekday5                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday5)) & LOG_fTy2Weekday5Mask))
// Sa
#define ParamLOG_fTy2Weekday6                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday6)) & LOG_fTy2Weekday6Mask))
// So
#define ParamLOG_fTy2Weekday7                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Weekday7)) & LOG_fTy2Weekday7Mask))
// Tag
#define ParamLOG_fTy2Day                             ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Day)) & LOG_fTy2DayMask) >> LOG_fTy2DayShift)
// Wochentag
#define ParamLOG_fTy2IsWeekday                       ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2IsWeekday)) & LOG_fTy2IsWeekdayMask))
// Monat
#define ParamLOG_fTy2Month                           ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy2Month)) & LOG_fTy2MonthMask) >> LOG_fTy2MonthShift)
// Mo
#define ParamLOG_fTy3Weekday1                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday1)) & LOG_fTy3Weekday1Mask))
// Di
#define ParamLOG_fTy3Weekday2                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday2)) & LOG_fTy3Weekday2Mask))
// Mi
#define ParamLOG_fTy3Weekday3                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday3)) & LOG_fTy3Weekday3Mask))
// Do
#define ParamLOG_fTy3Weekday4                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday4)) & LOG_fTy3Weekday4Mask))
// Fr
#define ParamLOG_fTy3Weekday5                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday5)) & LOG_fTy3Weekday5Mask))
// Sa
#define ParamLOG_fTy3Weekday6                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday6)) & LOG_fTy3Weekday6Mask))
// So
#define ParamLOG_fTy3Weekday7                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Weekday7)) & LOG_fTy3Weekday7Mask))
// Tag
#define ParamLOG_fTy3Day                             ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Day)) & LOG_fTy3DayMask) >> LOG_fTy3DayShift)
// Wochentag
#define ParamLOG_fTy3IsWeekday                       ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3IsWeekday)) & LOG_fTy3IsWeekdayMask))
// Monat
#define ParamLOG_fTy3Month                           ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy3Month)) & LOG_fTy3MonthMask) >> LOG_fTy3MonthShift)
// Mo
#define ParamLOG_fTy4Weekday1                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday1)) & LOG_fTy4Weekday1Mask))
// Di
#define ParamLOG_fTy4Weekday2                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday2)) & LOG_fTy4Weekday2Mask))
// Mi
#define ParamLOG_fTy4Weekday3                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday3)) & LOG_fTy4Weekday3Mask))
// Do
#define ParamLOG_fTy4Weekday4                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday4)) & LOG_fTy4Weekday4Mask))
// Fr
#define ParamLOG_fTy4Weekday5                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday5)) & LOG_fTy4Weekday5Mask))
// Sa
#define ParamLOG_fTy4Weekday6                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday6)) & LOG_fTy4Weekday6Mask))
// So
#define ParamLOG_fTy4Weekday7                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Weekday7)) & LOG_fTy4Weekday7Mask))
// Tag
#define ParamLOG_fTy4Day                             ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Day)) & LOG_fTy4DayMask) >> LOG_fTy4DayShift)
// Wochentag
#define ParamLOG_fTy4IsWeekday                       ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4IsWeekday)) & LOG_fTy4IsWeekdayMask))
// Monat
#define ParamLOG_fTy4Month                           ((knx.paramByte(LOG_ParamCalcIndex(LOG_fTy4Month)) & LOG_fTy4MonthMask) >> LOG_fTy4MonthShift)
// Interner Eingang 3
#define ParamLOG_fI1                                 ((knx.paramByte(LOG_ParamCalcIndex(LOG_fI1)) & LOG_fI1Mask) >> LOG_fI1Shift)
// Art der Verknüpfung
#define ParamLOG_fI1Kind                             ((knx.paramByte(LOG_ParamCalcIndex(LOG_fI1Kind)) & LOG_fI1KindMask) >> LOG_fI1KindShift)
// Internen Eingang als Trigger nutzen(ist immer logisch EIN)
#define ParamLOG_fI1AsTrigger                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fI1AsTrigger)) & LOG_fI1AsTriggerMask))
// Internen Eingang verbinden mit Kanal Nr.
#define ParamLOG_fI1Function                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fI1Function)))
// Internen Eingang verbinden mit Kanal Nr.
#define ParamLOG_fI1FunctionRel                      ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fI1FunctionRel)))
// Interner Eingang 4
#define ParamLOG_fI2                                 ((knx.paramByte(LOG_ParamCalcIndex(LOG_fI2)) & LOG_fI2Mask) >> LOG_fI2Shift)
// Art der Verknüpfung
#define ParamLOG_fI2Kind                             ((knx.paramByte(LOG_ParamCalcIndex(LOG_fI2Kind)) & LOG_fI2KindMask) >> LOG_fI2KindShift)
// Internen Eingang als Trigger nutzen(ist immer logisch EIN)
#define ParamLOG_fI2AsTrigger                        ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fI2AsTrigger)) & LOG_fI2AsTriggerMask))
// Internen Eingang verbinden mit Kanal Nr.
#define ParamLOG_fI2Function                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fI2Function)))
// Internen Eingang verbinden mit Kanal Nr.
#define ParamLOG_fI2FunctionRel                      ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fI2FunctionRel)))
// Zeit für Treppenlicht
#define ParamLOG_fOStairtimeBase                     ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOStairtimeBase)) & LOG_fOStairtimeBaseMask) >> LOG_fOStairtimeBaseShift)
// Zeit für Treppenlicht
#define ParamLOG_fOStairtimeTime                     (knx.paramWord(LOG_ParamCalcIndex(LOG_fOStairtimeTime)) & LOG_fOStairtimeTimeMask)
// Zeit für Treppenlicht (in Millisekunden)
#define ParamLOG_fOStairtimeTimeMS                   (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fOStairtimeTime))))
// Treppenlicht blinkt im Rhythmus
#define ParamLOG_fOBlinkBase                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOBlinkBase)) & LOG_fOBlinkBaseMask) >> LOG_fOBlinkBaseShift)
// Treppenlicht blinkt im Rhythmus
#define ParamLOG_fOBlinkTime                         (knx.paramWord(LOG_ParamCalcIndex(LOG_fOBlinkTime)) & LOG_fOBlinkTimeMask)
// Treppenlicht blinkt im Rhythmus (in Millisekunden)
#define ParamLOG_fOBlinkTimeMS                       (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fOBlinkTime))))
// EINschalten wird verzögert um
#define ParamLOG_fODelayOnBase                       ((knx.paramByte(LOG_ParamCalcIndex(LOG_fODelayOnBase)) & LOG_fODelayOnBaseMask) >> LOG_fODelayOnBaseShift)
// EINschalten wird verzögert um
#define ParamLOG_fODelayOnTime                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fODelayOnTime)) & LOG_fODelayOnTimeMask)
// EINschalten wird verzögert um (in Millisekunden)
#define ParamLOG_fODelayOnTimeMS                     (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fODelayOnTime))))
// AUSschalten wird verzögert um
#define ParamLOG_fODelayOffBase                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fODelayOffBase)) & LOG_fODelayOffBaseMask) >> LOG_fODelayOffBaseShift)
// AUSschalten wird verzögert um
#define ParamLOG_fODelayOffTime                      (knx.paramWord(LOG_ParamCalcIndex(LOG_fODelayOffTime)) & LOG_fODelayOffTimeMask)
// AUSschalten wird verzögert um (in Millisekunden)
#define ParamLOG_fODelayOffTimeMS                    (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fODelayOffTime))))
// EIN-Telegramm wird wiederholt alle
#define ParamLOG_fORepeatOnBase                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fORepeatOnBase)) & LOG_fORepeatOnBaseMask) >> LOG_fORepeatOnBaseShift)
// EIN-Telegramm wird wiederholt alle
#define ParamLOG_fORepeatOnTime                      (knx.paramWord(LOG_ParamCalcIndex(LOG_fORepeatOnTime)) & LOG_fORepeatOnTimeMask)
// EIN-Telegramm wird wiederholt alle (in Millisekunden)
#define ParamLOG_fORepeatOnTimeMS                    (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fORepeatOnTime))))
// AUS-Telegramm wird wiederholt alle
#define ParamLOG_fORepeatOffBase                     ((knx.paramByte(LOG_ParamCalcIndex(LOG_fORepeatOffBase)) & LOG_fORepeatOffBaseMask) >> LOG_fORepeatOffBaseShift)
// AUS-Telegramm wird wiederholt alle
#define ParamLOG_fORepeatOffTime                     (knx.paramWord(LOG_ParamCalcIndex(LOG_fORepeatOffTime)) & LOG_fORepeatOffTimeMask)
// AUS-Telegramm wird wiederholt alle (in Millisekunden)
#define ParamLOG_fORepeatOffTimeMS                   (paramDelay(knx.paramWord(LOG_ParamCalcIndex(LOG_fORepeatOffTime))))
// Ausgang schaltet zeitverzögert
#define ParamLOG_fODelay                             ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fODelay)) & LOG_fODelayMask))
// Erneutes EIN führt zu
#define ParamLOG_fODelayOnRepeat                     ((knx.paramByte(LOG_ParamCalcIndex(LOG_fODelayOnRepeat)) & LOG_fODelayOnRepeatMask) >> LOG_fODelayOnRepeatShift)
// Darauffolgendes AUS führt zu
#define ParamLOG_fODelayOnReset                      ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fODelayOnReset)) & LOG_fODelayOnResetMask))
// Erneutes AUS führt zu
#define ParamLOG_fODelayOffRepeat                    ((knx.paramByte(LOG_ParamCalcIndex(LOG_fODelayOffRepeat)) & LOG_fODelayOffRepeatMask) >> LOG_fODelayOffRepeatShift)
// Darauffolgendes EIN führt zu
#define ParamLOG_fODelayOffReset                     ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fODelayOffReset)) & LOG_fODelayOffResetMask))
// Ausgang hat eine Treppenlichtfunktion
#define ParamLOG_fOStair                             ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fOStair)) & LOG_fOStairMask))
// Treppenlicht kann verlängert werden
#define ParamLOG_fORetrigger                         ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fORetrigger)) & LOG_fORetriggerMask))
// Treppenlicht kann ausgeschaltet werden
#define ParamLOG_fOStairOff                          ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fOStairOff)) & LOG_fOStairOffMask))
// Ausgang wiederholt zyklisch
#define ParamLOG_fORepeat                            ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fORepeat)) & LOG_fORepeatMask))
// Wiederholungsfilter
#define ParamLOG_fOOutputFilter                      ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOutputFilter)) & LOG_fOOutputFilterMask) >> LOG_fOOutputFilterShift)
// Sendeverhalten für Ausgang
#define ParamLOG_fOSendOnChange                      ((bool)(knx.paramByte(LOG_ParamCalcIndex(LOG_fOSendOnChange)) & LOG_fOSendOnChangeMask))
// DPT für Ausgang
#define ParamLOG_fODpt                               (knx.paramByte(LOG_ParamCalcIndex(LOG_fODpt)))
// Wert für EIN senden?
#define ParamLOG_fOOn                                (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOn)))
// Wert für EIN senden?
#define ParamLOG_fOOnBuzzer                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnBuzzer)))
// Wert für EIN senden?
#define ParamLOG_fOOnLed                             (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnLed)))
// Wert für EIN senden?
#define ParamLOG_fOOnAll                             (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnAll)))
//     Wert für EIN senden als
#define ParamLOG_fOOnTone                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnTone)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt1                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt1)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt2                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt2)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt3Dir                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt3Dir)) & LOG_fOOnDpt3DirMask) >> LOG_fOOnDpt3DirShift)
// 
#define ParamLOG_fOOnDpt3Dim                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt3Dim)) & LOG_fOOnDpt3DimMask)
//     Wert für EIN senden als 
#define ParamLOG_fOOnDpt5                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt5)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt5001                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt5001)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt6                            ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt6)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt7                            (knx.paramWord(LOG_ParamCalcIndex(LOG_fOOnDpt7)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt8                            ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fOOnDpt8)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt9                            (knx.paramFloat(LOG_ParamCalcIndex(LOG_fOOnDpt9), Float_Enc_IEEE754Single))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt12                           (knx.paramInt(LOG_ParamCalcIndex(LOG_fOOnDpt12)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt13                           ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fOOnDpt13)))
//     Wert für EIN senden als
#define ParamLOG_fOOnDpt14                           (knx.paramFloat(LOG_ParamCalcIndex(LOG_fOOnDpt14), Float_Enc_IEEE754Single))
//     Wert für EIN senden als 
#define ParamLOG_fOOnDpt16                           (knx.paramData(LOG_ParamCalcIndex(LOG_fOOnDpt16)))
//     Wert für EIN senden als 
#define ParamLOG_fOOnDpt17                           (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnDpt17)))
//     Wert für EIN senden als (3-Byte-RGB)
#define ParamLOG_fOOnRGB                             ((knx.paramInt(LOG_ParamCalcIndex(LOG_fOOnRGB)) & LOG_fOOnRGBMask) >> LOG_fOOnRGBShift)
// 
#define ParamLOG_fOOnPAArea                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnPAArea)) & LOG_fOOnPAAreaMask) >> LOG_fOOnPAAreaShift)
// 
#define ParamLOG_fOOnPALine                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnPALine)) & LOG_fOOnPALineMask)
// 
#define ParamLOG_fOOnPADevice                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnPADevice)))
//     Wert für EIN ermitteln als
#define ParamLOG_fOOnFunction                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnFunction)))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fOOnKOKind                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnKOKind)) & LOG_fOOnKOKindMask) >> LOG_fOOnKOKindShift)
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fOOnKONumber                        (knx.paramWord(LOG_ParamCalcIndex(LOG_fOOnKONumber)))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fOOnKONumberRel                     ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fOOnKONumberRel)))
//     DPT des Kommunikationsobjekts
#define ParamLOG_fOOnKODpt                           (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnKODpt)))
//     Wert für EIN an ein zusätzliches    KO senden?
#define ParamLOG_fOOnKOSend                          ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOnKOSend)) & LOG_fOOnKOSendMask) >> LOG_fOOnKOSendShift)
//         Nummer des zusätzlichen KO
#define ParamLOG_fOOnKOSendNumber                    (knx.paramWord(LOG_ParamCalcIndex(LOG_fOOnKOSendNumber)))
//         Nummer des zusätzlichen KO
#define ParamLOG_fOOnKOSendNumberRel                 ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fOOnKOSendNumberRel)))
// Wert für AUS senden?
#define ParamLOG_fOOff                               (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOff)))
// Wert für AUS senden?
#define ParamLOG_fOOffBuzzer                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffBuzzer)))
// Wert für AUS senden?
#define ParamLOG_fOOffLed                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffLed)))
// Wert für AUS senden?
#define ParamLOG_fOOffAll                            (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffAll)))
//     Wert für AUS senden als
#define ParamLOG_fOOffTone                           (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffTone)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt1                           (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt1)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt2                           (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt2)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt3Dir                        ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt3Dir)) & LOG_fOOffDpt3DirMask) >> LOG_fOOffDpt3DirShift)
// 
#define ParamLOG_fOOffDpt3Dim                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt3Dim)) & LOG_fOOffDpt3DimMask)
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt5                           (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt5)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt5001                        (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt5001)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt6                           ((int8_t)knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt6)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt7                           (knx.paramWord(LOG_ParamCalcIndex(LOG_fOOffDpt7)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt8                           ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fOOffDpt8)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt9                           (knx.paramFloat(LOG_ParamCalcIndex(LOG_fOOffDpt9), Float_Enc_IEEE754Single))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt12                          (knx.paramInt(LOG_ParamCalcIndex(LOG_fOOffDpt12)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt13                          ((int32_t)knx.paramInt(LOG_ParamCalcIndex(LOG_fOOffDpt13)))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt14                          (knx.paramFloat(LOG_ParamCalcIndex(LOG_fOOffDpt14), Float_Enc_IEEE754Single))
//     Wert für AUS senden als
#define ParamLOG_fOOffDpt16                          (knx.paramData(LOG_ParamCalcIndex(LOG_fOOffDpt16)))
//     Wert für AUS senden als 
#define ParamLOG_fOOffDpt17                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffDpt17)))
//     Wert für AUS senden als (3-Byte-RGB)
#define ParamLOG_fOOffRGB                            ((knx.paramInt(LOG_ParamCalcIndex(LOG_fOOffRGB)) & LOG_fOOffRGBMask) >> LOG_fOOffRGBShift)
// 
#define ParamLOG_fOOffPAArea                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffPAArea)) & LOG_fOOffPAAreaMask) >> LOG_fOOffPAAreaShift)
// 
#define ParamLOG_fOOffPALine                         (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffPALine)) & LOG_fOOffPALineMask)
// 
#define ParamLOG_fOOffPADevice                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffPADevice)))
//     Wert für AUS ermitteln als
#define ParamLOG_fOOffFunction                       (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffFunction)))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fOOffKOKind                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffKOKind)) & LOG_fOOffKOKindMask) >> LOG_fOOffKOKindShift)
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fOOffKONumber                       (knx.paramWord(LOG_ParamCalcIndex(LOG_fOOffKONumber)))
//     Nummer des Kommunikationsobjekts
#define ParamLOG_fOOffKONumberRel                    ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fOOffKONumberRel)))
//     DPT des Kommunikationsobjekts
#define ParamLOG_fOOffKODpt                          (knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffKODpt)))
//     Wert für AUS an ein zusätzliches    KO senden?
#define ParamLOG_fOOffKOSend                         ((knx.paramByte(LOG_ParamCalcIndex(LOG_fOOffKOSend)) & LOG_fOOffKOSendMask) >> LOG_fOOffKOSendShift)
//         Nummer des zusätzlichen KO
#define ParamLOG_fOOffKOSendNumber                   (knx.paramWord(LOG_ParamCalcIndex(LOG_fOOffKOSendNumber)))
//         Nummer des zusätzlichen KO
#define ParamLOG_fOOffKOSendNumberRel                ((int16_t)knx.paramWord(LOG_ParamCalcIndex(LOG_fOOffKOSendNumberRel)))

// deprecated
#define LOG_KoOffset 420

// Communication objects per channel (multiple occurrence)
#define LOG_KoBlockOffset 420
#define LOG_KoBlockSize 3

#define LOG_KoCalcNumber(index) (index + LOG_KoBlockOffset + _channelIndex * LOG_KoBlockSize)
#define LOG_KoCalcIndex(number) ((number >= LOG_KoCalcNumber(0) && number < LOG_KoCalcNumber(LOG_KoBlockSize)) ? (number - LOG_KoBlockOffset) % LOG_KoBlockSize : -1)
#define LOG_KoCalcChannel(number) ((number >= LOG_KoBlockOffset && number < LOG_KoBlockOffset + LOG_ChannelCount * LOG_KoBlockSize) ? (number - LOG_KoBlockOffset) / LOG_KoBlockSize : -1)

#define LOG_KoKOfE1 0
#define LOG_KoKOfE2 1
#define LOG_KoKOfO 2

// Eingang 1
#define KoLOG_KOfE1                               (knx.getGroupObject(LOG_KoCalcNumber(LOG_KoKOfE1)))
// Eingang 2
#define KoLOG_KOfE2                               (knx.getGroupObject(LOG_KoCalcNumber(LOG_KoKOfE2)))
// Ausgang
#define KoLOG_KOfO                                (knx.getGroupObject(LOG_KoCalcNumber(LOG_KoKOfO)))

#define DFA_VisibleChannels                     12407      // uint8_t
#define DFA_DiagnoseAccess                      12408      // 1 Bit, Bit 7
#define     DFA_DiagnoseAccessMask 0x80
#define     DFA_DiagnoseAccessShift 7

// Verfügbare Kanäle
#define ParamDFA_VisibleChannels                     (knx.paramByte(DFA_VisibleChannels))
// Zugriff über Diagnose-Objekt
#define ParamDFA_DiagnoseAccess                      ((bool)(knx.paramByte(DFA_DiagnoseAccess) & DFA_DiagnoseAccessMask))

#define DFA_ChannelCount 4

// Parameter per channel
#define DFA_ParamBlockOffset 12409
#define DFA_ParamBlockSize 790
#define DFA_ParamCalcIndex(index) (index + DFA_ParamBlockOffset + _channelIndex * DFA_ParamBlockSize)

#define DFA_aActive                              0      // 2 Bits, Bit 7-6
#define     DFA_aActiveMask 0xC0
#define     DFA_aActiveShift 6
#define DFA_aStartPause                          0      // 2 Bits, Bit 5-4
#define     DFA_aStartPauseMask 0x30
#define     DFA_aStartPauseShift 4
#define DFA_aStateRestore                        0      // 2 Bits, Bit 3-2
#define     DFA_aStateRestoreMask 0x0C
#define     DFA_aStateRestoreShift 2
#define DFA_aStateSetting                        1      // 2 Bits, Bit 7-6
#define     DFA_aStateSettingMask 0xC0
#define     DFA_aStateSettingShift 6
#define DFA_aStateSettingSame                    1      // 2 Bits, Bit 5-4
#define     DFA_aStateSettingSameMask 0x30
#define     DFA_aStateSettingSameShift 4
#define DFA_az0                                  2      // uint8_t
#define DFA_aStartupDelayBase                    3      // 2 Bits, Bit 7-6
#define     DFA_aStartupDelayBaseMask 0xC0
#define     DFA_aStartupDelayBaseShift 6
#define DFA_aStartupDelayTime                    3      // 14 Bits, Bit 13-0
#define     DFA_aStartupDelayTimeMask 0x3FFF
#define     DFA_aStartupDelayTimeShift 0
#define DFA_aSymbolPairAB                        5      // 1 Bit, Bit 7
#define     DFA_aSymbolPairABMask 0x80
#define     DFA_aSymbolPairABShift 7
#define DFA_aSymbolPairCD                        5      // 1 Bit, Bit 6
#define     DFA_aSymbolPairCDMask 0x40
#define     DFA_aSymbolPairCDShift 6
#define DFA_aSymbolPairEF                        5      // 1 Bit, Bit 5
#define     DFA_aSymbolPairEFMask 0x20
#define     DFA_aSymbolPairEFShift 5
#define DFA_aSymbolPairGH                        5      // 1 Bit, Bit 4
#define     DFA_aSymbolPairGHMask 0x10
#define     DFA_aSymbolPairGHShift 4
#define DFA_aOutput1Dpt                          6      // 8 Bits, Bit 7-0
#define DFA_aOutput1IntervalBase                 7      // 2 Bits, Bit 7-6
#define     DFA_aOutput1IntervalBaseMask 0xC0
#define     DFA_aOutput1IntervalBaseShift 6
#define DFA_aOutput1IntervalTime                 7      // 14 Bits, Bit 13-0
#define     DFA_aOutput1IntervalTimeMask 0x3FFF
#define     DFA_aOutput1IntervalTimeShift 0
#define DFA_aOutput2Dpt                          9      // 8 Bits, Bit 7-0
#define DFA_aOutput2IntervalBase                10      // 2 Bits, Bit 7-6
#define     DFA_aOutput2IntervalBaseMask 0xC0
#define     DFA_aOutput2IntervalBaseShift 6
#define DFA_aOutput2IntervalTime                10      // 14 Bits, Bit 13-0
#define     DFA_aOutput2IntervalTimeMask 0x3FFF
#define     DFA_aOutput2IntervalTimeShift 0
#define DFA_aOutput3Dpt                         12      // 8 Bits, Bit 7-0
#define DFA_aOutput3IntervalBase                13      // 2 Bits, Bit 7-6
#define     DFA_aOutput3IntervalBaseMask 0xC0
#define     DFA_aOutput3IntervalBaseShift 6
#define DFA_aOutput3IntervalTime                13      // 14 Bits, Bit 13-0
#define     DFA_aOutput3IntervalTimeMask 0x3FFF
#define     DFA_aOutput3IntervalTimeShift 0
#define DFA_aOutput4Dpt                         15      // 8 Bits, Bit 7-0
#define DFA_aOutput4IntervalBase                16      // 2 Bits, Bit 7-6
#define     DFA_aOutput4IntervalBaseMask 0xC0
#define     DFA_aOutput4IntervalBaseShift 6
#define DFA_aOutput4IntervalTime                16      // 14 Bits, Bit 13-0
#define     DFA_aOutput4IntervalTimeMask 0x3FFF
#define     DFA_aOutput4IntervalTimeShift 0
#define DFA_aSymbolAInput                       18      // 2 Bits, Bit 7-6
#define     DFA_aSymbolAInputMask 0xC0
#define     DFA_aSymbolAInputShift 6
#define DFA_aSymbolAKoNumber                    19      // 15 Bits, Bit 15-1
#define     DFA_aSymbolAKoNumberMask 0xFFFE
#define     DFA_aSymbolAKoNumberShift 1
#define DFA_aSymbolALogicNumber                 19      // uint8_t
#define DFA_aSymbolATrigger                     21      // 2 Bits, Bit 7-6
#define     DFA_aSymbolATriggerMask 0xC0
#define     DFA_aSymbolATriggerShift 6
#define DFA_aSymbolBInput                       22      // 2 Bits, Bit 7-6
#define     DFA_aSymbolBInputMask 0xC0
#define     DFA_aSymbolBInputShift 6
#define DFA_aSymbolBKoNumber                    23      // 15 Bits, Bit 15-1
#define     DFA_aSymbolBKoNumberMask 0xFFFE
#define     DFA_aSymbolBKoNumberShift 1
#define DFA_aSymbolBLogicNumber                 23      // uint8_t
#define DFA_aSymbolBTrigger                     25      // 2 Bits, Bit 7-6
#define     DFA_aSymbolBTriggerMask 0xC0
#define     DFA_aSymbolBTriggerShift 6
#define DFA_aSymbolCInput                       26      // 2 Bits, Bit 7-6
#define     DFA_aSymbolCInputMask 0xC0
#define     DFA_aSymbolCInputShift 6
#define DFA_aSymbolCKoNumber                    27      // 15 Bits, Bit 15-1
#define     DFA_aSymbolCKoNumberMask 0xFFFE
#define     DFA_aSymbolCKoNumberShift 1
#define DFA_aSymbolCLogicNumber                 27      // uint8_t
#define DFA_aSymbolCTrigger                     29      // 2 Bits, Bit 7-6
#define     DFA_aSymbolCTriggerMask 0xC0
#define     DFA_aSymbolCTriggerShift 6
#define DFA_aSymbolDInput                       30      // 2 Bits, Bit 7-6
#define     DFA_aSymbolDInputMask 0xC0
#define     DFA_aSymbolDInputShift 6
#define DFA_aSymbolDKoNumber                    31      // 15 Bits, Bit 15-1
#define     DFA_aSymbolDKoNumberMask 0xFFFE
#define     DFA_aSymbolDKoNumberShift 1
#define DFA_aSymbolDLogicNumber                 31      // uint8_t
#define DFA_aSymbolDTrigger                     33      // 2 Bits, Bit 7-6
#define     DFA_aSymbolDTriggerMask 0xC0
#define     DFA_aSymbolDTriggerShift 6
#define DFA_aSymbolEInput                       34      // 2 Bits, Bit 7-6
#define     DFA_aSymbolEInputMask 0xC0
#define     DFA_aSymbolEInputShift 6
#define DFA_aSymbolEKoNumber                    35      // 15 Bits, Bit 15-1
#define     DFA_aSymbolEKoNumberMask 0xFFFE
#define     DFA_aSymbolEKoNumberShift 1
#define DFA_aSymbolELogicNumber                 35      // uint8_t
#define DFA_aSymbolETrigger                     37      // 2 Bits, Bit 7-6
#define     DFA_aSymbolETriggerMask 0xC0
#define     DFA_aSymbolETriggerShift 6
#define DFA_aSymbolFInput                       38      // 2 Bits, Bit 7-6
#define     DFA_aSymbolFInputMask 0xC0
#define     DFA_aSymbolFInputShift 6
#define DFA_aSymbolFKoNumber                    39      // 15 Bits, Bit 15-1
#define     DFA_aSymbolFKoNumberMask 0xFFFE
#define     DFA_aSymbolFKoNumberShift 1
#define DFA_aSymbolFLogicNumber                 39      // uint8_t
#define DFA_aSymbolFTrigger                     41      // 2 Bits, Bit 7-6
#define     DFA_aSymbolFTriggerMask 0xC0
#define     DFA_aSymbolFTriggerShift 6
#define DFA_aSymbolGInput                       42      // 2 Bits, Bit 7-6
#define     DFA_aSymbolGInputMask 0xC0
#define     DFA_aSymbolGInputShift 6
#define DFA_aSymbolGKoNumber                    43      // 15 Bits, Bit 15-1
#define     DFA_aSymbolGKoNumberMask 0xFFFE
#define     DFA_aSymbolGKoNumberShift 1
#define DFA_aSymbolGLogicNumber                 43      // uint8_t
#define DFA_aSymbolGTrigger                     45      // 2 Bits, Bit 7-6
#define     DFA_aSymbolGTriggerMask 0xC0
#define     DFA_aSymbolGTriggerShift 6
#define DFA_aSymbolHInput                       46      // 2 Bits, Bit 7-6
#define     DFA_aSymbolHInputMask 0xC0
#define     DFA_aSymbolHInputShift 6
#define DFA_aSymbolHKoNumber                    47      // 15 Bits, Bit 15-1
#define     DFA_aSymbolHKoNumberMask 0xFFFE
#define     DFA_aSymbolHKoNumberShift 1
#define DFA_aSymbolHLogicNumber                 47      // uint8_t
#define DFA_aSymbolHTrigger                     49      // 2 Bits, Bit 7-6
#define     DFA_aSymbolHTriggerMask 0xC0
#define     DFA_aSymbolHTriggerShift 6
#define DFA_aSymbolTInput                       50      // 2 Bits, Bit 7-6
#define     DFA_aSymbolTInputMask 0xC0
#define     DFA_aSymbolTInputShift 6
#define DFA_aSymbolTKoNumber                    51      // 15 Bits, Bit 15-1
#define     DFA_aSymbolTKoNumberMask 0xFFFE
#define     DFA_aSymbolTKoNumberShift 1
#define DFA_aSymbolTLogicNumber                 51      // uint8_t
#define DFA_aSymbolTTrigger                     53      // 2 Bits, Bit 7-6
#define     DFA_aSymbolTTriggerMask 0xC0
#define     DFA_aSymbolTTriggerShift 6
#define DFA_aCaLOG                              54      // uint8_t
#define DFA_aCaT                                55      // 8 Bits, Bit 7-0
#define DFA_aCaF                                56      // 8 Bits, Bit 7-0
#define DFA_aCaU                                57      // 8 Bits, Bit 7-0
#define DFA_aCbLOG                              58      // uint8_t
#define DFA_aCbT                                59      // 8 Bits, Bit 7-0
#define DFA_aCbF                                60      // 8 Bits, Bit 7-0
#define DFA_aCbU                                61      // 8 Bits, Bit 7-0
#define DFA_aCcLOG                              62      // uint8_t
#define DFA_aCcT                                63      // 8 Bits, Bit 7-0
#define DFA_aCcF                                64      // 8 Bits, Bit 7-0
#define DFA_aCcU                                65      // 8 Bits, Bit 7-0
#define DFA_aCdLOG                              66      // uint8_t
#define DFA_aCdT                                67      // 8 Bits, Bit 7-0
#define DFA_aCdF                                68      // 8 Bits, Bit 7-0
#define DFA_aCdU                                69      // 8 Bits, Bit 7-0
#define DFA_aCeLOG                              70      // uint8_t
#define DFA_aCeT                                71      // 8 Bits, Bit 7-0
#define DFA_aCeF                                72      // 8 Bits, Bit 7-0
#define DFA_aCeU                                73      // 8 Bits, Bit 7-0
#define DFA_aCfLOG                              74      // uint8_t
#define DFA_aCfT                                75      // 8 Bits, Bit 7-0
#define DFA_aCfF                                76      // 8 Bits, Bit 7-0
#define DFA_aCfU                                77      // 8 Bits, Bit 7-0
#define DFA_aCgLOG                              78      // uint8_t
#define DFA_aCgT                                79      // 8 Bits, Bit 7-0
#define DFA_aCgF                                80      // 8 Bits, Bit 7-0
#define DFA_aCgU                                81      // 8 Bits, Bit 7-0
#define DFA_aChLOG                              82      // uint8_t
#define DFA_aChT                                83      // 8 Bits, Bit 7-0
#define DFA_aChF                                84      // 8 Bits, Bit 7-0
#define DFA_aChU                                85      // 8 Bits, Bit 7-0
#define DFA_aCiLOG                              86      // uint8_t
#define DFA_aCiT                                87      // 8 Bits, Bit 7-0
#define DFA_aCiF                                88      // 8 Bits, Bit 7-0
#define DFA_aCiU                                89      // 8 Bits, Bit 7-0
#define DFA_aCjLOG                              90      // uint8_t
#define DFA_aCjT                                91      // 8 Bits, Bit 7-0
#define DFA_aCjF                                92      // 8 Bits, Bit 7-0
#define DFA_aCjU                                93      // 8 Bits, Bit 7-0
#define DFA_aCkLOG                              94      // uint8_t
#define DFA_aCkT                                95      // 8 Bits, Bit 7-0
#define DFA_aCkF                                96      // 8 Bits, Bit 7-0
#define DFA_aCkU                                97      // 8 Bits, Bit 7-0
#define DFA_aClLOG                              98      // uint8_t
#define DFA_aClT                                99      // 8 Bits, Bit 7-0
#define DFA_aClF                                100      // 8 Bits, Bit 7-0
#define DFA_aClU                                101      // 8 Bits, Bit 7-0
#define DFA_aCmLOG                              102      // uint8_t
#define DFA_aCmT                                103      // 8 Bits, Bit 7-0
#define DFA_aCmF                                104      // 8 Bits, Bit 7-0
#define DFA_aCmU                                105      // 8 Bits, Bit 7-0
#define DFA_aCnLOG                              106      // uint8_t
#define DFA_aCnT                                107      // 8 Bits, Bit 7-0
#define DFA_aCnF                                108      // 8 Bits, Bit 7-0
#define DFA_aCnU                                109      // 8 Bits, Bit 7-0
#define DFA_aCoLOG                              110      // uint8_t
#define DFA_aCoT                                111      // 8 Bits, Bit 7-0
#define DFA_aCoF                                112      // 8 Bits, Bit 7-0
#define DFA_aCoU                                113      // 8 Bits, Bit 7-0
#define DFA_aCpLOG                              114      // uint8_t
#define DFA_aCpT                                115      // 8 Bits, Bit 7-0
#define DFA_aCpF                                116      // 8 Bits, Bit 7-0
#define DFA_aCpU                                117      // 8 Bits, Bit 7-0
#define DFA_ad01A                               119      // 8 Bits, Bit 7-0
#define DFA_ad01B                               120      // 8 Bits, Bit 7-0
#define DFA_ad01C                               121      // 8 Bits, Bit 7-0
#define DFA_ad01D                               122      // 8 Bits, Bit 7-0
#define DFA_ad01E                               123      // 8 Bits, Bit 7-0
#define DFA_ad01F                               124      // 8 Bits, Bit 7-0
#define DFA_ad01G                               125      // 8 Bits, Bit 7-0
#define DFA_ad01H                               126      // 8 Bits, Bit 7-0
#define DFA_ad01T                               127      // 8 Bits, Bit 7-0
#define DFA_ad01TBase                           128      // 2 Bits, Bit 7-6
#define     DFA_ad01TBaseMask 0xC0
#define     DFA_ad01TBaseShift 6
#define DFA_ad01TTime                           128      // 14 Bits, Bit 13-0
#define     DFA_ad01TTimeMask 0x3FFF
#define     DFA_ad01TTimeShift 0
#define DFA_az01o1Send                          130      // 8 Bits, Bit 7-0
#define DFA_az01o1Dpt1                          131      // 8 Bits, Bit 7-0
#define DFA_az01o1Dpt2                          131      // 8 Bits, Bit 7-0
#define DFA_az01o1Dpt5                          131      // uint8_t
#define DFA_az01o1Dpt5001                       131      // uint8_t
#define DFA_az01o1Dpt6                          131      // int8_t
#define DFA_az01o1Dpt7                          131      // uint16_t
#define DFA_az01o1Dpt8                          131      // int16_t
#define DFA_az01o1Dpt9                          131      // float
#define DFA_az01o1Dpt12                         131      // uint32_t
#define DFA_az01o1Dpt13                         131      // int32_t
#define DFA_az01o1Dpt14                         131      // float
#define DFA_az01o1Dpt17                         131      // 8 Bits, Bit 7-0
#define DFA_az01o1Dpt232                        131      // 24 Bits, Bit 31-8
#define     DFA_az01o1Dpt232Mask 0xFFFFFF00
#define     DFA_az01o1Dpt232Shift 8
#define DFA_az01o2Send                          135      // 8 Bits, Bit 7-0
#define DFA_az01o2Dpt1                          136      // 8 Bits, Bit 7-0
#define DFA_az01o2Dpt2                          136      // 8 Bits, Bit 7-0
#define DFA_az01o2Dpt5                          136      // uint8_t
#define DFA_az01o2Dpt5001                       136      // uint8_t
#define DFA_az01o2Dpt6                          136      // int8_t
#define DFA_az01o2Dpt7                          136      // uint16_t
#define DFA_az01o2Dpt8                          136      // int16_t
#define DFA_az01o2Dpt9                          136      // float
#define DFA_az01o2Dpt12                         136      // uint32_t
#define DFA_az01o2Dpt13                         136      // int32_t
#define DFA_az01o2Dpt14                         136      // float
#define DFA_az01o2Dpt17                         136      // 8 Bits, Bit 7-0
#define DFA_az01o2Dpt232                        136      // 24 Bits, Bit 31-8
#define     DFA_az01o2Dpt232Mask 0xFFFFFF00
#define     DFA_az01o2Dpt232Shift 8
#define DFA_az01o3Send                          140      // 8 Bits, Bit 7-0
#define DFA_az01o3Dpt1                          141      // 8 Bits, Bit 7-0
#define DFA_az01o3Dpt2                          141      // 8 Bits, Bit 7-0
#define DFA_az01o3Dpt5                          141      // uint8_t
#define DFA_az01o3Dpt5001                       141      // uint8_t
#define DFA_az01o3Dpt6                          141      // int8_t
#define DFA_az01o3Dpt7                          141      // uint16_t
#define DFA_az01o3Dpt8                          141      // int16_t
#define DFA_az01o3Dpt9                          141      // float
#define DFA_az01o3Dpt12                         141      // uint32_t
#define DFA_az01o3Dpt13                         141      // int32_t
#define DFA_az01o3Dpt14                         141      // float
#define DFA_az01o3Dpt17                         141      // 8 Bits, Bit 7-0
#define DFA_az01o3Dpt232                        141      // 24 Bits, Bit 31-8
#define     DFA_az01o3Dpt232Mask 0xFFFFFF00
#define     DFA_az01o3Dpt232Shift 8
#define DFA_az01o4Send                          145      // 8 Bits, Bit 7-0
#define DFA_az01o4Dpt1                          146      // 8 Bits, Bit 7-0
#define DFA_az01o4Dpt2                          146      // 8 Bits, Bit 7-0
#define DFA_az01o4Dpt5                          146      // uint8_t
#define DFA_az01o4Dpt5001                       146      // uint8_t
#define DFA_az01o4Dpt6                          146      // int8_t
#define DFA_az01o4Dpt7                          146      // uint16_t
#define DFA_az01o4Dpt8                          146      // int16_t
#define DFA_az01o4Dpt9                          146      // float
#define DFA_az01o4Dpt12                         146      // uint32_t
#define DFA_az01o4Dpt13                         146      // int32_t
#define DFA_az01o4Dpt14                         146      // float
#define DFA_az01o4Dpt16                         146      // char*, 14 Byte
#define DFA_az01o4Dpt17                         146      // 8 Bits, Bit 7-0
#define DFA_az01o4Dpt232                        146      // 24 Bits, Bit 31-8
#define     DFA_az01o4Dpt232Mask 0xFFFFFF00
#define     DFA_az01o4Dpt232Shift 8
#define DFA_ad02A                               161      // 8 Bits, Bit 7-0
#define DFA_ad02B                               162      // 8 Bits, Bit 7-0
#define DFA_ad02C                               163      // 8 Bits, Bit 7-0
#define DFA_ad02D                               164      // 8 Bits, Bit 7-0
#define DFA_ad02E                               165      // 8 Bits, Bit 7-0
#define DFA_ad02F                               166      // 8 Bits, Bit 7-0
#define DFA_ad02G                               167      // 8 Bits, Bit 7-0
#define DFA_ad02H                               168      // 8 Bits, Bit 7-0
#define DFA_ad02T                               169      // 8 Bits, Bit 7-0
#define DFA_ad02TBase                           170      // 2 Bits, Bit 7-6
#define     DFA_ad02TBaseMask 0xC0
#define     DFA_ad02TBaseShift 6
#define DFA_ad02TTime                           170      // 14 Bits, Bit 13-0
#define     DFA_ad02TTimeMask 0x3FFF
#define     DFA_ad02TTimeShift 0
#define DFA_az02o1Send                          172      // 8 Bits, Bit 7-0
#define DFA_az02o1Dpt1                          173      // 8 Bits, Bit 7-0
#define DFA_az02o1Dpt2                          173      // 8 Bits, Bit 7-0
#define DFA_az02o1Dpt5                          173      // uint8_t
#define DFA_az02o1Dpt5001                       173      // uint8_t
#define DFA_az02o1Dpt6                          173      // int8_t
#define DFA_az02o1Dpt7                          173      // uint16_t
#define DFA_az02o1Dpt8                          173      // int16_t
#define DFA_az02o1Dpt9                          173      // float
#define DFA_az02o1Dpt12                         173      // uint32_t
#define DFA_az02o1Dpt13                         173      // int32_t
#define DFA_az02o1Dpt14                         173      // float
#define DFA_az02o1Dpt17                         173      // 8 Bits, Bit 7-0
#define DFA_az02o1Dpt232                        173      // 24 Bits, Bit 31-8
#define     DFA_az02o1Dpt232Mask 0xFFFFFF00
#define     DFA_az02o1Dpt232Shift 8
#define DFA_az02o2Send                          177      // 8 Bits, Bit 7-0
#define DFA_az02o2Dpt1                          178      // 8 Bits, Bit 7-0
#define DFA_az02o2Dpt2                          178      // 8 Bits, Bit 7-0
#define DFA_az02o2Dpt5                          178      // uint8_t
#define DFA_az02o2Dpt5001                       178      // uint8_t
#define DFA_az02o2Dpt6                          178      // int8_t
#define DFA_az02o2Dpt7                          178      // uint16_t
#define DFA_az02o2Dpt8                          178      // int16_t
#define DFA_az02o2Dpt9                          178      // float
#define DFA_az02o2Dpt12                         178      // uint32_t
#define DFA_az02o2Dpt13                         178      // int32_t
#define DFA_az02o2Dpt14                         178      // float
#define DFA_az02o2Dpt17                         178      // 8 Bits, Bit 7-0
#define DFA_az02o2Dpt232                        178      // 24 Bits, Bit 31-8
#define     DFA_az02o2Dpt232Mask 0xFFFFFF00
#define     DFA_az02o2Dpt232Shift 8
#define DFA_az02o3Send                          182      // 8 Bits, Bit 7-0
#define DFA_az02o3Dpt1                          183      // 8 Bits, Bit 7-0
#define DFA_az02o3Dpt2                          183      // 8 Bits, Bit 7-0
#define DFA_az02o3Dpt5                          183      // uint8_t
#define DFA_az02o3Dpt5001                       183      // uint8_t
#define DFA_az02o3Dpt6                          183      // int8_t
#define DFA_az02o3Dpt7                          183      // uint16_t
#define DFA_az02o3Dpt8                          183      // int16_t
#define DFA_az02o3Dpt9                          183      // float
#define DFA_az02o3Dpt12                         183      // uint32_t
#define DFA_az02o3Dpt13                         183      // int32_t
#define DFA_az02o3Dpt14                         183      // float
#define DFA_az02o3Dpt17                         183      // 8 Bits, Bit 7-0
#define DFA_az02o3Dpt232                        183      // 24 Bits, Bit 31-8
#define     DFA_az02o3Dpt232Mask 0xFFFFFF00
#define     DFA_az02o3Dpt232Shift 8
#define DFA_az02o4Send                          187      // 8 Bits, Bit 7-0
#define DFA_az02o4Dpt1                          188      // 8 Bits, Bit 7-0
#define DFA_az02o4Dpt2                          188      // 8 Bits, Bit 7-0
#define DFA_az02o4Dpt5                          188      // uint8_t
#define DFA_az02o4Dpt5001                       188      // uint8_t
#define DFA_az02o4Dpt6                          188      // int8_t
#define DFA_az02o4Dpt7                          188      // uint16_t
#define DFA_az02o4Dpt8                          188      // int16_t
#define DFA_az02o4Dpt9                          188      // float
#define DFA_az02o4Dpt12                         188      // uint32_t
#define DFA_az02o4Dpt13                         188      // int32_t
#define DFA_az02o4Dpt14                         188      // float
#define DFA_az02o4Dpt16                         188      // char*, 14 Byte
#define DFA_az02o4Dpt17                         188      // 8 Bits, Bit 7-0
#define DFA_az02o4Dpt232                        188      // 24 Bits, Bit 31-8
#define     DFA_az02o4Dpt232Mask 0xFFFFFF00
#define     DFA_az02o4Dpt232Shift 8
#define DFA_ad03A                               203      // 8 Bits, Bit 7-0
#define DFA_ad03B                               204      // 8 Bits, Bit 7-0
#define DFA_ad03C                               205      // 8 Bits, Bit 7-0
#define DFA_ad03D                               206      // 8 Bits, Bit 7-0
#define DFA_ad03E                               207      // 8 Bits, Bit 7-0
#define DFA_ad03F                               208      // 8 Bits, Bit 7-0
#define DFA_ad03G                               209      // 8 Bits, Bit 7-0
#define DFA_ad03H                               210      // 8 Bits, Bit 7-0
#define DFA_ad03T                               211      // 8 Bits, Bit 7-0
#define DFA_ad03TBase                           212      // 2 Bits, Bit 7-6
#define     DFA_ad03TBaseMask 0xC0
#define     DFA_ad03TBaseShift 6
#define DFA_ad03TTime                           212      // 14 Bits, Bit 13-0
#define     DFA_ad03TTimeMask 0x3FFF
#define     DFA_ad03TTimeShift 0
#define DFA_az03o1Send                          214      // 8 Bits, Bit 7-0
#define DFA_az03o1Dpt1                          215      // 8 Bits, Bit 7-0
#define DFA_az03o1Dpt2                          215      // 8 Bits, Bit 7-0
#define DFA_az03o1Dpt5                          215      // uint8_t
#define DFA_az03o1Dpt5001                       215      // uint8_t
#define DFA_az03o1Dpt6                          215      // int8_t
#define DFA_az03o1Dpt7                          215      // uint16_t
#define DFA_az03o1Dpt8                          215      // int16_t
#define DFA_az03o1Dpt9                          215      // float
#define DFA_az03o1Dpt12                         215      // uint32_t
#define DFA_az03o1Dpt13                         215      // int32_t
#define DFA_az03o1Dpt14                         215      // float
#define DFA_az03o1Dpt17                         215      // 8 Bits, Bit 7-0
#define DFA_az03o1Dpt232                        215      // 24 Bits, Bit 31-8
#define     DFA_az03o1Dpt232Mask 0xFFFFFF00
#define     DFA_az03o1Dpt232Shift 8
#define DFA_az03o2Send                          219      // 8 Bits, Bit 7-0
#define DFA_az03o2Dpt1                          220      // 8 Bits, Bit 7-0
#define DFA_az03o2Dpt2                          220      // 8 Bits, Bit 7-0
#define DFA_az03o2Dpt5                          220      // uint8_t
#define DFA_az03o2Dpt5001                       220      // uint8_t
#define DFA_az03o2Dpt6                          220      // int8_t
#define DFA_az03o2Dpt7                          220      // uint16_t
#define DFA_az03o2Dpt8                          220      // int16_t
#define DFA_az03o2Dpt9                          220      // float
#define DFA_az03o2Dpt12                         220      // uint32_t
#define DFA_az03o2Dpt13                         220      // int32_t
#define DFA_az03o2Dpt14                         220      // float
#define DFA_az03o2Dpt17                         220      // 8 Bits, Bit 7-0
#define DFA_az03o2Dpt232                        220      // 24 Bits, Bit 31-8
#define     DFA_az03o2Dpt232Mask 0xFFFFFF00
#define     DFA_az03o2Dpt232Shift 8
#define DFA_az03o3Send                          224      // 8 Bits, Bit 7-0
#define DFA_az03o3Dpt1                          225      // 8 Bits, Bit 7-0
#define DFA_az03o3Dpt2                          225      // 8 Bits, Bit 7-0
#define DFA_az03o3Dpt5                          225      // uint8_t
#define DFA_az03o3Dpt5001                       225      // uint8_t
#define DFA_az03o3Dpt6                          225      // int8_t
#define DFA_az03o3Dpt7                          225      // uint16_t
#define DFA_az03o3Dpt8                          225      // int16_t
#define DFA_az03o3Dpt9                          225      // float
#define DFA_az03o3Dpt12                         225      // uint32_t
#define DFA_az03o3Dpt13                         225      // int32_t
#define DFA_az03o3Dpt14                         225      // float
#define DFA_az03o3Dpt17                         225      // 8 Bits, Bit 7-0
#define DFA_az03o3Dpt232                        225      // 24 Bits, Bit 31-8
#define     DFA_az03o3Dpt232Mask 0xFFFFFF00
#define     DFA_az03o3Dpt232Shift 8
#define DFA_az03o4Send                          229      // 8 Bits, Bit 7-0
#define DFA_az03o4Dpt1                          230      // 8 Bits, Bit 7-0
#define DFA_az03o4Dpt2                          230      // 8 Bits, Bit 7-0
#define DFA_az03o4Dpt5                          230      // uint8_t
#define DFA_az03o4Dpt5001                       230      // uint8_t
#define DFA_az03o4Dpt6                          230      // int8_t
#define DFA_az03o4Dpt7                          230      // uint16_t
#define DFA_az03o4Dpt8                          230      // int16_t
#define DFA_az03o4Dpt9                          230      // float
#define DFA_az03o4Dpt12                         230      // uint32_t
#define DFA_az03o4Dpt13                         230      // int32_t
#define DFA_az03o4Dpt14                         230      // float
#define DFA_az03o4Dpt16                         230      // char*, 14 Byte
#define DFA_az03o4Dpt17                         230      // 8 Bits, Bit 7-0
#define DFA_az03o4Dpt232                        230      // 24 Bits, Bit 31-8
#define     DFA_az03o4Dpt232Mask 0xFFFFFF00
#define     DFA_az03o4Dpt232Shift 8
#define DFA_ad04A                               245      // 8 Bits, Bit 7-0
#define DFA_ad04B                               246      // 8 Bits, Bit 7-0
#define DFA_ad04C                               247      // 8 Bits, Bit 7-0
#define DFA_ad04D                               248      // 8 Bits, Bit 7-0
#define DFA_ad04E                               249      // 8 Bits, Bit 7-0
#define DFA_ad04F                               250      // 8 Bits, Bit 7-0
#define DFA_ad04G                               251      // 8 Bits, Bit 7-0
#define DFA_ad04H                               252      // 8 Bits, Bit 7-0
#define DFA_ad04T                               253      // 8 Bits, Bit 7-0
#define DFA_ad04TBase                           254      // 2 Bits, Bit 7-6
#define     DFA_ad04TBaseMask 0xC0
#define     DFA_ad04TBaseShift 6
#define DFA_ad04TTime                           254      // 14 Bits, Bit 13-0
#define     DFA_ad04TTimeMask 0x3FFF
#define     DFA_ad04TTimeShift 0
#define DFA_az04o1Send                          256      // 8 Bits, Bit 7-0
#define DFA_az04o1Dpt1                          257      // 8 Bits, Bit 7-0
#define DFA_az04o1Dpt2                          257      // 8 Bits, Bit 7-0
#define DFA_az04o1Dpt5                          257      // uint8_t
#define DFA_az04o1Dpt5001                       257      // uint8_t
#define DFA_az04o1Dpt6                          257      // int8_t
#define DFA_az04o1Dpt7                          257      // uint16_t
#define DFA_az04o1Dpt8                          257      // int16_t
#define DFA_az04o1Dpt9                          257      // float
#define DFA_az04o1Dpt12                         257      // uint32_t
#define DFA_az04o1Dpt13                         257      // int32_t
#define DFA_az04o1Dpt14                         257      // float
#define DFA_az04o1Dpt17                         257      // 8 Bits, Bit 7-0
#define DFA_az04o1Dpt232                        257      // 24 Bits, Bit 31-8
#define     DFA_az04o1Dpt232Mask 0xFFFFFF00
#define     DFA_az04o1Dpt232Shift 8
#define DFA_az04o2Send                          261      // 8 Bits, Bit 7-0
#define DFA_az04o2Dpt1                          262      // 8 Bits, Bit 7-0
#define DFA_az04o2Dpt2                          262      // 8 Bits, Bit 7-0
#define DFA_az04o2Dpt5                          262      // uint8_t
#define DFA_az04o2Dpt5001                       262      // uint8_t
#define DFA_az04o2Dpt6                          262      // int8_t
#define DFA_az04o2Dpt7                          262      // uint16_t
#define DFA_az04o2Dpt8                          262      // int16_t
#define DFA_az04o2Dpt9                          262      // float
#define DFA_az04o2Dpt12                         262      // uint32_t
#define DFA_az04o2Dpt13                         262      // int32_t
#define DFA_az04o2Dpt14                         262      // float
#define DFA_az04o2Dpt17                         262      // 8 Bits, Bit 7-0
#define DFA_az04o2Dpt232                        262      // 24 Bits, Bit 31-8
#define     DFA_az04o2Dpt232Mask 0xFFFFFF00
#define     DFA_az04o2Dpt232Shift 8
#define DFA_az04o3Send                          266      // 8 Bits, Bit 7-0
#define DFA_az04o3Dpt1                          267      // 8 Bits, Bit 7-0
#define DFA_az04o3Dpt2                          267      // 8 Bits, Bit 7-0
#define DFA_az04o3Dpt5                          267      // uint8_t
#define DFA_az04o3Dpt5001                       267      // uint8_t
#define DFA_az04o3Dpt6                          267      // int8_t
#define DFA_az04o3Dpt7                          267      // uint16_t
#define DFA_az04o3Dpt8                          267      // int16_t
#define DFA_az04o3Dpt9                          267      // float
#define DFA_az04o3Dpt12                         267      // uint32_t
#define DFA_az04o3Dpt13                         267      // int32_t
#define DFA_az04o3Dpt14                         267      // float
#define DFA_az04o3Dpt17                         267      // 8 Bits, Bit 7-0
#define DFA_az04o3Dpt232                        267      // 24 Bits, Bit 31-8
#define     DFA_az04o3Dpt232Mask 0xFFFFFF00
#define     DFA_az04o3Dpt232Shift 8
#define DFA_az04o4Send                          271      // 8 Bits, Bit 7-0
#define DFA_az04o4Dpt1                          272      // 8 Bits, Bit 7-0
#define DFA_az04o4Dpt2                          272      // 8 Bits, Bit 7-0
#define DFA_az04o4Dpt5                          272      // uint8_t
#define DFA_az04o4Dpt5001                       272      // uint8_t
#define DFA_az04o4Dpt6                          272      // int8_t
#define DFA_az04o4Dpt7                          272      // uint16_t
#define DFA_az04o4Dpt8                          272      // int16_t
#define DFA_az04o4Dpt9                          272      // float
#define DFA_az04o4Dpt12                         272      // uint32_t
#define DFA_az04o4Dpt13                         272      // int32_t
#define DFA_az04o4Dpt14                         272      // float
#define DFA_az04o4Dpt16                         272      // char*, 14 Byte
#define DFA_az04o4Dpt17                         272      // 8 Bits, Bit 7-0
#define DFA_az04o4Dpt232                        272      // 24 Bits, Bit 31-8
#define     DFA_az04o4Dpt232Mask 0xFFFFFF00
#define     DFA_az04o4Dpt232Shift 8
#define DFA_ad05A                               287      // 8 Bits, Bit 7-0
#define DFA_ad05B                               288      // 8 Bits, Bit 7-0
#define DFA_ad05C                               289      // 8 Bits, Bit 7-0
#define DFA_ad05D                               290      // 8 Bits, Bit 7-0
#define DFA_ad05E                               291      // 8 Bits, Bit 7-0
#define DFA_ad05F                               292      // 8 Bits, Bit 7-0
#define DFA_ad05G                               293      // 8 Bits, Bit 7-0
#define DFA_ad05H                               294      // 8 Bits, Bit 7-0
#define DFA_ad05T                               295      // 8 Bits, Bit 7-0
#define DFA_ad05TBase                           296      // 2 Bits, Bit 7-6
#define     DFA_ad05TBaseMask 0xC0
#define     DFA_ad05TBaseShift 6
#define DFA_ad05TTime                           296      // 14 Bits, Bit 13-0
#define     DFA_ad05TTimeMask 0x3FFF
#define     DFA_ad05TTimeShift 0
#define DFA_az05o1Send                          298      // 8 Bits, Bit 7-0
#define DFA_az05o1Dpt1                          299      // 8 Bits, Bit 7-0
#define DFA_az05o1Dpt2                          299      // 8 Bits, Bit 7-0
#define DFA_az05o1Dpt5                          299      // uint8_t
#define DFA_az05o1Dpt5001                       299      // uint8_t
#define DFA_az05o1Dpt6                          299      // int8_t
#define DFA_az05o1Dpt7                          299      // uint16_t
#define DFA_az05o1Dpt8                          299      // int16_t
#define DFA_az05o1Dpt9                          299      // float
#define DFA_az05o1Dpt12                         299      // uint32_t
#define DFA_az05o1Dpt13                         299      // int32_t
#define DFA_az05o1Dpt14                         299      // float
#define DFA_az05o1Dpt17                         299      // 8 Bits, Bit 7-0
#define DFA_az05o1Dpt232                        299      // 24 Bits, Bit 31-8
#define     DFA_az05o1Dpt232Mask 0xFFFFFF00
#define     DFA_az05o1Dpt232Shift 8
#define DFA_az05o2Send                          303      // 8 Bits, Bit 7-0
#define DFA_az05o2Dpt1                          304      // 8 Bits, Bit 7-0
#define DFA_az05o2Dpt2                          304      // 8 Bits, Bit 7-0
#define DFA_az05o2Dpt5                          304      // uint8_t
#define DFA_az05o2Dpt5001                       304      // uint8_t
#define DFA_az05o2Dpt6                          304      // int8_t
#define DFA_az05o2Dpt7                          304      // uint16_t
#define DFA_az05o2Dpt8                          304      // int16_t
#define DFA_az05o2Dpt9                          304      // float
#define DFA_az05o2Dpt12                         304      // uint32_t
#define DFA_az05o2Dpt13                         304      // int32_t
#define DFA_az05o2Dpt14                         304      // float
#define DFA_az05o2Dpt17                         304      // 8 Bits, Bit 7-0
#define DFA_az05o2Dpt232                        304      // 24 Bits, Bit 31-8
#define     DFA_az05o2Dpt232Mask 0xFFFFFF00
#define     DFA_az05o2Dpt232Shift 8
#define DFA_az05o3Send                          308      // 8 Bits, Bit 7-0
#define DFA_az05o3Dpt1                          309      // 8 Bits, Bit 7-0
#define DFA_az05o3Dpt2                          309      // 8 Bits, Bit 7-0
#define DFA_az05o3Dpt5                          309      // uint8_t
#define DFA_az05o3Dpt5001                       309      // uint8_t
#define DFA_az05o3Dpt6                          309      // int8_t
#define DFA_az05o3Dpt7                          309      // uint16_t
#define DFA_az05o3Dpt8                          309      // int16_t
#define DFA_az05o3Dpt9                          309      // float
#define DFA_az05o3Dpt12                         309      // uint32_t
#define DFA_az05o3Dpt13                         309      // int32_t
#define DFA_az05o3Dpt14                         309      // float
#define DFA_az05o3Dpt17                         309      // 8 Bits, Bit 7-0
#define DFA_az05o3Dpt232                        309      // 24 Bits, Bit 31-8
#define     DFA_az05o3Dpt232Mask 0xFFFFFF00
#define     DFA_az05o3Dpt232Shift 8
#define DFA_az05o4Send                          313      // 8 Bits, Bit 7-0
#define DFA_az05o4Dpt1                          314      // 8 Bits, Bit 7-0
#define DFA_az05o4Dpt2                          314      // 8 Bits, Bit 7-0
#define DFA_az05o4Dpt5                          314      // uint8_t
#define DFA_az05o4Dpt5001                       314      // uint8_t
#define DFA_az05o4Dpt6                          314      // int8_t
#define DFA_az05o4Dpt7                          314      // uint16_t
#define DFA_az05o4Dpt8                          314      // int16_t
#define DFA_az05o4Dpt9                          314      // float
#define DFA_az05o4Dpt12                         314      // uint32_t
#define DFA_az05o4Dpt13                         314      // int32_t
#define DFA_az05o4Dpt14                         314      // float
#define DFA_az05o4Dpt16                         314      // char*, 14 Byte
#define DFA_az05o4Dpt17                         314      // 8 Bits, Bit 7-0
#define DFA_az05o4Dpt232                        314      // 24 Bits, Bit 31-8
#define     DFA_az05o4Dpt232Mask 0xFFFFFF00
#define     DFA_az05o4Dpt232Shift 8
#define DFA_ad06A                               329      // 8 Bits, Bit 7-0
#define DFA_ad06B                               330      // 8 Bits, Bit 7-0
#define DFA_ad06C                               331      // 8 Bits, Bit 7-0
#define DFA_ad06D                               332      // 8 Bits, Bit 7-0
#define DFA_ad06E                               333      // 8 Bits, Bit 7-0
#define DFA_ad06F                               334      // 8 Bits, Bit 7-0
#define DFA_ad06G                               335      // 8 Bits, Bit 7-0
#define DFA_ad06H                               336      // 8 Bits, Bit 7-0
#define DFA_ad06T                               337      // 8 Bits, Bit 7-0
#define DFA_ad06TBase                           338      // 2 Bits, Bit 7-6
#define     DFA_ad06TBaseMask 0xC0
#define     DFA_ad06TBaseShift 6
#define DFA_ad06TTime                           338      // 14 Bits, Bit 13-0
#define     DFA_ad06TTimeMask 0x3FFF
#define     DFA_ad06TTimeShift 0
#define DFA_az06o1Send                          340      // 8 Bits, Bit 7-0
#define DFA_az06o1Dpt1                          341      // 8 Bits, Bit 7-0
#define DFA_az06o1Dpt2                          341      // 8 Bits, Bit 7-0
#define DFA_az06o1Dpt5                          341      // uint8_t
#define DFA_az06o1Dpt5001                       341      // uint8_t
#define DFA_az06o1Dpt6                          341      // int8_t
#define DFA_az06o1Dpt7                          341      // uint16_t
#define DFA_az06o1Dpt8                          341      // int16_t
#define DFA_az06o1Dpt9                          341      // float
#define DFA_az06o1Dpt12                         341      // uint32_t
#define DFA_az06o1Dpt13                         341      // int32_t
#define DFA_az06o1Dpt14                         341      // float
#define DFA_az06o1Dpt17                         341      // 8 Bits, Bit 7-0
#define DFA_az06o1Dpt232                        341      // 24 Bits, Bit 31-8
#define     DFA_az06o1Dpt232Mask 0xFFFFFF00
#define     DFA_az06o1Dpt232Shift 8
#define DFA_az06o2Send                          345      // 8 Bits, Bit 7-0
#define DFA_az06o2Dpt1                          346      // 8 Bits, Bit 7-0
#define DFA_az06o2Dpt2                          346      // 8 Bits, Bit 7-0
#define DFA_az06o2Dpt5                          346      // uint8_t
#define DFA_az06o2Dpt5001                       346      // uint8_t
#define DFA_az06o2Dpt6                          346      // int8_t
#define DFA_az06o2Dpt7                          346      // uint16_t
#define DFA_az06o2Dpt8                          346      // int16_t
#define DFA_az06o2Dpt9                          346      // float
#define DFA_az06o2Dpt12                         346      // uint32_t
#define DFA_az06o2Dpt13                         346      // int32_t
#define DFA_az06o2Dpt14                         346      // float
#define DFA_az06o2Dpt17                         346      // 8 Bits, Bit 7-0
#define DFA_az06o2Dpt232                        346      // 24 Bits, Bit 31-8
#define     DFA_az06o2Dpt232Mask 0xFFFFFF00
#define     DFA_az06o2Dpt232Shift 8
#define DFA_az06o3Send                          350      // 8 Bits, Bit 7-0
#define DFA_az06o3Dpt1                          351      // 8 Bits, Bit 7-0
#define DFA_az06o3Dpt2                          351      // 8 Bits, Bit 7-0
#define DFA_az06o3Dpt5                          351      // uint8_t
#define DFA_az06o3Dpt5001                       351      // uint8_t
#define DFA_az06o3Dpt6                          351      // int8_t
#define DFA_az06o3Dpt7                          351      // uint16_t
#define DFA_az06o3Dpt8                          351      // int16_t
#define DFA_az06o3Dpt9                          351      // float
#define DFA_az06o3Dpt12                         351      // uint32_t
#define DFA_az06o3Dpt13                         351      // int32_t
#define DFA_az06o3Dpt14                         351      // float
#define DFA_az06o3Dpt17                         351      // 8 Bits, Bit 7-0
#define DFA_az06o3Dpt232                        351      // 24 Bits, Bit 31-8
#define     DFA_az06o3Dpt232Mask 0xFFFFFF00
#define     DFA_az06o3Dpt232Shift 8
#define DFA_az06o4Send                          355      // 8 Bits, Bit 7-0
#define DFA_az06o4Dpt1                          356      // 8 Bits, Bit 7-0
#define DFA_az06o4Dpt2                          356      // 8 Bits, Bit 7-0
#define DFA_az06o4Dpt5                          356      // uint8_t
#define DFA_az06o4Dpt5001                       356      // uint8_t
#define DFA_az06o4Dpt6                          356      // int8_t
#define DFA_az06o4Dpt7                          356      // uint16_t
#define DFA_az06o4Dpt8                          356      // int16_t
#define DFA_az06o4Dpt9                          356      // float
#define DFA_az06o4Dpt12                         356      // uint32_t
#define DFA_az06o4Dpt13                         356      // int32_t
#define DFA_az06o4Dpt14                         356      // float
#define DFA_az06o4Dpt16                         356      // char*, 14 Byte
#define DFA_az06o4Dpt17                         356      // 8 Bits, Bit 7-0
#define DFA_az06o4Dpt232                        356      // 24 Bits, Bit 31-8
#define     DFA_az06o4Dpt232Mask 0xFFFFFF00
#define     DFA_az06o4Dpt232Shift 8
#define DFA_ad07A                               371      // 8 Bits, Bit 7-0
#define DFA_ad07B                               372      // 8 Bits, Bit 7-0
#define DFA_ad07C                               373      // 8 Bits, Bit 7-0
#define DFA_ad07D                               374      // 8 Bits, Bit 7-0
#define DFA_ad07E                               375      // 8 Bits, Bit 7-0
#define DFA_ad07F                               376      // 8 Bits, Bit 7-0
#define DFA_ad07G                               377      // 8 Bits, Bit 7-0
#define DFA_ad07H                               378      // 8 Bits, Bit 7-0
#define DFA_ad07T                               379      // 8 Bits, Bit 7-0
#define DFA_ad07TBase                           380      // 2 Bits, Bit 7-6
#define     DFA_ad07TBaseMask 0xC0
#define     DFA_ad07TBaseShift 6
#define DFA_ad07TTime                           380      // 14 Bits, Bit 13-0
#define     DFA_ad07TTimeMask 0x3FFF
#define     DFA_ad07TTimeShift 0
#define DFA_az07o1Send                          382      // 8 Bits, Bit 7-0
#define DFA_az07o1Dpt1                          383      // 8 Bits, Bit 7-0
#define DFA_az07o1Dpt2                          383      // 8 Bits, Bit 7-0
#define DFA_az07o1Dpt5                          383      // uint8_t
#define DFA_az07o1Dpt5001                       383      // uint8_t
#define DFA_az07o1Dpt6                          383      // int8_t
#define DFA_az07o1Dpt7                          383      // uint16_t
#define DFA_az07o1Dpt8                          383      // int16_t
#define DFA_az07o1Dpt9                          383      // float
#define DFA_az07o1Dpt12                         383      // uint32_t
#define DFA_az07o1Dpt13                         383      // int32_t
#define DFA_az07o1Dpt14                         383      // float
#define DFA_az07o1Dpt17                         383      // 8 Bits, Bit 7-0
#define DFA_az07o1Dpt232                        383      // 24 Bits, Bit 31-8
#define     DFA_az07o1Dpt232Mask 0xFFFFFF00
#define     DFA_az07o1Dpt232Shift 8
#define DFA_az07o2Send                          387      // 8 Bits, Bit 7-0
#define DFA_az07o2Dpt1                          388      // 8 Bits, Bit 7-0
#define DFA_az07o2Dpt2                          388      // 8 Bits, Bit 7-0
#define DFA_az07o2Dpt5                          388      // uint8_t
#define DFA_az07o2Dpt5001                       388      // uint8_t
#define DFA_az07o2Dpt6                          388      // int8_t
#define DFA_az07o2Dpt7                          388      // uint16_t
#define DFA_az07o2Dpt8                          388      // int16_t
#define DFA_az07o2Dpt9                          388      // float
#define DFA_az07o2Dpt12                         388      // uint32_t
#define DFA_az07o2Dpt13                         388      // int32_t
#define DFA_az07o2Dpt14                         388      // float
#define DFA_az07o2Dpt17                         388      // 8 Bits, Bit 7-0
#define DFA_az07o2Dpt232                        388      // 24 Bits, Bit 31-8
#define     DFA_az07o2Dpt232Mask 0xFFFFFF00
#define     DFA_az07o2Dpt232Shift 8
#define DFA_az07o3Send                          392      // 8 Bits, Bit 7-0
#define DFA_az07o3Dpt1                          393      // 8 Bits, Bit 7-0
#define DFA_az07o3Dpt2                          393      // 8 Bits, Bit 7-0
#define DFA_az07o3Dpt5                          393      // uint8_t
#define DFA_az07o3Dpt5001                       393      // uint8_t
#define DFA_az07o3Dpt6                          393      // int8_t
#define DFA_az07o3Dpt7                          393      // uint16_t
#define DFA_az07o3Dpt8                          393      // int16_t
#define DFA_az07o3Dpt9                          393      // float
#define DFA_az07o3Dpt12                         393      // uint32_t
#define DFA_az07o3Dpt13                         393      // int32_t
#define DFA_az07o3Dpt14                         393      // float
#define DFA_az07o3Dpt17                         393      // 8 Bits, Bit 7-0
#define DFA_az07o3Dpt232                        393      // 24 Bits, Bit 31-8
#define     DFA_az07o3Dpt232Mask 0xFFFFFF00
#define     DFA_az07o3Dpt232Shift 8
#define DFA_az07o4Send                          397      // 8 Bits, Bit 7-0
#define DFA_az07o4Dpt1                          398      // 8 Bits, Bit 7-0
#define DFA_az07o4Dpt2                          398      // 8 Bits, Bit 7-0
#define DFA_az07o4Dpt5                          398      // uint8_t
#define DFA_az07o4Dpt5001                       398      // uint8_t
#define DFA_az07o4Dpt6                          398      // int8_t
#define DFA_az07o4Dpt7                          398      // uint16_t
#define DFA_az07o4Dpt8                          398      // int16_t
#define DFA_az07o4Dpt9                          398      // float
#define DFA_az07o4Dpt12                         398      // uint32_t
#define DFA_az07o4Dpt13                         398      // int32_t
#define DFA_az07o4Dpt14                         398      // float
#define DFA_az07o4Dpt16                         398      // char*, 14 Byte
#define DFA_az07o4Dpt17                         398      // 8 Bits, Bit 7-0
#define DFA_az07o4Dpt232                        398      // 24 Bits, Bit 31-8
#define     DFA_az07o4Dpt232Mask 0xFFFFFF00
#define     DFA_az07o4Dpt232Shift 8
#define DFA_ad08A                               413      // 8 Bits, Bit 7-0
#define DFA_ad08B                               414      // 8 Bits, Bit 7-0
#define DFA_ad08C                               415      // 8 Bits, Bit 7-0
#define DFA_ad08D                               416      // 8 Bits, Bit 7-0
#define DFA_ad08E                               417      // 8 Bits, Bit 7-0
#define DFA_ad08F                               418      // 8 Bits, Bit 7-0
#define DFA_ad08G                               419      // 8 Bits, Bit 7-0
#define DFA_ad08H                               420      // 8 Bits, Bit 7-0
#define DFA_ad08T                               421      // 8 Bits, Bit 7-0
#define DFA_ad08TBase                           422      // 2 Bits, Bit 7-6
#define     DFA_ad08TBaseMask 0xC0
#define     DFA_ad08TBaseShift 6
#define DFA_ad08TTime                           422      // 14 Bits, Bit 13-0
#define     DFA_ad08TTimeMask 0x3FFF
#define     DFA_ad08TTimeShift 0
#define DFA_az08o1Send                          424      // 8 Bits, Bit 7-0
#define DFA_az08o1Dpt1                          425      // 8 Bits, Bit 7-0
#define DFA_az08o1Dpt2                          425      // 8 Bits, Bit 7-0
#define DFA_az08o1Dpt5                          425      // uint8_t
#define DFA_az08o1Dpt5001                       425      // uint8_t
#define DFA_az08o1Dpt6                          425      // int8_t
#define DFA_az08o1Dpt7                          425      // uint16_t
#define DFA_az08o1Dpt8                          425      // int16_t
#define DFA_az08o1Dpt9                          425      // float
#define DFA_az08o1Dpt12                         425      // uint32_t
#define DFA_az08o1Dpt13                         425      // int32_t
#define DFA_az08o1Dpt14                         425      // float
#define DFA_az08o1Dpt17                         425      // 8 Bits, Bit 7-0
#define DFA_az08o1Dpt232                        425      // 24 Bits, Bit 31-8
#define     DFA_az08o1Dpt232Mask 0xFFFFFF00
#define     DFA_az08o1Dpt232Shift 8
#define DFA_az08o2Send                          429      // 8 Bits, Bit 7-0
#define DFA_az08o2Dpt1                          430      // 8 Bits, Bit 7-0
#define DFA_az08o2Dpt2                          430      // 8 Bits, Bit 7-0
#define DFA_az08o2Dpt5                          430      // uint8_t
#define DFA_az08o2Dpt5001                       430      // uint8_t
#define DFA_az08o2Dpt6                          430      // int8_t
#define DFA_az08o2Dpt7                          430      // uint16_t
#define DFA_az08o2Dpt8                          430      // int16_t
#define DFA_az08o2Dpt9                          430      // float
#define DFA_az08o2Dpt12                         430      // uint32_t
#define DFA_az08o2Dpt13                         430      // int32_t
#define DFA_az08o2Dpt14                         430      // float
#define DFA_az08o2Dpt17                         430      // 8 Bits, Bit 7-0
#define DFA_az08o2Dpt232                        430      // 24 Bits, Bit 31-8
#define     DFA_az08o2Dpt232Mask 0xFFFFFF00
#define     DFA_az08o2Dpt232Shift 8
#define DFA_az08o3Send                          434      // 8 Bits, Bit 7-0
#define DFA_az08o3Dpt1                          435      // 8 Bits, Bit 7-0
#define DFA_az08o3Dpt2                          435      // 8 Bits, Bit 7-0
#define DFA_az08o3Dpt5                          435      // uint8_t
#define DFA_az08o3Dpt5001                       435      // uint8_t
#define DFA_az08o3Dpt6                          435      // int8_t
#define DFA_az08o3Dpt7                          435      // uint16_t
#define DFA_az08o3Dpt8                          435      // int16_t
#define DFA_az08o3Dpt9                          435      // float
#define DFA_az08o3Dpt12                         435      // uint32_t
#define DFA_az08o3Dpt13                         435      // int32_t
#define DFA_az08o3Dpt14                         435      // float
#define DFA_az08o3Dpt17                         435      // 8 Bits, Bit 7-0
#define DFA_az08o3Dpt232                        435      // 24 Bits, Bit 31-8
#define     DFA_az08o3Dpt232Mask 0xFFFFFF00
#define     DFA_az08o3Dpt232Shift 8
#define DFA_az08o4Send                          439      // 8 Bits, Bit 7-0
#define DFA_az08o4Dpt1                          440      // 8 Bits, Bit 7-0
#define DFA_az08o4Dpt2                          440      // 8 Bits, Bit 7-0
#define DFA_az08o4Dpt5                          440      // uint8_t
#define DFA_az08o4Dpt5001                       440      // uint8_t
#define DFA_az08o4Dpt6                          440      // int8_t
#define DFA_az08o4Dpt7                          440      // uint16_t
#define DFA_az08o4Dpt8                          440      // int16_t
#define DFA_az08o4Dpt9                          440      // float
#define DFA_az08o4Dpt12                         440      // uint32_t
#define DFA_az08o4Dpt13                         440      // int32_t
#define DFA_az08o4Dpt14                         440      // float
#define DFA_az08o4Dpt16                         440      // char*, 14 Byte
#define DFA_az08o4Dpt17                         440      // 8 Bits, Bit 7-0
#define DFA_az08o4Dpt232                        440      // 24 Bits, Bit 31-8
#define     DFA_az08o4Dpt232Mask 0xFFFFFF00
#define     DFA_az08o4Dpt232Shift 8
#define DFA_ad09A                               455      // 8 Bits, Bit 7-0
#define DFA_ad09B                               456      // 8 Bits, Bit 7-0
#define DFA_ad09C                               457      // 8 Bits, Bit 7-0
#define DFA_ad09D                               458      // 8 Bits, Bit 7-0
#define DFA_ad09E                               459      // 8 Bits, Bit 7-0
#define DFA_ad09F                               460      // 8 Bits, Bit 7-0
#define DFA_ad09G                               461      // 8 Bits, Bit 7-0
#define DFA_ad09H                               462      // 8 Bits, Bit 7-0
#define DFA_ad09T                               463      // 8 Bits, Bit 7-0
#define DFA_ad09TBase                           464      // 2 Bits, Bit 7-6
#define     DFA_ad09TBaseMask 0xC0
#define     DFA_ad09TBaseShift 6
#define DFA_ad09TTime                           464      // 14 Bits, Bit 13-0
#define     DFA_ad09TTimeMask 0x3FFF
#define     DFA_ad09TTimeShift 0
#define DFA_az09o1Send                          466      // 8 Bits, Bit 7-0
#define DFA_az09o1Dpt1                          467      // 8 Bits, Bit 7-0
#define DFA_az09o1Dpt2                          467      // 8 Bits, Bit 7-0
#define DFA_az09o1Dpt5                          467      // uint8_t
#define DFA_az09o1Dpt5001                       467      // uint8_t
#define DFA_az09o1Dpt6                          467      // int8_t
#define DFA_az09o1Dpt7                          467      // uint16_t
#define DFA_az09o1Dpt8                          467      // int16_t
#define DFA_az09o1Dpt9                          467      // float
#define DFA_az09o1Dpt12                         467      // uint32_t
#define DFA_az09o1Dpt13                         467      // int32_t
#define DFA_az09o1Dpt14                         467      // float
#define DFA_az09o1Dpt17                         467      // 8 Bits, Bit 7-0
#define DFA_az09o1Dpt232                        467      // 24 Bits, Bit 31-8
#define     DFA_az09o1Dpt232Mask 0xFFFFFF00
#define     DFA_az09o1Dpt232Shift 8
#define DFA_az09o2Send                          471      // 8 Bits, Bit 7-0
#define DFA_az09o2Dpt1                          472      // 8 Bits, Bit 7-0
#define DFA_az09o2Dpt2                          472      // 8 Bits, Bit 7-0
#define DFA_az09o2Dpt5                          472      // uint8_t
#define DFA_az09o2Dpt5001                       472      // uint8_t
#define DFA_az09o2Dpt6                          472      // int8_t
#define DFA_az09o2Dpt7                          472      // uint16_t
#define DFA_az09o2Dpt8                          472      // int16_t
#define DFA_az09o2Dpt9                          472      // float
#define DFA_az09o2Dpt12                         472      // uint32_t
#define DFA_az09o2Dpt13                         472      // int32_t
#define DFA_az09o2Dpt14                         472      // float
#define DFA_az09o2Dpt17                         472      // 8 Bits, Bit 7-0
#define DFA_az09o2Dpt232                        472      // 24 Bits, Bit 31-8
#define     DFA_az09o2Dpt232Mask 0xFFFFFF00
#define     DFA_az09o2Dpt232Shift 8
#define DFA_az09o3Send                          476      // 8 Bits, Bit 7-0
#define DFA_az09o3Dpt1                          477      // 8 Bits, Bit 7-0
#define DFA_az09o3Dpt2                          477      // 8 Bits, Bit 7-0
#define DFA_az09o3Dpt5                          477      // uint8_t
#define DFA_az09o3Dpt5001                       477      // uint8_t
#define DFA_az09o3Dpt6                          477      // int8_t
#define DFA_az09o3Dpt7                          477      // uint16_t
#define DFA_az09o3Dpt8                          477      // int16_t
#define DFA_az09o3Dpt9                          477      // float
#define DFA_az09o3Dpt12                         477      // uint32_t
#define DFA_az09o3Dpt13                         477      // int32_t
#define DFA_az09o3Dpt14                         477      // float
#define DFA_az09o3Dpt17                         477      // 8 Bits, Bit 7-0
#define DFA_az09o3Dpt232                        477      // 24 Bits, Bit 31-8
#define     DFA_az09o3Dpt232Mask 0xFFFFFF00
#define     DFA_az09o3Dpt232Shift 8
#define DFA_az09o4Send                          481      // 8 Bits, Bit 7-0
#define DFA_az09o4Dpt1                          482      // 8 Bits, Bit 7-0
#define DFA_az09o4Dpt2                          482      // 8 Bits, Bit 7-0
#define DFA_az09o4Dpt5                          482      // uint8_t
#define DFA_az09o4Dpt5001                       482      // uint8_t
#define DFA_az09o4Dpt6                          482      // int8_t
#define DFA_az09o4Dpt7                          482      // uint16_t
#define DFA_az09o4Dpt8                          482      // int16_t
#define DFA_az09o4Dpt9                          482      // float
#define DFA_az09o4Dpt12                         482      // uint32_t
#define DFA_az09o4Dpt13                         482      // int32_t
#define DFA_az09o4Dpt14                         482      // float
#define DFA_az09o4Dpt16                         482      // char*, 14 Byte
#define DFA_az09o4Dpt17                         482      // 8 Bits, Bit 7-0
#define DFA_az09o4Dpt232                        482      // 24 Bits, Bit 31-8
#define     DFA_az09o4Dpt232Mask 0xFFFFFF00
#define     DFA_az09o4Dpt232Shift 8
#define DFA_ad10A                               497      // 8 Bits, Bit 7-0
#define DFA_ad10B                               498      // 8 Bits, Bit 7-0
#define DFA_ad10C                               499      // 8 Bits, Bit 7-0
#define DFA_ad10D                               500      // 8 Bits, Bit 7-0
#define DFA_ad10E                               501      // 8 Bits, Bit 7-0
#define DFA_ad10F                               502      // 8 Bits, Bit 7-0
#define DFA_ad10G                               503      // 8 Bits, Bit 7-0
#define DFA_ad10H                               504      // 8 Bits, Bit 7-0
#define DFA_ad10T                               505      // 8 Bits, Bit 7-0
#define DFA_ad10TBase                           506      // 2 Bits, Bit 7-6
#define     DFA_ad10TBaseMask 0xC0
#define     DFA_ad10TBaseShift 6
#define DFA_ad10TTime                           506      // 14 Bits, Bit 13-0
#define     DFA_ad10TTimeMask 0x3FFF
#define     DFA_ad10TTimeShift 0
#define DFA_az10o1Send                          508      // 8 Bits, Bit 7-0
#define DFA_az10o1Dpt1                          509      // 8 Bits, Bit 7-0
#define DFA_az10o1Dpt2                          509      // 8 Bits, Bit 7-0
#define DFA_az10o1Dpt5                          509      // uint8_t
#define DFA_az10o1Dpt5001                       509      // uint8_t
#define DFA_az10o1Dpt6                          509      // int8_t
#define DFA_az10o1Dpt7                          509      // uint16_t
#define DFA_az10o1Dpt8                          509      // int16_t
#define DFA_az10o1Dpt9                          509      // float
#define DFA_az10o1Dpt12                         509      // uint32_t
#define DFA_az10o1Dpt13                         509      // int32_t
#define DFA_az10o1Dpt14                         509      // float
#define DFA_az10o1Dpt17                         509      // 8 Bits, Bit 7-0
#define DFA_az10o1Dpt232                        509      // 24 Bits, Bit 31-8
#define     DFA_az10o1Dpt232Mask 0xFFFFFF00
#define     DFA_az10o1Dpt232Shift 8
#define DFA_az10o2Send                          513      // 8 Bits, Bit 7-0
#define DFA_az10o2Dpt1                          514      // 8 Bits, Bit 7-0
#define DFA_az10o2Dpt2                          514      // 8 Bits, Bit 7-0
#define DFA_az10o2Dpt5                          514      // uint8_t
#define DFA_az10o2Dpt5001                       514      // uint8_t
#define DFA_az10o2Dpt6                          514      // int8_t
#define DFA_az10o2Dpt7                          514      // uint16_t
#define DFA_az10o2Dpt8                          514      // int16_t
#define DFA_az10o2Dpt9                          514      // float
#define DFA_az10o2Dpt12                         514      // uint32_t
#define DFA_az10o2Dpt13                         514      // int32_t
#define DFA_az10o2Dpt14                         514      // float
#define DFA_az10o2Dpt17                         514      // 8 Bits, Bit 7-0
#define DFA_az10o2Dpt232                        514      // 24 Bits, Bit 31-8
#define     DFA_az10o2Dpt232Mask 0xFFFFFF00
#define     DFA_az10o2Dpt232Shift 8
#define DFA_az10o3Send                          518      // 8 Bits, Bit 7-0
#define DFA_az10o3Dpt1                          519      // 8 Bits, Bit 7-0
#define DFA_az10o3Dpt2                          519      // 8 Bits, Bit 7-0
#define DFA_az10o3Dpt5                          519      // uint8_t
#define DFA_az10o3Dpt5001                       519      // uint8_t
#define DFA_az10o3Dpt6                          519      // int8_t
#define DFA_az10o3Dpt7                          519      // uint16_t
#define DFA_az10o3Dpt8                          519      // int16_t
#define DFA_az10o3Dpt9                          519      // float
#define DFA_az10o3Dpt12                         519      // uint32_t
#define DFA_az10o3Dpt13                         519      // int32_t
#define DFA_az10o3Dpt14                         519      // float
#define DFA_az10o3Dpt17                         519      // 8 Bits, Bit 7-0
#define DFA_az10o3Dpt232                        519      // 24 Bits, Bit 31-8
#define     DFA_az10o3Dpt232Mask 0xFFFFFF00
#define     DFA_az10o3Dpt232Shift 8
#define DFA_az10o4Send                          523      // 8 Bits, Bit 7-0
#define DFA_az10o4Dpt1                          524      // 8 Bits, Bit 7-0
#define DFA_az10o4Dpt2                          524      // 8 Bits, Bit 7-0
#define DFA_az10o4Dpt5                          524      // uint8_t
#define DFA_az10o4Dpt5001                       524      // uint8_t
#define DFA_az10o4Dpt6                          524      // int8_t
#define DFA_az10o4Dpt7                          524      // uint16_t
#define DFA_az10o4Dpt8                          524      // int16_t
#define DFA_az10o4Dpt9                          524      // float
#define DFA_az10o4Dpt12                         524      // uint32_t
#define DFA_az10o4Dpt13                         524      // int32_t
#define DFA_az10o4Dpt14                         524      // float
#define DFA_az10o4Dpt16                         524      // char*, 14 Byte
#define DFA_az10o4Dpt17                         524      // 8 Bits, Bit 7-0
#define DFA_az10o4Dpt232                        524      // 24 Bits, Bit 31-8
#define     DFA_az10o4Dpt232Mask 0xFFFFFF00
#define     DFA_az10o4Dpt232Shift 8
#define DFA_ad11A                               539      // 8 Bits, Bit 7-0
#define DFA_ad11B                               540      // 8 Bits, Bit 7-0
#define DFA_ad11C                               541      // 8 Bits, Bit 7-0
#define DFA_ad11D                               542      // 8 Bits, Bit 7-0
#define DFA_ad11E                               543      // 8 Bits, Bit 7-0
#define DFA_ad11F                               544      // 8 Bits, Bit 7-0
#define DFA_ad11G                               545      // 8 Bits, Bit 7-0
#define DFA_ad11H                               546      // 8 Bits, Bit 7-0
#define DFA_ad11T                               547      // 8 Bits, Bit 7-0
#define DFA_ad11TBase                           548      // 2 Bits, Bit 7-6
#define     DFA_ad11TBaseMask 0xC0
#define     DFA_ad11TBaseShift 6
#define DFA_ad11TTime                           548      // 14 Bits, Bit 13-0
#define     DFA_ad11TTimeMask 0x3FFF
#define     DFA_ad11TTimeShift 0
#define DFA_az11o1Send                          550      // 8 Bits, Bit 7-0
#define DFA_az11o1Dpt1                          551      // 8 Bits, Bit 7-0
#define DFA_az11o1Dpt2                          551      // 8 Bits, Bit 7-0
#define DFA_az11o1Dpt5                          551      // uint8_t
#define DFA_az11o1Dpt5001                       551      // uint8_t
#define DFA_az11o1Dpt6                          551      // int8_t
#define DFA_az11o1Dpt7                          551      // uint16_t
#define DFA_az11o1Dpt8                          551      // int16_t
#define DFA_az11o1Dpt9                          551      // float
#define DFA_az11o1Dpt12                         551      // uint32_t
#define DFA_az11o1Dpt13                         551      // int32_t
#define DFA_az11o1Dpt14                         551      // float
#define DFA_az11o1Dpt17                         551      // 8 Bits, Bit 7-0
#define DFA_az11o1Dpt232                        551      // 24 Bits, Bit 31-8
#define     DFA_az11o1Dpt232Mask 0xFFFFFF00
#define     DFA_az11o1Dpt232Shift 8
#define DFA_az11o2Send                          555      // 8 Bits, Bit 7-0
#define DFA_az11o2Dpt1                          556      // 8 Bits, Bit 7-0
#define DFA_az11o2Dpt2                          556      // 8 Bits, Bit 7-0
#define DFA_az11o2Dpt5                          556      // uint8_t
#define DFA_az11o2Dpt5001                       556      // uint8_t
#define DFA_az11o2Dpt6                          556      // int8_t
#define DFA_az11o2Dpt7                          556      // uint16_t
#define DFA_az11o2Dpt8                          556      // int16_t
#define DFA_az11o2Dpt9                          556      // float
#define DFA_az11o2Dpt12                         556      // uint32_t
#define DFA_az11o2Dpt13                         556      // int32_t
#define DFA_az11o2Dpt14                         556      // float
#define DFA_az11o2Dpt17                         556      // 8 Bits, Bit 7-0
#define DFA_az11o2Dpt232                        556      // 24 Bits, Bit 31-8
#define     DFA_az11o2Dpt232Mask 0xFFFFFF00
#define     DFA_az11o2Dpt232Shift 8
#define DFA_az11o3Send                          560      // 8 Bits, Bit 7-0
#define DFA_az11o3Dpt1                          561      // 8 Bits, Bit 7-0
#define DFA_az11o3Dpt2                          561      // 8 Bits, Bit 7-0
#define DFA_az11o3Dpt5                          561      // uint8_t
#define DFA_az11o3Dpt5001                       561      // uint8_t
#define DFA_az11o3Dpt6                          561      // int8_t
#define DFA_az11o3Dpt7                          561      // uint16_t
#define DFA_az11o3Dpt8                          561      // int16_t
#define DFA_az11o3Dpt9                          561      // float
#define DFA_az11o3Dpt12                         561      // uint32_t
#define DFA_az11o3Dpt13                         561      // int32_t
#define DFA_az11o3Dpt14                         561      // float
#define DFA_az11o3Dpt17                         561      // 8 Bits, Bit 7-0
#define DFA_az11o3Dpt232                        561      // 24 Bits, Bit 31-8
#define     DFA_az11o3Dpt232Mask 0xFFFFFF00
#define     DFA_az11o3Dpt232Shift 8
#define DFA_az11o4Send                          565      // 8 Bits, Bit 7-0
#define DFA_az11o4Dpt1                          566      // 8 Bits, Bit 7-0
#define DFA_az11o4Dpt2                          566      // 8 Bits, Bit 7-0
#define DFA_az11o4Dpt5                          566      // uint8_t
#define DFA_az11o4Dpt5001                       566      // uint8_t
#define DFA_az11o4Dpt6                          566      // int8_t
#define DFA_az11o4Dpt7                          566      // uint16_t
#define DFA_az11o4Dpt8                          566      // int16_t
#define DFA_az11o4Dpt9                          566      // float
#define DFA_az11o4Dpt12                         566      // uint32_t
#define DFA_az11o4Dpt13                         566      // int32_t
#define DFA_az11o4Dpt14                         566      // float
#define DFA_az11o4Dpt16                         566      // char*, 14 Byte
#define DFA_az11o4Dpt17                         566      // 8 Bits, Bit 7-0
#define DFA_az11o4Dpt232                        566      // 24 Bits, Bit 31-8
#define     DFA_az11o4Dpt232Mask 0xFFFFFF00
#define     DFA_az11o4Dpt232Shift 8
#define DFA_ad12A                               581      // 8 Bits, Bit 7-0
#define DFA_ad12B                               582      // 8 Bits, Bit 7-0
#define DFA_ad12C                               583      // 8 Bits, Bit 7-0
#define DFA_ad12D                               584      // 8 Bits, Bit 7-0
#define DFA_ad12E                               585      // 8 Bits, Bit 7-0
#define DFA_ad12F                               586      // 8 Bits, Bit 7-0
#define DFA_ad12G                               587      // 8 Bits, Bit 7-0
#define DFA_ad12H                               588      // 8 Bits, Bit 7-0
#define DFA_ad12T                               589      // 8 Bits, Bit 7-0
#define DFA_ad12TBase                           590      // 2 Bits, Bit 7-6
#define     DFA_ad12TBaseMask 0xC0
#define     DFA_ad12TBaseShift 6
#define DFA_ad12TTime                           590      // 14 Bits, Bit 13-0
#define     DFA_ad12TTimeMask 0x3FFF
#define     DFA_ad12TTimeShift 0
#define DFA_az12o1Send                          592      // 8 Bits, Bit 7-0
#define DFA_az12o1Dpt1                          593      // 8 Bits, Bit 7-0
#define DFA_az12o1Dpt2                          593      // 8 Bits, Bit 7-0
#define DFA_az12o1Dpt5                          593      // uint8_t
#define DFA_az12o1Dpt5001                       593      // uint8_t
#define DFA_az12o1Dpt6                          593      // int8_t
#define DFA_az12o1Dpt7                          593      // uint16_t
#define DFA_az12o1Dpt8                          593      // int16_t
#define DFA_az12o1Dpt9                          593      // float
#define DFA_az12o1Dpt12                         593      // uint32_t
#define DFA_az12o1Dpt13                         593      // int32_t
#define DFA_az12o1Dpt14                         593      // float
#define DFA_az12o1Dpt17                         593      // 8 Bits, Bit 7-0
#define DFA_az12o1Dpt232                        593      // 24 Bits, Bit 31-8
#define     DFA_az12o1Dpt232Mask 0xFFFFFF00
#define     DFA_az12o1Dpt232Shift 8
#define DFA_az12o2Send                          597      // 8 Bits, Bit 7-0
#define DFA_az12o2Dpt1                          598      // 8 Bits, Bit 7-0
#define DFA_az12o2Dpt2                          598      // 8 Bits, Bit 7-0
#define DFA_az12o2Dpt5                          598      // uint8_t
#define DFA_az12o2Dpt5001                       598      // uint8_t
#define DFA_az12o2Dpt6                          598      // int8_t
#define DFA_az12o2Dpt7                          598      // uint16_t
#define DFA_az12o2Dpt8                          598      // int16_t
#define DFA_az12o2Dpt9                          598      // float
#define DFA_az12o2Dpt12                         598      // uint32_t
#define DFA_az12o2Dpt13                         598      // int32_t
#define DFA_az12o2Dpt14                         598      // float
#define DFA_az12o2Dpt17                         598      // 8 Bits, Bit 7-0
#define DFA_az12o2Dpt232                        598      // 24 Bits, Bit 31-8
#define     DFA_az12o2Dpt232Mask 0xFFFFFF00
#define     DFA_az12o2Dpt232Shift 8
#define DFA_az12o3Send                          602      // 8 Bits, Bit 7-0
#define DFA_az12o3Dpt1                          603      // 8 Bits, Bit 7-0
#define DFA_az12o3Dpt2                          603      // 8 Bits, Bit 7-0
#define DFA_az12o3Dpt5                          603      // uint8_t
#define DFA_az12o3Dpt5001                       603      // uint8_t
#define DFA_az12o3Dpt6                          603      // int8_t
#define DFA_az12o3Dpt7                          603      // uint16_t
#define DFA_az12o3Dpt8                          603      // int16_t
#define DFA_az12o3Dpt9                          603      // float
#define DFA_az12o3Dpt12                         603      // uint32_t
#define DFA_az12o3Dpt13                         603      // int32_t
#define DFA_az12o3Dpt14                         603      // float
#define DFA_az12o3Dpt17                         603      // 8 Bits, Bit 7-0
#define DFA_az12o3Dpt232                        603      // 24 Bits, Bit 31-8
#define     DFA_az12o3Dpt232Mask 0xFFFFFF00
#define     DFA_az12o3Dpt232Shift 8
#define DFA_az12o4Send                          607      // 8 Bits, Bit 7-0
#define DFA_az12o4Dpt1                          608      // 8 Bits, Bit 7-0
#define DFA_az12o4Dpt2                          608      // 8 Bits, Bit 7-0
#define DFA_az12o4Dpt5                          608      // uint8_t
#define DFA_az12o4Dpt5001                       608      // uint8_t
#define DFA_az12o4Dpt6                          608      // int8_t
#define DFA_az12o4Dpt7                          608      // uint16_t
#define DFA_az12o4Dpt8                          608      // int16_t
#define DFA_az12o4Dpt9                          608      // float
#define DFA_az12o4Dpt12                         608      // uint32_t
#define DFA_az12o4Dpt13                         608      // int32_t
#define DFA_az12o4Dpt14                         608      // float
#define DFA_az12o4Dpt16                         608      // char*, 14 Byte
#define DFA_az12o4Dpt17                         608      // 8 Bits, Bit 7-0
#define DFA_az12o4Dpt232                        608      // 24 Bits, Bit 31-8
#define     DFA_az12o4Dpt232Mask 0xFFFFFF00
#define     DFA_az12o4Dpt232Shift 8
#define DFA_ad13A                               623      // 8 Bits, Bit 7-0
#define DFA_ad13B                               624      // 8 Bits, Bit 7-0
#define DFA_ad13C                               625      // 8 Bits, Bit 7-0
#define DFA_ad13D                               626      // 8 Bits, Bit 7-0
#define DFA_ad13E                               627      // 8 Bits, Bit 7-0
#define DFA_ad13F                               628      // 8 Bits, Bit 7-0
#define DFA_ad13G                               629      // 8 Bits, Bit 7-0
#define DFA_ad13H                               630      // 8 Bits, Bit 7-0
#define DFA_ad13T                               631      // 8 Bits, Bit 7-0
#define DFA_ad13TBase                           632      // 2 Bits, Bit 7-6
#define     DFA_ad13TBaseMask 0xC0
#define     DFA_ad13TBaseShift 6
#define DFA_ad13TTime                           632      // 14 Bits, Bit 13-0
#define     DFA_ad13TTimeMask 0x3FFF
#define     DFA_ad13TTimeShift 0
#define DFA_az13o1Send                          634      // 8 Bits, Bit 7-0
#define DFA_az13o1Dpt1                          635      // 8 Bits, Bit 7-0
#define DFA_az13o1Dpt2                          635      // 8 Bits, Bit 7-0
#define DFA_az13o1Dpt5                          635      // uint8_t
#define DFA_az13o1Dpt5001                       635      // uint8_t
#define DFA_az13o1Dpt6                          635      // int8_t
#define DFA_az13o1Dpt7                          635      // uint16_t
#define DFA_az13o1Dpt8                          635      // int16_t
#define DFA_az13o1Dpt9                          635      // float
#define DFA_az13o1Dpt12                         635      // uint32_t
#define DFA_az13o1Dpt13                         635      // int32_t
#define DFA_az13o1Dpt14                         635      // float
#define DFA_az13o1Dpt17                         635      // 8 Bits, Bit 7-0
#define DFA_az13o1Dpt232                        635      // 24 Bits, Bit 31-8
#define     DFA_az13o1Dpt232Mask 0xFFFFFF00
#define     DFA_az13o1Dpt232Shift 8
#define DFA_az13o2Send                          639      // 8 Bits, Bit 7-0
#define DFA_az13o2Dpt1                          640      // 8 Bits, Bit 7-0
#define DFA_az13o2Dpt2                          640      // 8 Bits, Bit 7-0
#define DFA_az13o2Dpt5                          640      // uint8_t
#define DFA_az13o2Dpt5001                       640      // uint8_t
#define DFA_az13o2Dpt6                          640      // int8_t
#define DFA_az13o2Dpt7                          640      // uint16_t
#define DFA_az13o2Dpt8                          640      // int16_t
#define DFA_az13o2Dpt9                          640      // float
#define DFA_az13o2Dpt12                         640      // uint32_t
#define DFA_az13o2Dpt13                         640      // int32_t
#define DFA_az13o2Dpt14                         640      // float
#define DFA_az13o2Dpt17                         640      // 8 Bits, Bit 7-0
#define DFA_az13o2Dpt232                        640      // 24 Bits, Bit 31-8
#define     DFA_az13o2Dpt232Mask 0xFFFFFF00
#define     DFA_az13o2Dpt232Shift 8
#define DFA_az13o3Send                          644      // 8 Bits, Bit 7-0
#define DFA_az13o3Dpt1                          645      // 8 Bits, Bit 7-0
#define DFA_az13o3Dpt2                          645      // 8 Bits, Bit 7-0
#define DFA_az13o3Dpt5                          645      // uint8_t
#define DFA_az13o3Dpt5001                       645      // uint8_t
#define DFA_az13o3Dpt6                          645      // int8_t
#define DFA_az13o3Dpt7                          645      // uint16_t
#define DFA_az13o3Dpt8                          645      // int16_t
#define DFA_az13o3Dpt9                          645      // float
#define DFA_az13o3Dpt12                         645      // uint32_t
#define DFA_az13o3Dpt13                         645      // int32_t
#define DFA_az13o3Dpt14                         645      // float
#define DFA_az13o3Dpt17                         645      // 8 Bits, Bit 7-0
#define DFA_az13o3Dpt232                        645      // 24 Bits, Bit 31-8
#define     DFA_az13o3Dpt232Mask 0xFFFFFF00
#define     DFA_az13o3Dpt232Shift 8
#define DFA_az13o4Send                          649      // 8 Bits, Bit 7-0
#define DFA_az13o4Dpt1                          650      // 8 Bits, Bit 7-0
#define DFA_az13o4Dpt2                          650      // 8 Bits, Bit 7-0
#define DFA_az13o4Dpt5                          650      // uint8_t
#define DFA_az13o4Dpt5001                       650      // uint8_t
#define DFA_az13o4Dpt6                          650      // int8_t
#define DFA_az13o4Dpt7                          650      // uint16_t
#define DFA_az13o4Dpt8                          650      // int16_t
#define DFA_az13o4Dpt9                          650      // float
#define DFA_az13o4Dpt12                         650      // uint32_t
#define DFA_az13o4Dpt13                         650      // int32_t
#define DFA_az13o4Dpt14                         650      // float
#define DFA_az13o4Dpt16                         650      // char*, 14 Byte
#define DFA_az13o4Dpt17                         650      // 8 Bits, Bit 7-0
#define DFA_az13o4Dpt232                        650      // 24 Bits, Bit 31-8
#define     DFA_az13o4Dpt232Mask 0xFFFFFF00
#define     DFA_az13o4Dpt232Shift 8
#define DFA_ad14A                               665      // 8 Bits, Bit 7-0
#define DFA_ad14B                               666      // 8 Bits, Bit 7-0
#define DFA_ad14C                               667      // 8 Bits, Bit 7-0
#define DFA_ad14D                               668      // 8 Bits, Bit 7-0
#define DFA_ad14E                               669      // 8 Bits, Bit 7-0
#define DFA_ad14F                               670      // 8 Bits, Bit 7-0
#define DFA_ad14G                               671      // 8 Bits, Bit 7-0
#define DFA_ad14H                               672      // 8 Bits, Bit 7-0
#define DFA_ad14T                               673      // 8 Bits, Bit 7-0
#define DFA_ad14TBase                           674      // 2 Bits, Bit 7-6
#define     DFA_ad14TBaseMask 0xC0
#define     DFA_ad14TBaseShift 6
#define DFA_ad14TTime                           674      // 14 Bits, Bit 13-0
#define     DFA_ad14TTimeMask 0x3FFF
#define     DFA_ad14TTimeShift 0
#define DFA_az14o1Send                          676      // 8 Bits, Bit 7-0
#define DFA_az14o1Dpt1                          677      // 8 Bits, Bit 7-0
#define DFA_az14o1Dpt2                          677      // 8 Bits, Bit 7-0
#define DFA_az14o1Dpt5                          677      // uint8_t
#define DFA_az14o1Dpt5001                       677      // uint8_t
#define DFA_az14o1Dpt6                          677      // int8_t
#define DFA_az14o1Dpt7                          677      // uint16_t
#define DFA_az14o1Dpt8                          677      // int16_t
#define DFA_az14o1Dpt9                          677      // float
#define DFA_az14o1Dpt12                         677      // uint32_t
#define DFA_az14o1Dpt13                         677      // int32_t
#define DFA_az14o1Dpt14                         677      // float
#define DFA_az14o1Dpt17                         677      // 8 Bits, Bit 7-0
#define DFA_az14o1Dpt232                        677      // 24 Bits, Bit 31-8
#define     DFA_az14o1Dpt232Mask 0xFFFFFF00
#define     DFA_az14o1Dpt232Shift 8
#define DFA_az14o2Send                          681      // 8 Bits, Bit 7-0
#define DFA_az14o2Dpt1                          682      // 8 Bits, Bit 7-0
#define DFA_az14o2Dpt2                          682      // 8 Bits, Bit 7-0
#define DFA_az14o2Dpt5                          682      // uint8_t
#define DFA_az14o2Dpt5001                       682      // uint8_t
#define DFA_az14o2Dpt6                          682      // int8_t
#define DFA_az14o2Dpt7                          682      // uint16_t
#define DFA_az14o2Dpt8                          682      // int16_t
#define DFA_az14o2Dpt9                          682      // float
#define DFA_az14o2Dpt12                         682      // uint32_t
#define DFA_az14o2Dpt13                         682      // int32_t
#define DFA_az14o2Dpt14                         682      // float
#define DFA_az14o2Dpt17                         682      // 8 Bits, Bit 7-0
#define DFA_az14o2Dpt232                        682      // 24 Bits, Bit 31-8
#define     DFA_az14o2Dpt232Mask 0xFFFFFF00
#define     DFA_az14o2Dpt232Shift 8
#define DFA_az14o3Send                          686      // 8 Bits, Bit 7-0
#define DFA_az14o3Dpt1                          687      // 8 Bits, Bit 7-0
#define DFA_az14o3Dpt2                          687      // 8 Bits, Bit 7-0
#define DFA_az14o3Dpt5                          687      // uint8_t
#define DFA_az14o3Dpt5001                       687      // uint8_t
#define DFA_az14o3Dpt6                          687      // int8_t
#define DFA_az14o3Dpt7                          687      // uint16_t
#define DFA_az14o3Dpt8                          687      // int16_t
#define DFA_az14o3Dpt9                          687      // float
#define DFA_az14o3Dpt12                         687      // uint32_t
#define DFA_az14o3Dpt13                         687      // int32_t
#define DFA_az14o3Dpt14                         687      // float
#define DFA_az14o3Dpt17                         687      // 8 Bits, Bit 7-0
#define DFA_az14o3Dpt232                        687      // 24 Bits, Bit 31-8
#define     DFA_az14o3Dpt232Mask 0xFFFFFF00
#define     DFA_az14o3Dpt232Shift 8
#define DFA_az14o4Send                          691      // 8 Bits, Bit 7-0
#define DFA_az14o4Dpt1                          692      // 8 Bits, Bit 7-0
#define DFA_az14o4Dpt2                          692      // 8 Bits, Bit 7-0
#define DFA_az14o4Dpt5                          692      // uint8_t
#define DFA_az14o4Dpt5001                       692      // uint8_t
#define DFA_az14o4Dpt6                          692      // int8_t
#define DFA_az14o4Dpt7                          692      // uint16_t
#define DFA_az14o4Dpt8                          692      // int16_t
#define DFA_az14o4Dpt9                          692      // float
#define DFA_az14o4Dpt12                         692      // uint32_t
#define DFA_az14o4Dpt13                         692      // int32_t
#define DFA_az14o4Dpt14                         692      // float
#define DFA_az14o4Dpt16                         692      // char*, 14 Byte
#define DFA_az14o4Dpt17                         692      // 8 Bits, Bit 7-0
#define DFA_az14o4Dpt232                        692      // 24 Bits, Bit 31-8
#define     DFA_az14o4Dpt232Mask 0xFFFFFF00
#define     DFA_az14o4Dpt232Shift 8
#define DFA_ad15A                               707      // 8 Bits, Bit 7-0
#define DFA_ad15B                               708      // 8 Bits, Bit 7-0
#define DFA_ad15C                               709      // 8 Bits, Bit 7-0
#define DFA_ad15D                               710      // 8 Bits, Bit 7-0
#define DFA_ad15E                               711      // 8 Bits, Bit 7-0
#define DFA_ad15F                               712      // 8 Bits, Bit 7-0
#define DFA_ad15G                               713      // 8 Bits, Bit 7-0
#define DFA_ad15H                               714      // 8 Bits, Bit 7-0
#define DFA_ad15T                               715      // 8 Bits, Bit 7-0
#define DFA_ad15TBase                           716      // 2 Bits, Bit 7-6
#define     DFA_ad15TBaseMask 0xC0
#define     DFA_ad15TBaseShift 6
#define DFA_ad15TTime                           716      // 14 Bits, Bit 13-0
#define     DFA_ad15TTimeMask 0x3FFF
#define     DFA_ad15TTimeShift 0
#define DFA_az15o1Send                          718      // 8 Bits, Bit 7-0
#define DFA_az15o1Dpt1                          719      // 8 Bits, Bit 7-0
#define DFA_az15o1Dpt2                          719      // 8 Bits, Bit 7-0
#define DFA_az15o1Dpt5                          719      // uint8_t
#define DFA_az15o1Dpt5001                       719      // uint8_t
#define DFA_az15o1Dpt6                          719      // int8_t
#define DFA_az15o1Dpt7                          719      // uint16_t
#define DFA_az15o1Dpt8                          719      // int16_t
#define DFA_az15o1Dpt9                          719      // float
#define DFA_az15o1Dpt12                         719      // uint32_t
#define DFA_az15o1Dpt13                         719      // int32_t
#define DFA_az15o1Dpt14                         719      // float
#define DFA_az15o1Dpt17                         719      // 8 Bits, Bit 7-0
#define DFA_az15o1Dpt232                        719      // 24 Bits, Bit 31-8
#define     DFA_az15o1Dpt232Mask 0xFFFFFF00
#define     DFA_az15o1Dpt232Shift 8
#define DFA_az15o2Send                          723      // 8 Bits, Bit 7-0
#define DFA_az15o2Dpt1                          724      // 8 Bits, Bit 7-0
#define DFA_az15o2Dpt2                          724      // 8 Bits, Bit 7-0
#define DFA_az15o2Dpt5                          724      // uint8_t
#define DFA_az15o2Dpt5001                       724      // uint8_t
#define DFA_az15o2Dpt6                          724      // int8_t
#define DFA_az15o2Dpt7                          724      // uint16_t
#define DFA_az15o2Dpt8                          724      // int16_t
#define DFA_az15o2Dpt9                          724      // float
#define DFA_az15o2Dpt12                         724      // uint32_t
#define DFA_az15o2Dpt13                         724      // int32_t
#define DFA_az15o2Dpt14                         724      // float
#define DFA_az15o2Dpt17                         724      // 8 Bits, Bit 7-0
#define DFA_az15o2Dpt232                        724      // 24 Bits, Bit 31-8
#define     DFA_az15o2Dpt232Mask 0xFFFFFF00
#define     DFA_az15o2Dpt232Shift 8
#define DFA_az15o3Send                          728      // 8 Bits, Bit 7-0
#define DFA_az15o3Dpt1                          729      // 8 Bits, Bit 7-0
#define DFA_az15o3Dpt2                          729      // 8 Bits, Bit 7-0
#define DFA_az15o3Dpt5                          729      // uint8_t
#define DFA_az15o3Dpt5001                       729      // uint8_t
#define DFA_az15o3Dpt6                          729      // int8_t
#define DFA_az15o3Dpt7                          729      // uint16_t
#define DFA_az15o3Dpt8                          729      // int16_t
#define DFA_az15o3Dpt9                          729      // float
#define DFA_az15o3Dpt12                         729      // uint32_t
#define DFA_az15o3Dpt13                         729      // int32_t
#define DFA_az15o3Dpt14                         729      // float
#define DFA_az15o3Dpt17                         729      // 8 Bits, Bit 7-0
#define DFA_az15o3Dpt232                        729      // 24 Bits, Bit 31-8
#define     DFA_az15o3Dpt232Mask 0xFFFFFF00
#define     DFA_az15o3Dpt232Shift 8
#define DFA_az15o4Send                          733      // 8 Bits, Bit 7-0
#define DFA_az15o4Dpt1                          734      // 8 Bits, Bit 7-0
#define DFA_az15o4Dpt2                          734      // 8 Bits, Bit 7-0
#define DFA_az15o4Dpt5                          734      // uint8_t
#define DFA_az15o4Dpt5001                       734      // uint8_t
#define DFA_az15o4Dpt6                          734      // int8_t
#define DFA_az15o4Dpt7                          734      // uint16_t
#define DFA_az15o4Dpt8                          734      // int16_t
#define DFA_az15o4Dpt9                          734      // float
#define DFA_az15o4Dpt12                         734      // uint32_t
#define DFA_az15o4Dpt13                         734      // int32_t
#define DFA_az15o4Dpt14                         734      // float
#define DFA_az15o4Dpt16                         734      // char*, 14 Byte
#define DFA_az15o4Dpt17                         734      // 8 Bits, Bit 7-0
#define DFA_az15o4Dpt232                        734      // 24 Bits, Bit 31-8
#define     DFA_az15o4Dpt232Mask 0xFFFFFF00
#define     DFA_az15o4Dpt232Shift 8
#define DFA_ad16A                               749      // 8 Bits, Bit 7-0
#define DFA_ad16B                               750      // 8 Bits, Bit 7-0
#define DFA_ad16C                               751      // 8 Bits, Bit 7-0
#define DFA_ad16D                               752      // 8 Bits, Bit 7-0
#define DFA_ad16E                               753      // 8 Bits, Bit 7-0
#define DFA_ad16F                               754      // 8 Bits, Bit 7-0
#define DFA_ad16G                               755      // 8 Bits, Bit 7-0
#define DFA_ad16H                               756      // 8 Bits, Bit 7-0
#define DFA_ad16T                               757      // 8 Bits, Bit 7-0
#define DFA_ad16TBase                           758      // 2 Bits, Bit 7-6
#define     DFA_ad16TBaseMask 0xC0
#define     DFA_ad16TBaseShift 6
#define DFA_ad16TTime                           758      // 14 Bits, Bit 13-0
#define     DFA_ad16TTimeMask 0x3FFF
#define     DFA_ad16TTimeShift 0
#define DFA_az16o1Send                          760      // 8 Bits, Bit 7-0
#define DFA_az16o1Dpt1                          761      // 8 Bits, Bit 7-0
#define DFA_az16o1Dpt2                          761      // 8 Bits, Bit 7-0
#define DFA_az16o1Dpt5                          761      // uint8_t
#define DFA_az16o1Dpt5001                       761      // uint8_t
#define DFA_az16o1Dpt6                          761      // int8_t
#define DFA_az16o1Dpt7                          761      // uint16_t
#define DFA_az16o1Dpt8                          761      // int16_t
#define DFA_az16o1Dpt9                          761      // float
#define DFA_az16o1Dpt12                         761      // uint32_t
#define DFA_az16o1Dpt13                         761      // int32_t
#define DFA_az16o1Dpt14                         761      // float
#define DFA_az16o1Dpt17                         761      // 8 Bits, Bit 7-0
#define DFA_az16o1Dpt232                        761      // 24 Bits, Bit 31-8
#define     DFA_az16o1Dpt232Mask 0xFFFFFF00
#define     DFA_az16o1Dpt232Shift 8
#define DFA_az16o2Send                          765      // 8 Bits, Bit 7-0
#define DFA_az16o2Dpt1                          766      // 8 Bits, Bit 7-0
#define DFA_az16o2Dpt2                          766      // 8 Bits, Bit 7-0
#define DFA_az16o2Dpt5                          766      // uint8_t
#define DFA_az16o2Dpt5001                       766      // uint8_t
#define DFA_az16o2Dpt6                          766      // int8_t
#define DFA_az16o2Dpt7                          766      // uint16_t
#define DFA_az16o2Dpt8                          766      // int16_t
#define DFA_az16o2Dpt9                          766      // float
#define DFA_az16o2Dpt12                         766      // uint32_t
#define DFA_az16o2Dpt13                         766      // int32_t
#define DFA_az16o2Dpt14                         766      // float
#define DFA_az16o2Dpt17                         766      // 8 Bits, Bit 7-0
#define DFA_az16o2Dpt232                        766      // 24 Bits, Bit 31-8
#define     DFA_az16o2Dpt232Mask 0xFFFFFF00
#define     DFA_az16o2Dpt232Shift 8
#define DFA_az16o3Send                          770      // 8 Bits, Bit 7-0
#define DFA_az16o3Dpt1                          771      // 8 Bits, Bit 7-0
#define DFA_az16o3Dpt2                          771      // 8 Bits, Bit 7-0
#define DFA_az16o3Dpt5                          771      // uint8_t
#define DFA_az16o3Dpt5001                       771      // uint8_t
#define DFA_az16o3Dpt6                          771      // int8_t
#define DFA_az16o3Dpt7                          771      // uint16_t
#define DFA_az16o3Dpt8                          771      // int16_t
#define DFA_az16o3Dpt9                          771      // float
#define DFA_az16o3Dpt12                         771      // uint32_t
#define DFA_az16o3Dpt13                         771      // int32_t
#define DFA_az16o3Dpt14                         771      // float
#define DFA_az16o3Dpt17                         771      // 8 Bits, Bit 7-0
#define DFA_az16o3Dpt232                        771      // 24 Bits, Bit 31-8
#define     DFA_az16o3Dpt232Mask 0xFFFFFF00
#define     DFA_az16o3Dpt232Shift 8
#define DFA_az16o4Send                          775      // 8 Bits, Bit 7-0
#define DFA_az16o4Dpt1                          776      // 8 Bits, Bit 7-0
#define DFA_az16o4Dpt2                          776      // 8 Bits, Bit 7-0
#define DFA_az16o4Dpt5                          776      // uint8_t
#define DFA_az16o4Dpt5001                       776      // uint8_t
#define DFA_az16o4Dpt6                          776      // int8_t
#define DFA_az16o4Dpt7                          776      // uint16_t
#define DFA_az16o4Dpt8                          776      // int16_t
#define DFA_az16o4Dpt9                          776      // float
#define DFA_az16o4Dpt12                         776      // uint32_t
#define DFA_az16o4Dpt13                         776      // int32_t
#define DFA_az16o4Dpt14                         776      // float
#define DFA_az16o4Dpt16                         776      // char*, 14 Byte
#define DFA_az16o4Dpt17                         776      // 8 Bits, Bit 7-0
#define DFA_az16o4Dpt232                        776      // 24 Bits, Bit 31-8
#define     DFA_az16o4Dpt232Mask 0xFFFFFF00
#define     DFA_az16o4Dpt232Shift 8

// Kanal verwenden?
#define ParamDFA_aActive                             ((knx.paramByte(DFA_ParamCalcIndex(DFA_aActive)) & DFA_aActiveMask) >> DFA_aActiveShift)
// Pausieren erlauben
#define ParamDFA_aStartPause                         ((knx.paramByte(DFA_ParamCalcIndex(DFA_aStartPause)) & DFA_aStartPauseMask) >> DFA_aStartPauseShift)
// Rekonstruktion bei erneutem Start
#define ParamDFA_aStateRestore                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aStateRestore)) & DFA_aStateRestoreMask) >> DFA_aStateRestoreShift)
// Direktes Setzen von Zustand erlauben?
#define ParamDFA_aStateSetting                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aStateSetting)) & DFA_aStateSettingMask) >> DFA_aStateSettingShift)
// Erneutes Setzen von aktuellem Zustand
#define ParamDFA_aStateSettingSame                   ((knx.paramByte(DFA_ParamCalcIndex(DFA_aStateSettingSame)) & DFA_aStateSettingSameMask) >> DFA_aStateSettingSameShift)
// Startzustand
#define ParamDFA_az0                                 (knx.paramByte(DFA_ParamCalcIndex(DFA_az0)))
// Einschaltverzögerung Zeitbasis
#define ParamDFA_aStartupDelayBase                   ((knx.paramByte(DFA_ParamCalcIndex(DFA_aStartupDelayBase)) & DFA_aStartupDelayBaseMask) >> DFA_aStartupDelayBaseShift)
// Einschaltverzögerung Zeit
#define ParamDFA_aStartupDelayTime                   (knx.paramWord(DFA_ParamCalcIndex(DFA_aStartupDelayTime)) & DFA_aStartupDelayTimeMask)
// Einschaltverzögerung Zeit (in Millisekunden)
#define ParamDFA_aStartupDelayTimeMS                 (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_aStartupDelayTime))))
// Kombination A/B
#define ParamDFA_aSymbolPairAB                       ((bool)(knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolPairAB)) & DFA_aSymbolPairABMask))
// Kombination C/D
#define ParamDFA_aSymbolPairCD                       ((bool)(knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolPairCD)) & DFA_aSymbolPairCDMask))
// Kombination E/F
#define ParamDFA_aSymbolPairEF                       ((bool)(knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolPairEF)) & DFA_aSymbolPairEFMask))
// Kombination G/H
#define ParamDFA_aSymbolPairGH                       ((bool)(knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolPairGH)) & DFA_aSymbolPairGHMask))
// Datentyp Ausgabe 1
#define ParamDFA_aOutput1Dpt                         (knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput1Dpt)))
// Sendeintervall Zeitbasis
#define ParamDFA_aOutput1IntervalBase                ((knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput1IntervalBase)) & DFA_aOutput1IntervalBaseMask) >> DFA_aOutput1IntervalBaseShift)
// Sendeintervall Zeit
#define ParamDFA_aOutput1IntervalTime                (knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput1IntervalTime)) & DFA_aOutput1IntervalTimeMask)
// Sendeintervall Zeit (in Millisekunden)
#define ParamDFA_aOutput1IntervalTimeMS              (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput1IntervalTime))))
// Datentyp Ausgabe 2
#define ParamDFA_aOutput2Dpt                         (knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput2Dpt)))
// Sendeintervall Zeitbasis
#define ParamDFA_aOutput2IntervalBase                ((knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput2IntervalBase)) & DFA_aOutput2IntervalBaseMask) >> DFA_aOutput2IntervalBaseShift)
// Sendeintervall Zeit
#define ParamDFA_aOutput2IntervalTime                (knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput2IntervalTime)) & DFA_aOutput2IntervalTimeMask)
// Sendeintervall Zeit (in Millisekunden)
#define ParamDFA_aOutput2IntervalTimeMS              (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput2IntervalTime))))
// Datentyp Ausgabe 3
#define ParamDFA_aOutput3Dpt                         (knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput3Dpt)))
// Sendeintervall Zeitbasis
#define ParamDFA_aOutput3IntervalBase                ((knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput3IntervalBase)) & DFA_aOutput3IntervalBaseMask) >> DFA_aOutput3IntervalBaseShift)
// Sendeintervall Zeit
#define ParamDFA_aOutput3IntervalTime                (knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput3IntervalTime)) & DFA_aOutput3IntervalTimeMask)
// Sendeintervall Zeit (in Millisekunden)
#define ParamDFA_aOutput3IntervalTimeMS              (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput3IntervalTime))))
// Datentyp Ausgabe 4
#define ParamDFA_aOutput4Dpt                         (knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput4Dpt)))
// Sendeintervall Zeitbasis
#define ParamDFA_aOutput4IntervalBase                ((knx.paramByte(DFA_ParamCalcIndex(DFA_aOutput4IntervalBase)) & DFA_aOutput4IntervalBaseMask) >> DFA_aOutput4IntervalBaseShift)
// Sendeintervall Zeit
#define ParamDFA_aOutput4IntervalTime                (knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput4IntervalTime)) & DFA_aOutput4IntervalTimeMask)
// Sendeintervall Zeit (in Millisekunden)
#define ParamDFA_aOutput4IntervalTimeMS              (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_aOutput4IntervalTime))))
// KO Eingabe 1
#define ParamDFA_aSymbolAInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolAInput)) & DFA_aSymbolAInputMask) >> DFA_aSymbolAInputShift)
// KO-Nummer Eingabe 1
#define ParamDFA_aSymbolAKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolAKoNumber)) & DFA_aSymbolAKoNumberMask) >> DFA_aSymbolAKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 1
#define ParamDFA_aSymbolALogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolALogicNumber)))
// Eingabewert 1
#define ParamDFA_aSymbolATrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolATrigger)) & DFA_aSymbolATriggerMask) >> DFA_aSymbolATriggerShift)
// KO Eingabe 2
#define ParamDFA_aSymbolBInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolBInput)) & DFA_aSymbolBInputMask) >> DFA_aSymbolBInputShift)
// KO-Nummer Eingabe 2
#define ParamDFA_aSymbolBKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolBKoNumber)) & DFA_aSymbolBKoNumberMask) >> DFA_aSymbolBKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 2
#define ParamDFA_aSymbolBLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolBLogicNumber)))
// Eingabewert 2
#define ParamDFA_aSymbolBTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolBTrigger)) & DFA_aSymbolBTriggerMask) >> DFA_aSymbolBTriggerShift)
// KO Eingabe 3
#define ParamDFA_aSymbolCInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolCInput)) & DFA_aSymbolCInputMask) >> DFA_aSymbolCInputShift)
// KO-Nummer Eingabe 3
#define ParamDFA_aSymbolCKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolCKoNumber)) & DFA_aSymbolCKoNumberMask) >> DFA_aSymbolCKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 3
#define ParamDFA_aSymbolCLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolCLogicNumber)))
// Eingabewert 3
#define ParamDFA_aSymbolCTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolCTrigger)) & DFA_aSymbolCTriggerMask) >> DFA_aSymbolCTriggerShift)
// KO Eingabe 4
#define ParamDFA_aSymbolDInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolDInput)) & DFA_aSymbolDInputMask) >> DFA_aSymbolDInputShift)
// KO-Nummer Eingabe 4
#define ParamDFA_aSymbolDKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolDKoNumber)) & DFA_aSymbolDKoNumberMask) >> DFA_aSymbolDKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 4
#define ParamDFA_aSymbolDLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolDLogicNumber)))
// Eingabewert 4
#define ParamDFA_aSymbolDTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolDTrigger)) & DFA_aSymbolDTriggerMask) >> DFA_aSymbolDTriggerShift)
// KO Eingabe 5
#define ParamDFA_aSymbolEInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolEInput)) & DFA_aSymbolEInputMask) >> DFA_aSymbolEInputShift)
// KO-Nummer Eingabe 5
#define ParamDFA_aSymbolEKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolEKoNumber)) & DFA_aSymbolEKoNumberMask) >> DFA_aSymbolEKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 5
#define ParamDFA_aSymbolELogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolELogicNumber)))
// Eingabewert 5
#define ParamDFA_aSymbolETrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolETrigger)) & DFA_aSymbolETriggerMask) >> DFA_aSymbolETriggerShift)
// KO Eingabe 6
#define ParamDFA_aSymbolFInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolFInput)) & DFA_aSymbolFInputMask) >> DFA_aSymbolFInputShift)
// KO-Nummer Eingabe 6
#define ParamDFA_aSymbolFKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolFKoNumber)) & DFA_aSymbolFKoNumberMask) >> DFA_aSymbolFKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 6
#define ParamDFA_aSymbolFLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolFLogicNumber)))
// Eingabewert 6
#define ParamDFA_aSymbolFTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolFTrigger)) & DFA_aSymbolFTriggerMask) >> DFA_aSymbolFTriggerShift)
// KO Eingabe 7
#define ParamDFA_aSymbolGInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolGInput)) & DFA_aSymbolGInputMask) >> DFA_aSymbolGInputShift)
// KO-Nummer Eingabe 7
#define ParamDFA_aSymbolGKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolGKoNumber)) & DFA_aSymbolGKoNumberMask) >> DFA_aSymbolGKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 7
#define ParamDFA_aSymbolGLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolGLogicNumber)))
// Eingabewert 7
#define ParamDFA_aSymbolGTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolGTrigger)) & DFA_aSymbolGTriggerMask) >> DFA_aSymbolGTriggerShift)
// KO Eingabe 8
#define ParamDFA_aSymbolHInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolHInput)) & DFA_aSymbolHInputMask) >> DFA_aSymbolHInputShift)
// KO-Nummer Eingabe 8
#define ParamDFA_aSymbolHKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolHKoNumber)) & DFA_aSymbolHKoNumberMask) >> DFA_aSymbolHKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer Eingabe 8
#define ParamDFA_aSymbolHLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolHLogicNumber)))
// Eingabewert 8
#define ParamDFA_aSymbolHTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolHTrigger)) & DFA_aSymbolHTriggerMask) >> DFA_aSymbolHTriggerShift)
// Direktes Auslösen von Timeout (Symbol T)
#define ParamDFA_aSymbolTInput                       ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolTInput)) & DFA_aSymbolTInputMask) >> DFA_aSymbolTInputShift)
// KO-Nummer für Symbol T
#define ParamDFA_aSymbolTKoNumber                    ((knx.paramWord(DFA_ParamCalcIndex(DFA_aSymbolTKoNumber)) & DFA_aSymbolTKoNumberMask) >> DFA_aSymbolTKoNumberShift)
// Logik-Kanal-Ausgangs-Nummer für Symbol T
#define ParamDFA_aSymbolTLogicNumber                 (knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolTLogicNumber)))
// Eingabewert zum Auslösen von Timeout
#define ParamDFA_aSymbolTTrigger                     ((knx.paramByte(DFA_ParamCalcIndex(DFA_aSymbolTTrigger)) & DFA_aSymbolTTriggerMask) >> DFA_aSymbolTTriggerShift)
// Bedingungsauswertung 1 - Logikkanal
#define ParamDFA_aCaLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCaLOG)))
// Bedingungsauswertung 1 - Folgezustand bei 1
#define ParamDFA_aCaT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCaT)))
// Bedingungsauswertung 1 - Folgezustand bei 0
#define ParamDFA_aCaF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCaF)))
// Bedingungsauswertung 1 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCaU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCaU)))
// Bedingungsauswertung 2 - Logikkanal
#define ParamDFA_aCbLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCbLOG)))
// Bedingungsauswertung 2 - Folgezustand bei 1
#define ParamDFA_aCbT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCbT)))
// Bedingungsauswertung 2 - Folgezustand bei 0
#define ParamDFA_aCbF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCbF)))
// Bedingungsauswertung 2 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCbU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCbU)))
// Bedingungsauswertung 3 - Logikkanal
#define ParamDFA_aCcLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCcLOG)))
// Bedingungsauswertung 3 - Folgezustand bei 1
#define ParamDFA_aCcT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCcT)))
// Bedingungsauswertung 3 - Folgezustand bei 0
#define ParamDFA_aCcF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCcF)))
// Bedingungsauswertung 3 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCcU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCcU)))
// Bedingungsauswertung 4 - Logikkanal
#define ParamDFA_aCdLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCdLOG)))
// Bedingungsauswertung 4 - Folgezustand bei 1
#define ParamDFA_aCdT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCdT)))
// Bedingungsauswertung 4 - Folgezustand bei 0
#define ParamDFA_aCdF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCdF)))
// Bedingungsauswertung 4 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCdU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCdU)))
// Bedingungsauswertung 5 - Logikkanal
#define ParamDFA_aCeLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCeLOG)))
// Bedingungsauswertung 5 - Folgezustand bei 1
#define ParamDFA_aCeT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCeT)))
// Bedingungsauswertung 5 - Folgezustand bei 0
#define ParamDFA_aCeF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCeF)))
// Bedingungsauswertung 5 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCeU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCeU)))
// Bedingungsauswertung 6 - Logikkanal
#define ParamDFA_aCfLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCfLOG)))
// Bedingungsauswertung 6 - Folgezustand bei 1
#define ParamDFA_aCfT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCfT)))
// Bedingungsauswertung 6 - Folgezustand bei 0
#define ParamDFA_aCfF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCfF)))
// Bedingungsauswertung 6 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCfU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCfU)))
// Bedingungsauswertung 7 - Logikkanal
#define ParamDFA_aCgLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCgLOG)))
// Bedingungsauswertung 7 - Folgezustand bei 1
#define ParamDFA_aCgT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCgT)))
// Bedingungsauswertung 7 - Folgezustand bei 0
#define ParamDFA_aCgF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCgF)))
// Bedingungsauswertung 7 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCgU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCgU)))
// Bedingungsauswertung 8 - Logikkanal
#define ParamDFA_aChLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aChLOG)))
// Bedingungsauswertung 8 - Folgezustand bei 1
#define ParamDFA_aChT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aChT)))
// Bedingungsauswertung 8 - Folgezustand bei 0
#define ParamDFA_aChF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aChF)))
// Bedingungsauswertung 8 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aChU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aChU)))
// Bedingungsauswertung 9 - Logikkanal
#define ParamDFA_aCiLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCiLOG)))
// Bedingungsauswertung 9 - Folgezustand bei 1
#define ParamDFA_aCiT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCiT)))
// Bedingungsauswertung 9 - Folgezustand bei 0
#define ParamDFA_aCiF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCiF)))
// Bedingungsauswertung 9 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCiU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCiU)))
// Bedingungsauswertung 10 - Logikkanal
#define ParamDFA_aCjLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCjLOG)))
// Bedingungsauswertung 10 - Folgezustand bei 1
#define ParamDFA_aCjT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCjT)))
// Bedingungsauswertung 10 - Folgezustand bei 0
#define ParamDFA_aCjF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCjF)))
// Bedingungsauswertung 10 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCjU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCjU)))
// Bedingungsauswertung 11 - Logikkanal
#define ParamDFA_aCkLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCkLOG)))
// Bedingungsauswertung 11 - Folgezustand bei 1
#define ParamDFA_aCkT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCkT)))
// Bedingungsauswertung 11 - Folgezustand bei 0
#define ParamDFA_aCkF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCkF)))
// Bedingungsauswertung 11 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCkU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCkU)))
// Bedingungsauswertung 12 - Logikkanal
#define ParamDFA_aClLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aClLOG)))
// Bedingungsauswertung 12 - Folgezustand bei 1
#define ParamDFA_aClT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aClT)))
// Bedingungsauswertung 12 - Folgezustand bei 0
#define ParamDFA_aClF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aClF)))
// Bedingungsauswertung 12 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aClU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aClU)))
// Bedingungsauswertung 13 - Logikkanal
#define ParamDFA_aCmLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCmLOG)))
// Bedingungsauswertung 13 - Folgezustand bei 1
#define ParamDFA_aCmT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCmT)))
// Bedingungsauswertung 13 - Folgezustand bei 0
#define ParamDFA_aCmF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCmF)))
// Bedingungsauswertung 13 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCmU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCmU)))
// Bedingungsauswertung 14 - Logikkanal
#define ParamDFA_aCnLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCnLOG)))
// Bedingungsauswertung 14 - Folgezustand bei 1
#define ParamDFA_aCnT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCnT)))
// Bedingungsauswertung 14 - Folgezustand bei 0
#define ParamDFA_aCnF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCnF)))
// Bedingungsauswertung 14 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCnU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCnU)))
// Bedingungsauswertung 15 - Logikkanal
#define ParamDFA_aCoLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCoLOG)))
// Bedingungsauswertung 15 - Folgezustand bei 1
#define ParamDFA_aCoT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCoT)))
// Bedingungsauswertung 15 - Folgezustand bei 0
#define ParamDFA_aCoF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCoF)))
// Bedingungsauswertung 15 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCoU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCoU)))
// Bedingungsauswertung 16 - Logikkanal
#define ParamDFA_aCpLOG                              (knx.paramByte(DFA_ParamCalcIndex(DFA_aCpLOG)))
// Bedingungsauswertung 16 - Folgezustand bei 1
#define ParamDFA_aCpT                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCpT)))
// Bedingungsauswertung 16 - Folgezustand bei 0
#define ParamDFA_aCpF                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCpF)))
// Bedingungsauswertung 16 - Folgezustand bei UNDEFINIERT
#define ParamDFA_aCpU                                (knx.paramByte(DFA_ParamCalcIndex(DFA_aCpU)))
// trans(01,1)
#define ParamDFA_ad01A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01A)))
// trans(01,2)
#define ParamDFA_ad01B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01B)))
// trans(01,3)
#define ParamDFA_ad01C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01C)))
// trans(01,4)
#define ParamDFA_ad01D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01D)))
// trans(01,5)
#define ParamDFA_ad01E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01E)))
// trans(01,6)
#define ParamDFA_ad01F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01F)))
// trans(01,7)
#define ParamDFA_ad01G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01G)))
// trans(01,8)
#define ParamDFA_ad01H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01H)))
// trans(01,timeout)
#define ParamDFA_ad01T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad01T)))
// Zeitbasis
#define ParamDFA_ad01TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad01TBase)) & DFA_ad01TBaseMask) >> DFA_ad01TBaseShift)
// Zeit
#define ParamDFA_ad01TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad01TTime)) & DFA_ad01TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad01TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad01TTime))))
// Sendeverhalten Zustand 1
#define ParamDFA_az01o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Send)))
// Ausgabewert Zustand 1 (DPT 1)
#define ParamDFA_az01o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Dpt1)))
// Ausgabewert Zustand 1 (DPT 2)
#define ParamDFA_az01o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Dpt2)))
// Ausgabewert Zustand 1 (DPT 5)
#define ParamDFA_az01o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Dpt5)))
// Ausgabewert Zustand 1 (DPT 5.001)
#define ParamDFA_az01o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Dpt5001)))
// Ausgabewert Zustand 1 (DPT 6)
#define ParamDFA_az01o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Dpt6)))
// Ausgabewert Zustand 1 (DPT 7)
#define ParamDFA_az01o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az01o1Dpt7)))
// Ausgabewert Zustand 1 (DPT 8)
#define ParamDFA_az01o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az01o1Dpt8)))
// Ausgabewert Zustand 1 (DPT 9)
#define ParamDFA_az01o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 12)
#define ParamDFA_az01o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az01o1Dpt12)))
// Ausgabewert Zustand 1 (DPT 13)
#define ParamDFA_az01o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az01o1Dpt13)))
// Ausgabewert Zustand 1 (DPT 14)
#define ParamDFA_az01o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 17)
#define ParamDFA_az01o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o1Dpt17)))
// Ausgabewert Zustand 1 (DPT 232)
#define ParamDFA_az01o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az01o1Dpt232)) & DFA_az01o1Dpt232Mask) >> DFA_az01o1Dpt232Shift)
// Sendeverhalten Zustand 1
#define ParamDFA_az01o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Send)))
// Ausgabewert Zustand 1 (DPT 1)
#define ParamDFA_az01o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Dpt1)))
// Ausgabewert Zustand 1 (DPT 2)
#define ParamDFA_az01o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Dpt2)))
// Ausgabewert Zustand 1 (DPT 5)
#define ParamDFA_az01o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Dpt5)))
// Ausgabewert Zustand 1 (DPT 5.001)
#define ParamDFA_az01o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Dpt5001)))
// Ausgabewert Zustand 1 (DPT 6)
#define ParamDFA_az01o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Dpt6)))
// Ausgabewert Zustand 1 (DPT 7)
#define ParamDFA_az01o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az01o2Dpt7)))
// Ausgabewert Zustand 1 (DPT 8)
#define ParamDFA_az01o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az01o2Dpt8)))
// Ausgabewert Zustand 1 (DPT 9)
#define ParamDFA_az01o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 12)
#define ParamDFA_az01o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az01o2Dpt12)))
// Ausgabewert Zustand 1 (DPT 13)
#define ParamDFA_az01o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az01o2Dpt13)))
// Ausgabewert Zustand 1 (DPT 14)
#define ParamDFA_az01o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 17)
#define ParamDFA_az01o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o2Dpt17)))
// Ausgabewert Zustand 1 (DPT 232)
#define ParamDFA_az01o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az01o2Dpt232)) & DFA_az01o2Dpt232Mask) >> DFA_az01o2Dpt232Shift)
// Sendeverhalten Zustand 1
#define ParamDFA_az01o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Send)))
// Ausgabewert Zustand 1 (DPT 1)
#define ParamDFA_az01o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Dpt1)))
// Ausgabewert Zustand 1 (DPT 2)
#define ParamDFA_az01o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Dpt2)))
// Ausgabewert Zustand 1 (DPT 5)
#define ParamDFA_az01o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Dpt5)))
// Ausgabewert Zustand 1 (DPT 5.001)
#define ParamDFA_az01o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Dpt5001)))
// Ausgabewert Zustand 1 (DPT 6)
#define ParamDFA_az01o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Dpt6)))
// Ausgabewert Zustand 1 (DPT 7)
#define ParamDFA_az01o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az01o3Dpt7)))
// Ausgabewert Zustand 1 (DPT 8)
#define ParamDFA_az01o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az01o3Dpt8)))
// Ausgabewert Zustand 1 (DPT 9)
#define ParamDFA_az01o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 12)
#define ParamDFA_az01o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az01o3Dpt12)))
// Ausgabewert Zustand 1 (DPT 13)
#define ParamDFA_az01o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az01o3Dpt13)))
// Ausgabewert Zustand 1 (DPT 14)
#define ParamDFA_az01o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 17)
#define ParamDFA_az01o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o3Dpt17)))
// Ausgabewert Zustand 1 (DPT 232)
#define ParamDFA_az01o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az01o3Dpt232)) & DFA_az01o3Dpt232Mask) >> DFA_az01o3Dpt232Shift)
// Sendeverhalten Zustand 1
#define ParamDFA_az01o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Send)))
// Ausgabewert Zustand 1 (DPT 1)
#define ParamDFA_az01o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Dpt1)))
// Ausgabewert Zustand 1 (DPT 2)
#define ParamDFA_az01o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Dpt2)))
// Ausgabewert Zustand 1 (DPT 5)
#define ParamDFA_az01o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Dpt5)))
// Ausgabewert Zustand 1 (DPT 5.001)
#define ParamDFA_az01o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Dpt5001)))
// Ausgabewert Zustand 1 (DPT 6)
#define ParamDFA_az01o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Dpt6)))
// Ausgabewert Zustand 1 (DPT 7)
#define ParamDFA_az01o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az01o4Dpt7)))
// Ausgabewert Zustand 1 (DPT 8)
#define ParamDFA_az01o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az01o4Dpt8)))
// Ausgabewert Zustand 1 (DPT 9)
#define ParamDFA_az01o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 12)
#define ParamDFA_az01o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az01o4Dpt12)))
// Ausgabewert Zustand 1 (DPT 13)
#define ParamDFA_az01o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az01o4Dpt13)))
// Ausgabewert Zustand 1 (DPT 14)
#define ParamDFA_az01o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az01o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 1 (DPT 16)
#define ParamDFA_az01o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az01o4Dpt16)))
// Ausgabewert Zustand 1 (DPT 17)
#define ParamDFA_az01o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az01o4Dpt17)))
// Ausgabewert Zustand 1 (DPT 232)
#define ParamDFA_az01o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az01o4Dpt232)) & DFA_az01o4Dpt232Mask) >> DFA_az01o4Dpt232Shift)
// trans(02,1)
#define ParamDFA_ad02A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02A)))
// trans(02,2)
#define ParamDFA_ad02B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02B)))
// trans(02,3)
#define ParamDFA_ad02C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02C)))
// trans(02,4)
#define ParamDFA_ad02D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02D)))
// trans(02,5)
#define ParamDFA_ad02E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02E)))
// trans(02,6)
#define ParamDFA_ad02F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02F)))
// trans(02,7)
#define ParamDFA_ad02G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02G)))
// trans(02,8)
#define ParamDFA_ad02H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02H)))
// trans(02,timeout)
#define ParamDFA_ad02T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad02T)))
// Zeitbasis
#define ParamDFA_ad02TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad02TBase)) & DFA_ad02TBaseMask) >> DFA_ad02TBaseShift)
// Zeit
#define ParamDFA_ad02TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad02TTime)) & DFA_ad02TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad02TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad02TTime))))
// Sendeverhalten Zustand 2
#define ParamDFA_az02o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Send)))
// Ausgabewert Zustand 2 (DPT 1)
#define ParamDFA_az02o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Dpt1)))
// Ausgabewert Zustand 2 (DPT 2)
#define ParamDFA_az02o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Dpt2)))
// Ausgabewert Zustand 2 (DPT 5)
#define ParamDFA_az02o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Dpt5)))
// Ausgabewert Zustand 2 (DPT 5.001)
#define ParamDFA_az02o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Dpt5001)))
// Ausgabewert Zustand 2 (DPT 6)
#define ParamDFA_az02o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Dpt6)))
// Ausgabewert Zustand 2 (DPT 7)
#define ParamDFA_az02o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az02o1Dpt7)))
// Ausgabewert Zustand 2 (DPT 8)
#define ParamDFA_az02o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az02o1Dpt8)))
// Ausgabewert Zustand 2 (DPT 9)
#define ParamDFA_az02o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 12)
#define ParamDFA_az02o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az02o1Dpt12)))
// Ausgabewert Zustand 2 (DPT 13)
#define ParamDFA_az02o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az02o1Dpt13)))
// Ausgabewert Zustand 2 (DPT 14)
#define ParamDFA_az02o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 17)
#define ParamDFA_az02o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o1Dpt17)))
// Ausgabewert Zustand 2 (DPT 232)
#define ParamDFA_az02o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az02o1Dpt232)) & DFA_az02o1Dpt232Mask) >> DFA_az02o1Dpt232Shift)
// Sendeverhalten Zustand 2
#define ParamDFA_az02o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Send)))
// Ausgabewert Zustand 2 (DPT 1)
#define ParamDFA_az02o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Dpt1)))
// Ausgabewert Zustand 2 (DPT 2)
#define ParamDFA_az02o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Dpt2)))
// Ausgabewert Zustand 2 (DPT 5)
#define ParamDFA_az02o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Dpt5)))
// Ausgabewert Zustand 2 (DPT 5.001)
#define ParamDFA_az02o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Dpt5001)))
// Ausgabewert Zustand 2 (DPT 6)
#define ParamDFA_az02o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Dpt6)))
// Ausgabewert Zustand 2 (DPT 7)
#define ParamDFA_az02o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az02o2Dpt7)))
// Ausgabewert Zustand 2 (DPT 8)
#define ParamDFA_az02o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az02o2Dpt8)))
// Ausgabewert Zustand 2 (DPT 9)
#define ParamDFA_az02o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 12)
#define ParamDFA_az02o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az02o2Dpt12)))
// Ausgabewert Zustand 2 (DPT 13)
#define ParamDFA_az02o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az02o2Dpt13)))
// Ausgabewert Zustand 2 (DPT 14)
#define ParamDFA_az02o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 17)
#define ParamDFA_az02o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o2Dpt17)))
// Ausgabewert Zustand 2 (DPT 232)
#define ParamDFA_az02o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az02o2Dpt232)) & DFA_az02o2Dpt232Mask) >> DFA_az02o2Dpt232Shift)
// Sendeverhalten Zustand 2
#define ParamDFA_az02o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Send)))
// Ausgabewert Zustand 2 (DPT 1)
#define ParamDFA_az02o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Dpt1)))
// Ausgabewert Zustand 2 (DPT 2)
#define ParamDFA_az02o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Dpt2)))
// Ausgabewert Zustand 2 (DPT 5)
#define ParamDFA_az02o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Dpt5)))
// Ausgabewert Zustand 2 (DPT 5.001)
#define ParamDFA_az02o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Dpt5001)))
// Ausgabewert Zustand 2 (DPT 6)
#define ParamDFA_az02o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Dpt6)))
// Ausgabewert Zustand 2 (DPT 7)
#define ParamDFA_az02o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az02o3Dpt7)))
// Ausgabewert Zustand 2 (DPT 8)
#define ParamDFA_az02o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az02o3Dpt8)))
// Ausgabewert Zustand 2 (DPT 9)
#define ParamDFA_az02o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 12)
#define ParamDFA_az02o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az02o3Dpt12)))
// Ausgabewert Zustand 2 (DPT 13)
#define ParamDFA_az02o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az02o3Dpt13)))
// Ausgabewert Zustand 2 (DPT 14)
#define ParamDFA_az02o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 17)
#define ParamDFA_az02o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o3Dpt17)))
// Ausgabewert Zustand 2 (DPT 232)
#define ParamDFA_az02o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az02o3Dpt232)) & DFA_az02o3Dpt232Mask) >> DFA_az02o3Dpt232Shift)
// Sendeverhalten Zustand 2
#define ParamDFA_az02o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Send)))
// Ausgabewert Zustand 2 (DPT 1)
#define ParamDFA_az02o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Dpt1)))
// Ausgabewert Zustand 2 (DPT 2)
#define ParamDFA_az02o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Dpt2)))
// Ausgabewert Zustand 2 (DPT 5)
#define ParamDFA_az02o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Dpt5)))
// Ausgabewert Zustand 2 (DPT 5.001)
#define ParamDFA_az02o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Dpt5001)))
// Ausgabewert Zustand 2 (DPT 6)
#define ParamDFA_az02o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Dpt6)))
// Ausgabewert Zustand 2 (DPT 7)
#define ParamDFA_az02o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az02o4Dpt7)))
// Ausgabewert Zustand 2 (DPT 8)
#define ParamDFA_az02o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az02o4Dpt8)))
// Ausgabewert Zustand 2 (DPT 9)
#define ParamDFA_az02o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 12)
#define ParamDFA_az02o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az02o4Dpt12)))
// Ausgabewert Zustand 2 (DPT 13)
#define ParamDFA_az02o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az02o4Dpt13)))
// Ausgabewert Zustand 2 (DPT 14)
#define ParamDFA_az02o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az02o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 2 (DPT 16)
#define ParamDFA_az02o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az02o4Dpt16)))
// Ausgabewert Zustand 2 (DPT 17)
#define ParamDFA_az02o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az02o4Dpt17)))
// Ausgabewert Zustand 2 (DPT 232)
#define ParamDFA_az02o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az02o4Dpt232)) & DFA_az02o4Dpt232Mask) >> DFA_az02o4Dpt232Shift)
// trans(03,1)
#define ParamDFA_ad03A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03A)))
// trans(03,2)
#define ParamDFA_ad03B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03B)))
// trans(03,3)
#define ParamDFA_ad03C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03C)))
// trans(03,4)
#define ParamDFA_ad03D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03D)))
// trans(03,5)
#define ParamDFA_ad03E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03E)))
// trans(03,6)
#define ParamDFA_ad03F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03F)))
// trans(03,7)
#define ParamDFA_ad03G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03G)))
// trans(03,8)
#define ParamDFA_ad03H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03H)))
// trans(03,timeout)
#define ParamDFA_ad03T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad03T)))
// Zeitbasis
#define ParamDFA_ad03TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad03TBase)) & DFA_ad03TBaseMask) >> DFA_ad03TBaseShift)
// Zeit
#define ParamDFA_ad03TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad03TTime)) & DFA_ad03TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad03TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad03TTime))))
// Sendeverhalten Zustand 3
#define ParamDFA_az03o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Send)))
// Ausgabewert Zustand 3 (DPT 1)
#define ParamDFA_az03o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Dpt1)))
// Ausgabewert Zustand 3 (DPT 2)
#define ParamDFA_az03o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Dpt2)))
// Ausgabewert Zustand 3 (DPT 5)
#define ParamDFA_az03o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Dpt5)))
// Ausgabewert Zustand 3 (DPT 5.001)
#define ParamDFA_az03o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Dpt5001)))
// Ausgabewert Zustand 3 (DPT 6)
#define ParamDFA_az03o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Dpt6)))
// Ausgabewert Zustand 3 (DPT 7)
#define ParamDFA_az03o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az03o1Dpt7)))
// Ausgabewert Zustand 3 (DPT 8)
#define ParamDFA_az03o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az03o1Dpt8)))
// Ausgabewert Zustand 3 (DPT 9)
#define ParamDFA_az03o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 12)
#define ParamDFA_az03o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az03o1Dpt12)))
// Ausgabewert Zustand 3 (DPT 13)
#define ParamDFA_az03o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az03o1Dpt13)))
// Ausgabewert Zustand 3 (DPT 14)
#define ParamDFA_az03o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 17)
#define ParamDFA_az03o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o1Dpt17)))
// Ausgabewert Zustand 3 (DPT 232)
#define ParamDFA_az03o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az03o1Dpt232)) & DFA_az03o1Dpt232Mask) >> DFA_az03o1Dpt232Shift)
// Sendeverhalten Zustand 3
#define ParamDFA_az03o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Send)))
// Ausgabewert Zustand 3 (DPT 1)
#define ParamDFA_az03o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Dpt1)))
// Ausgabewert Zustand 3 (DPT 2)
#define ParamDFA_az03o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Dpt2)))
// Ausgabewert Zustand 3 (DPT 5)
#define ParamDFA_az03o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Dpt5)))
// Ausgabewert Zustand 3 (DPT 5.001)
#define ParamDFA_az03o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Dpt5001)))
// Ausgabewert Zustand 3 (DPT 6)
#define ParamDFA_az03o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Dpt6)))
// Ausgabewert Zustand 3 (DPT 7)
#define ParamDFA_az03o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az03o2Dpt7)))
// Ausgabewert Zustand 3 (DPT 8)
#define ParamDFA_az03o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az03o2Dpt8)))
// Ausgabewert Zustand 3 (DPT 9)
#define ParamDFA_az03o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 12)
#define ParamDFA_az03o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az03o2Dpt12)))
// Ausgabewert Zustand 3 (DPT 13)
#define ParamDFA_az03o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az03o2Dpt13)))
// Ausgabewert Zustand 3 (DPT 14)
#define ParamDFA_az03o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 17)
#define ParamDFA_az03o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o2Dpt17)))
// Ausgabewert Zustand 3 (DPT 232)
#define ParamDFA_az03o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az03o2Dpt232)) & DFA_az03o2Dpt232Mask) >> DFA_az03o2Dpt232Shift)
// Sendeverhalten Zustand 3
#define ParamDFA_az03o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Send)))
// Ausgabewert Zustand 3 (DPT 1)
#define ParamDFA_az03o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Dpt1)))
// Ausgabewert Zustand 3 (DPT 2)
#define ParamDFA_az03o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Dpt2)))
// Ausgabewert Zustand 3 (DPT 5)
#define ParamDFA_az03o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Dpt5)))
// Ausgabewert Zustand 3 (DPT 5.001)
#define ParamDFA_az03o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Dpt5001)))
// Ausgabewert Zustand 3 (DPT 6)
#define ParamDFA_az03o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Dpt6)))
// Ausgabewert Zustand 3 (DPT 7)
#define ParamDFA_az03o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az03o3Dpt7)))
// Ausgabewert Zustand 3 (DPT 8)
#define ParamDFA_az03o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az03o3Dpt8)))
// Ausgabewert Zustand 3 (DPT 9)
#define ParamDFA_az03o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 12)
#define ParamDFA_az03o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az03o3Dpt12)))
// Ausgabewert Zustand 3 (DPT 13)
#define ParamDFA_az03o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az03o3Dpt13)))
// Ausgabewert Zustand 3 (DPT 14)
#define ParamDFA_az03o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 17)
#define ParamDFA_az03o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o3Dpt17)))
// Ausgabewert Zustand 3 (DPT 232)
#define ParamDFA_az03o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az03o3Dpt232)) & DFA_az03o3Dpt232Mask) >> DFA_az03o3Dpt232Shift)
// Sendeverhalten Zustand 3
#define ParamDFA_az03o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Send)))
// Ausgabewert Zustand 3 (DPT 1)
#define ParamDFA_az03o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Dpt1)))
// Ausgabewert Zustand 3 (DPT 2)
#define ParamDFA_az03o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Dpt2)))
// Ausgabewert Zustand 3 (DPT 5)
#define ParamDFA_az03o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Dpt5)))
// Ausgabewert Zustand 3 (DPT 5.001)
#define ParamDFA_az03o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Dpt5001)))
// Ausgabewert Zustand 3 (DPT 6)
#define ParamDFA_az03o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Dpt6)))
// Ausgabewert Zustand 3 (DPT 7)
#define ParamDFA_az03o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az03o4Dpt7)))
// Ausgabewert Zustand 3 (DPT 8)
#define ParamDFA_az03o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az03o4Dpt8)))
// Ausgabewert Zustand 3 (DPT 9)
#define ParamDFA_az03o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 12)
#define ParamDFA_az03o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az03o4Dpt12)))
// Ausgabewert Zustand 3 (DPT 13)
#define ParamDFA_az03o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az03o4Dpt13)))
// Ausgabewert Zustand 3 (DPT 14)
#define ParamDFA_az03o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az03o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 3 (DPT 16)
#define ParamDFA_az03o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az03o4Dpt16)))
// Ausgabewert Zustand 3 (DPT 17)
#define ParamDFA_az03o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az03o4Dpt17)))
// Ausgabewert Zustand 3 (DPT 232)
#define ParamDFA_az03o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az03o4Dpt232)) & DFA_az03o4Dpt232Mask) >> DFA_az03o4Dpt232Shift)
// trans(04,1)
#define ParamDFA_ad04A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04A)))
// trans(04,2)
#define ParamDFA_ad04B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04B)))
// trans(04,3)
#define ParamDFA_ad04C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04C)))
// trans(04,4)
#define ParamDFA_ad04D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04D)))
// trans(04,5)
#define ParamDFA_ad04E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04E)))
// trans(04,6)
#define ParamDFA_ad04F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04F)))
// trans(04,7)
#define ParamDFA_ad04G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04G)))
// trans(04,8)
#define ParamDFA_ad04H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04H)))
// trans(04,timeout)
#define ParamDFA_ad04T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad04T)))
// Zeitbasis
#define ParamDFA_ad04TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad04TBase)) & DFA_ad04TBaseMask) >> DFA_ad04TBaseShift)
// Zeit
#define ParamDFA_ad04TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad04TTime)) & DFA_ad04TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad04TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad04TTime))))
// Sendeverhalten Zustand 4
#define ParamDFA_az04o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Send)))
// Ausgabewert Zustand 4 (DPT 1)
#define ParamDFA_az04o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Dpt1)))
// Ausgabewert Zustand 4 (DPT 2)
#define ParamDFA_az04o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Dpt2)))
// Ausgabewert Zustand 4 (DPT 5)
#define ParamDFA_az04o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Dpt5)))
// Ausgabewert Zustand 4 (DPT 5.001)
#define ParamDFA_az04o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Dpt5001)))
// Ausgabewert Zustand 4 (DPT 6)
#define ParamDFA_az04o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Dpt6)))
// Ausgabewert Zustand 4 (DPT 7)
#define ParamDFA_az04o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az04o1Dpt7)))
// Ausgabewert Zustand 4 (DPT 8)
#define ParamDFA_az04o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az04o1Dpt8)))
// Ausgabewert Zustand 4 (DPT 9)
#define ParamDFA_az04o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 12)
#define ParamDFA_az04o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az04o1Dpt12)))
// Ausgabewert Zustand 4 (DPT 13)
#define ParamDFA_az04o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az04o1Dpt13)))
// Ausgabewert Zustand 4 (DPT 14)
#define ParamDFA_az04o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 17)
#define ParamDFA_az04o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o1Dpt17)))
// Ausgabewert Zustand 4 (DPT 232)
#define ParamDFA_az04o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az04o1Dpt232)) & DFA_az04o1Dpt232Mask) >> DFA_az04o1Dpt232Shift)
// Sendeverhalten Zustand 4
#define ParamDFA_az04o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Send)))
// Ausgabewert Zustand 4 (DPT 1)
#define ParamDFA_az04o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Dpt1)))
// Ausgabewert Zustand 4 (DPT 2)
#define ParamDFA_az04o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Dpt2)))
// Ausgabewert Zustand 4 (DPT 5)
#define ParamDFA_az04o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Dpt5)))
// Ausgabewert Zustand 4 (DPT 5.001)
#define ParamDFA_az04o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Dpt5001)))
// Ausgabewert Zustand 4 (DPT 6)
#define ParamDFA_az04o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Dpt6)))
// Ausgabewert Zustand 4 (DPT 7)
#define ParamDFA_az04o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az04o2Dpt7)))
// Ausgabewert Zustand 4 (DPT 8)
#define ParamDFA_az04o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az04o2Dpt8)))
// Ausgabewert Zustand 4 (DPT 9)
#define ParamDFA_az04o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 12)
#define ParamDFA_az04o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az04o2Dpt12)))
// Ausgabewert Zustand 4 (DPT 13)
#define ParamDFA_az04o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az04o2Dpt13)))
// Ausgabewert Zustand 4 (DPT 14)
#define ParamDFA_az04o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 17)
#define ParamDFA_az04o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o2Dpt17)))
// Ausgabewert Zustand 4 (DPT 232)
#define ParamDFA_az04o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az04o2Dpt232)) & DFA_az04o2Dpt232Mask) >> DFA_az04o2Dpt232Shift)
// Sendeverhalten Zustand 4
#define ParamDFA_az04o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Send)))
// Ausgabewert Zustand 4 (DPT 1)
#define ParamDFA_az04o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Dpt1)))
// Ausgabewert Zustand 4 (DPT 2)
#define ParamDFA_az04o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Dpt2)))
// Ausgabewert Zustand 4 (DPT 5)
#define ParamDFA_az04o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Dpt5)))
// Ausgabewert Zustand 4 (DPT 5.001)
#define ParamDFA_az04o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Dpt5001)))
// Ausgabewert Zustand 4 (DPT 6)
#define ParamDFA_az04o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Dpt6)))
// Ausgabewert Zustand 4 (DPT 7)
#define ParamDFA_az04o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az04o3Dpt7)))
// Ausgabewert Zustand 4 (DPT 8)
#define ParamDFA_az04o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az04o3Dpt8)))
// Ausgabewert Zustand 4 (DPT 9)
#define ParamDFA_az04o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 12)
#define ParamDFA_az04o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az04o3Dpt12)))
// Ausgabewert Zustand 4 (DPT 13)
#define ParamDFA_az04o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az04o3Dpt13)))
// Ausgabewert Zustand 4 (DPT 14)
#define ParamDFA_az04o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 17)
#define ParamDFA_az04o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o3Dpt17)))
// Ausgabewert Zustand 4 (DPT 232)
#define ParamDFA_az04o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az04o3Dpt232)) & DFA_az04o3Dpt232Mask) >> DFA_az04o3Dpt232Shift)
// Sendeverhalten Zustand 4
#define ParamDFA_az04o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Send)))
// Ausgabewert Zustand 4 (DPT 1)
#define ParamDFA_az04o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Dpt1)))
// Ausgabewert Zustand 4 (DPT 2)
#define ParamDFA_az04o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Dpt2)))
// Ausgabewert Zustand 4 (DPT 5)
#define ParamDFA_az04o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Dpt5)))
// Ausgabewert Zustand 4 (DPT 5.001)
#define ParamDFA_az04o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Dpt5001)))
// Ausgabewert Zustand 4 (DPT 6)
#define ParamDFA_az04o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Dpt6)))
// Ausgabewert Zustand 4 (DPT 7)
#define ParamDFA_az04o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az04o4Dpt7)))
// Ausgabewert Zustand 4 (DPT 8)
#define ParamDFA_az04o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az04o4Dpt8)))
// Ausgabewert Zustand 4 (DPT 9)
#define ParamDFA_az04o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 12)
#define ParamDFA_az04o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az04o4Dpt12)))
// Ausgabewert Zustand 4 (DPT 13)
#define ParamDFA_az04o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az04o4Dpt13)))
// Ausgabewert Zustand 4 (DPT 14)
#define ParamDFA_az04o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az04o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 4 (DPT 16)
#define ParamDFA_az04o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az04o4Dpt16)))
// Ausgabewert Zustand 4 (DPT 17)
#define ParamDFA_az04o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az04o4Dpt17)))
// Ausgabewert Zustand 4 (DPT 232)
#define ParamDFA_az04o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az04o4Dpt232)) & DFA_az04o4Dpt232Mask) >> DFA_az04o4Dpt232Shift)
// trans(05,1)
#define ParamDFA_ad05A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05A)))
// trans(05,2)
#define ParamDFA_ad05B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05B)))
// trans(05,3)
#define ParamDFA_ad05C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05C)))
// trans(05,4)
#define ParamDFA_ad05D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05D)))
// trans(05,5)
#define ParamDFA_ad05E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05E)))
// trans(05,6)
#define ParamDFA_ad05F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05F)))
// trans(05,7)
#define ParamDFA_ad05G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05G)))
// trans(05,8)
#define ParamDFA_ad05H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05H)))
// trans(05,timeout)
#define ParamDFA_ad05T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad05T)))
// Zeitbasis
#define ParamDFA_ad05TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad05TBase)) & DFA_ad05TBaseMask) >> DFA_ad05TBaseShift)
// Zeit
#define ParamDFA_ad05TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad05TTime)) & DFA_ad05TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad05TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad05TTime))))
// Sendeverhalten Zustand 5
#define ParamDFA_az05o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Send)))
// Ausgabewert Zustand 5 (DPT 1)
#define ParamDFA_az05o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Dpt1)))
// Ausgabewert Zustand 5 (DPT 2)
#define ParamDFA_az05o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Dpt2)))
// Ausgabewert Zustand 5 (DPT 5)
#define ParamDFA_az05o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Dpt5)))
// Ausgabewert Zustand 5 (DPT 5.001)
#define ParamDFA_az05o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Dpt5001)))
// Ausgabewert Zustand 5 (DPT 6)
#define ParamDFA_az05o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Dpt6)))
// Ausgabewert Zustand 5 (DPT 7)
#define ParamDFA_az05o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az05o1Dpt7)))
// Ausgabewert Zustand 5 (DPT 8)
#define ParamDFA_az05o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az05o1Dpt8)))
// Ausgabewert Zustand 5 (DPT 9)
#define ParamDFA_az05o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 12)
#define ParamDFA_az05o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az05o1Dpt12)))
// Ausgabewert Zustand 5 (DPT 13)
#define ParamDFA_az05o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az05o1Dpt13)))
// Ausgabewert Zustand 5 (DPT 14)
#define ParamDFA_az05o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 17)
#define ParamDFA_az05o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o1Dpt17)))
// Ausgabewert Zustand 5 (DPT 232)
#define ParamDFA_az05o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az05o1Dpt232)) & DFA_az05o1Dpt232Mask) >> DFA_az05o1Dpt232Shift)
// Sendeverhalten Zustand 5
#define ParamDFA_az05o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Send)))
// Ausgabewert Zustand 5 (DPT 1)
#define ParamDFA_az05o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Dpt1)))
// Ausgabewert Zustand 5 (DPT 2)
#define ParamDFA_az05o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Dpt2)))
// Ausgabewert Zustand 5 (DPT 5)
#define ParamDFA_az05o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Dpt5)))
// Ausgabewert Zustand 5 (DPT 5.001)
#define ParamDFA_az05o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Dpt5001)))
// Ausgabewert Zustand 5 (DPT 6)
#define ParamDFA_az05o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Dpt6)))
// Ausgabewert Zustand 5 (DPT 7)
#define ParamDFA_az05o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az05o2Dpt7)))
// Ausgabewert Zustand 5 (DPT 8)
#define ParamDFA_az05o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az05o2Dpt8)))
// Ausgabewert Zustand 5 (DPT 9)
#define ParamDFA_az05o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 12)
#define ParamDFA_az05o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az05o2Dpt12)))
// Ausgabewert Zustand 5 (DPT 13)
#define ParamDFA_az05o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az05o2Dpt13)))
// Ausgabewert Zustand 5 (DPT 14)
#define ParamDFA_az05o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 17)
#define ParamDFA_az05o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o2Dpt17)))
// Ausgabewert Zustand 5 (DPT 232)
#define ParamDFA_az05o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az05o2Dpt232)) & DFA_az05o2Dpt232Mask) >> DFA_az05o2Dpt232Shift)
// Sendeverhalten Zustand 5
#define ParamDFA_az05o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Send)))
// Ausgabewert Zustand 5 (DPT 1)
#define ParamDFA_az05o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Dpt1)))
// Ausgabewert Zustand 5 (DPT 2)
#define ParamDFA_az05o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Dpt2)))
// Ausgabewert Zustand 5 (DPT 5)
#define ParamDFA_az05o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Dpt5)))
// Ausgabewert Zustand 5 (DPT 5.001)
#define ParamDFA_az05o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Dpt5001)))
// Ausgabewert Zustand 5 (DPT 6)
#define ParamDFA_az05o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Dpt6)))
// Ausgabewert Zustand 5 (DPT 7)
#define ParamDFA_az05o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az05o3Dpt7)))
// Ausgabewert Zustand 5 (DPT 8)
#define ParamDFA_az05o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az05o3Dpt8)))
// Ausgabewert Zustand 5 (DPT 9)
#define ParamDFA_az05o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 12)
#define ParamDFA_az05o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az05o3Dpt12)))
// Ausgabewert Zustand 5 (DPT 13)
#define ParamDFA_az05o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az05o3Dpt13)))
// Ausgabewert Zustand 5 (DPT 14)
#define ParamDFA_az05o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 17)
#define ParamDFA_az05o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o3Dpt17)))
// Ausgabewert Zustand 5 (DPT 232)
#define ParamDFA_az05o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az05o3Dpt232)) & DFA_az05o3Dpt232Mask) >> DFA_az05o3Dpt232Shift)
// Sendeverhalten Zustand 5
#define ParamDFA_az05o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Send)))
// Ausgabewert Zustand 5 (DPT 1)
#define ParamDFA_az05o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Dpt1)))
// Ausgabewert Zustand 5 (DPT 2)
#define ParamDFA_az05o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Dpt2)))
// Ausgabewert Zustand 5 (DPT 5)
#define ParamDFA_az05o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Dpt5)))
// Ausgabewert Zustand 5 (DPT 5.001)
#define ParamDFA_az05o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Dpt5001)))
// Ausgabewert Zustand 5 (DPT 6)
#define ParamDFA_az05o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Dpt6)))
// Ausgabewert Zustand 5 (DPT 7)
#define ParamDFA_az05o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az05o4Dpt7)))
// Ausgabewert Zustand 5 (DPT 8)
#define ParamDFA_az05o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az05o4Dpt8)))
// Ausgabewert Zustand 5 (DPT 9)
#define ParamDFA_az05o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 12)
#define ParamDFA_az05o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az05o4Dpt12)))
// Ausgabewert Zustand 5 (DPT 13)
#define ParamDFA_az05o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az05o4Dpt13)))
// Ausgabewert Zustand 5 (DPT 14)
#define ParamDFA_az05o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az05o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 5 (DPT 16)
#define ParamDFA_az05o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az05o4Dpt16)))
// Ausgabewert Zustand 5 (DPT 17)
#define ParamDFA_az05o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az05o4Dpt17)))
// Ausgabewert Zustand 5 (DPT 232)
#define ParamDFA_az05o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az05o4Dpt232)) & DFA_az05o4Dpt232Mask) >> DFA_az05o4Dpt232Shift)
// trans(06,1)
#define ParamDFA_ad06A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06A)))
// trans(06,2)
#define ParamDFA_ad06B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06B)))
// trans(06,3)
#define ParamDFA_ad06C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06C)))
// trans(06,4)
#define ParamDFA_ad06D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06D)))
// trans(06,5)
#define ParamDFA_ad06E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06E)))
// trans(06,6)
#define ParamDFA_ad06F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06F)))
// trans(06,7)
#define ParamDFA_ad06G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06G)))
// trans(06,8)
#define ParamDFA_ad06H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06H)))
// trans(06,timeout)
#define ParamDFA_ad06T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad06T)))
// Zeitbasis
#define ParamDFA_ad06TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad06TBase)) & DFA_ad06TBaseMask) >> DFA_ad06TBaseShift)
// Zeit
#define ParamDFA_ad06TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad06TTime)) & DFA_ad06TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad06TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad06TTime))))
// Sendeverhalten Zustand 6
#define ParamDFA_az06o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Send)))
// Ausgabewert Zustand 6 (DPT 1)
#define ParamDFA_az06o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Dpt1)))
// Ausgabewert Zustand 6 (DPT 2)
#define ParamDFA_az06o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Dpt2)))
// Ausgabewert Zustand 6 (DPT 5)
#define ParamDFA_az06o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Dpt5)))
// Ausgabewert Zustand 6 (DPT 5.001)
#define ParamDFA_az06o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Dpt5001)))
// Ausgabewert Zustand 6 (DPT 6)
#define ParamDFA_az06o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Dpt6)))
// Ausgabewert Zustand 6 (DPT 7)
#define ParamDFA_az06o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az06o1Dpt7)))
// Ausgabewert Zustand 6 (DPT 8)
#define ParamDFA_az06o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az06o1Dpt8)))
// Ausgabewert Zustand 6 (DPT 9)
#define ParamDFA_az06o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 12)
#define ParamDFA_az06o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az06o1Dpt12)))
// Ausgabewert Zustand 6 (DPT 13)
#define ParamDFA_az06o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az06o1Dpt13)))
// Ausgabewert Zustand 6 (DPT 14)
#define ParamDFA_az06o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 17)
#define ParamDFA_az06o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o1Dpt17)))
// Ausgabewert Zustand 6 (DPT 232)
#define ParamDFA_az06o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az06o1Dpt232)) & DFA_az06o1Dpt232Mask) >> DFA_az06o1Dpt232Shift)
// Sendeverhalten Zustand 6
#define ParamDFA_az06o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Send)))
// Ausgabewert Zustand 6 (DPT 1)
#define ParamDFA_az06o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Dpt1)))
// Ausgabewert Zustand 6 (DPT 2)
#define ParamDFA_az06o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Dpt2)))
// Ausgabewert Zustand 6 (DPT 5)
#define ParamDFA_az06o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Dpt5)))
// Ausgabewert Zustand 6 (DPT 5.001)
#define ParamDFA_az06o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Dpt5001)))
// Ausgabewert Zustand 6 (DPT 6)
#define ParamDFA_az06o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Dpt6)))
// Ausgabewert Zustand 6 (DPT 7)
#define ParamDFA_az06o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az06o2Dpt7)))
// Ausgabewert Zustand 6 (DPT 8)
#define ParamDFA_az06o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az06o2Dpt8)))
// Ausgabewert Zustand 6 (DPT 9)
#define ParamDFA_az06o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 12)
#define ParamDFA_az06o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az06o2Dpt12)))
// Ausgabewert Zustand 6 (DPT 13)
#define ParamDFA_az06o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az06o2Dpt13)))
// Ausgabewert Zustand 6 (DPT 14)
#define ParamDFA_az06o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 17)
#define ParamDFA_az06o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o2Dpt17)))
// Ausgabewert Zustand 6 (DPT 232)
#define ParamDFA_az06o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az06o2Dpt232)) & DFA_az06o2Dpt232Mask) >> DFA_az06o2Dpt232Shift)
// Sendeverhalten Zustand 6
#define ParamDFA_az06o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Send)))
// Ausgabewert Zustand 6 (DPT 1)
#define ParamDFA_az06o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Dpt1)))
// Ausgabewert Zustand 6 (DPT 2)
#define ParamDFA_az06o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Dpt2)))
// Ausgabewert Zustand 6 (DPT 5)
#define ParamDFA_az06o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Dpt5)))
// Ausgabewert Zustand 6 (DPT 5.001)
#define ParamDFA_az06o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Dpt5001)))
// Ausgabewert Zustand 6 (DPT 6)
#define ParamDFA_az06o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Dpt6)))
// Ausgabewert Zustand 6 (DPT 7)
#define ParamDFA_az06o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az06o3Dpt7)))
// Ausgabewert Zustand 6 (DPT 8)
#define ParamDFA_az06o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az06o3Dpt8)))
// Ausgabewert Zustand 6 (DPT 9)
#define ParamDFA_az06o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 12)
#define ParamDFA_az06o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az06o3Dpt12)))
// Ausgabewert Zustand 6 (DPT 13)
#define ParamDFA_az06o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az06o3Dpt13)))
// Ausgabewert Zustand 6 (DPT 14)
#define ParamDFA_az06o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 17)
#define ParamDFA_az06o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o3Dpt17)))
// Ausgabewert Zustand 6 (DPT 232)
#define ParamDFA_az06o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az06o3Dpt232)) & DFA_az06o3Dpt232Mask) >> DFA_az06o3Dpt232Shift)
// Sendeverhalten Zustand 6
#define ParamDFA_az06o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Send)))
// Ausgabewert Zustand 6 (DPT 1)
#define ParamDFA_az06o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Dpt1)))
// Ausgabewert Zustand 6 (DPT 2)
#define ParamDFA_az06o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Dpt2)))
// Ausgabewert Zustand 6 (DPT 5)
#define ParamDFA_az06o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Dpt5)))
// Ausgabewert Zustand 6 (DPT 5.001)
#define ParamDFA_az06o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Dpt5001)))
// Ausgabewert Zustand 6 (DPT 6)
#define ParamDFA_az06o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Dpt6)))
// Ausgabewert Zustand 6 (DPT 7)
#define ParamDFA_az06o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az06o4Dpt7)))
// Ausgabewert Zustand 6 (DPT 8)
#define ParamDFA_az06o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az06o4Dpt8)))
// Ausgabewert Zustand 6 (DPT 9)
#define ParamDFA_az06o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 12)
#define ParamDFA_az06o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az06o4Dpt12)))
// Ausgabewert Zustand 6 (DPT 13)
#define ParamDFA_az06o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az06o4Dpt13)))
// Ausgabewert Zustand 6 (DPT 14)
#define ParamDFA_az06o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az06o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 6 (DPT 16)
#define ParamDFA_az06o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az06o4Dpt16)))
// Ausgabewert Zustand 6 (DPT 17)
#define ParamDFA_az06o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az06o4Dpt17)))
// Ausgabewert Zustand 6 (DPT 232)
#define ParamDFA_az06o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az06o4Dpt232)) & DFA_az06o4Dpt232Mask) >> DFA_az06o4Dpt232Shift)
// trans(07,1)
#define ParamDFA_ad07A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07A)))
// trans(07,2)
#define ParamDFA_ad07B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07B)))
// trans(07,3)
#define ParamDFA_ad07C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07C)))
// trans(07,4)
#define ParamDFA_ad07D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07D)))
// trans(07,5)
#define ParamDFA_ad07E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07E)))
// trans(07,6)
#define ParamDFA_ad07F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07F)))
// trans(07,7)
#define ParamDFA_ad07G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07G)))
// trans(07,8)
#define ParamDFA_ad07H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07H)))
// trans(07,timeout)
#define ParamDFA_ad07T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad07T)))
// Zeitbasis
#define ParamDFA_ad07TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad07TBase)) & DFA_ad07TBaseMask) >> DFA_ad07TBaseShift)
// Zeit
#define ParamDFA_ad07TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad07TTime)) & DFA_ad07TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad07TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad07TTime))))
// Sendeverhalten Zustand 7
#define ParamDFA_az07o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Send)))
// Ausgabewert Zustand 7 (DPT 1)
#define ParamDFA_az07o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Dpt1)))
// Ausgabewert Zustand 7 (DPT 2)
#define ParamDFA_az07o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Dpt2)))
// Ausgabewert Zustand 7 (DPT 5)
#define ParamDFA_az07o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Dpt5)))
// Ausgabewert Zustand 7 (DPT 5.001)
#define ParamDFA_az07o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Dpt5001)))
// Ausgabewert Zustand 7 (DPT 6)
#define ParamDFA_az07o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Dpt6)))
// Ausgabewert Zustand 7 (DPT 7)
#define ParamDFA_az07o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az07o1Dpt7)))
// Ausgabewert Zustand 7 (DPT 8)
#define ParamDFA_az07o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az07o1Dpt8)))
// Ausgabewert Zustand 7 (DPT 9)
#define ParamDFA_az07o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 12)
#define ParamDFA_az07o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az07o1Dpt12)))
// Ausgabewert Zustand 7 (DPT 13)
#define ParamDFA_az07o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az07o1Dpt13)))
// Ausgabewert Zustand 7 (DPT 14)
#define ParamDFA_az07o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 17)
#define ParamDFA_az07o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o1Dpt17)))
// Ausgabewert Zustand 7 (DPT 232)
#define ParamDFA_az07o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az07o1Dpt232)) & DFA_az07o1Dpt232Mask) >> DFA_az07o1Dpt232Shift)
// Sendeverhalten Zustand 7
#define ParamDFA_az07o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Send)))
// Ausgabewert Zustand 7 (DPT 1)
#define ParamDFA_az07o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Dpt1)))
// Ausgabewert Zustand 7 (DPT 2)
#define ParamDFA_az07o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Dpt2)))
// Ausgabewert Zustand 7 (DPT 5)
#define ParamDFA_az07o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Dpt5)))
// Ausgabewert Zustand 7 (DPT 5.001)
#define ParamDFA_az07o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Dpt5001)))
// Ausgabewert Zustand 7 (DPT 6)
#define ParamDFA_az07o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Dpt6)))
// Ausgabewert Zustand 7 (DPT 7)
#define ParamDFA_az07o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az07o2Dpt7)))
// Ausgabewert Zustand 7 (DPT 8)
#define ParamDFA_az07o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az07o2Dpt8)))
// Ausgabewert Zustand 7 (DPT 9)
#define ParamDFA_az07o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 12)
#define ParamDFA_az07o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az07o2Dpt12)))
// Ausgabewert Zustand 7 (DPT 13)
#define ParamDFA_az07o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az07o2Dpt13)))
// Ausgabewert Zustand 7 (DPT 14)
#define ParamDFA_az07o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 17)
#define ParamDFA_az07o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o2Dpt17)))
// Ausgabewert Zustand 7 (DPT 232)
#define ParamDFA_az07o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az07o2Dpt232)) & DFA_az07o2Dpt232Mask) >> DFA_az07o2Dpt232Shift)
// Sendeverhalten Zustand 7
#define ParamDFA_az07o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Send)))
// Ausgabewert Zustand 7 (DPT 1)
#define ParamDFA_az07o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Dpt1)))
// Ausgabewert Zustand 7 (DPT 2)
#define ParamDFA_az07o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Dpt2)))
// Ausgabewert Zustand 7 (DPT 5)
#define ParamDFA_az07o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Dpt5)))
// Ausgabewert Zustand 7 (DPT 5.001)
#define ParamDFA_az07o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Dpt5001)))
// Ausgabewert Zustand 7 (DPT 6)
#define ParamDFA_az07o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Dpt6)))
// Ausgabewert Zustand 7 (DPT 7)
#define ParamDFA_az07o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az07o3Dpt7)))
// Ausgabewert Zustand 7 (DPT 8)
#define ParamDFA_az07o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az07o3Dpt8)))
// Ausgabewert Zustand 7 (DPT 9)
#define ParamDFA_az07o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 12)
#define ParamDFA_az07o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az07o3Dpt12)))
// Ausgabewert Zustand 7 (DPT 13)
#define ParamDFA_az07o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az07o3Dpt13)))
// Ausgabewert Zustand 7 (DPT 14)
#define ParamDFA_az07o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 17)
#define ParamDFA_az07o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o3Dpt17)))
// Ausgabewert Zustand 7 (DPT 232)
#define ParamDFA_az07o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az07o3Dpt232)) & DFA_az07o3Dpt232Mask) >> DFA_az07o3Dpt232Shift)
// Sendeverhalten Zustand 7
#define ParamDFA_az07o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Send)))
// Ausgabewert Zustand 7 (DPT 1)
#define ParamDFA_az07o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Dpt1)))
// Ausgabewert Zustand 7 (DPT 2)
#define ParamDFA_az07o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Dpt2)))
// Ausgabewert Zustand 7 (DPT 5)
#define ParamDFA_az07o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Dpt5)))
// Ausgabewert Zustand 7 (DPT 5.001)
#define ParamDFA_az07o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Dpt5001)))
// Ausgabewert Zustand 7 (DPT 6)
#define ParamDFA_az07o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Dpt6)))
// Ausgabewert Zustand 7 (DPT 7)
#define ParamDFA_az07o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az07o4Dpt7)))
// Ausgabewert Zustand 7 (DPT 8)
#define ParamDFA_az07o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az07o4Dpt8)))
// Ausgabewert Zustand 7 (DPT 9)
#define ParamDFA_az07o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 12)
#define ParamDFA_az07o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az07o4Dpt12)))
// Ausgabewert Zustand 7 (DPT 13)
#define ParamDFA_az07o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az07o4Dpt13)))
// Ausgabewert Zustand 7 (DPT 14)
#define ParamDFA_az07o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az07o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 7 (DPT 16)
#define ParamDFA_az07o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az07o4Dpt16)))
// Ausgabewert Zustand 7 (DPT 17)
#define ParamDFA_az07o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az07o4Dpt17)))
// Ausgabewert Zustand 7 (DPT 232)
#define ParamDFA_az07o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az07o4Dpt232)) & DFA_az07o4Dpt232Mask) >> DFA_az07o4Dpt232Shift)
// trans(08,1)
#define ParamDFA_ad08A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08A)))
// trans(08,2)
#define ParamDFA_ad08B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08B)))
// trans(08,3)
#define ParamDFA_ad08C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08C)))
// trans(08,4)
#define ParamDFA_ad08D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08D)))
// trans(08,5)
#define ParamDFA_ad08E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08E)))
// trans(08,6)
#define ParamDFA_ad08F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08F)))
// trans(08,7)
#define ParamDFA_ad08G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08G)))
// trans(08,8)
#define ParamDFA_ad08H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08H)))
// trans(08,timeout)
#define ParamDFA_ad08T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad08T)))
// Zeitbasis
#define ParamDFA_ad08TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad08TBase)) & DFA_ad08TBaseMask) >> DFA_ad08TBaseShift)
// Zeit
#define ParamDFA_ad08TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad08TTime)) & DFA_ad08TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad08TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad08TTime))))
// Sendeverhalten Zustand 8
#define ParamDFA_az08o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Send)))
// Ausgabewert Zustand 8 (DPT 1)
#define ParamDFA_az08o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Dpt1)))
// Ausgabewert Zustand 8 (DPT 2)
#define ParamDFA_az08o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Dpt2)))
// Ausgabewert Zustand 8 (DPT 5)
#define ParamDFA_az08o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Dpt5)))
// Ausgabewert Zustand 8 (DPT 5.001)
#define ParamDFA_az08o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Dpt5001)))
// Ausgabewert Zustand 8 (DPT 6)
#define ParamDFA_az08o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Dpt6)))
// Ausgabewert Zustand 8 (DPT 7)
#define ParamDFA_az08o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az08o1Dpt7)))
// Ausgabewert Zustand 8 (DPT 8)
#define ParamDFA_az08o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az08o1Dpt8)))
// Ausgabewert Zustand 8 (DPT 9)
#define ParamDFA_az08o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 12)
#define ParamDFA_az08o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az08o1Dpt12)))
// Ausgabewert Zustand 8 (DPT 13)
#define ParamDFA_az08o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az08o1Dpt13)))
// Ausgabewert Zustand 8 (DPT 14)
#define ParamDFA_az08o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 17)
#define ParamDFA_az08o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o1Dpt17)))
// Ausgabewert Zustand 8 (DPT 232)
#define ParamDFA_az08o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az08o1Dpt232)) & DFA_az08o1Dpt232Mask) >> DFA_az08o1Dpt232Shift)
// Sendeverhalten Zustand 8
#define ParamDFA_az08o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Send)))
// Ausgabewert Zustand 8 (DPT 1)
#define ParamDFA_az08o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Dpt1)))
// Ausgabewert Zustand 8 (DPT 2)
#define ParamDFA_az08o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Dpt2)))
// Ausgabewert Zustand 8 (DPT 5)
#define ParamDFA_az08o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Dpt5)))
// Ausgabewert Zustand 8 (DPT 5.001)
#define ParamDFA_az08o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Dpt5001)))
// Ausgabewert Zustand 8 (DPT 6)
#define ParamDFA_az08o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Dpt6)))
// Ausgabewert Zustand 8 (DPT 7)
#define ParamDFA_az08o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az08o2Dpt7)))
// Ausgabewert Zustand 8 (DPT 8)
#define ParamDFA_az08o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az08o2Dpt8)))
// Ausgabewert Zustand 8 (DPT 9)
#define ParamDFA_az08o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 12)
#define ParamDFA_az08o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az08o2Dpt12)))
// Ausgabewert Zustand 8 (DPT 13)
#define ParamDFA_az08o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az08o2Dpt13)))
// Ausgabewert Zustand 8 (DPT 14)
#define ParamDFA_az08o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 17)
#define ParamDFA_az08o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o2Dpt17)))
// Ausgabewert Zustand 8 (DPT 232)
#define ParamDFA_az08o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az08o2Dpt232)) & DFA_az08o2Dpt232Mask) >> DFA_az08o2Dpt232Shift)
// Sendeverhalten Zustand 8
#define ParamDFA_az08o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Send)))
// Ausgabewert Zustand 8 (DPT 1)
#define ParamDFA_az08o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Dpt1)))
// Ausgabewert Zustand 8 (DPT 2)
#define ParamDFA_az08o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Dpt2)))
// Ausgabewert Zustand 8 (DPT 5)
#define ParamDFA_az08o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Dpt5)))
// Ausgabewert Zustand 8 (DPT 5.001)
#define ParamDFA_az08o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Dpt5001)))
// Ausgabewert Zustand 8 (DPT 6)
#define ParamDFA_az08o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Dpt6)))
// Ausgabewert Zustand 8 (DPT 7)
#define ParamDFA_az08o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az08o3Dpt7)))
// Ausgabewert Zustand 8 (DPT 8)
#define ParamDFA_az08o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az08o3Dpt8)))
// Ausgabewert Zustand 8 (DPT 9)
#define ParamDFA_az08o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 12)
#define ParamDFA_az08o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az08o3Dpt12)))
// Ausgabewert Zustand 8 (DPT 13)
#define ParamDFA_az08o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az08o3Dpt13)))
// Ausgabewert Zustand 8 (DPT 14)
#define ParamDFA_az08o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 17)
#define ParamDFA_az08o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o3Dpt17)))
// Ausgabewert Zustand 8 (DPT 232)
#define ParamDFA_az08o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az08o3Dpt232)) & DFA_az08o3Dpt232Mask) >> DFA_az08o3Dpt232Shift)
// Sendeverhalten Zustand 8
#define ParamDFA_az08o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Send)))
// Ausgabewert Zustand 8 (DPT 1)
#define ParamDFA_az08o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Dpt1)))
// Ausgabewert Zustand 8 (DPT 2)
#define ParamDFA_az08o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Dpt2)))
// Ausgabewert Zustand 8 (DPT 5)
#define ParamDFA_az08o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Dpt5)))
// Ausgabewert Zustand 8 (DPT 5.001)
#define ParamDFA_az08o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Dpt5001)))
// Ausgabewert Zustand 8 (DPT 6)
#define ParamDFA_az08o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Dpt6)))
// Ausgabewert Zustand 8 (DPT 7)
#define ParamDFA_az08o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az08o4Dpt7)))
// Ausgabewert Zustand 8 (DPT 8)
#define ParamDFA_az08o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az08o4Dpt8)))
// Ausgabewert Zustand 8 (DPT 9)
#define ParamDFA_az08o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 12)
#define ParamDFA_az08o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az08o4Dpt12)))
// Ausgabewert Zustand 8 (DPT 13)
#define ParamDFA_az08o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az08o4Dpt13)))
// Ausgabewert Zustand 8 (DPT 14)
#define ParamDFA_az08o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az08o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 8 (DPT 16)
#define ParamDFA_az08o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az08o4Dpt16)))
// Ausgabewert Zustand 8 (DPT 17)
#define ParamDFA_az08o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az08o4Dpt17)))
// Ausgabewert Zustand 8 (DPT 232)
#define ParamDFA_az08o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az08o4Dpt232)) & DFA_az08o4Dpt232Mask) >> DFA_az08o4Dpt232Shift)
// trans(09,1)
#define ParamDFA_ad09A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09A)))
// trans(09,2)
#define ParamDFA_ad09B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09B)))
// trans(09,3)
#define ParamDFA_ad09C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09C)))
// trans(09,4)
#define ParamDFA_ad09D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09D)))
// trans(09,5)
#define ParamDFA_ad09E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09E)))
// trans(09,6)
#define ParamDFA_ad09F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09F)))
// trans(09,7)
#define ParamDFA_ad09G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09G)))
// trans(09,8)
#define ParamDFA_ad09H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09H)))
// trans(09,timeout)
#define ParamDFA_ad09T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad09T)))
// Zeitbasis
#define ParamDFA_ad09TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad09TBase)) & DFA_ad09TBaseMask) >> DFA_ad09TBaseShift)
// Zeit
#define ParamDFA_ad09TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad09TTime)) & DFA_ad09TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad09TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad09TTime))))
// Sendeverhalten Zustand 9
#define ParamDFA_az09o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Send)))
// Ausgabewert Zustand 9 (DPT 1)
#define ParamDFA_az09o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Dpt1)))
// Ausgabewert Zustand 9 (DPT 2)
#define ParamDFA_az09o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Dpt2)))
// Ausgabewert Zustand 9 (DPT 5)
#define ParamDFA_az09o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Dpt5)))
// Ausgabewert Zustand 9 (DPT 5.001)
#define ParamDFA_az09o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Dpt5001)))
// Ausgabewert Zustand 9 (DPT 6)
#define ParamDFA_az09o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Dpt6)))
// Ausgabewert Zustand 9 (DPT 7)
#define ParamDFA_az09o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az09o1Dpt7)))
// Ausgabewert Zustand 9 (DPT 8)
#define ParamDFA_az09o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az09o1Dpt8)))
// Ausgabewert Zustand 9 (DPT 9)
#define ParamDFA_az09o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 12)
#define ParamDFA_az09o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az09o1Dpt12)))
// Ausgabewert Zustand 9 (DPT 13)
#define ParamDFA_az09o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az09o1Dpt13)))
// Ausgabewert Zustand 9 (DPT 14)
#define ParamDFA_az09o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 17)
#define ParamDFA_az09o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o1Dpt17)))
// Ausgabewert Zustand 9 (DPT 232)
#define ParamDFA_az09o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az09o1Dpt232)) & DFA_az09o1Dpt232Mask) >> DFA_az09o1Dpt232Shift)
// Sendeverhalten Zustand 9
#define ParamDFA_az09o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Send)))
// Ausgabewert Zustand 9 (DPT 1)
#define ParamDFA_az09o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Dpt1)))
// Ausgabewert Zustand 9 (DPT 2)
#define ParamDFA_az09o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Dpt2)))
// Ausgabewert Zustand 9 (DPT 5)
#define ParamDFA_az09o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Dpt5)))
// Ausgabewert Zustand 9 (DPT 5.001)
#define ParamDFA_az09o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Dpt5001)))
// Ausgabewert Zustand 9 (DPT 6)
#define ParamDFA_az09o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Dpt6)))
// Ausgabewert Zustand 9 (DPT 7)
#define ParamDFA_az09o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az09o2Dpt7)))
// Ausgabewert Zustand 9 (DPT 8)
#define ParamDFA_az09o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az09o2Dpt8)))
// Ausgabewert Zustand 9 (DPT 9)
#define ParamDFA_az09o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 12)
#define ParamDFA_az09o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az09o2Dpt12)))
// Ausgabewert Zustand 9 (DPT 13)
#define ParamDFA_az09o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az09o2Dpt13)))
// Ausgabewert Zustand 9 (DPT 14)
#define ParamDFA_az09o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 17)
#define ParamDFA_az09o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o2Dpt17)))
// Ausgabewert Zustand 9 (DPT 232)
#define ParamDFA_az09o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az09o2Dpt232)) & DFA_az09o2Dpt232Mask) >> DFA_az09o2Dpt232Shift)
// Sendeverhalten Zustand 9
#define ParamDFA_az09o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Send)))
// Ausgabewert Zustand 9 (DPT 1)
#define ParamDFA_az09o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Dpt1)))
// Ausgabewert Zustand 9 (DPT 2)
#define ParamDFA_az09o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Dpt2)))
// Ausgabewert Zustand 9 (DPT 5)
#define ParamDFA_az09o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Dpt5)))
// Ausgabewert Zustand 9 (DPT 5.001)
#define ParamDFA_az09o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Dpt5001)))
// Ausgabewert Zustand 9 (DPT 6)
#define ParamDFA_az09o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Dpt6)))
// Ausgabewert Zustand 9 (DPT 7)
#define ParamDFA_az09o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az09o3Dpt7)))
// Ausgabewert Zustand 9 (DPT 8)
#define ParamDFA_az09o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az09o3Dpt8)))
// Ausgabewert Zustand 9 (DPT 9)
#define ParamDFA_az09o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 12)
#define ParamDFA_az09o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az09o3Dpt12)))
// Ausgabewert Zustand 9 (DPT 13)
#define ParamDFA_az09o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az09o3Dpt13)))
// Ausgabewert Zustand 9 (DPT 14)
#define ParamDFA_az09o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 17)
#define ParamDFA_az09o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o3Dpt17)))
// Ausgabewert Zustand 9 (DPT 232)
#define ParamDFA_az09o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az09o3Dpt232)) & DFA_az09o3Dpt232Mask) >> DFA_az09o3Dpt232Shift)
// Sendeverhalten Zustand 9
#define ParamDFA_az09o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Send)))
// Ausgabewert Zustand 9 (DPT 1)
#define ParamDFA_az09o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Dpt1)))
// Ausgabewert Zustand 9 (DPT 2)
#define ParamDFA_az09o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Dpt2)))
// Ausgabewert Zustand 9 (DPT 5)
#define ParamDFA_az09o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Dpt5)))
// Ausgabewert Zustand 9 (DPT 5.001)
#define ParamDFA_az09o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Dpt5001)))
// Ausgabewert Zustand 9 (DPT 6)
#define ParamDFA_az09o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Dpt6)))
// Ausgabewert Zustand 9 (DPT 7)
#define ParamDFA_az09o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az09o4Dpt7)))
// Ausgabewert Zustand 9 (DPT 8)
#define ParamDFA_az09o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az09o4Dpt8)))
// Ausgabewert Zustand 9 (DPT 9)
#define ParamDFA_az09o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 12)
#define ParamDFA_az09o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az09o4Dpt12)))
// Ausgabewert Zustand 9 (DPT 13)
#define ParamDFA_az09o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az09o4Dpt13)))
// Ausgabewert Zustand 9 (DPT 14)
#define ParamDFA_az09o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az09o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 9 (DPT 16)
#define ParamDFA_az09o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az09o4Dpt16)))
// Ausgabewert Zustand 9 (DPT 17)
#define ParamDFA_az09o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az09o4Dpt17)))
// Ausgabewert Zustand 9 (DPT 232)
#define ParamDFA_az09o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az09o4Dpt232)) & DFA_az09o4Dpt232Mask) >> DFA_az09o4Dpt232Shift)
// trans(10,1)
#define ParamDFA_ad10A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10A)))
// trans(10,2)
#define ParamDFA_ad10B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10B)))
// trans(10,3)
#define ParamDFA_ad10C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10C)))
// trans(10,4)
#define ParamDFA_ad10D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10D)))
// trans(10,5)
#define ParamDFA_ad10E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10E)))
// trans(10,6)
#define ParamDFA_ad10F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10F)))
// trans(10,7)
#define ParamDFA_ad10G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10G)))
// trans(10,8)
#define ParamDFA_ad10H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10H)))
// trans(10,timeout)
#define ParamDFA_ad10T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad10T)))
// Zeitbasis
#define ParamDFA_ad10TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad10TBase)) & DFA_ad10TBaseMask) >> DFA_ad10TBaseShift)
// Zeit
#define ParamDFA_ad10TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad10TTime)) & DFA_ad10TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad10TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad10TTime))))
// Sendeverhalten Zustand 10
#define ParamDFA_az10o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Send)))
// Ausgabewert Zustand 10 (DPT 1)
#define ParamDFA_az10o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Dpt1)))
// Ausgabewert Zustand 10 (DPT 2)
#define ParamDFA_az10o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Dpt2)))
// Ausgabewert Zustand 10 (DPT 5)
#define ParamDFA_az10o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Dpt5)))
// Ausgabewert Zustand 10 (DPT 5.001)
#define ParamDFA_az10o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Dpt5001)))
// Ausgabewert Zustand 10 (DPT 6)
#define ParamDFA_az10o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Dpt6)))
// Ausgabewert Zustand 10 (DPT 7)
#define ParamDFA_az10o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az10o1Dpt7)))
// Ausgabewert Zustand 10 (DPT 8)
#define ParamDFA_az10o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az10o1Dpt8)))
// Ausgabewert Zustand 10 (DPT 9)
#define ParamDFA_az10o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 12)
#define ParamDFA_az10o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az10o1Dpt12)))
// Ausgabewert Zustand 10 (DPT 13)
#define ParamDFA_az10o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az10o1Dpt13)))
// Ausgabewert Zustand 10 (DPT 14)
#define ParamDFA_az10o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 17)
#define ParamDFA_az10o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o1Dpt17)))
// Ausgabewert Zustand 10 (DPT 232)
#define ParamDFA_az10o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az10o1Dpt232)) & DFA_az10o1Dpt232Mask) >> DFA_az10o1Dpt232Shift)
// Sendeverhalten Zustand 10
#define ParamDFA_az10o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Send)))
// Ausgabewert Zustand 10 (DPT 1)
#define ParamDFA_az10o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Dpt1)))
// Ausgabewert Zustand 10 (DPT 2)
#define ParamDFA_az10o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Dpt2)))
// Ausgabewert Zustand 10 (DPT 5)
#define ParamDFA_az10o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Dpt5)))
// Ausgabewert Zustand 10 (DPT 5.001)
#define ParamDFA_az10o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Dpt5001)))
// Ausgabewert Zustand 10 (DPT 6)
#define ParamDFA_az10o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Dpt6)))
// Ausgabewert Zustand 10 (DPT 7)
#define ParamDFA_az10o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az10o2Dpt7)))
// Ausgabewert Zustand 10 (DPT 8)
#define ParamDFA_az10o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az10o2Dpt8)))
// Ausgabewert Zustand 10 (DPT 9)
#define ParamDFA_az10o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 12)
#define ParamDFA_az10o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az10o2Dpt12)))
// Ausgabewert Zustand 10 (DPT 13)
#define ParamDFA_az10o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az10o2Dpt13)))
// Ausgabewert Zustand 10 (DPT 14)
#define ParamDFA_az10o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 17)
#define ParamDFA_az10o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o2Dpt17)))
// Ausgabewert Zustand 10 (DPT 232)
#define ParamDFA_az10o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az10o2Dpt232)) & DFA_az10o2Dpt232Mask) >> DFA_az10o2Dpt232Shift)
// Sendeverhalten Zustand 10
#define ParamDFA_az10o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Send)))
// Ausgabewert Zustand 10 (DPT 1)
#define ParamDFA_az10o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Dpt1)))
// Ausgabewert Zustand 10 (DPT 2)
#define ParamDFA_az10o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Dpt2)))
// Ausgabewert Zustand 10 (DPT 5)
#define ParamDFA_az10o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Dpt5)))
// Ausgabewert Zustand 10 (DPT 5.001)
#define ParamDFA_az10o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Dpt5001)))
// Ausgabewert Zustand 10 (DPT 6)
#define ParamDFA_az10o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Dpt6)))
// Ausgabewert Zustand 10 (DPT 7)
#define ParamDFA_az10o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az10o3Dpt7)))
// Ausgabewert Zustand 10 (DPT 8)
#define ParamDFA_az10o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az10o3Dpt8)))
// Ausgabewert Zustand 10 (DPT 9)
#define ParamDFA_az10o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 12)
#define ParamDFA_az10o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az10o3Dpt12)))
// Ausgabewert Zustand 10 (DPT 13)
#define ParamDFA_az10o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az10o3Dpt13)))
// Ausgabewert Zustand 10 (DPT 14)
#define ParamDFA_az10o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 17)
#define ParamDFA_az10o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o3Dpt17)))
// Ausgabewert Zustand 10 (DPT 232)
#define ParamDFA_az10o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az10o3Dpt232)) & DFA_az10o3Dpt232Mask) >> DFA_az10o3Dpt232Shift)
// Sendeverhalten Zustand 10
#define ParamDFA_az10o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Send)))
// Ausgabewert Zustand 10 (DPT 1)
#define ParamDFA_az10o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Dpt1)))
// Ausgabewert Zustand 10 (DPT 2)
#define ParamDFA_az10o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Dpt2)))
// Ausgabewert Zustand 10 (DPT 5)
#define ParamDFA_az10o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Dpt5)))
// Ausgabewert Zustand 10 (DPT 5.001)
#define ParamDFA_az10o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Dpt5001)))
// Ausgabewert Zustand 10 (DPT 6)
#define ParamDFA_az10o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Dpt6)))
// Ausgabewert Zustand 10 (DPT 7)
#define ParamDFA_az10o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az10o4Dpt7)))
// Ausgabewert Zustand 10 (DPT 8)
#define ParamDFA_az10o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az10o4Dpt8)))
// Ausgabewert Zustand 10 (DPT 9)
#define ParamDFA_az10o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 12)
#define ParamDFA_az10o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az10o4Dpt12)))
// Ausgabewert Zustand 10 (DPT 13)
#define ParamDFA_az10o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az10o4Dpt13)))
// Ausgabewert Zustand 10 (DPT 14)
#define ParamDFA_az10o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az10o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 10 (DPT 16)
#define ParamDFA_az10o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az10o4Dpt16)))
// Ausgabewert Zustand 10 (DPT 17)
#define ParamDFA_az10o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az10o4Dpt17)))
// Ausgabewert Zustand 10 (DPT 232)
#define ParamDFA_az10o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az10o4Dpt232)) & DFA_az10o4Dpt232Mask) >> DFA_az10o4Dpt232Shift)
// trans(11,1)
#define ParamDFA_ad11A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11A)))
// trans(11,2)
#define ParamDFA_ad11B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11B)))
// trans(11,3)
#define ParamDFA_ad11C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11C)))
// trans(11,4)
#define ParamDFA_ad11D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11D)))
// trans(11,5)
#define ParamDFA_ad11E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11E)))
// trans(11,6)
#define ParamDFA_ad11F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11F)))
// trans(11,7)
#define ParamDFA_ad11G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11G)))
// trans(11,8)
#define ParamDFA_ad11H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11H)))
// trans(11,timeout)
#define ParamDFA_ad11T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad11T)))
// Zeitbasis
#define ParamDFA_ad11TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad11TBase)) & DFA_ad11TBaseMask) >> DFA_ad11TBaseShift)
// Zeit
#define ParamDFA_ad11TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad11TTime)) & DFA_ad11TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad11TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad11TTime))))
// Sendeverhalten Zustand 11
#define ParamDFA_az11o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Send)))
// Ausgabewert Zustand 11 (DPT 1)
#define ParamDFA_az11o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Dpt1)))
// Ausgabewert Zustand 11 (DPT 2)
#define ParamDFA_az11o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Dpt2)))
// Ausgabewert Zustand 11 (DPT 5)
#define ParamDFA_az11o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Dpt5)))
// Ausgabewert Zustand 11 (DPT 5.001)
#define ParamDFA_az11o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Dpt5001)))
// Ausgabewert Zustand 11 (DPT 6)
#define ParamDFA_az11o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Dpt6)))
// Ausgabewert Zustand 11 (DPT 7)
#define ParamDFA_az11o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az11o1Dpt7)))
// Ausgabewert Zustand 11 (DPT 8)
#define ParamDFA_az11o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az11o1Dpt8)))
// Ausgabewert Zustand 11 (DPT 9)
#define ParamDFA_az11o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 12)
#define ParamDFA_az11o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az11o1Dpt12)))
// Ausgabewert Zustand 11 (DPT 13)
#define ParamDFA_az11o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az11o1Dpt13)))
// Ausgabewert Zustand 11 (DPT 14)
#define ParamDFA_az11o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 17)
#define ParamDFA_az11o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o1Dpt17)))
// Ausgabewert Zustand 11 (DPT 232)
#define ParamDFA_az11o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az11o1Dpt232)) & DFA_az11o1Dpt232Mask) >> DFA_az11o1Dpt232Shift)
// Sendeverhalten Zustand 11
#define ParamDFA_az11o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Send)))
// Ausgabewert Zustand 11 (DPT 1)
#define ParamDFA_az11o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Dpt1)))
// Ausgabewert Zustand 11 (DPT 2)
#define ParamDFA_az11o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Dpt2)))
// Ausgabewert Zustand 11 (DPT 5)
#define ParamDFA_az11o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Dpt5)))
// Ausgabewert Zustand 11 (DPT 5.001)
#define ParamDFA_az11o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Dpt5001)))
// Ausgabewert Zustand 11 (DPT 6)
#define ParamDFA_az11o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Dpt6)))
// Ausgabewert Zustand 11 (DPT 7)
#define ParamDFA_az11o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az11o2Dpt7)))
// Ausgabewert Zustand 11 (DPT 8)
#define ParamDFA_az11o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az11o2Dpt8)))
// Ausgabewert Zustand 11 (DPT 9)
#define ParamDFA_az11o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 12)
#define ParamDFA_az11o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az11o2Dpt12)))
// Ausgabewert Zustand 11 (DPT 13)
#define ParamDFA_az11o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az11o2Dpt13)))
// Ausgabewert Zustand 11 (DPT 14)
#define ParamDFA_az11o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 17)
#define ParamDFA_az11o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o2Dpt17)))
// Ausgabewert Zustand 11 (DPT 232)
#define ParamDFA_az11o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az11o2Dpt232)) & DFA_az11o2Dpt232Mask) >> DFA_az11o2Dpt232Shift)
// Sendeverhalten Zustand 11
#define ParamDFA_az11o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Send)))
// Ausgabewert Zustand 11 (DPT 1)
#define ParamDFA_az11o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Dpt1)))
// Ausgabewert Zustand 11 (DPT 2)
#define ParamDFA_az11o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Dpt2)))
// Ausgabewert Zustand 11 (DPT 5)
#define ParamDFA_az11o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Dpt5)))
// Ausgabewert Zustand 11 (DPT 5.001)
#define ParamDFA_az11o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Dpt5001)))
// Ausgabewert Zustand 11 (DPT 6)
#define ParamDFA_az11o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Dpt6)))
// Ausgabewert Zustand 11 (DPT 7)
#define ParamDFA_az11o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az11o3Dpt7)))
// Ausgabewert Zustand 11 (DPT 8)
#define ParamDFA_az11o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az11o3Dpt8)))
// Ausgabewert Zustand 11 (DPT 9)
#define ParamDFA_az11o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 12)
#define ParamDFA_az11o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az11o3Dpt12)))
// Ausgabewert Zustand 11 (DPT 13)
#define ParamDFA_az11o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az11o3Dpt13)))
// Ausgabewert Zustand 11 (DPT 14)
#define ParamDFA_az11o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 17)
#define ParamDFA_az11o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o3Dpt17)))
// Ausgabewert Zustand 11 (DPT 232)
#define ParamDFA_az11o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az11o3Dpt232)) & DFA_az11o3Dpt232Mask) >> DFA_az11o3Dpt232Shift)
// Sendeverhalten Zustand 11
#define ParamDFA_az11o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Send)))
// Ausgabewert Zustand 11 (DPT 1)
#define ParamDFA_az11o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Dpt1)))
// Ausgabewert Zustand 11 (DPT 2)
#define ParamDFA_az11o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Dpt2)))
// Ausgabewert Zustand 11 (DPT 5)
#define ParamDFA_az11o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Dpt5)))
// Ausgabewert Zustand 11 (DPT 5.001)
#define ParamDFA_az11o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Dpt5001)))
// Ausgabewert Zustand 11 (DPT 6)
#define ParamDFA_az11o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Dpt6)))
// Ausgabewert Zustand 11 (DPT 7)
#define ParamDFA_az11o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az11o4Dpt7)))
// Ausgabewert Zustand 11 (DPT 8)
#define ParamDFA_az11o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az11o4Dpt8)))
// Ausgabewert Zustand 11 (DPT 9)
#define ParamDFA_az11o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 12)
#define ParamDFA_az11o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az11o4Dpt12)))
// Ausgabewert Zustand 11 (DPT 13)
#define ParamDFA_az11o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az11o4Dpt13)))
// Ausgabewert Zustand 11 (DPT 14)
#define ParamDFA_az11o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az11o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 11 (DPT 16)
#define ParamDFA_az11o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az11o4Dpt16)))
// Ausgabewert Zustand 11 (DPT 17)
#define ParamDFA_az11o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az11o4Dpt17)))
// Ausgabewert Zustand 11 (DPT 232)
#define ParamDFA_az11o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az11o4Dpt232)) & DFA_az11o4Dpt232Mask) >> DFA_az11o4Dpt232Shift)
// trans(12,1)
#define ParamDFA_ad12A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12A)))
// trans(12,2)
#define ParamDFA_ad12B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12B)))
// trans(12,3)
#define ParamDFA_ad12C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12C)))
// trans(12,4)
#define ParamDFA_ad12D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12D)))
// trans(12,5)
#define ParamDFA_ad12E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12E)))
// trans(12,6)
#define ParamDFA_ad12F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12F)))
// trans(12,7)
#define ParamDFA_ad12G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12G)))
// trans(12,8)
#define ParamDFA_ad12H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12H)))
// trans(12,timeout)
#define ParamDFA_ad12T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad12T)))
// Zeitbasis
#define ParamDFA_ad12TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad12TBase)) & DFA_ad12TBaseMask) >> DFA_ad12TBaseShift)
// Zeit
#define ParamDFA_ad12TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad12TTime)) & DFA_ad12TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad12TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad12TTime))))
// Sendeverhalten Zustand 12
#define ParamDFA_az12o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Send)))
// Ausgabewert Zustand 12 (DPT 1)
#define ParamDFA_az12o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Dpt1)))
// Ausgabewert Zustand 12 (DPT 2)
#define ParamDFA_az12o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Dpt2)))
// Ausgabewert Zustand 12 (DPT 5)
#define ParamDFA_az12o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Dpt5)))
// Ausgabewert Zustand 12 (DPT 5.001)
#define ParamDFA_az12o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Dpt5001)))
// Ausgabewert Zustand 12 (DPT 6)
#define ParamDFA_az12o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Dpt6)))
// Ausgabewert Zustand 12 (DPT 7)
#define ParamDFA_az12o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az12o1Dpt7)))
// Ausgabewert Zustand 12 (DPT 8)
#define ParamDFA_az12o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az12o1Dpt8)))
// Ausgabewert Zustand 12 (DPT 9)
#define ParamDFA_az12o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 12)
#define ParamDFA_az12o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az12o1Dpt12)))
// Ausgabewert Zustand 12 (DPT 13)
#define ParamDFA_az12o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az12o1Dpt13)))
// Ausgabewert Zustand 12 (DPT 14)
#define ParamDFA_az12o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 17)
#define ParamDFA_az12o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o1Dpt17)))
// Ausgabewert Zustand 12 (DPT 232)
#define ParamDFA_az12o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az12o1Dpt232)) & DFA_az12o1Dpt232Mask) >> DFA_az12o1Dpt232Shift)
// Sendeverhalten Zustand 12
#define ParamDFA_az12o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Send)))
// Ausgabewert Zustand 12 (DPT 1)
#define ParamDFA_az12o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Dpt1)))
// Ausgabewert Zustand 12 (DPT 2)
#define ParamDFA_az12o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Dpt2)))
// Ausgabewert Zustand 12 (DPT 5)
#define ParamDFA_az12o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Dpt5)))
// Ausgabewert Zustand 12 (DPT 5.001)
#define ParamDFA_az12o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Dpt5001)))
// Ausgabewert Zustand 12 (DPT 6)
#define ParamDFA_az12o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Dpt6)))
// Ausgabewert Zustand 12 (DPT 7)
#define ParamDFA_az12o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az12o2Dpt7)))
// Ausgabewert Zustand 12 (DPT 8)
#define ParamDFA_az12o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az12o2Dpt8)))
// Ausgabewert Zustand 12 (DPT 9)
#define ParamDFA_az12o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 12)
#define ParamDFA_az12o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az12o2Dpt12)))
// Ausgabewert Zustand 12 (DPT 13)
#define ParamDFA_az12o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az12o2Dpt13)))
// Ausgabewert Zustand 12 (DPT 14)
#define ParamDFA_az12o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 17)
#define ParamDFA_az12o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o2Dpt17)))
// Ausgabewert Zustand 12 (DPT 232)
#define ParamDFA_az12o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az12o2Dpt232)) & DFA_az12o2Dpt232Mask) >> DFA_az12o2Dpt232Shift)
// Sendeverhalten Zustand 12
#define ParamDFA_az12o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Send)))
// Ausgabewert Zustand 12 (DPT 1)
#define ParamDFA_az12o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Dpt1)))
// Ausgabewert Zustand 12 (DPT 2)
#define ParamDFA_az12o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Dpt2)))
// Ausgabewert Zustand 12 (DPT 5)
#define ParamDFA_az12o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Dpt5)))
// Ausgabewert Zustand 12 (DPT 5.001)
#define ParamDFA_az12o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Dpt5001)))
// Ausgabewert Zustand 12 (DPT 6)
#define ParamDFA_az12o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Dpt6)))
// Ausgabewert Zustand 12 (DPT 7)
#define ParamDFA_az12o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az12o3Dpt7)))
// Ausgabewert Zustand 12 (DPT 8)
#define ParamDFA_az12o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az12o3Dpt8)))
// Ausgabewert Zustand 12 (DPT 9)
#define ParamDFA_az12o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 12)
#define ParamDFA_az12o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az12o3Dpt12)))
// Ausgabewert Zustand 12 (DPT 13)
#define ParamDFA_az12o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az12o3Dpt13)))
// Ausgabewert Zustand 12 (DPT 14)
#define ParamDFA_az12o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 17)
#define ParamDFA_az12o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o3Dpt17)))
// Ausgabewert Zustand 12 (DPT 232)
#define ParamDFA_az12o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az12o3Dpt232)) & DFA_az12o3Dpt232Mask) >> DFA_az12o3Dpt232Shift)
// Sendeverhalten Zustand 12
#define ParamDFA_az12o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Send)))
// Ausgabewert Zustand 12 (DPT 1)
#define ParamDFA_az12o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Dpt1)))
// Ausgabewert Zustand 12 (DPT 2)
#define ParamDFA_az12o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Dpt2)))
// Ausgabewert Zustand 12 (DPT 5)
#define ParamDFA_az12o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Dpt5)))
// Ausgabewert Zustand 12 (DPT 5.001)
#define ParamDFA_az12o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Dpt5001)))
// Ausgabewert Zustand 12 (DPT 6)
#define ParamDFA_az12o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Dpt6)))
// Ausgabewert Zustand 12 (DPT 7)
#define ParamDFA_az12o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az12o4Dpt7)))
// Ausgabewert Zustand 12 (DPT 8)
#define ParamDFA_az12o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az12o4Dpt8)))
// Ausgabewert Zustand 12 (DPT 9)
#define ParamDFA_az12o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 12)
#define ParamDFA_az12o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az12o4Dpt12)))
// Ausgabewert Zustand 12 (DPT 13)
#define ParamDFA_az12o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az12o4Dpt13)))
// Ausgabewert Zustand 12 (DPT 14)
#define ParamDFA_az12o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az12o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 12 (DPT 16)
#define ParamDFA_az12o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az12o4Dpt16)))
// Ausgabewert Zustand 12 (DPT 17)
#define ParamDFA_az12o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az12o4Dpt17)))
// Ausgabewert Zustand 12 (DPT 232)
#define ParamDFA_az12o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az12o4Dpt232)) & DFA_az12o4Dpt232Mask) >> DFA_az12o4Dpt232Shift)
// trans(13,1)
#define ParamDFA_ad13A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13A)))
// trans(13,2)
#define ParamDFA_ad13B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13B)))
// trans(13,3)
#define ParamDFA_ad13C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13C)))
// trans(13,4)
#define ParamDFA_ad13D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13D)))
// trans(13,5)
#define ParamDFA_ad13E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13E)))
// trans(13,6)
#define ParamDFA_ad13F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13F)))
// trans(13,7)
#define ParamDFA_ad13G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13G)))
// trans(13,8)
#define ParamDFA_ad13H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13H)))
// trans(13,timeout)
#define ParamDFA_ad13T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad13T)))
// Zeitbasis
#define ParamDFA_ad13TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad13TBase)) & DFA_ad13TBaseMask) >> DFA_ad13TBaseShift)
// Zeit
#define ParamDFA_ad13TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad13TTime)) & DFA_ad13TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad13TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad13TTime))))
// Sendeverhalten Zustand 13
#define ParamDFA_az13o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Send)))
// Ausgabewert Zustand 13 (DPT 1)
#define ParamDFA_az13o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Dpt1)))
// Ausgabewert Zustand 13 (DPT 2)
#define ParamDFA_az13o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Dpt2)))
// Ausgabewert Zustand 13 (DPT 5)
#define ParamDFA_az13o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Dpt5)))
// Ausgabewert Zustand 13 (DPT 5.001)
#define ParamDFA_az13o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Dpt5001)))
// Ausgabewert Zustand 13 (DPT 6)
#define ParamDFA_az13o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Dpt6)))
// Ausgabewert Zustand 13 (DPT 7)
#define ParamDFA_az13o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az13o1Dpt7)))
// Ausgabewert Zustand 13 (DPT 8)
#define ParamDFA_az13o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az13o1Dpt8)))
// Ausgabewert Zustand 13 (DPT 9)
#define ParamDFA_az13o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 12)
#define ParamDFA_az13o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az13o1Dpt12)))
// Ausgabewert Zustand 13 (DPT 13)
#define ParamDFA_az13o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az13o1Dpt13)))
// Ausgabewert Zustand 13 (DPT 14)
#define ParamDFA_az13o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 17)
#define ParamDFA_az13o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o1Dpt17)))
// Ausgabewert Zustand 13 (DPT 232)
#define ParamDFA_az13o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az13o1Dpt232)) & DFA_az13o1Dpt232Mask) >> DFA_az13o1Dpt232Shift)
// Sendeverhalten Zustand 13
#define ParamDFA_az13o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Send)))
// Ausgabewert Zustand 13 (DPT 1)
#define ParamDFA_az13o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Dpt1)))
// Ausgabewert Zustand 13 (DPT 2)
#define ParamDFA_az13o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Dpt2)))
// Ausgabewert Zustand 13 (DPT 5)
#define ParamDFA_az13o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Dpt5)))
// Ausgabewert Zustand 13 (DPT 5.001)
#define ParamDFA_az13o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Dpt5001)))
// Ausgabewert Zustand 13 (DPT 6)
#define ParamDFA_az13o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Dpt6)))
// Ausgabewert Zustand 13 (DPT 7)
#define ParamDFA_az13o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az13o2Dpt7)))
// Ausgabewert Zustand 13 (DPT 8)
#define ParamDFA_az13o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az13o2Dpt8)))
// Ausgabewert Zustand 13 (DPT 9)
#define ParamDFA_az13o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 12)
#define ParamDFA_az13o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az13o2Dpt12)))
// Ausgabewert Zustand 13 (DPT 13)
#define ParamDFA_az13o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az13o2Dpt13)))
// Ausgabewert Zustand 13 (DPT 14)
#define ParamDFA_az13o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 17)
#define ParamDFA_az13o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o2Dpt17)))
// Ausgabewert Zustand 13 (DPT 232)
#define ParamDFA_az13o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az13o2Dpt232)) & DFA_az13o2Dpt232Mask) >> DFA_az13o2Dpt232Shift)
// Sendeverhalten Zustand 13
#define ParamDFA_az13o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Send)))
// Ausgabewert Zustand 13 (DPT 1)
#define ParamDFA_az13o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Dpt1)))
// Ausgabewert Zustand 13 (DPT 2)
#define ParamDFA_az13o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Dpt2)))
// Ausgabewert Zustand 13 (DPT 5)
#define ParamDFA_az13o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Dpt5)))
// Ausgabewert Zustand 13 (DPT 5.001)
#define ParamDFA_az13o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Dpt5001)))
// Ausgabewert Zustand 13 (DPT 6)
#define ParamDFA_az13o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Dpt6)))
// Ausgabewert Zustand 13 (DPT 7)
#define ParamDFA_az13o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az13o3Dpt7)))
// Ausgabewert Zustand 13 (DPT 8)
#define ParamDFA_az13o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az13o3Dpt8)))
// Ausgabewert Zustand 13 (DPT 9)
#define ParamDFA_az13o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 12)
#define ParamDFA_az13o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az13o3Dpt12)))
// Ausgabewert Zustand 13 (DPT 13)
#define ParamDFA_az13o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az13o3Dpt13)))
// Ausgabewert Zustand 13 (DPT 14)
#define ParamDFA_az13o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 17)
#define ParamDFA_az13o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o3Dpt17)))
// Ausgabewert Zustand 13 (DPT 232)
#define ParamDFA_az13o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az13o3Dpt232)) & DFA_az13o3Dpt232Mask) >> DFA_az13o3Dpt232Shift)
// Sendeverhalten Zustand 13
#define ParamDFA_az13o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Send)))
// Ausgabewert Zustand 13 (DPT 1)
#define ParamDFA_az13o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Dpt1)))
// Ausgabewert Zustand 13 (DPT 2)
#define ParamDFA_az13o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Dpt2)))
// Ausgabewert Zustand 13 (DPT 5)
#define ParamDFA_az13o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Dpt5)))
// Ausgabewert Zustand 13 (DPT 5.001)
#define ParamDFA_az13o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Dpt5001)))
// Ausgabewert Zustand 13 (DPT 6)
#define ParamDFA_az13o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Dpt6)))
// Ausgabewert Zustand 13 (DPT 7)
#define ParamDFA_az13o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az13o4Dpt7)))
// Ausgabewert Zustand 13 (DPT 8)
#define ParamDFA_az13o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az13o4Dpt8)))
// Ausgabewert Zustand 13 (DPT 9)
#define ParamDFA_az13o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 12)
#define ParamDFA_az13o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az13o4Dpt12)))
// Ausgabewert Zustand 13 (DPT 13)
#define ParamDFA_az13o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az13o4Dpt13)))
// Ausgabewert Zustand 13 (DPT 14)
#define ParamDFA_az13o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az13o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 13 (DPT 16)
#define ParamDFA_az13o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az13o4Dpt16)))
// Ausgabewert Zustand 13 (DPT 17)
#define ParamDFA_az13o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az13o4Dpt17)))
// Ausgabewert Zustand 13 (DPT 232)
#define ParamDFA_az13o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az13o4Dpt232)) & DFA_az13o4Dpt232Mask) >> DFA_az13o4Dpt232Shift)
// trans(14,1)
#define ParamDFA_ad14A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14A)))
// trans(14,2)
#define ParamDFA_ad14B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14B)))
// trans(14,3)
#define ParamDFA_ad14C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14C)))
// trans(14,4)
#define ParamDFA_ad14D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14D)))
// trans(14,5)
#define ParamDFA_ad14E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14E)))
// trans(14,6)
#define ParamDFA_ad14F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14F)))
// trans(14,7)
#define ParamDFA_ad14G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14G)))
// trans(14,8)
#define ParamDFA_ad14H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14H)))
// trans(14,timeout)
#define ParamDFA_ad14T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad14T)))
// Zeitbasis
#define ParamDFA_ad14TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad14TBase)) & DFA_ad14TBaseMask) >> DFA_ad14TBaseShift)
// Zeit
#define ParamDFA_ad14TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad14TTime)) & DFA_ad14TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad14TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad14TTime))))
// Sendeverhalten Zustand 14
#define ParamDFA_az14o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Send)))
// Ausgabewert Zustand 14 (DPT 1)
#define ParamDFA_az14o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Dpt1)))
// Ausgabewert Zustand 14 (DPT 2)
#define ParamDFA_az14o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Dpt2)))
// Ausgabewert Zustand 14 (DPT 5)
#define ParamDFA_az14o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Dpt5)))
// Ausgabewert Zustand 14 (DPT 5.001)
#define ParamDFA_az14o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Dpt5001)))
// Ausgabewert Zustand 14 (DPT 6)
#define ParamDFA_az14o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Dpt6)))
// Ausgabewert Zustand 14 (DPT 7)
#define ParamDFA_az14o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az14o1Dpt7)))
// Ausgabewert Zustand 14 (DPT 8)
#define ParamDFA_az14o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az14o1Dpt8)))
// Ausgabewert Zustand 14 (DPT 9)
#define ParamDFA_az14o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 12)
#define ParamDFA_az14o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az14o1Dpt12)))
// Ausgabewert Zustand 14 (DPT 13)
#define ParamDFA_az14o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az14o1Dpt13)))
// Ausgabewert Zustand 14 (DPT 14)
#define ParamDFA_az14o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 17)
#define ParamDFA_az14o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o1Dpt17)))
// Ausgabewert Zustand 14 (DPT 232)
#define ParamDFA_az14o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az14o1Dpt232)) & DFA_az14o1Dpt232Mask) >> DFA_az14o1Dpt232Shift)
// Sendeverhalten Zustand 14
#define ParamDFA_az14o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Send)))
// Ausgabewert Zustand 14 (DPT 1)
#define ParamDFA_az14o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Dpt1)))
// Ausgabewert Zustand 14 (DPT 2)
#define ParamDFA_az14o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Dpt2)))
// Ausgabewert Zustand 14 (DPT 5)
#define ParamDFA_az14o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Dpt5)))
// Ausgabewert Zustand 14 (DPT 5.001)
#define ParamDFA_az14o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Dpt5001)))
// Ausgabewert Zustand 14 (DPT 6)
#define ParamDFA_az14o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Dpt6)))
// Ausgabewert Zustand 14 (DPT 7)
#define ParamDFA_az14o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az14o2Dpt7)))
// Ausgabewert Zustand 14 (DPT 8)
#define ParamDFA_az14o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az14o2Dpt8)))
// Ausgabewert Zustand 14 (DPT 9)
#define ParamDFA_az14o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 12)
#define ParamDFA_az14o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az14o2Dpt12)))
// Ausgabewert Zustand 14 (DPT 13)
#define ParamDFA_az14o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az14o2Dpt13)))
// Ausgabewert Zustand 14 (DPT 14)
#define ParamDFA_az14o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 17)
#define ParamDFA_az14o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o2Dpt17)))
// Ausgabewert Zustand 14 (DPT 232)
#define ParamDFA_az14o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az14o2Dpt232)) & DFA_az14o2Dpt232Mask) >> DFA_az14o2Dpt232Shift)
// Sendeverhalten Zustand 14
#define ParamDFA_az14o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Send)))
// Ausgabewert Zustand 14 (DPT 1)
#define ParamDFA_az14o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Dpt1)))
// Ausgabewert Zustand 14 (DPT 2)
#define ParamDFA_az14o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Dpt2)))
// Ausgabewert Zustand 14 (DPT 5)
#define ParamDFA_az14o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Dpt5)))
// Ausgabewert Zustand 14 (DPT 5.001)
#define ParamDFA_az14o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Dpt5001)))
// Ausgabewert Zustand 14 (DPT 6)
#define ParamDFA_az14o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Dpt6)))
// Ausgabewert Zustand 14 (DPT 7)
#define ParamDFA_az14o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az14o3Dpt7)))
// Ausgabewert Zustand 14 (DPT 8)
#define ParamDFA_az14o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az14o3Dpt8)))
// Ausgabewert Zustand 14 (DPT 9)
#define ParamDFA_az14o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 12)
#define ParamDFA_az14o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az14o3Dpt12)))
// Ausgabewert Zustand 14 (DPT 13)
#define ParamDFA_az14o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az14o3Dpt13)))
// Ausgabewert Zustand 14 (DPT 14)
#define ParamDFA_az14o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 17)
#define ParamDFA_az14o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o3Dpt17)))
// Ausgabewert Zustand 14 (DPT 232)
#define ParamDFA_az14o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az14o3Dpt232)) & DFA_az14o3Dpt232Mask) >> DFA_az14o3Dpt232Shift)
// Sendeverhalten Zustand 14
#define ParamDFA_az14o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Send)))
// Ausgabewert Zustand 14 (DPT 1)
#define ParamDFA_az14o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Dpt1)))
// Ausgabewert Zustand 14 (DPT 2)
#define ParamDFA_az14o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Dpt2)))
// Ausgabewert Zustand 14 (DPT 5)
#define ParamDFA_az14o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Dpt5)))
// Ausgabewert Zustand 14 (DPT 5.001)
#define ParamDFA_az14o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Dpt5001)))
// Ausgabewert Zustand 14 (DPT 6)
#define ParamDFA_az14o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Dpt6)))
// Ausgabewert Zustand 14 (DPT 7)
#define ParamDFA_az14o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az14o4Dpt7)))
// Ausgabewert Zustand 14 (DPT 8)
#define ParamDFA_az14o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az14o4Dpt8)))
// Ausgabewert Zustand 14 (DPT 9)
#define ParamDFA_az14o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 12)
#define ParamDFA_az14o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az14o4Dpt12)))
// Ausgabewert Zustand 14 (DPT 13)
#define ParamDFA_az14o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az14o4Dpt13)))
// Ausgabewert Zustand 14 (DPT 14)
#define ParamDFA_az14o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az14o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 14 (DPT 16)
#define ParamDFA_az14o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az14o4Dpt16)))
// Ausgabewert Zustand 14 (DPT 17)
#define ParamDFA_az14o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az14o4Dpt17)))
// Ausgabewert Zustand 14 (DPT 232)
#define ParamDFA_az14o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az14o4Dpt232)) & DFA_az14o4Dpt232Mask) >> DFA_az14o4Dpt232Shift)
// trans(15,1)
#define ParamDFA_ad15A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15A)))
// trans(15,2)
#define ParamDFA_ad15B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15B)))
// trans(15,3)
#define ParamDFA_ad15C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15C)))
// trans(15,4)
#define ParamDFA_ad15D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15D)))
// trans(15,5)
#define ParamDFA_ad15E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15E)))
// trans(15,6)
#define ParamDFA_ad15F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15F)))
// trans(15,7)
#define ParamDFA_ad15G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15G)))
// trans(15,8)
#define ParamDFA_ad15H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15H)))
// trans(15,timeout)
#define ParamDFA_ad15T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad15T)))
// Zeitbasis
#define ParamDFA_ad15TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad15TBase)) & DFA_ad15TBaseMask) >> DFA_ad15TBaseShift)
// Zeit
#define ParamDFA_ad15TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad15TTime)) & DFA_ad15TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad15TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad15TTime))))
// Sendeverhalten Zustand 15
#define ParamDFA_az15o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Send)))
// Ausgabewert Zustand 15 (DPT 1)
#define ParamDFA_az15o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Dpt1)))
// Ausgabewert Zustand 15 (DPT 2)
#define ParamDFA_az15o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Dpt2)))
// Ausgabewert Zustand 15 (DPT 5)
#define ParamDFA_az15o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Dpt5)))
// Ausgabewert Zustand 15 (DPT 5.001)
#define ParamDFA_az15o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Dpt5001)))
// Ausgabewert Zustand 15 (DPT 6)
#define ParamDFA_az15o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Dpt6)))
// Ausgabewert Zustand 15 (DPT 7)
#define ParamDFA_az15o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az15o1Dpt7)))
// Ausgabewert Zustand 15 (DPT 8)
#define ParamDFA_az15o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az15o1Dpt8)))
// Ausgabewert Zustand 15 (DPT 9)
#define ParamDFA_az15o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 12)
#define ParamDFA_az15o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az15o1Dpt12)))
// Ausgabewert Zustand 15 (DPT 13)
#define ParamDFA_az15o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az15o1Dpt13)))
// Ausgabewert Zustand 15 (DPT 14)
#define ParamDFA_az15o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 17)
#define ParamDFA_az15o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o1Dpt17)))
// Ausgabewert Zustand 15 (DPT 232)
#define ParamDFA_az15o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az15o1Dpt232)) & DFA_az15o1Dpt232Mask) >> DFA_az15o1Dpt232Shift)
// Sendeverhalten Zustand 15
#define ParamDFA_az15o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Send)))
// Ausgabewert Zustand 15 (DPT 1)
#define ParamDFA_az15o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Dpt1)))
// Ausgabewert Zustand 15 (DPT 2)
#define ParamDFA_az15o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Dpt2)))
// Ausgabewert Zustand 15 (DPT 5)
#define ParamDFA_az15o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Dpt5)))
// Ausgabewert Zustand 15 (DPT 5.001)
#define ParamDFA_az15o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Dpt5001)))
// Ausgabewert Zustand 15 (DPT 6)
#define ParamDFA_az15o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Dpt6)))
// Ausgabewert Zustand 15 (DPT 7)
#define ParamDFA_az15o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az15o2Dpt7)))
// Ausgabewert Zustand 15 (DPT 8)
#define ParamDFA_az15o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az15o2Dpt8)))
// Ausgabewert Zustand 15 (DPT 9)
#define ParamDFA_az15o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 12)
#define ParamDFA_az15o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az15o2Dpt12)))
// Ausgabewert Zustand 15 (DPT 13)
#define ParamDFA_az15o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az15o2Dpt13)))
// Ausgabewert Zustand 15 (DPT 14)
#define ParamDFA_az15o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 17)
#define ParamDFA_az15o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o2Dpt17)))
// Ausgabewert Zustand 15 (DPT 232)
#define ParamDFA_az15o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az15o2Dpt232)) & DFA_az15o2Dpt232Mask) >> DFA_az15o2Dpt232Shift)
// Sendeverhalten Zustand 15
#define ParamDFA_az15o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Send)))
// Ausgabewert Zustand 15 (DPT 1)
#define ParamDFA_az15o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Dpt1)))
// Ausgabewert Zustand 15 (DPT 2)
#define ParamDFA_az15o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Dpt2)))
// Ausgabewert Zustand 15 (DPT 5)
#define ParamDFA_az15o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Dpt5)))
// Ausgabewert Zustand 15 (DPT 5.001)
#define ParamDFA_az15o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Dpt5001)))
// Ausgabewert Zustand 15 (DPT 6)
#define ParamDFA_az15o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Dpt6)))
// Ausgabewert Zustand 15 (DPT 7)
#define ParamDFA_az15o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az15o3Dpt7)))
// Ausgabewert Zustand 15 (DPT 8)
#define ParamDFA_az15o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az15o3Dpt8)))
// Ausgabewert Zustand 15 (DPT 9)
#define ParamDFA_az15o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 12)
#define ParamDFA_az15o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az15o3Dpt12)))
// Ausgabewert Zustand 15 (DPT 13)
#define ParamDFA_az15o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az15o3Dpt13)))
// Ausgabewert Zustand 15 (DPT 14)
#define ParamDFA_az15o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 17)
#define ParamDFA_az15o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o3Dpt17)))
// Ausgabewert Zustand 15 (DPT 232)
#define ParamDFA_az15o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az15o3Dpt232)) & DFA_az15o3Dpt232Mask) >> DFA_az15o3Dpt232Shift)
// Sendeverhalten Zustand 15
#define ParamDFA_az15o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Send)))
// Ausgabewert Zustand 15 (DPT 1)
#define ParamDFA_az15o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Dpt1)))
// Ausgabewert Zustand 15 (DPT 2)
#define ParamDFA_az15o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Dpt2)))
// Ausgabewert Zustand 15 (DPT 5)
#define ParamDFA_az15o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Dpt5)))
// Ausgabewert Zustand 15 (DPT 5.001)
#define ParamDFA_az15o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Dpt5001)))
// Ausgabewert Zustand 15 (DPT 6)
#define ParamDFA_az15o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Dpt6)))
// Ausgabewert Zustand 15 (DPT 7)
#define ParamDFA_az15o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az15o4Dpt7)))
// Ausgabewert Zustand 15 (DPT 8)
#define ParamDFA_az15o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az15o4Dpt8)))
// Ausgabewert Zustand 15 (DPT 9)
#define ParamDFA_az15o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 12)
#define ParamDFA_az15o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az15o4Dpt12)))
// Ausgabewert Zustand 15 (DPT 13)
#define ParamDFA_az15o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az15o4Dpt13)))
// Ausgabewert Zustand 15 (DPT 14)
#define ParamDFA_az15o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az15o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 15 (DPT 16)
#define ParamDFA_az15o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az15o4Dpt16)))
// Ausgabewert Zustand 15 (DPT 17)
#define ParamDFA_az15o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az15o4Dpt17)))
// Ausgabewert Zustand 15 (DPT 232)
#define ParamDFA_az15o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az15o4Dpt232)) & DFA_az15o4Dpt232Mask) >> DFA_az15o4Dpt232Shift)
// trans(16,1)
#define ParamDFA_ad16A                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16A)))
// trans(16,2)
#define ParamDFA_ad16B                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16B)))
// trans(16,3)
#define ParamDFA_ad16C                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16C)))
// trans(16,4)
#define ParamDFA_ad16D                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16D)))
// trans(16,5)
#define ParamDFA_ad16E                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16E)))
// trans(16,6)
#define ParamDFA_ad16F                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16F)))
// trans(16,7)
#define ParamDFA_ad16G                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16G)))
// trans(16,8)
#define ParamDFA_ad16H                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16H)))
// trans(16,timeout)
#define ParamDFA_ad16T                               (knx.paramByte(DFA_ParamCalcIndex(DFA_ad16T)))
// Zeitbasis
#define ParamDFA_ad16TBase                           ((knx.paramByte(DFA_ParamCalcIndex(DFA_ad16TBase)) & DFA_ad16TBaseMask) >> DFA_ad16TBaseShift)
// Zeit
#define ParamDFA_ad16TTime                           (knx.paramWord(DFA_ParamCalcIndex(DFA_ad16TTime)) & DFA_ad16TTimeMask)
// Zeit (in Millisekunden)
#define ParamDFA_ad16TTimeMS                         (paramDelay(knx.paramWord(DFA_ParamCalcIndex(DFA_ad16TTime))))
// Sendeverhalten Zustand 16
#define ParamDFA_az16o1Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Send)))
// Ausgabewert Zustand 16 (DPT 1)
#define ParamDFA_az16o1Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Dpt1)))
// Ausgabewert Zustand 16 (DPT 2)
#define ParamDFA_az16o1Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Dpt2)))
// Ausgabewert Zustand 16 (DPT 5)
#define ParamDFA_az16o1Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Dpt5)))
// Ausgabewert Zustand 16 (DPT 5.001)
#define ParamDFA_az16o1Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Dpt5001)))
// Ausgabewert Zustand 16 (DPT 6)
#define ParamDFA_az16o1Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Dpt6)))
// Ausgabewert Zustand 16 (DPT 7)
#define ParamDFA_az16o1Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az16o1Dpt7)))
// Ausgabewert Zustand 16 (DPT 8)
#define ParamDFA_az16o1Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az16o1Dpt8)))
// Ausgabewert Zustand 16 (DPT 9)
#define ParamDFA_az16o1Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o1Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 12)
#define ParamDFA_az16o1Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az16o1Dpt12)))
// Ausgabewert Zustand 16 (DPT 13)
#define ParamDFA_az16o1Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az16o1Dpt13)))
// Ausgabewert Zustand 16 (DPT 14)
#define ParamDFA_az16o1Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o1Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 17)
#define ParamDFA_az16o1Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o1Dpt17)))
// Ausgabewert Zustand 16 (DPT 232)
#define ParamDFA_az16o1Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az16o1Dpt232)) & DFA_az16o1Dpt232Mask) >> DFA_az16o1Dpt232Shift)
// Sendeverhalten Zustand 16
#define ParamDFA_az16o2Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Send)))
// Ausgabewert Zustand 16 (DPT 1)
#define ParamDFA_az16o2Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Dpt1)))
// Ausgabewert Zustand 16 (DPT 2)
#define ParamDFA_az16o2Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Dpt2)))
// Ausgabewert Zustand 16 (DPT 5)
#define ParamDFA_az16o2Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Dpt5)))
// Ausgabewert Zustand 16 (DPT 5.001)
#define ParamDFA_az16o2Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Dpt5001)))
// Ausgabewert Zustand 16 (DPT 6)
#define ParamDFA_az16o2Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Dpt6)))
// Ausgabewert Zustand 16 (DPT 7)
#define ParamDFA_az16o2Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az16o2Dpt7)))
// Ausgabewert Zustand 16 (DPT 8)
#define ParamDFA_az16o2Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az16o2Dpt8)))
// Ausgabewert Zustand 16 (DPT 9)
#define ParamDFA_az16o2Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o2Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 12)
#define ParamDFA_az16o2Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az16o2Dpt12)))
// Ausgabewert Zustand 16 (DPT 13)
#define ParamDFA_az16o2Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az16o2Dpt13)))
// Ausgabewert Zustand 16 (DPT 14)
#define ParamDFA_az16o2Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o2Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 17)
#define ParamDFA_az16o2Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o2Dpt17)))
// Ausgabewert Zustand 16 (DPT 232)
#define ParamDFA_az16o2Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az16o2Dpt232)) & DFA_az16o2Dpt232Mask) >> DFA_az16o2Dpt232Shift)
// Sendeverhalten Zustand 16
#define ParamDFA_az16o3Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Send)))
// Ausgabewert Zustand 16 (DPT 1)
#define ParamDFA_az16o3Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Dpt1)))
// Ausgabewert Zustand 16 (DPT 2)
#define ParamDFA_az16o3Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Dpt2)))
// Ausgabewert Zustand 16 (DPT 5)
#define ParamDFA_az16o3Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Dpt5)))
// Ausgabewert Zustand 16 (DPT 5.001)
#define ParamDFA_az16o3Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Dpt5001)))
// Ausgabewert Zustand 16 (DPT 6)
#define ParamDFA_az16o3Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Dpt6)))
// Ausgabewert Zustand 16 (DPT 7)
#define ParamDFA_az16o3Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az16o3Dpt7)))
// Ausgabewert Zustand 16 (DPT 8)
#define ParamDFA_az16o3Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az16o3Dpt8)))
// Ausgabewert Zustand 16 (DPT 9)
#define ParamDFA_az16o3Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o3Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 12)
#define ParamDFA_az16o3Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az16o3Dpt12)))
// Ausgabewert Zustand 16 (DPT 13)
#define ParamDFA_az16o3Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az16o3Dpt13)))
// Ausgabewert Zustand 16 (DPT 14)
#define ParamDFA_az16o3Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o3Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 17)
#define ParamDFA_az16o3Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o3Dpt17)))
// Ausgabewert Zustand 16 (DPT 232)
#define ParamDFA_az16o3Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az16o3Dpt232)) & DFA_az16o3Dpt232Mask) >> DFA_az16o3Dpt232Shift)
// Sendeverhalten Zustand 16
#define ParamDFA_az16o4Send                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Send)))
// Ausgabewert Zustand 16 (DPT 1)
#define ParamDFA_az16o4Dpt1                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Dpt1)))
// Ausgabewert Zustand 16 (DPT 2)
#define ParamDFA_az16o4Dpt2                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Dpt2)))
// Ausgabewert Zustand 16 (DPT 5)
#define ParamDFA_az16o4Dpt5                          (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Dpt5)))
// Ausgabewert Zustand 16 (DPT 5.001)
#define ParamDFA_az16o4Dpt5001                       (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Dpt5001)))
// Ausgabewert Zustand 16 (DPT 6)
#define ParamDFA_az16o4Dpt6                          ((int8_t)knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Dpt6)))
// Ausgabewert Zustand 16 (DPT 7)
#define ParamDFA_az16o4Dpt7                          (knx.paramWord(DFA_ParamCalcIndex(DFA_az16o4Dpt7)))
// Ausgabewert Zustand 16 (DPT 8)
#define ParamDFA_az16o4Dpt8                          ((int16_t)knx.paramWord(DFA_ParamCalcIndex(DFA_az16o4Dpt8)))
// Ausgabewert Zustand 16 (DPT 9)
#define ParamDFA_az16o4Dpt9                          (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o4Dpt9), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 12)
#define ParamDFA_az16o4Dpt12                         (knx.paramInt(DFA_ParamCalcIndex(DFA_az16o4Dpt12)))
// Ausgabewert Zustand 16 (DPT 13)
#define ParamDFA_az16o4Dpt13                         ((int32_t)knx.paramInt(DFA_ParamCalcIndex(DFA_az16o4Dpt13)))
// Ausgabewert Zustand 16 (DPT 14)
#define ParamDFA_az16o4Dpt14                         (knx.paramFloat(DFA_ParamCalcIndex(DFA_az16o4Dpt14), Float_Enc_IEEE754Single))
// Ausgabewert Zustand 16 (DPT 16)
#define ParamDFA_az16o4Dpt16                         (knx.paramData(DFA_ParamCalcIndex(DFA_az16o4Dpt16)))
// Ausgabewert Zustand 16 (DPT 17)
#define ParamDFA_az16o4Dpt17                         (knx.paramByte(DFA_ParamCalcIndex(DFA_az16o4Dpt17)))
// Ausgabewert Zustand 16 (DPT 232)
#define ParamDFA_az16o4Dpt232                        ((knx.paramInt(DFA_ParamCalcIndex(DFA_az16o4Dpt232)) & DFA_az16o4Dpt232Mask) >> DFA_az16o4Dpt232Shift)

// deprecated
#define DFA_KoOffset 720

// Communication objects per channel (multiple occurrence)
#define DFA_KoBlockOffset 720
#define DFA_KoBlockSize 30

#define DFA_KoCalcNumber(index) (index + DFA_KoBlockOffset + _channelIndex * DFA_KoBlockSize)
#define DFA_KoCalcIndex(number) ((number >= DFA_KoCalcNumber(0) && number < DFA_KoCalcNumber(DFA_KoBlockSize)) ? (number - DFA_KoBlockOffset) % DFA_KoBlockSize : -1)
#define DFA_KoCalcChannel(number) ((number >= DFA_KoBlockOffset && number < DFA_KoBlockOffset + DFA_ChannelCount * DFA_KoBlockSize) ? (number - DFA_KoBlockOffset) / DFA_KoBlockSize : -1)

#define DFA_KoKOaRunning 0
#define DFA_KoKOaRunSet 1
#define DFA_KoKOaState 2
#define DFA_KoKOaStateI 3
#define DFA_KoKOaInput1 11
#define DFA_KoKOaInput2 12
#define DFA_KoKOaInput3 13
#define DFA_KoKOaInput4 14
#define DFA_KoKOaInput5 15
#define DFA_KoKOaInput6 16
#define DFA_KoKOaInput7 17
#define DFA_KoKOaInput8 18
#define DFA_KoKOaInputT 19
#define DFA_KoKOaOutput1 21
#define DFA_KoKOaOutput2 22
#define DFA_KoKOaOutput3 23
#define DFA_KoKOaOutput4 24
#define DFA_KoKOaDummy14 29

// läuft?
#define KoDFA_KOaRunning                          (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaRunning)))
// starten/pausieren
#define KoDFA_KOaRunSet                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaRunSet)))
// Zustand
#define KoDFA_KOaState                            (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaState)))
// Zustand setzen
#define KoDFA_KOaStateI                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaStateI)))
// Eingang 1
#define KoDFA_KOaInput1                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput1)))
// Eingang 2
#define KoDFA_KOaInput2                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput2)))
// Eingang 3
#define KoDFA_KOaInput3                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput3)))
// Eingang 4
#define KoDFA_KOaInput4                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput4)))
// Eingang 5
#define KoDFA_KOaInput5                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput5)))
// Eingang 6
#define KoDFA_KOaInput6                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput6)))
// Eingang 7
#define KoDFA_KOaInput7                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput7)))
// Eingang 8
#define KoDFA_KOaInput8                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInput8)))
// Eingang T
#define KoDFA_KOaInputT                           (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaInputT)))
// Wertausgang 1
#define KoDFA_KOaOutput1                          (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaOutput1)))
// Wertausgang 2
#define KoDFA_KOaOutput2                          (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaOutput2)))
// Wertausgang 3
#define KoDFA_KOaOutput3                          (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaOutput3)))
// Wertausgang 4
#define KoDFA_KOaOutput4                          (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaOutput4)))
// Dummy14
#define KoDFA_KOaDummy14                          (knx.getGroupObject(DFA_KoCalcNumber(DFA_KoKOaDummy14)))



// Header generation for Module 'BASE_KommentarModule'

#define BASE_KommentarModuleCount 0
#define BASE_KommentarModuleModuleParamSize 0
#define BASE_KommentarModuleSubmodulesParamSize 0
#define BASE_KommentarModuleParamSize 0
#define BASE_KommentarModuleParamOffset 15569
#define BASE_KommentarModuleCalcIndex(index, m1) (index + BASE_KommentarModuleParamOffset + _channelIndex * BASE_KommentarModuleCount * BASE_KommentarModuleParamSize + m1 * BASE_KommentarModuleParamSize)



#ifdef MAIN_FirmwareRevision
#ifndef FIRMWARE_REVISION
#define FIRMWARE_REVISION MAIN_FirmwareRevision
#endif
#endif
#ifdef MAIN_FirmwareName
#ifndef FIRMWARE_NAME
#define FIRMWARE_NAME MAIN_FirmwareName
#endif
#endif
