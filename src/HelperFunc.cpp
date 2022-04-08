#include "HelperFunc.h"
#include <Arduino.h>
#include "GardenControl.h"


bool delayCheck(uint32_t iOldTimer, uint32_t iDuration)
{
    return millis() - iOldTimer >= iDuration;
}

uint16_t getParBEM(uint16_t PAR, uint8_t CH)
{
    return  BEM_ParamBlockOffset + (CH * BEM_ParamBlockSize) + PAR;
}

uint16_t getComBEM(uint16_t COM, uint8_t CH)
{
    return  BEM_KoOffset + (CH * BEM_KoBlockSize) + COM;
}

uint16_t getParREL(uint16_t PAR, uint8_t CH)
{
    return  REL_ParamBlockOffset + (CH * REL_ParamBlockSize) + PAR;
}

uint16_t getComREL(uint16_t COM, uint8_t CH)
{
    return  REL_KoOffset + (CH * REL_KoBlockSize) + COM;
}

uint16_t getParADC(uint16_t PAR, uint8_t CH)
{
    return  ADC_ParamBlockOffset + (CH * ADC_ParamBlockSize) + PAR;
}

uint16_t getComADC(uint16_t COM, uint8_t CH)
{
    return  ADC_KoOffset + (CH * ADC_KoBlockSize) + COM;
}
