#pragma once

#include <stdint.h>
#include <Arduino.h>

//Variant handling
//#define V1
//#define V2

#define KNXcallback
#define ADC_enable
//#define ADC_enable_Output
#define IOExp_enable
//#define IOExp_enable_Output
#define Opto_IN
//#define Opto_IN_Output
#define BinInputs
//#define BinInputs_Output
//#define S0Inputs
//#define S0Inputs_Output
#define ImplInput
//#define ImplInput_Output



#define ADC_Div_5V  0
#define ADC_Div_12V 1


// IO Expander TOP
#define IO_5V_EN     0
#define IO_HW_ID1    1
#define IO_HW_ID2    2
#define IO_HW_ID3    3
#define IO_5V_fault  4
#define IO_Set_DIV_1 5
#define IO_Set_DIV_2 6
#define IO_Set_DIV_3 7

// IO Expander BOT
#define Ventil_1  1
#define Ventil_2  2
#define Ventil_3  3
#define Ventil_4  4
#define Ventil_5  5
#define Ventil_6  6
#define Ventil_7  7
#define Ventil_8  8
#define Ventil_9  9
#define Ventil_10 10
#define Ventil_11 11
#define Ventil_12 12
#define Relais_1  12  
#define Relais_2  13
#define Relais_3  14 
#define BOT_5V_EN 15

#define RelaisOffset 11


#define SERIAL_PORT Serial   // Serial port for Arduino Zero

// MUst be updated to the new Project HW !!!
#define PROG_LED_PIN 24
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 13
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN 38 
#define LED_YELLOW_PIN 27  //nicht 22 weil es in V1 für GPIO_5V_EN verwendet wird, 27 frei gewählt weil in HW nicht belegt!
// Set i2c address
#define  i2cAddr_IO           0x27
#define  i2cAddr_IO_Bot       0x20
#define  i2cADC               0x68
#define  i2cADC_BOT           0x6E

// SSR Reilais
#define GPIO_SSR_EN 2

// Status Isolated 5V
#define GPIO_5V_status 8

// enable 5V
#define GPIO_5V_EN 22

// Inputs Optocoupler
#define OptoIN_1 12
#define OptoIN_2 11
#define OptoIN_3 10
#define OptoIN_4  9









