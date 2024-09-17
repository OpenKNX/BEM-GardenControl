#include <Arduino.h>
#include <stdint.h>

#include "Device_setup.h"
#include "ErrorHandling.h"
#include "MCP23017.h"
#include "OpenKNX.h"
#include "PCA9554.h"
#include "PCA9555.h"
#include "SystemFailureHandling.h"

PCA9554 pca9554(i2cAddr_IO_Top_PCA9554, &Wire1); // Create an object at this address
PCA9555 pca9555(i2cAddr_IO_Bot_PCA9555, &Wire1); // Create an object at this address
MCP23017 mcp23017_TOP(i2cAddr_IO_Top_MCP23017, &Wire1);
MCP23017 mcp23017_BOT(i2cAddr_IO_Bot_MCP23017, &Wire1);

bool init_flag_PCA9555 = false;
bool init_flag_PCA9554 = false;

uint8_t failureCounter = 0;
uint8_t failureCounter2 = 0;

void init_IOExpander_GPIOs_TOP()
{
    // check if +5V iso is available
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID())
        {
            // HW with PCA9554
            case HW_1_0:
            case HW_2_0:
            case HW_2_1:
                SERIAL_PORT.print("  PCA9554: ");
                if (pca9554.isConnected())
                {
                    pca9554.begin();

                    switch (get_HW_ID())
                    {
                        case HW_1_0:

                            pca9554.digitalWrite(5, HIGH); // Set Volt div 1 = HIGH
                            pca9554.digitalWrite(6, HIGH); // Set Volt div 2 = HIGH
                            pca9554.digitalWrite(7, HIGH); // Set Volt div 3 = HIGH

                            pca9554.pinMode(0, OUTPUT); // ADC VCC 5V Output Enable
                            pca9554.pinMode(1, INPUT);  // HW_ID 1
                            pca9554.pinMode(2, INPUT);  // HW_ID 2
                            pca9554.pinMode(3, INPUT);  // HW_ID 3
                            pca9554.pinMode(4, INPUT);  // ADC VCC +5V Output Fault
                            pca9554.pinMode(5, OUTPUT); // Volt div 1
                            pca9554.pinMode(6, OUTPUT); // Volt div 2
                            pca9554.pinMode(7, OUTPUT); // Volt div 3
                            break;
                        case HW_2_0:
                        case HW_2_1:
                            pca9554.pinMode(0, OUTPUT); // ADC VCC 5V Output Enable
                            pca9554.pinMode(1, INPUT);  // HW_ID 1
                            pca9554.pinMode(2, INPUT);  // HW_ID 2
                            pca9554.pinMode(3, INPUT);  // HW_ID 3
                            pca9554.pinMode(4, INPUT);  // ADC VCC +5V Output Fault
                            pca9554.pinMode(5, INPUT);  // ADC VCC +12V Output Fault
                            pca9554.pinMode(6, OUTPUT); // HSS_SEL
                            pca9554.pinMode(7, OUTPUT); // +24V SW Enable
                            break;

                        default:
                            SERIAL_PORT.println(" wrong HW_ID Io Exp Init");
                            break;
                    }

                    SERIAL_PORT.print(pca9554.readRegister(Reg_input_Ports), BIN);

                    SERIAL_PORT.println(" OK");

                    init_flag_PCA9554 = true;
                }
                else
                {
                    SERIAL_PORT.println("NOK");
                }
                break;

            // HW with MCP230017
            case HW_3_0:
                SERIAL_PORT.print("  MCP230017_Top: ");

                if (mcp23017_TOP.isConnected())
                {
                    mcp23017_TOP.begin();

                    mcp23017_TOP.pinMode(0, INPUT_PULLUP); // HW-ID1
                    mcp23017_TOP.pinMode(1, INPUT_PULLUP); // HW-ID1
                    mcp23017_TOP.pinMode(2, INPUT_PULLUP); // HW-ID1
                    mcp23017_TOP.pinMode(3, OUTPUT);       // +5V_EN
                    mcp23017_TOP.pinMode(4, OUTPUT);       // +12V_EN
                    mcp23017_TOP.pinMode(5, OUTPUT);       // +24V_EN
                    mcp23017_TOP.pinMode(6, INPUT);       // +24V_1_EN (optional)
                    mcp23017_TOP.pinMode(7, INPUT);       // +24V_2_EN (optional)

                    mcp23017_TOP.pinMode(8, INPUT);  // HSS_Sense1
                    mcp23017_TOP.pinMode(9, INPUT);  // HSS_Sense2
                    mcp23017_TOP.pinMode(10, INPUT); // +5V_fault
                    mcp23017_TOP.pinMode(11, INPUT); // +12V_+24V_fault
                    mcp23017_TOP.pinMode(12, INPUT); // BGPIO5 (optional)
                    mcp23017_TOP.pinMode(13, INPUT); // BGPIO6 (optional)

                    SERIAL_PORT.println(" OK");
                    init_flag_PCA9554 = true;
                }
                else
                {
                    SERIAL_PORT.println("NOK");
                }
                //******************************************************************************************************************************************************* */
                break;

            default:
                SERIAL_PORT.println(" wrong HW_ID Io Exp Init");
                break;
        }
    }
    else
    {
        SERIAL_PORT.println("  PCA9554 ERROR: no +5V_Iso");
    }
}

void init_IOExpander_GPIOs_BOT()
{
    // check if +5V iso is available
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_BOT())
        {
            case HW_BOT_1_0:
            case HW_BOT_2_0:
            case HW_BOT_2_1:
                SERIAL_PORT.print("  PCA9555: ");
                if (pca9555.isConnected())
                {
                    SERIAL_PORT.println("OK");
                    pca9555.begin();
                    pca9555.digitalWriteAllToLow();
                    pca9555.pinModeAllOutputs();
                    init_flag_PCA9555 = true;
                }
                else
                {
                    SERIAL_PORT.println("NOK");
                }
                break;

            case HW_BOT_5_0:
                SERIAL_PORT.print("  MCP230017: ");
                if (mcp23017_BOT.isConnected())
                {
                    mcp23017_TOP.begin();

                    for (int i = 0; i < 16; i++)
                    {
                        mcp23017_BOT.digitalWrite(i, LOW);
                        delay(1);
                        mcp23017_BOT.pinMode(i, OUTPUT);
                    }
                    SERIAL_PORT.println("OK");
                    init_flag_PCA9555 = true;
                }
                else
                {
                    SERIAL_PORT.println("NOK");
                }
                break;
            default:
                SERIAL_PORT.println("Wrong HW-ID init_IOExpander_GPIOs_BOT");
        }
    }
    else
    {
        SERIAL_PORT.println("  IO-EXP BOT ERROR: no +5V_Iso");
    }
}

void clearInitFlags_IOExp()
{
    init_flag_PCA9554 = false;
    init_flag_PCA9555 = false;
}

void set_IOExpander_TOP_Output(uint8_t ch, bool state)
{
    // check if +5V iso is available
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_TOP())
        {
            case HW_1_0:
            case HW_2_0:
            case HW_2_1:
                if (init_flag_PCA9554)
                {
                    pca9554.digitalWrite(ch, state);
                }
                else
                {
                    init_IOExpander_GPIOs_TOP();
                    pca9554.digitalWrite(ch, state);
                }
                break;

            case HW_3_0:
                if (init_flag_PCA9554)
                {
                    mcp23017_TOP.digitalWrite(ch, state);
                    SERIAL_PORT.print("MCP23017_TOP.write ");
                    SERIAL_PORT.print(state);
                    SERIAL_PORT.print("  CH");
                    SERIAL_PORT.println(ch);
                }
                else
                {
                    init_IOExpander_GPIOs_TOP();
                    mcp23017_TOP.digitalWrite(ch, state);
                }
                break;
            default:
                SERIAL_PORT.println("Wrong HW-ID Get_IOexpander_Top_Input");
                break;
        }
    }
}

bool get_IOExpander_TOP_Input(uint8_t ch)
{
    // check if +5V iso is available
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_TOP())
        {
            case HW_1_0:
            case HW_2_0:
            case HW_2_1:
                if (init_flag_PCA9554)
                {
                    return pca9554.digitalRead(ch);
                }
                else
                {
                    init_IOExpander_GPIOs_TOP();
                    return pca9554.digitalRead(ch);
                }
                break;
            case HW_3_0:
                if (init_flag_PCA9554)
                {
                    return mcp23017_TOP.digitalRead(ch);
                }
                else
                {
                    init_IOExpander_GPIOs_TOP();
                    return mcp23017_TOP.digitalRead(ch);
                }
                break;
            default:
                SERIAL_PORT.println("Wrong HW-ID Get_IOexpander_Top_Input");
                return 0;
                break;
        }
    }
    else
    {
        return 0;
    }
}

bool get_IOExpander_BOT_Input(uint8_t ch)
{

    // check if +5V iso is available
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_BOT())
        {
            case HW_BOT_1_0:
            case HW_BOT_2_0:
            case HW_BOT_2_1:
                if (init_flag_PCA9555)
                {
                    return pca9555.digitalRead(ch);
                }
                else
                {
                    init_IOExpander_GPIOs_BOT();
                    return pca9555.digitalRead(ch);
                }
                break;

            case HW_BOT_5_0:
                if (init_flag_PCA9555)
                {
                    return mcp23017_BOT.digitalRead(ch);
                }
                else
                {
                    init_IOExpander_GPIOs_TOP();
                    return mcp23017_BOT.digitalRead(ch);
                }
                //**************************************************************************************************************************************************************** */
                break;
            default:
                SERIAL_PORT.println("Wrong HW-ID Get_IOexpander_BOT_INput");
                return 0;
                break;
        }
    }
    else
    {
        return 0;
    }
}

void set_IOExpander_BOT_Output(uint8_t ch, bool state)
{
    // check if +5V iso is available
    if (!get_24V_AC_Error())
    {
        switch (get_HW_ID_BOT())
        {
            case HW_BOT_1_0:
            case HW_BOT_2_0:
            case HW_BOT_2_1:
                if (init_flag_PCA9555)
                {
                    pca9555.digitalWrite(ch, state);
                }
                else
                {
                    init_IOExpander_GPIOs_BOT();
                    pca9555.digitalWrite(ch, state);
                }
                break;
            case HW_BOT_5_0:
                if (init_flag_PCA9555)
                {
                    mcp23017_BOT.digitalWrite(ch, state);
                    SERIAL_PORT.print("MCP23017_BOT.write ");
                    SERIAL_PORT.print(state);
                    SERIAL_PORT.print("  CH");
                    SERIAL_PORT.println(ch);
                    break;
                }
                else
                {
                    init_IOExpander_GPIOs_BOT();
                    mcp23017_BOT.digitalWrite(ch, state);
                    SERIAL_PORT.println("::::: Write");
                }

                break;
            default:
                SERIAL_PORT.println("Wrong HW-ID Set_IOexpander_BOT_Input");
                break;
        }
    }
}

bool getStatus_Ventil(uint8_t ch)
{
    if (ch <= BEM_ChannelCount && ch > 0)
    {
        return get_IOExpander_BOT_Input(ch);
    }
    {
        return 0;
    }
}

bool getStatus_Relais(uint8_t nr)
{
    if (nr <= REL_ChannelCount && nr > 0)
    {
        nr = nr + RelaisOffset;
        return get_IOExpander_BOT_Input(nr);
    }
    else
    {
        return 0;
    }
}

void enable_5V(bool state)
{
    switch (get_HW_ID_TOP())
    {
        case HW_1_0:
        case HW_2_0:
        case HW_2_1:
            set_IOExpander_TOP_Output(IO_5V_EN, state);
            break;
        case HW_3_0:
            set_IOExpander_TOP_Output(IO_5V_EN_V3, state);
            break;
        default:
            SERIAL_PORT.println("Wrong HW-ID enable 5V");
            break;
    }
}

void set_ADC1_VoltageDiff(bool state)
{
    set_IOExpander_TOP_Output(IO_Set_DIV_1, state);
}

void set_ADC2_VoltageDiff(bool state)
{
    set_IOExpander_TOP_Output(IO_Set_DIV_2, state);
}

void set_ADC3_VoltageDiff(bool state)
{
    set_IOExpander_TOP_Output(IO_Set_DIV_3, state);
}

bool getInitFlag_PCA9555()
{
    return init_flag_PCA9555;
}

bool getInitFlag_PCA9554()
{
    return init_flag_PCA9554;
}

// only TEST

void set_IOExpander_BOT_Output_PCA9555(uint8_t ch, bool state)
{
    if (!get_24V_AC_Error())
    {
        if (init_flag_PCA9555)
        {
            pca9555.digitalWrite(ch, state);
        }
        else
        {
            init_IOExpander_GPIOs_BOT();
            pca9555.digitalWrite(ch, state);
        }
    }
}