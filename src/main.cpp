#include "FileTransferModule.h"
#include "Logic.h"
#include "OpenKNX.h"

#ifdef ARDUINO_ARCH_RP2040
    #pragma message "Pico Core Version: " ARDUINO_PICO_VERSION_STR
#endif

void setup()
{
    // change this also in library.json
    const uint8_t firmwareRevision = 0;
    openknx.init(firmwareRevision);
    openknx.addModule(1, openknxLogic);
    openknx.addModule(2, openknxGardenControlModule);
    openknx.addModule(9, openknxFileTransferModule);

    SERIAL_DEBUG.println("Start init HW TOP");
    read_HW_ID_TOP();
    print_HW_ID_TOP(get_HW_ID_TOP());
    initHW();
    SERIAL_DEBUG.println("Done");

    Serial1.setRX(17); // UART0 KNX
    Serial1.setTX(16); // UART0

    SERIAL_DEBUG.println("Start KNX");

    // bool TestLEDstate = false;

    SERIAL_DEBUG.println("Start OpenKNX");
    // pin or GPIO the programming led is connected to. Default is LED_BUILDIN
    // knx.ledPin(PROG_LED_PIN);
    // // is the led active on HIGH or low? Default is LOW
    // knx.ledPinActiveOn(PROG_LED_PIN_ACTIVE_ON);
    // // pin or GPIO programming button is connected to. Default is 0
    // knx.buttonPin(get_PROG_BUTTON_PIN());
    // Is the interrupt created in RISING or FALLING signal? Default is RISING
    // knx.buttonPinInterruptOn(PROG_BUTTON_PIN_INTERRUPT_ON);
    openknx.setup();
}

void loop()
{
    openknx.loop();
}