/*
	Library for the Microchip MCP23017 I2C port expander
	(c)sstaub 2023
	Original written by Limor Fried/Ladyada for Adafruit Industries
	under BSD license
*/

#ifndef MCP23017_H
#define MCP23017_H

#include "Arduino.h"
#include "Wire.h"

#define MCP23X17_ADDRESS  0x20

// registers port A
#define MCP23X17_IODIRA   0x00
#define MCP23X17_IPOLA    0x02
#define MCP23X17_GPINTENA 0x04
#define MCP23X17_DEFVALA  0x06
#define MCP23X17_INTCONA  0x08
#define MCP23X17_IOCONA   0x0A
#define MCP23X17_GPPUA    0x0C
#define MCP23X17_INTFA    0x0E
#define MCP23X17_INTCAPA  0x10
#define MCP23X17_GPIOA    0x12
#define MCP23X17_OLATA    0x14

// registers port B
#define MCP23X17_IODIRB   0x01
#define MCP23X17_IPOLB    0x03
#define MCP23X17_GPINTENB 0x05
#define MCP23X17_DEFVALB  0x07
#define MCP23X17_INTCONB  0x09
#define MCP23X17_IOCONB   0x0B
#define MCP23X17_GPPUB    0x0D
#define MCP23X17_INTFB    0x0F
#define MCP23X17_INTCAPB  0x11
#define MCP23X17_GPIOB    0x13
#define MCP23X17_OLATB    0x15

#define MCP23X17_INT_ERR  0xFF

/**
 * @brief Class for Microchip MCP23017
 * 
 */
class MCP23017 {
public:
	MCP23017(const uint8_t deviceAddress, TwoWire *wire); 
	void begin();
	bool isConnected();

	/**
	 * @brief Sets the pin mode to either INPUT, INPUT_PULLUP or OUTPUT
	 * 
	 * @param pin
	 * @param mode
	 */
	void pinMode(uint8_t pin, uint8_t mode);

	/**
	 * @brief Write HIGH or LOW to a pin.
	 * 
	 * @param pin
	 * @param value HIGH or LOW
	 */
	void digitalWrite(uint8_t pin, uint8_t value);

	/**
	 * @brief Read a pin.
	 * 
	 * @param  pin
	 * @return value 1 for HIGH and 0 for LOW
	 */
	uint8_t digitalRead(uint8_t pin);

	/**
	 * @brief Configures the interrupt system. Both port A and B are assigned to the same configuration.
	 * 
	 * @param mirroring OR both INTA and INTB pins
	 * @param openDrain set the INT pin to value or open drain
	 * @param polarity  set LOW or HIGH on interrupt
	 */
	void interruptSetup(uint8_t mirroring, uint8_t open, uint8_t polarity);

	/**
	 * @brief Setup a pin for interrupt.
	 * 
	 * @param pin
	 * @param mode CHANGE, FALLING, RISING.
	 */
	void interruptPin(uint8_t pin, uint8_t mode);

	/**
	 * @brief Get the last interrupt pin.
	 * 
	 * @return pin
	 */
	uint8_t lastInterruptPin();

	/**
	 * @brief Get the value of the last interrupt pin.
	 * 
	 * @return value
	 */
	uint8_t lastInterruptPinValue();

private:
	TwoWire *_wire;
	uint8_t _address;
	uint8_t bitForPin(uint8_t pin);
	uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);
	uint8_t read(uint8_t addr);
	void write(uint8_t addr, uint8_t value);
	void update(uint8_t p, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr);
	};

#endif
