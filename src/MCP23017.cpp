#include "MCP23017.h"

MCP23017::MCP23017(const uint8_t deviceAddress, TwoWire *wire) {
	_address = deviceAddress;
	_wire = wire;
	}

void MCP23017::begin() {
	write(MCP23X17_IODIRA, 0xff);
	write(MCP23X17_IODIRB, 0xff);
	}

bool MCP23017::isConnected()
{
  _wire->beginTransmission(_address);
  return ( _wire->endTransmission() == 0);
}

void MCP23017::pinMode(uint8_t pin, uint8_t mode) {
	update(pin, !mode, MCP23X17_IODIRA, MCP23X17_IODIRB);
	update(pin, 0, MCP23X17_GPPUA, MCP23X17_GPPUB);
	if(mode == INPUT_PULLUP) {
		update(pin, 1, MCP23X17_IODIRA, MCP23X17_IODIRB);
		update(pin, 1, MCP23X17_GPPUA, MCP23X17_GPPUB);
		}
	}

void MCP23017::digitalWrite(uint8_t pin, uint8_t value) {
	uint8_t gpio;
	uint8_t bit = bitForPin(pin);
	uint8_t regAddr = regForPin(pin, MCP23X17_OLATA, MCP23X17_OLATB);
	gpio = read(regAddr);
	bitWrite(gpio, bit, value);
	regAddr = regForPin(pin, MCP23X17_GPIOA, MCP23X17_GPIOB);
	write(regAddr, gpio);
	}

uint8_t MCP23017::digitalRead(uint8_t pin) {
	uint8_t bit = bitForPin(pin);
	uint8_t regAddr = regForPin(pin, MCP23X17_GPIOA, MCP23X17_GPIOB);
	return (read(regAddr) >> bit) & 0x1;
	}

void MCP23017::interruptSetup(uint8_t mirroring, uint8_t openDrain, uint8_t polarity) {
	uint8_t ioconfValue = read(MCP23X17_IOCONA);
	bitWrite(ioconfValue, 6, mirroring);
	bitWrite(ioconfValue, 2, openDrain);
	bitWrite(ioconfValue, 1, polarity);
	write(MCP23X17_IOCONA, ioconfValue);

	ioconfValue=read(MCP23X17_IOCONB);
	bitWrite(ioconfValue, 6, mirroring);
	bitWrite(ioconfValue, 2, openDrain);
	bitWrite(ioconfValue, 1, polarity);
	write(MCP23X17_IOCONB, ioconfValue);
	}

void MCP23017::interruptPin(uint8_t pin, uint8_t mode) {
	// set the pin interrupt control (0 means change, 1 means compare against given value);
	// if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt
	update(pin, (mode != CHANGE), MCP23X17_INTCONA, MCP23X17_INTCONB);
	// In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
	// In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
	update(pin, (mode == FALLING), MCP23X17_DEFVALA, MCP23X17_DEFVALB);
	// enable the pin for interrupt
	update(pin, HIGH, MCP23X17_GPINTENA, MCP23X17_GPINTENB);
	}

uint8_t MCP23017::lastInterruptPin() {
	uint8_t intf;
	intf = read(MCP23X17_INTFA);
	for(uint8_t i = 0; i < 8; i++) if (bitRead(intf, i)) return i;
	intf = read(MCP23X17_INTFB);
	for(uint8_t i = 0; i < 8; i++) if (bitRead(intf, i)) return i+8;
	return MCP23X17_INT_ERR;
	}

uint8_t MCP23017::lastInterruptPinValue() {
	uint8_t intPin = lastInterruptPin();
	if(intPin != MCP23X17_INT_ERR) {
		uint8_t intcapreg = regForPin(intPin, MCP23X17_INTCAPA, MCP23X17_INTCAPB);
		uint8_t bit = bitForPin(intPin);
		return (read(intcapreg) >> bit) & (0x01);
		}
	return MCP23X17_INT_ERR;
	}

uint8_t MCP23017::bitForPin(uint8_t pin) {
	return pin % 8;
	}

uint8_t MCP23017::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
	return(pin < 8) ? portAaddr : portBaddr;
	}

uint8_t MCP23017::read(uint8_t addr) {
	_wire->beginTransmission(_address);
	_wire->write(addr);
	_wire->endTransmission();
	_wire->requestFrom(_address, 1);
	return _wire->read();
	}

void MCP23017::write(uint8_t regAddr, uint8_t regValue) {
	_wire->beginTransmission(_address);
	_wire->write(regAddr);
	_wire->write(regValue);
	_wire->endTransmission();
	}

void MCP23017::update(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
	uint8_t bit = bitForPin(pin);
	regValue = read(regAddr);
	bitWrite(regValue, bit, pValue);
	write(regAddr, regValue);
	}
