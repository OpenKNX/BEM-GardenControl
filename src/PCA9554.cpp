#include <Arduino.h>
#include <Wire.h>
#include "Pca9554.h"

#define PCA9554_ADDRESS 0x38

#define PCA9554_REG_INP 0
#define PCA9554_REG_OUT 1
#define PCA9554_REG_POL 2
#define PCA9554_REG_CTRL 3

uint8_t pinNum2bitNum[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};



/***************************************************************************
 *
 * Constructor for the Pca9554Class class, not much here yet
 *
 **************************************************************************/
PCA9554::PCA9554(const uint8_t deviceAddress, TwoWire *wire)
{
	_address = deviceAddress;
	_wire = wire;
}

/***************************************************************************
 *
 * Begin method. This method must be called before using this library
 * either directly if the class is initializing the Wire library or by
 * calling this library's function begin(sda, scl) in which case that
 * function will call this one.
 *
 **************************************************************************/
void PCA9554::begin(void)
{
	// Read out default values from the registers to the shadow variables.
	m_inp = readRegister( PCA9554_REG_INP);
	m_out = readRegister( PCA9554_REG_OUT);
	m_pol = readRegister( PCA9554_REG_POL);
	m_ctrl = readRegister( PCA9554_REG_CTRL);
}



bool PCA9554::isConnected()
{
  _wire->beginTransmission(_address);
  return ( _wire->endTransmission() == 0);
}

/***************************************************************************
 *
 * Sets the desired pin mode
 *
 **************************************************************************/
boolean PCA9554::pinMode(uint8_t pin, uint8_t mode)
{
	// Make sure the pin number is OK
	if (pin >= sizeof pinNum2bitNum)
	{
		return false;
	}

	// Calculate the new control register value
	if (mode == OUTPUT)
	{
		m_ctrl &= ~pinNum2bitNum[pin];
	}
	else if (mode == INPUT)
	{
		m_ctrl |= pinNum2bitNum[pin];
	}
	else
	{
		return false;
	}
	writeRegister( PCA9554_REG_CTRL, m_ctrl);

	return true;
}

/***************************************************************************
 *
 *  Writes 8-bits to the specified destination register
 *
 **************************************************************************/
void PCA9554::writeRegister( uint8_t reg, uint8_t value)
{
	_wire->beginTransmission(_address);
	_wire->write((uint8_t)reg);
	_wire->write((uint8_t)value);
	_wire->endTransmission();
}


/***************************************************************************
 *
 * Reads 8-bits from the specified source register
 *
 **************************************************************************/
uint16_t PCA9554::readRegister( uint8_t reg)
{
	_wire->beginTransmission(_address);
	_wire->write(reg);
	_wire->endTransmission();
	_wire->requestFrom(_address, (uint8_t)1);
	return _wire->read();
}

/***************************************************************************
 *
 * Sets the desired pin polarity. This can be used to invert inverse
 * hardware logic.
 *
 **************************************************************************/
boolean PCA9554::pinPolarity(uint8_t pin, uint8_t polarity)
{
	// Make sure pin number is OK
	if (pin >= sizeof pinNum2bitNum)
	{
		return false;
	}

	if (polarity == INVERTED)
	{
		m_pol |= pinNum2bitNum[pin];
	}
	else if (polarity == NORMAL)
	{
		m_pol &= ~pinNum2bitNum[pin];
	}
	else
	{
		return false;
	}

	writeRegister( PCA9554_REG_POL, m_pol);

	return true;
}

/***************************************************************************
 *
 * Write digital value to pin
 *
 **************************************************************************/
boolean PCA9554::digitalWrite(uint8_t pin, boolean val)
{
	// Make sure pin number is OK
	if (pin >= sizeof pinNum2bitNum)
	{
		return false;
	}

	if (val == HIGH)
	{
		m_out |= pinNum2bitNum[pin];
	}
	else
	{
		m_out &= ~pinNum2bitNum[pin];
	}

	writeRegister( PCA9554_REG_OUT, m_out);
	return true;
}

/***************************************************************************
 *
 * Read digital value from pin.
 * Note, so far this function will fail silently if the pin parameter is
 * incorrectly specified.
 *
 **************************************************************************/
boolean PCA9554::digitalRead(uint8_t pin)
{
	return ((readRegister(PCA9554_REG_INP) >> pin) & 1);
}

