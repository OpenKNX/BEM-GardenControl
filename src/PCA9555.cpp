#include <Arduino.h>
#include <Wire.h>
#include "PCA9555.h"

/**
 * @name PCA9555 constructor
 * @param address I2C address of the IO Expander
 * Creates the class interface and sets the I2C Address of the port
 */
PCA9555::PCA9555(uint8_t address, TwoWire *wire)
{
  _wire = wire;
  _address = address; // save the address id
  _valueRegister = 0;
}

// Checks if PCA9555 is responsive. Refer to Wire.endTransmission() from Arduino for details.
bool PCA9555::begin()
{
  _configurationRegister = 0xFFFF;
  _valueRegister = 0x0000; 

  _wire->beginTransmission(_address);
  _wire->write(0x02); // Test Address
  _error = _wire->endTransmission();

  if (_error != 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool PCA9555::isConnected()
{
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}

uint16_t PCA9555::getConfigReg()
{
  return _configurationRegister;
}

/**
 * @name pinMode
 * @param pin       pin number
 * @param IOMode    mode of pin INPUT or OUTPUT
 * sets the mode of this IO pin
 */
void PCA9555::pinMode(uint8_t pin, uint8_t IOMode)
{

  //
  // check if valid pin first
  //
  if (pin <= 15)
  {
    //
    // now set the correct bit in the configuration register
    //
    if (IOMode == OUTPUT)
    {
      //
      // mask correct bit to 0 by inverting x so that only
      // the correct bit is LOW. The rest stays HIGH
      //
      _configurationRegister = _configurationRegister & ~(1 << pin);
    }
    else
    {
      //
      // or just the required bit to 1
      //
      _configurationRegister = _configurationRegister | (1 << pin);
    }
    //
    // write configuration register to chip
    //
    I2CSetValue(_address, NXP_CONFIG, _configurationRegister_low);
    I2CSetValue(_address, NXP_CONFIG + 1, _configurationRegister_high);
  }
}
/**
 * @name pinMode
 * @param pin       pin number
 * @param IOMode    mode of pin INPUT or OUTPUT
 * sets the mode of this IO pin
 */
void PCA9555::pinModeAllOutputs()
{

  _configurationRegister = 0;

  I2CSetValue(_address, NXP_CONFIG, _configurationRegister_low);
  I2CSetValue(_address, NXP_CONFIG + 1, _configurationRegister_high);
}

void pinModeAllOutputs();
/**
 * @name digitalRead Reads the high/low value of specified pin
 * @param pin
 * @return value of pin
 * Reads the selected pin.
 */
uint8_t PCA9555::digitalRead(uint8_t pin)
{
  uint16_t _inputData = 0;
  //
  // we wil only process pins <= 15
  //
  if (pin > 15)
    return 255;
  _inputData = I2CGetValue(_address, NXP_INPUT);
  _inputData |= I2CGetValue(_address, NXP_INPUT + 1) << 8;
  //
  // now mask the bit required and see if it is a HIGH
  //
  if ((_inputData & (1 << pin)) > 0)
  {
    //
    // the bit is HIGH otherwise we would return a LOW value
    //
    return HIGH;
  }
  else
  {
    return LOW;
  }
}

void PCA9555::digitalWrite(uint8_t pin, uint8_t value)
{
  //
  // check valid pin first
  //
  if (pin > 15)
  {
    _error = 255; // invalid pin
    return;       // exit
  }
  //
  // if the value is LOW we will and the register value with correct bit set to zero
  // if the value is HIGH we will or the register value with correct bit set to HIGH
  //
  if (value > 0)
  {
    //
    // this is a High value so we will or it with the value register
    //
    _valueRegister = _valueRegister | (1 << pin); // and OR bit in register
  }
  else
  {
    //
    // this is a LOW value so we have to AND it with 0 into the _valueRegister
    //
    _valueRegister = _valueRegister & ~(1 << pin); // AND all bits
  }
  I2CSetValue(_address, NXP_OUTPUT, _valueRegister_low);
  I2CSetValue(_address, NXP_OUTPUT + 1, _valueRegister_high);
}

void PCA9555::digitalWriteAllToLow()
{

  _valueRegister = 0; // all bits 0

  I2CSetValue(_address, NXP_OUTPUT, _valueRegister_low);
  I2CSetValue(_address, NXP_OUTPUT + 1, _valueRegister_high);
}

// This is the actual ISR
// Stores states of all pins in _stateOfPins
void PCA9555::pinStates()
{
  _stateOfPins = I2CGetValue(_address, NXP_INPUT);
  _stateOfPins |= I2CGetValue(_address, NXP_INPUT + 1) << 8;
}

// Returns to user the state of desired pin
uint8_t PCA9555::stateOfPin(uint8_t pin)
{
  if ((_stateOfPins & (1 << pin)) > 0)
  {
    //
    // the bit is HIGH otherwise we would return a LOW value
    //
    return HIGH;
  }
  else
  {
    return LOW;
  }
}

/**
 * @name setClock modifies the clock frequency for I2C communication
 * @param clockFrequency
 * clockFrequency: the value (in Hertz) of desired communication clock.
 * The PCA9555 supports a 400kHz clock.
 * Accepted values are:
 *    10000 low speed mode, supported on some processors
 *    100000, standard mode
 *    400000, fast mode
 */
void PCA9555::setClock(uint32_t clockFrequency)
{
  _wire->setClock(clockFrequency);
}

//
// low level hardware methods
//

/**
 * @name I2CGetValue
 * @param address Address of I2C chip
 * @param reg    Register to read from
 * @return data in register
 * Reads the data from addressed chip at selected register. \n
 * If the value is above 255, an error is set. \n
 * error codes : \n
 * 256 = either 0 or more than one byte is received from the chip
 */
uint16_t PCA9555::I2CGetValue(uint8_t address, uint8_t reg)
{
  uint16_t _inputData;
  //
  // read the address input register
  //
  _wire->beginTransmission(address); // setup read registers
  _wire->write(reg);
  _error = _wire->endTransmission();
  //
  // ask for 2 bytes to be returned
  //
  if (_wire->requestFrom((int)address, 1) != 1)
  {
    //
    // we are not receing the bytes we need
    //
    return 256; // error code is above normal data range
  };
  //
  // read both bytes
  //
  _inputData = _wire->read();
  return _inputData;
}

/**
 * @name I2CSetValue(uint8_t address, uint8_t reg, uint8_t value)
 * @param address Address of I2C chip
 * @param reg    register to write to
 * @param value    value to write to register
 * Write the value given to the register set to selected chip.
 */
void PCA9555::I2CSetValue(uint8_t address, uint8_t reg, uint8_t value)
{
  //
  // write output register to chip
  //
  _wire->beginTransmission(address); // setup direction registers
  _wire->write(reg);                 // pointer to configuration register address 0
  _wire->write(value);               // write config register low byte
  _error = _wire->endTransmission();
}