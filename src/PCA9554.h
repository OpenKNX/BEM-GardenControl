#ifndef __GUARD_PCA9554_H__
#define __GUARD_PCA9554_H__

#include <Arduino.h>
#include <Wire.h>

#define NORMAL 0
#define INVERTED 1

class PCA9554
{
protected:
public:
	PCA9554(const uint8_t deviceAddress, TwoWire *wire);
	void begin(void);
	boolean pinMode(uint8_t pin, uint8_t mode);
	boolean pinPolarity(uint8_t pin, uint8_t polarity);
	boolean digitalWrite(uint8_t pin, boolean val);
	boolean digitalRead(uint8_t pin);
	void writeRegister( uint8_t reg, uint8_t value);
	uint16_t readRegister( uint8_t reg);
	bool    isConnected();

private:
	TwoWire *_wire;
	uint8_t _address;

	uint8_t m_inp;
	uint8_t m_out;
	uint8_t m_pol;
	uint8_t m_ctrl;
};


#endif // ifndef __GUARD_PCA9554_H__