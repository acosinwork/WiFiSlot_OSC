#ifndef CONTROLS_H
#define CONTROLS_H

#include <Arduino.h>
#include <stdint.h>
#include <I2cio.h>

enum pinType {i2cioT, mainT};

class Pot
{
private:
	uint8_t _pin;
	pinType _type;
	uint16_t _lastValue = 0;
	uint16_t _sendedVal = 0;
	
	I2cio* _io;
	bool change = true;

public:
	void attach(uint8_t pin, pinType type = mainT);
	void attach(uint8_t pin, I2cio* io, pinType type = i2cioT);
	bool changed();
	void update();
	float read();
};

#endif //CONTROLS_H