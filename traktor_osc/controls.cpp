#include "controls.h"

void Pot::attach(uint8_t pin,  pinType type) {
  _pin = pin;
  _type = type;
}

void Pot::attach(uint8_t pin, I2cio* io, pinType type) {
  _io = io;
  attach(pin, type);
}

void Pot::update() {
  int value = 0;
  switch (_type) {
    case i2cioT:
      value = _io->analogRead(_pin);
      break;
    case mainT:
      value = analogRead(_pin);
      break;
  }
  float filteredVal = 0;
  if (value >= 0) {
    filteredVal = (float)value * 0.2 + (float)_lastValue * 0.8;
  }
  value = filteredVal;

  if (_sendedVal == _lastValue) {
    change = false;
  } else {
    change = true;
  }
  _lastValue = value;
}

float Pot::read() {
  _sendedVal = _lastValue;
  change = false;
  switch (_type) {
    case i2cioT:
      return 1.0 - (float)_lastValue / 4095.0;
      break;
    case mainT:
      return 1.0 - (float)_lastValue / 1023.0;
      break;
  }
}

bool Pot::changed() {
  return change;
}
