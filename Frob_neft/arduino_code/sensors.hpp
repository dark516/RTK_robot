#pragma once

struct LineSensor {
  byte pin;

  LineSensor(byte sensorPin) : pin(sensorPin) {
    pinMode(pin, INPUT);
  }
  int read() {
    return analogRead(pin);
  }
};
