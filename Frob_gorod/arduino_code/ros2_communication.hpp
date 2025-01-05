#pragma once
#include "motor_regulator.h"

int state = 3;

int counts[3] = {0, 0, 0};

extern Regulator left_regulator;
extern Regulator right_regulator;
//Команды с ROS ноды
enum Commands : uint8_t {
  SET_STATE = 0x10,
  ANS = 0x11,
};

//template <typename T> void serial_read(T& dest) {
//  wait_bytes(sizeof());
//  Serial.readBytes((uint8_t* ) &dest, sizeof(T));
//}
//
//template <typename T> void serial_send(T& src) {
//  Serial.write((uint8_t *) &src, sizeof(T));
//}

void wait_bytes(uint16_t b) {
  while (Serial.available() < b) {
    ; // Ожидание прибытия всего пакета
  }
}

byte getState(){
  wait_bytes(sizeof(int8_t));
  int8_t st = Serial.read();
  return st;
}

void getAns(){
  wait_bytes(sizeof(int8_t));
  int8_t ans = Serial.read();
  counts[ans]++;
}
void command_spin(){
  if (Serial.available() > 1) {
    uint8_t code = Serial.read();
    switch (code) {
    case SET_STATE: state = getState(); break;
    case ANS: getAns(); break;
    }
  }
}
