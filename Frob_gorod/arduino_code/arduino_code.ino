#include "motor_regulator.h"
#include "ros2_communication.hpp"
#include "sensors.hpp"

#define BASE_SPEED 100

int gray = 560;
//ЛЕВЫЙ МОТОР
void __left_motor_enc(); //Заголовок функции
//Создание экземпляра левого регулятора
Regulator left_regulator(
  Motor(7, 6),
  Encoder(3, 12, __left_motor_enc, false),
  PID(1.1, 0.01, 0.005, 100) 
);

void __left_motor_enc() {
  left_regulator.encoder.encoder_int();
}

//ПРАВЫЙ МОТОР
void __right_motor_enc(); //Заголовок функции
//Создание экземпляра правого регулятора
Regulator right_regulator(
  Motor(4, 5),
  Encoder(2, 13, __right_motor_enc, true),
  PID(1.1, 0.01, 0.005, 100) 
);

void __right_motor_enc() {
  right_regulator.encoder.encoder_int();
}

LineSensor lline(A2);
LineSensor rline(A1);
PID linePid(0.4, 0.0, 0.0, 100);

void setup() {
  Serial.begin(115200);
}

#define PT(x) Serial.print(x); Serial.print('\t')

void stop() {
  for (int i=0, s=255; i < 16; i++, s=-s){
    left_regulator.motor.set_pwmdir(s);
    right_regulator.motor.set_pwmdir(s);
    delay(5);
  }
  left_regulator.motor.set_pwmdir(0);
  right_regulator.motor.set_pwmdir(0);
}

int findMostFrequentNumber() {
  int mostFrequent = 0;
  int maxCount = counts[0];

  for (int i = 1; i < 3; i++) {
    if (counts[i] > maxCount) {
      maxCount = counts[i];
      mostFrequent = i;
    }
  }
  return mostFrequent;
}

void resetCounts() {
  for (int i = 0; i < 3; i++) {
    counts[i] = 0; // Сбрасываем счетчики
  }
}

void loop() {
  static unsigned long freq = millis();
  command_spin();
//  Serial.print(lline.read());
//  Serial.print("      ");
//  Serial.print(rline.read());
//  Serial.println();
//
  if (state == 1) { //Если робот выключен
    left_regulator.motor.set_pwmdir(0);
    right_regulator.motor.set_pwmdir(0);
//      left_regulator.motor.set_pwmdir(BASE_SPEED);
//      right_regulator.motor.set_pwmdir(BASE_SPEED);
//      delay(850);
//      stop();
//      left_regulator.motor.set_pwmdir(100);
//      right_regulator.motor.set_pwmdir(-100);
//      delay(340);
//      while (lline.read() > gray){
//        left_regulator.motor.set_pwmdir(100);
//        right_regulator.motor.set_pwmdir(-100);
//      }
//      stop();
//      exit(0);
    return;
  }
  
  int U = linePid.calc(lline.read() - rline.read());
  left_regulator.motor.set_pwmdir(BASE_SPEED+U);
  right_regulator.motor.set_pwmdir(BASE_SPEED-U);

  if (rline.read() < gray) {
    stop();
    delay(50);
    left_regulator.motor.set_pwmdir(BASE_SPEED);
    right_regulator.motor.set_pwmdir(BASE_SPEED);
    delay(850);
    stop();
    switch (findMostFrequentNumber()) {
    case 0:  break; // Прямо
    case 1:      // Блокирующий поврот налево  
      left_regulator.motor.set_pwmdir(-100);
      right_regulator.motor.set_pwmdir(100);
      delay(420);
      while (rline.read() > gray){
        left_regulator.motor.set_pwmdir(-100);
        right_regulator.motor.set_pwmdir(100);
      }
      stop(); 
      break;
    case 2:  // Блокирующий поврот направо
      left_regulator.motor.set_pwmdir(100);
      right_regulator.motor.set_pwmdir(-100);
      delay(420);
      while (lline.read() > gray){
        left_regulator.motor.set_pwmdir(100);
        right_regulator.motor.set_pwmdir(-100);
      }
      stop();
      break;
    }
    resetCounts();
  }
}
