#include "motor_regulator.h"
#include "ros2_communication.hpp"
#include "sensors.hpp"

#define PIN_TRIG 11
#define PIN_ECHO 12


#define BASE_SPEED 100
//int state = 3;

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

LineSensor lline(A3);
LineSensor rline(A2);
LineSensor mline(A1);
PID linePid(0.8, 0.0, 0.3, 100);

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  myservo.attach(9);
  oled.init();        // инициализация
  oled.clear();       // очистка
  oled.setScale(3);   // масштаб текста (1..4)
  oled.home();        // курсор в 0,0
  //oled.print(200);
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

void loop() {
  static unsigned long freq = millis();
  command_spin();
  
  if (millis() - freq >= 1000 * DT) {
    freq = millis();
    left_regulator.update(); //Не трогать.
    right_regulator.update(); //Не трогать.
  }

  if (state == 1) { //Если робот выключен
    left_regulator.motor.set_pwmdir(0);
    left_regulator.motor.set_pwmdir(0);
    return;
  } //Нужно написать 2 что бы стартануть робота или продолжить выполнение задания
  int ans = linePid.calc(lline.read() - rline.read());
  left_regulator.motor.set_pwmdir(BASE_SPEED+ans);
  right_regulator.motor.set_pwmdir(BASE_SPEED-ans);
  
}
