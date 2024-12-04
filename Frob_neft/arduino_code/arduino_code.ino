#include "motor_regulator.h"
#include "ros2_communication.hpp"
#include "sensors.hpp"

#define PIN_TRIG 11
#define PIN_ECHO 12


#define BASE_SPEED 175
bool crossFlag = 1;
byte nCross = 0;
int gray;


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
PID linePid(0.6, 0.001, 0.05, 100);

void setup() {
  //left_regulator.motor.set_pwmdir(255);
  //right_regulator.set_delta(1);
  //turnAngle(90);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
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
////
//  Serial.print(lline.read());
//  Serial.print("      ");
//  Serial.print(rline.read());
//  Serial.print("      ");
//  Serial.print(mline.read());
//  Serial.println();
//
//
  if (state == 1) { //Если робот выключен
    gray = mline.read() - min(rline.read(), lline.read()) / 2;
    crossFlag = 1;
    nCross = 0;
    return;
  }else if (state == 2) { //PAUSE
    stop();
    return;
  } //Нужно написать 3 что бы стартануть робота или продолжить выполнение задания
  //gray = mline.read() - max(rline.read(), lline.read()) / 2;
  int ans = linePid.calc(lline.read() - rline.read());
  left_regulator.motor.set_pwmdir(BASE_SPEED+ans);
  right_regulator.motor.set_pwmdir(BASE_SPEED-ans);
  if (mline.read() < 200) {
    nCross++;
    //Проезд на примерно пол корпуса вперед
    left_regulator.motor.set_pwmdir(150);
    right_regulator.motor.set_pwmdir(150);
    delay(280);
    stop();
    left_regulator.motor.set_pwmdir(0);
    right_regulator.motor.set_pwmdir(0);

    
    if ((nCross == 2) or (nCross == 5)) delay(100);
    else delay(100);

    if (nCross == 3) {
      left_regulator.motor.set_pwmdir(-150);
      right_regulator.motor.set_pwmdir(150);
      delay(260);
      while (rline.read() > 200){
        left_regulator.motor.set_pwmdir(-150);
        right_regulator.motor.set_pwmdir(150);
      }
      delay(260);
      while (rline.read() > 200){
        left_regulator.motor.set_pwmdir(-150);
        right_regulator.motor.set_pwmdir(150);
      }
      stop();
      delay(500);
      left_regulator.motor.set_pwmdir(-150);
      right_regulator.motor.set_pwmdir(150);
      delay(260);
      while (rline.read() > 200){
        left_regulator.motor.set_pwmdir(-150);
        right_regulator.motor.set_pwmdir(150);
      }
      stop();
      //Serial.println("Ловушка");  
    }else if (nCross == 6) {
      stop();
      state = 1;
      
    }else {
//        turnTicks = turnAngle(-90, 140);
//        while (abs(left_regulator.encoder.ticks - turn_lStart_ticks) < abs(turnTicks)) {}
//        stop();
      left_regulator.motor.set_pwmdir(-150);
      right_regulator.motor.set_pwmdir(150);
      delay(260);
      while (rline.read() > 200){
        left_regulator.motor.set_pwmdir(-150);
        right_regulator.motor.set_pwmdir(150);
      }
      stop();
      delay(100);
    }
  }
}
