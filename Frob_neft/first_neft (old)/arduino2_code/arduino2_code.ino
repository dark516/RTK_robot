#include <Wire.h>
#include <Octoliner.h>
#include "motor_regulator.h"
#include "ros2_communication.hpp"


#define PIN_TRIG 12
#define PIN_ECHO 11
long duration;

Motor left_motor(7, 6);
Motor right_motor(4, 5);

LineSens line(
  Octoliner(42),
  PID(0.4, 0.018, 0.007, 100) 
);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  line.init();
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

long readDist() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  duration = pulseIn(PIN_ECHO, HIGH);
  return (duration / 2) / 29.1;
  
}
int state = 1;
int black;
byte nStation = 0;
bool crossFlag = 1;
byte nCross = 0;
bool flagCow = 0;
#define BASE_SPEED 210

void loop() {
  if (Serial.available() >= 4) {  // Ждём, пока придёт 4 байта
    byte buffer[4];
    Serial.readBytes(buffer, 4);  // Читаем 4 байта
    state = *((int*)buffer);
  }
  //command_spin();
  if (state == 1) { //Если робот выключен
    black = (line.readSens(4) + line.readSens(3)) / 2 - 20;
    nStation = 0;
    crossFlag = 1;
    nCross = 0;
    return;
  }else if (state == 2) { //PAUSE
    left_motor.set_pwmdir(0);
    right_motor.set_pwmdir(0);
    return;
  } //Нужно написать 3 что бы стартануть робота или продолжить выполнение задания
  while (readDist() < 25) {
    if (!flagCow) {
      left_motor.set_pwmdir(-150);
      right_motor.set_pwmdir(-150);
      delay(50);
    }
    flagCow = 1;
    //delay(5); //Задержка проезда прямо
    left_motor.set_pwmdir(0);
    right_motor.set_pwmdir(0);
  }
  int ans = line.ans();
  left_motor.set_pwmdir(BASE_SPEED+ans);
  right_motor.set_pwmdir(BASE_SPEED-ans);
  flagCow = 0;
  if (line.readSens(0) > black) {
    nCross++;
    //Проезд на примерно пол корпуса вперед
    left_motor.set_pwmdir(150);
    right_motor.set_pwmdir(150);
    delay(160);
    left_motor.set_pwmdir(-150);
    right_motor.set_pwmdir(-150);
    delay(30); //Задержка проезда прямо
    left_motor.set_pwmdir(0);
    right_motor.set_pwmdir(0);

    
    if ((nCross == 2) or (nCross == 5)) delay(600);
    else delay(100);

    
    if (nCross == 3) {
      //Serial.println("Ловушка");  
    }else if (nCross == 6) {
      left_motor.set_pwmdir(-150);
      right_motor.set_pwmdir(-150);
      delay(10);
      left_motor.set_pwmdir(0);
      right_motor.set_pwmdir(0);
      state = 1;
      
    }else {
      //Serial.println("Поворот");
      while (line.readSens(1) > black - 30){
        left_motor.set_pwmdir(150);
        right_motor.set_pwmdir(-150);
      }
      while (line.readSens(1) < black){
        left_motor.set_pwmdir(150);
        right_motor.set_pwmdir(-150);
      }
      while (line.readSens(1) > black - 30){
        left_motor.set_pwmdir(150);
        right_motor.set_pwmdir(-150);
      }
    }
    left_motor.set_pwmdir(0);
    right_motor.set_pwmdir(0);
  }

}
