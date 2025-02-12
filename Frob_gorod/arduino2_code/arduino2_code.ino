volatile long ltick = 0;
volatile long rtick = 0;
int base = 130;
int gray = 500;
int state = 1;
int counts[3] = {0, 0, 0};
long t0 = millis();

#define DT 0.01

struct PID {
  float kp, ki, kd, max_i;
  float integral = 0;
  int old_error = 0;

  PID (float p, float i, float d, float mi) : kp(p), ki(i), kd(d), max_i(mi) {}
  
  int calc(int error) {
    integral += error * DT * ki;
    integral = constrain(integral, -max_i, max_i);
    float diff = (error - old_error) * kd / DT;
    old_error = error;
    return error * kp + integral + diff;
  }
};

PID linePid(0.1, 0, 0.03, 100);
enum Commands : uint8_t {
  SET_STATE = 0x10,
  ANS = 0x11,
};
void lenc(){
  if (digitalRead(12)) ltick ++;
  else ltick --;
}
void renc(){
  if (digitalRead(13)) rtick --;
  else rtick ++;
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

void setup(){
  //Инициализация энкодеров
  pinMode(3, INPUT);
  pinMode(12, INPUT);
  pinMode(2, INPUT);
  pinMode(13, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), lenc, RISING); // Левый
  attachInterrupt(digitalPinToInterrupt(2), renc, RISING);

  //Инициализация моторов
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A0, INPUT);
  Serial.begin(115200);
  
}
void setMotors(int l = 0, int r = 0){
  digitalWrite(7, l > 0);
  digitalWrite(4, r > 0);
  analogWrite(6, constrain(abs(l), 0, 255));
  analogWrite(5, constrain(abs(r), 0, 255));
}
void zEnc(){
  ltick = 0;
  rtick = 0;
}
void stopEnc(int time = 500){ //Остановка по энкодерам
  long t = millis();
  zEnc();
  float pk = 12;
  while(millis() - t <= time){
    setMotors((0 - ltick)*pk, (0 - rtick)*pk);
  }
  setMotors();
}
int readLline(){
  //return analogRead(A2);
  return map(analogRead(A2), 460, 850, 200, 800);
}
int readRline(){
  //return analogRead(A1);
  return map(analogRead(A1), 650, 979, 200, 800);
}
int readRRline(){
  //return analogRead(A0);
  return map(analogRead(A0), 450, 979, 200, 800);
}
//void lineTick(int speed = base){
//  float pk = 0.2;
//  setMotors(speed - (readLline() - readRline()) * pk, speed + (readLline() - readRline()) * pk);
//}

//void lineTick(int speed = base){
//  float pk = 0.3;
//  float dk = 0.02;
//  setMotors(speed - (readLline() - readRline()) * pk, speed + (readLline() - readRline()) * pk);
//}
void lineTick(int speed = base){
  int U = linePid.calc(readRline() - readLline());
  setMotors(speed + U, speed - U);
}

void syncMove(int speed){ // Синхронизированная езда
  float pk = 7;
  setMotors(speed + (rtick - ltick) * pk, speed + (ltick - rtick) * pk);
}
void goLineMM(int mm){
  zEnc();
  float koef = 1000/460;
  while(ltick < mm*koef){
    lineTick();
  }
  stopEnc();
}

void goMM(int mm){
  zEnc();
  float koef = 1000/460;
  while(abs(ltick) < abs(mm*koef)){
    if (mm > 0) syncMove(base);
    else syncMove(-base);
  }
  stopEnc();
}
void turnAngle(int angle){
   zEnc();
   int ticks = (angle * 237) / 90;
   while (abs(ltick) < abs(ticks)){
    if (angle < 0) setMotors(-base, base); 
    else setMotors(base, -base);
   }
   stopEnc();
}

void viravn(int time = 500){
  long t = millis();
  while (millis() - t < time){
    lineTick(0);
  }
  stopEnc();
}
void turnLline(){
  zEnc();
  setMotors(-140, 140);
  delay(300);
  while ((readLline() < gray) and (abs(rtick) < 237) and (readRline() < gray)){}
  viravn(1000);
}
void turnRline(){
  zEnc();
  setMotors(140, -140);
  delay(300);
  while ((readRline() < gray) and (abs(ltick) < 237) and (readLline() < gray)){}
  viravn(1000);
}
void loop(){
//  Serial.print(readLline());
//  Serial.print("   ");
//  Serial.print(readRline());
//  Serial.print("   ");
//  Serial.println(readRRline());
//


//  command_spin();
//  if (state == 1) {
//    setMotors();
//    return;
//  }
//  lineTick();
//  if ((readRRline() > gray) and (millis() - t0 > 2000) and (readRline() > gray)) {
//    t0 = millis();
//    stopEnc(200);
//    goMM(115);
//    switch (findMostFrequentNumber()) {
//      case 0:  break; // Прямо
//      case 1:      // Блокирующий поврот налево  
//        turnLline(); 
//        stopEnc(1000);
//        break;
//      case 2:  // Блокирующий поврот направо
//        turnRline();
//        stopEnc(1000);
//        break;
//      }
//      resetCounts();
//  }

  command_spin();
  if (state == 1) {
    setMotors();
    return;
  }
  lineTick();
  if ((readRRline() > gray) and (millis() - t0 > 2000)) {
    t0 = millis();
    stopEnc(200);
    goMM(115);
    switch (findMostFrequentNumber()) {
      case 0:  break; // Прямо
      case 1:      // Блокирующий поврот налево  
        turnLline();
        viravn(500);
        break;
      case 2:  // Блокирующий поврот направо
        turnRline();
        viravn(500);
        break;
      }
      resetCounts();
  }
}
