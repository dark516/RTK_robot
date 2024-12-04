#pragma once

#define DT 0.01
#define MAX_DELTA 13 //Максимальная фактическая скорость

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

struct Encoder {
  int speed = 0;
  volatile long ticks = 0, prev_ticks = 0;
  byte pin_a, pin_b;
  
  Encoder (byte pin_enc_a, byte pin_enc_b, void(*on_enc)(void)) : pin_a(pin_enc_a), pin_b(pin_enc_b) {
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_a), on_enc, RISING);
  }

  void encoder_int() {
    if (digitalRead(pin_b)) ticks++;
    else ticks--;
  }

  int calc_delta() {
    speed = ticks - prev_ticks;
    prev_ticks = ticks; 
  }
  
};

struct Motor {
  byte dir, speed;
  
  Motor (byte pin_dir, byte pin_speed) : dir(pin_dir), speed(pin_speed) {
    pinMode(dir, OUTPUT);
    pinMode(speed, OUTPUT);
  }

  void set_pwmdir(int pwm_dir) {
    int pwm = constrain(abs(pwm_dir), 0, 255);
    analogWrite(speed, pwm);
    digitalWrite(dir, pwm_dir > 0);
  }
};

struct Regulator { 

  Motor motor;
  Encoder encoder;
  PID pid;

  long next = 0;
  int delta = 0;
  
  
  Regulator (Motor&& motor, Encoder&& encoder, PID&& pid) : motor(motor), encoder(encoder), pid(pid) {}

  void update() {
    next += delta;
    motor.set_pwmdir(pid.calc(next - encoder.ticks));
    encoder.calc_delta();
  }

  void set_delta(int new_delta) {
    delta = constrain(new_delta, -MAX_DELTA, MAX_DELTA);
    if(new_delta == 0) {
      next = encoder.ticks;
      motor.set_pwmdir(0);
    }
  }
    
};


struct LineSens {
  Octoliner octoliner;
  PID pid;
  
  LineSens (Octoliner&& octoliner, PID&& pid) : octoliner(octoliner), pid(pid) {}

  void init() {
    octoliner.begin();
    octoliner.setSensitivity(200);
  }
  int readSens(byte n) {
    return octoliner.analogRead(n);
  }
  int ans() {
    return pid.calc(octoliner.analogRead(2)-octoliner.analogRead(5));
  }
};
