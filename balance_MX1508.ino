#include <Wire.h>

// MPU6050
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t acX,acY,acZ,tmp,gyX,gyY,gyZ;

float angle = 0;
float error_slow = 0;
float error_slower = 0;

// Motor Outputs (PWM)
int in1 = 10;
int in2 = 5;
int in3 = 9;
int in4 = 11;

// Timing
long t = 0;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  wakeUpMPU();
  
  error_slow = 0;
  error_slower = 0;
}

void loop() {
  t = millis();

  getMPU();
  angle = angle + gyY * .005; // if you want to include acceleration ((1-f) * (angle + gyY * .005) + f * acZ);
  
  float error = angle + error_slow + error_slower;

  error_slow += error * .04; // learn better set point short run
  error_slower += error * .001; // learn better set point long run
  
  error_slow *= 0.99;
  
  float Output = error * .2;
  
  if (Output > 255) {
    Output = 255;
  } else if (Output < -255) {
    Output = -255;
  }
  
  if (abs(angle) > 9000) {
    motorA(0);
    motorB(0);
    error_slow = 0;
    error_slower = 0;
    delay(5000);
  } else {
    motorA(int(Output));
    motorB(int(Output));
  }
  
  while (millis() <= t + 5);
}

void motorA(int spd) {
  if (spd < 0) {
    analogWrite(in2,0);
    analogWrite(in1,-spd);
  } else {
    analogWrite(in2,spd);
    analogWrite(in1,0);
  }
}

void motorB(int spd) {
  if (spd < 0) {
    analogWrite(in4,0);
    analogWrite(in3,-spd);
  } else {
    analogWrite(in4,spd);
    analogWrite(in3,0);
  }
}

void wakeUpMPU() {
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void getMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  acX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  acY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true);
  tmp = tmp/340.00+36.53; // to degrees C from datasheet
}

