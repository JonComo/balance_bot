#include <Wire.h>

// MPU6050
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t acX,acY,acZ,tmp,gyX,gyY,gyZ;
double aax,aay,aaz,at,agx,agy,agz;

// Motors
int in1 = 10;
int in2 = 5;
int in3 = 9;
int in4 = 11;

// Timing
long t = 0;

// PID
#include <PID_v1.h>
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, .5, 0, 0.0, DIRECT);

// Variables
float angle = 0;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);
  
  wakeUpMPU();
  
  delay(1000);
  
  Setpoint = 0; // forward is more negative
  for (int i = 0; i < 500; i ++) {
    t = millis();
    
    updateAngle();
    digitalWrite(13, i%10==0);
    while (millis() < t + 5);
  }
  
  digitalWrite(13, LOW);
  
  Setpoint = angle;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  
  //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  //TCCR0B = _BV(CS00);

  //Serial.begin(115200);
  //Serial.println("Hello Computer");
}

float ie = 0;
int last_output = 0;

void loop() {
  t = millis();
  updateAngle();
  //Serial.println(angle);
  
  Input = angle;
  //myPID.Compute(); // sets Output

  float error = (Setpoint - Input);

  Setpoint += (Setpoint - Input) * .2;
  
  ie += error * .05;
  ie *= 0.98;
  
  Output = error * 2.0;

  if (last_output > 0 && Output < 0) {
    ie = 0;
  } else if (last_output < 0 && Output > 0) {
    ie = 0;
  }
  
  last_output = Output;

  /*
  ie += Output * .0001;
  
  int lim = 100;
  if (ie > lim) {
    ie = lim;
  } else if (ie < -lim) {
    ie = -lim;
  }
  
  //Output += ie; */
  
  //float tilt = (acZ + 2600) / 10000.0 + gyY / 2000.0;
  //int mspeed = int(-35 * tilt * 255.0); */

  if (Output > 255) {
    Output = 255;
  } else if (Output < -255) {
    Output = -255;
  }
  
  if (abs(Input-Setpoint) > 9000) {
    motorA(0);
    motorB(0);
    ie = 0;
  } else {
    motorA(int(Output));
    motorB(int(Output));
  }
  
  while (millis() < t + 25);
}

void updateAngle() {
  getMPU();
  float f = 0.01;
  angle = ((1-f) * (angle + gyY * .025) + f * acZ);
  //angle *= 0.998;
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
  
  float spd = .4;
  // avg everything out
  aax += (acX-aax)*spd;
  aay += (acY-aay)*spd;
  aaz += (acZ-aaz)*spd;
  at += (tmp-at)*spd;
  agx += (gyX-agx)*spd;
  agy += (gyY-agy)*spd;
  agz += (gyZ-agz)*spd;
}

