#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#define Kp 10  // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 100 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define Ki 0


#define MaxSpeed 255   // max speed of the robot
#define BaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line

#define dir1_f 13
#define pwm1_f 12
#define dir2_f 26
#define pwm2_f 25
// #define dir1_b 26
// #define pwm1_b 25
// #define dir2_b 33
// #define pwm2_b 32


int P;
int I;
int D;
int rightMotorSpeed;
int leftMotorSpeed;
int lastError = 0; 


MPU6050 mpu6050(Wire);

long timer = 0;
long timer1 =0;
long timer2 =0;


void move(int motor, int Speed, int Direction) {
  boolean inPin1 = HIGH;

  if (Direction == 1) {
    inPin1 = HIGH;
  }
  if (Direction == 0) {
    inPin1 = LOW;
  }

  if (motor == 0) {
    digitalWrite(dir1_f, inPin1);
    analogWrite(pwm1_f, Speed);
    // digitalWrite(dir2_b, inPin1);
    // analogWrite(pwm2_b, Speed);
    if(millis() - timer1 > 1000){
    Serial.println(Speed);
    timer1 = millis();
    }
  }
  if (motor == 1) {
    digitalWrite(dir2_f, inPin1);
    analogWrite(pwm2_f, Speed);
    // digitalWrite(dir2_b, inPin1);
    // analogWrite(pwm2_b, Speed);
    if(millis() - timer2 > 1000){
    Serial.println(Speed);
    timer2 = millis();
  }

  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(pwm1_f, OUTPUT);
  pinMode(dir1_f, OUTPUT);
  pinMode(pwm2_f, OUTPUT);
  pinMode(dir2_f, OUTPUT);
}

void loop() {
  mpu6050.update();

  if(millis() - timer > 1000){
    
    // Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    // Serial.print("\taccX : ");Serial.print(mpu6050.getAccX());
    // Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    // Serial.print("\taccZ : ");Serial.print(mpu6050.getAccZ());
  
    // Serial.print("\tgyroX : ");Serial.print(mpu6050.getGyroX());
    // Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    // Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    // Serial.print("\taccAngleX : ");Serial.print(mpu6050.getAccAngleX());
    // Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    // Serial.print("\tgyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    // Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    // Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    // Serial.print("\tangleX : ");Serial.print(mpu6050.getAngleX());
    // Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.print("\n");
   // timer = millis();
   timer = millis();
  }

  int  error = mpu6050.getAngleZ() - 0;
  P = error;
  I = I + error;
  D = error - lastError;
  int motorSpeedDiff = Kp * P + Kd * D + Ki * I;
  lastError = error;

  rightMotorSpeed = BaseSpeed + motorSpeedDiff;
  leftMotorSpeed = BaseSpeed - motorSpeedDiff;
    
  

  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;

  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 0);
  
}