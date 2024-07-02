#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;

float acceleration = 200;
float maxSpeed = 99999999;

int left_step = 33;
int left_dir = 25;
int right_step = 26;
int right_dir = 27;

AccelStepper left(AccelStepper::DRIVER, left_step, left_dir);
AccelStepper right(AccelStepper::DRIVER, right_step, right_dir);

int servoLeftPin = 14;
int servoRightPin = 12;

float servoLeft_Offset = 0;
float servoRight_Offset = 0;

Servo servoLeft;
Servo servoRight;

float angle = 270;

void writeServo(float angle){
  servoLeft.write(angle+servoLeft_Offset);
  servoRight.write(270-angle+servoRight_Offset);
}

void writeStep(float var){
  left.setSpeed(var);
  right.setSpeed(-var);
  left.runSpeed();
  right.runSpeed();
}

void initStep(){
  left.setAcceleration(acceleration);
  right.setAcceleration(acceleration);
  left.setMaxSpeed(maxSpeed);
  right.setMaxSpeed(maxSpeed);
}

void initMpu6050(){
  if (!mpu.begin()){
    Serial.println("Failed find mpu6050");
  }
  Serial.println("MPU 6050 INIT ENd");
}

void initServo(){
  Serial.println("Start Servo Setting");
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);

  servoLeft.attach(servoLeftPin, 1000, 2000);
  servoRight.attach(servoRightPin, 1000, 2000);
  writeServo(angle);
  Serial.println("Servo Ready");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  left.setSpeed(300);
  initMpu6050();
  initServo();
  initStep();
}
sensors_event_t a, g, temp;
float accX, accY, accZ;
int d= 1;

long unsigned old_time = millis();

float Kp = 100; // PID제어 P 값
float Ki = 5; // PID제어 I 값
float Kd = 10; // PID제어 D 값

float pre_error;

void loop() {
  // put your main code here, to run repeatedly:
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  Serial.println(String(accX));
  Serial.println(String(accY));
  Serial.println(String(accZ));

  float error = 0-accY;
  int P = Kp * error;
  float delta_time = millis()-old_time;
  int I = Ki * error * delta_time;
  int D = Kd * (error - pre_error) / delta_time;
  int pid = P + I + D; 
  writeStep(pid);
  pre_error = error;

  writeServo(angle);
}
