#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <Stepper.h>
#include <ESP32Servo.h>
Adafruit_MPU6050 mpu;

const int stepsPerRevolution = 200;

Stepper left(stepsPerRevolution, 4,0,2,15);//15, 2, 0, 4);

int servoLeftPin = 14;
int servoRightPin = 12;

float servoLeft_Offset = -28;
float servoRight_Offset = -80;

Servo servoLeft;
Servo servoRight;

float angle = 90;

void writeServo(float angle){
  servoLeft.write(angle+servoLeft_Offset);
  servoRight.write(270-angle+servoRight_Offset);
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
}
sensors_event_t a, g, temp;
float accX, accY, accZ;
int d= 1;
void loop() {
  // put your main code here, to run repeatedly:
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  Serial.println(String(accX));
  Serial.println(String(accY));
  Serial.println(String(accZ));
  left.step(stepsPerRevolution*d);
  delay(100);
}
