#include <Adafruit_MPU6050.h>
#include <Arduino.h>
Adafruit_MPU6050 mpu;

void initMpu6050(){
  if (!mpu.begin()){
    Serial.println("Failed find mpu6050");
    while (1){ delay(10); }
  }
  Serial.println("MPU 6050 INIT ENd");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initMpu6050();
}
sensors_event_t a, g, temp;
float accX, accY, accZ;

void loop() {
  // put your main code here, to run repeatedly:
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  Serial.println(String(accX));
  Serial.println(String(accY));
  Serial.println(String(accZ));
  delay(100);
}
