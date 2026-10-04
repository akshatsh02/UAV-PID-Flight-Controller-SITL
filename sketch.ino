#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;
Servo pitchServo;
Servo rollServo;

float filteredPitch = 0.0;
float filteredRoll = 0.0;
unsigned long lastTime = 0;

// PID Tuning Parameters
float Kp = 1.5;  // Proportional
float Ki = 0.05; // Integral
float Kd = 0.8;  // Derivative

float pitchErrorSum = 0.0;
float lastPitchError = 0.0;
float rollErrorSum = 0.0;
float lastRollError = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pitchServo.attach(18);
  rollServo.attach(19);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001; 
  lastTime = currentTime;

  float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accelRoll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float gyroPitchRate = g.gyro.x * 180.0 / PI; 
  float gyroRollRate = g.gyro.y * 180.0 / PI;

  filteredPitch = 0.98 * (filteredPitch + gyroPitchRate * dt) + 0.02 * accelPitch;
  filteredRoll = 0.98 * (filteredRoll + gyroRollRate * dt) + 0.02 * accelRoll;

  // PID Calculation: PITCH
  float pitchError = 0.0 - filteredPitch; 
  pitchErrorSum += (pitchError * dt);
  pitchErrorSum = constrain(pitchErrorSum, -50, 50); 
  float pitchDerivative = (pitchError - lastPitchError) / dt;
  float pitchPIDOutput = (Kp * pitchError) + (Ki * pitchErrorSum) + (Kd * pitchDerivative);
  lastPitchError = pitchError;

  // PID Calculation: ROLL
  float rollError = 0.0 - filteredRoll; 
  rollErrorSum += (rollError * dt);
  rollErrorSum = constrain(rollErrorSum, -50, 50);
  float rollDerivative = (rollError - lastRollError) / dt;
  float rollPIDOutput = (Kp * rollError) + (Ki * rollErrorSum) + (Kd * rollDerivative);
  lastRollError = rollError;

  int pitchServoAngle = constrain(90 + pitchPIDOutput, 0, 180);
  int rollServoAngle = constrain(90 + rollPIDOutput, 0, 180);

  pitchServo.write(pitchServoAngle);
  rollServo.write(rollServoAngle);

  Serial.print("Drone_Pitch:");
  Serial.print(filteredPitch);
  Serial.print(", PID_Servo_Correction:");
  Serial.println(pitchServoAngle - 90); 

  delay(20); 
}