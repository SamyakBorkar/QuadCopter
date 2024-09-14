#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>


// Servo objects for each ESC
Servo esc1, esc2, esc3, esc4;

// Pin assignments for motors (ESC)
int escPin1 = 25, escPin2 = 26, escPin3 = 32, escPin4 = 33;

// MPU6050 object
MPU6050 mpu;

// Variables to store orientation data
double roll, pitch, yaw;
double rollAccel, pitchAccel;  // Accelerometer-based roll and pitch
double rollGyro, pitchGyro, yawGyro;  // Gyroscope-based roll, pitch, and yaw

// Complementary filter constants
double alpha = 0.98;  // Weighting factor for the complementary filter
double dt = 0.01;  // Loop time (in seconds)

// PID Variables for Roll
double rollInput, rollOutput, rollSetpoint;
double Kp = 1.5, Ki = 0.5, Kd = 0.1;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

// PID Variables for Pitch
double pitchInput, pitchOutput, pitchSetpoint;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

// PID Variables for Yaw (optional, depending on control complexity)
double yawInput, yawOutput, yawSetpoint;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);  // Initialize serial communication

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize ESCs
  esc1.attach(escPin1);
  esc2.attach(escPin2);
  esc3.attach(escPin3);
  esc4.attach(escPin4);
  
  // Set PID tunings
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-400, 400);  // Limit output to motor speed range
  rollPID.SetSampleTime(10);  // Sample every 10ms

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-400, 400);
  pitchPID.SetSampleTime(10);

  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-400, 400);
  yawPID.SetSampleTime(10);

  // Start motors at minimum throttle
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);  // Wait for ESCs to arm
}

void loop() {
  // Read MPU6050 Data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate accelerometer-based roll and pitch
  rollAccel = atan2(ay, az) * 180 / PI;
  pitchAccel = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Convert raw gyroscope values to degrees/sec
  rollGyro = gx / 131.0;  // Gyroscope sensitivity 131 LSB/(°/s)
  pitchGyro = gy / 131.0;
  yawGyro = gz / 131.0;

  // Complementary filter to combine accelerometer and gyroscope data
  roll = alpha * (roll + rollGyro * dt) + (1 - alpha) * rollAccel;
  pitch = alpha * (pitch + pitchGyro * dt) + (1 - alpha) * pitchAccel;
  yaw = yaw + yawGyro * dt;  // Yaw is only based on the gyroscope

  // Print filtered values to the serial monitor
  Serial.println("=== Filtered MPU6050 Readings ===");
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.println(yaw);

  // Set the target orientation
  rollSetpoint = 0;  // Set the target as 0 for level flight
  pitchSetpoint = 0;
  yawSetpoint = 0;

  // Update PID Inputs
  rollInput = roll;
  pitchInput = pitch;
  yawInput = yaw;

  // Compute PID outputs
  rollPID.Compute();
  pitchPID.Compute();
  yawPID.Compute();

  // Adjust motor speeds based on PID output
  int motor1Speed = 1000 + rollOutput + pitchOutput - yawOutput;
  int motor2Speed = 1000 - rollOutput + pitchOutput + yawOutput;
  int motor3Speed = 1000 - rollOutput - pitchOutput - yawOutput;
  int motor4Speed = 1000 + rollOutput - pitchOutput + yawOutput;

  // Constrain motor speeds to valid range
  motor1Speed = constrain(motor1Speed, 1000, 2000);
  motor2Speed = constrain(motor2Speed, 1000, 2000);
  motor3Speed = constrain(motor3Speed, 1000, 2000);
  motor4Speed = constrain(motor4Speed, 1000, 2000);

  // Send motor speeds to ESCs
  esc1.writeMicroseconds(motor1Speed);
  esc2.writeMicroseconds(motor2Speed);
  esc3.writeMicroseconds(motor3Speed);
  esc4.writeMicroseconds(motor4Speed);

  // Print motor speeds to the serial monitor
  Serial.print("Motor Speeds: ");
  Serial.print(motor1Speed); Serial.print(", ");
  Serial.print(motor2Speed); Serial.print(", ");
  Serial.print(motor3Speed); Serial.print(", ");
  Serial.println(motor4Speed);

  delay(10);  // 10ms delay for the complementary filter loop
}