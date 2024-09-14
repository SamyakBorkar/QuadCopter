#include <ESP32Servo.h>  // Use ESP32Servo instead of the standard Servo library
#include<Arduino.h>
// Create Servo objects for each ESC
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// Pin assignments for ESP32
int receiverPin1 = 13;  // Pin connected to the receiver's throttle channel for Motor 1 (CH1)
int receiverPin2 = 12;  // Pin connected to the receiver's throttle channel for Motor 2 (CH2)
int receiverPin3 = 14;  // Pin connected to the receiver's throttle channel for Motor 3 (CH3)
int receiverPin4 = 27;  // Pin connected to the receiver's throttle channel for Motor 4 (CH4)

int escPin1 = 25;  // Pin connected to the ESC signal for Motor 1
int escPin2 = 26;  // Pin connected to the ESC signal for Motor 2
int escPin3 = 32;  // Pin connected to the ESC signal for Motor 3
int escPin4 = 33;  // Pin connected to the ESC signal for Motor 4

// Function to calibrate the ESC
void calibrateESC(Servo &esc, const String &motorName) {
  Serial.print(motorName);
  Serial.println(" Calibration Mode");
  
  // Full throttle calibration
  Serial.print("Set full throttle to calibrate ");
  Serial.println(motorName);
  esc.writeMicroseconds(2000);  // Set full throttle (max)
  delay(3000);  // Wait for 3 seconds (ESC beeps will confirm full throttle)
  
  // Minimum throttle calibration
  Serial.print("Set minimum throttle to calibrate ");
  Serial.println(motorName);
  esc.writeMicroseconds(1000);  // Set minimum throttle (min)
  delay(3000);  // Wait for 3 seconds (ESC beeps will confirm min throttle)
  
  Serial.print(motorName);
  Serial.println(" Calibration Completed.");
  esc.writeMicroseconds(1000);  // Arm the ESC by setting throttle to minimum
  delay(2000);  // Wait for the ESC to arm
}

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
  
  // Set receiver pins as input
  pinMode(receiverPin1, INPUT);
  pinMode(receiverPin2, INPUT);
  pinMode(receiverPin3, INPUT);
  pinMode(receiverPin4, INPUT);
  
  // Attach the ESCs to their respective pins
  esc1.attach(escPin1);
  esc2.attach(escPin2);
  esc3.attach(escPin3);
  esc4.attach(escPin4);

  // Wait for user to input 'c' in the Serial Monitor to start calibration for each ESC
  Serial.println("Enter 'c' in the Serial Monitor to calibrate all ESCs, or wait to proceed.");
  long startTime = millis();
  while (millis() - startTime < 10000) {  // Wait for 10 seconds for user input
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 'c') {
        calibrateESC(esc1, "Motor 1");
        calibrateESC(esc2, "Motor 2");
        calibrateESC(esc3, "Motor 3");
        calibrateESC(esc4, "Motor 4");
        break;
      }
    }
  }
  
  Serial.println("Ready to control the motors with the transmitter.");
  esc1.writeMicroseconds(1000);  // Start with minimum throttle for all ESCs
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);  // Wait for ESCs to arm
}

void loop() {
  // Read the PWM signals from the receiver for each motor
  int throttleInput1 = pulseIn(receiverPin1, HIGH);  
  int throttleInput2 = pulseIn(receiverPin2, HIGH);
  int throttleInput3 = pulseIn(receiverPin3, HIGH);
  int throttleInput4 = pulseIn(receiverPin4, HIGH);
  
  // Print the raw throttle inputs to the Serial Monitor
  Serial.print("Throttle Input Motor 1: ");
  Serial.println(throttleInput1);
  Serial.print("Throttle Input Motor 2: ");
  Serial.println(throttleInput2);
  Serial.print("Throttle Input Motor 3: ");
  Serial.println(throttleInput3);
  Serial.print("Throttle Input Motor 4: ");
  Serial.println(throttleInput4);

  // Map the throttle inputs (1000-2000 microseconds) to the ESC range
  int escSpeed1 = map(throttleInput1, 1000, 2000, 1000, 2000);  
  int escSpeed2 = map(throttleInput2, 1000, 2000, 1000, 2000);  
  int escSpeed3 = map(throttleInput3, 1000, 2000, 1000, 2000);  
  int escSpeed4 = map(throttleInput4, 1000, 2000, 1000, 2000);  

  // Print the mapped ESC speeds to the Serial Monitor
  Serial.print("ESC Speed Motor 1: ");
  Serial.println(escSpeed1);
  Serial.print("ESC Speed Motor 2: ");
  Serial.println(escSpeed2);
  Serial.print("ESC Speed Motor 3: ");
  Serial.println(escSpeed3);
  Serial.print("ESC Speed Motor 4: ");
  Serial.println(escSpeed4);

  // Set the ESC speeds based on the throttle inputs
  esc1.writeMicroseconds(escSpeed1);
  esc2.writeMicroseconds(escSpeed2);
  esc3.writeMicroseconds(escSpeed3);
  esc4.writeMicroseconds(escSpeed4);

  delay(100);  // Add delay to avoid flooding the Serial Monitor
}