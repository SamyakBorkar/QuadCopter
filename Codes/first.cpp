#include <Arduino.h>
#include <Servo.h>

#define JOYSTICK_PIN A0
#define ESC_PIN_1 2
#define ESC_PIN_2 3
#define ESC_PIN_3 4
#define ESC_PIN_4 5

Servo esc1, esc2, esc3, esc4;

void setup() {
  esc1.attach(ESC_PIN_1, 1000, 2000);
  esc2.attach(ESC_PIN_2, 1000, 2000);
  esc3.attach(ESC_PIN_3, 1000, 2000);
  esc4.attach(ESC_PIN_4, 1000, 2000);

  // Initialize all ESCs to 0 speed and upward thrust
  esc1.writeMicroseconds(1500); // Set midpoint for throttle
  esc2.writeMicroseconds(1500); // Set midpoint for throttle
  esc3.writeMicroseconds(1500); // Set midpoint for throttle
  esc4.writeMicroseconds(1500); // Set midpoint for throttle

  delay(2000); // Delay to allow ESCs to initialize
}

void loop() {
  int joystickValue = analogRead(JOYSTICK_PIN);
  joystickValue = constrain(joystickValue, 550, 1023); // Read upper half of joystick value from center.
  
  // Map joystick value to throttle signal
  int throttle = map(joystickValue, 550, 1023, 1000, 2000); // Range for ESC throttle
  
  // Write throttle signal to each ESC and specify motor rotation directions
  // Front-left motor (ESC_PIN_1) and rear-right motor (ESC_PIN_4) spin clockwise
  esc1.writeMicroseconds(throttle); // CW
  esc4.writeMicroseconds(throttle); // CW
  
  // Rear-left motor (ESC_PIN_3) and front-right motor (ESC_PIN_2) spin counter-clockwise
  esc3.writeMicroseconds(throttle); // CCW
  esc2.writeMicroseconds(throttle); // CCW
}
