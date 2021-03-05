#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Arduino.h"
#include <Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
#define SERVO_PIN_1 PA5
// #define SERVO_PIN_2 PB7

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo servo1;
Servo servo2;
int pos = 0;

void setup()
{
  servo1.attach(SERVO_PIN_1);
  // servo2.attach(SERVO_PIN_2);
  Serial.begin(9600);
}
void loop()
{
  Serial.print("HELLO: ");
for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  } 
};
