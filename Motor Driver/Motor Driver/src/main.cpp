#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Arduino.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
#define MOTOR_PIN_1A PA_6
#define MOTOR_PIN_1B PA_7
#define MOTOR_PIN_2A PB_1
#define MOTOR_PIN_2B PB_0
#define SPEED_CONTROL PA1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void runMotor(int speed);

double speed = 0;
int inc = 10;
void setup()
{
  pinMode(MOTOR_PIN_1A, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SPEED_CONTROL, INPUT_ANALOG);
  Serial.begin(9600);
}

void loop()
{
  double voltage = analogRead(SPEED_CONTROL) / 1024.0 * 3.3;
  speed = (voltage - 1.65) / 1.65 * 1024;
  // if (speed > 1024)
  //   inc = -10;
  // else if (speed < -1024)
  //   inc = 10;
  // speed = speed + inc;

  Serial.println(speed);
  runMotor(speed);

  delay(50);
};

int wheelBalance = 40;
void runMotor(int speed){
  if (speed >= 0){
    pwm_start(MOTOR_PIN_1A, 512, speed + wheelBalance, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_1B, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_2A, 512, speed - wheelBalance, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_2B, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else{
    pwm_start(MOTOR_PIN_1A, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_1B, 512, -speed - wheelBalance, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_2A, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_2B, 512, -speed + wheelBalance, RESOLUTION_10B_COMPARE_FORMAT);
  }
};