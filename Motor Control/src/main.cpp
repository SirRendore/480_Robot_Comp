#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Arduino.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
#define MOTOR_PIN_1 PA_6
#define MOTOR_PIN_2 PA_7
#define SENSOR_PIN PA5
#define P_CONTROL PB0
#define D_CONTROL PB1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void runMotor(int speed);

const int irDesired = 450;
int speed = 0;
int irValue = 0;
int p = 0;
int d = 0;
int error = 0;
int prevError = 0;
int p_value;
int d_value;

void setup()
{
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(P_CONTROL, INPUT_ANALOG);
  pinMode(D_CONTROL, INPUT_ANALOG);
  pinMode(SENSOR_PIN, INPUT_ANALOG);

  Serial.begin(9600);
}

void loop()
{
  p = analogRead(P_CONTROL);
  d = analogRead(D_CONTROL);

  irValue = analogRead(SENSOR_PIN);
  error = irDesired - irValue;

  p_value = p*error;
  d_value = d*(error - prevError);

  speed = (p_value + d_value)/1000;

  if (speed <= 250) {
    if (speed >100)
      speed = 250;
    else if(speed > 0)
      speed = 0;
  }
  if (speed >= -250){
    if (speed < -100)
      speed = -250;
    else if(speed < 0)
      speed = 0;
  }

  runMotor(speed);

  // Serial.println(speed);
  Serial.print(" P: ");
  Serial.print(p);
  Serial.print(" D: ");
  Serial.print(d);
  Serial.print(" IR: ");
  Serial.print(irValue);
  Serial.print(" Speed: ");
  Serial.println(speed);

  delay(5);
  prevError = error;
};

void runMotor(int speed){
  if (speed >= 0){
    pwm_start(MOTOR_PIN_1, 512, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_2, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else{
    pwm_start(MOTOR_PIN_2, 512, -speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_PIN_1, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
};