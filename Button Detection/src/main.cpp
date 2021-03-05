#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Arduino.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
#define SWITCH_SENSOR PB5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void handle_interrupt();
void handle_interrupt_schmitt();
volatile int i = 0;
volatile int j = 0;

void setup()
{
  pinMode(SWITCH_SENSOR, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(SWITCH_SENSOR), handle_interrupt, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Displays "Hello world!" on the screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
}

void loop()
{
  j++;
  display.clearDisplay();
  display.setCursor(0, 0);

  digitalWrite(LED_BUILTIN, HIGH);
  display.print("Total Loops: ");
  display.println(j);
  display.print("Collisions: ");
  display.print(i);

  display.display();
};

void handle_interrupt()
{
  digitalWrite(LED_BUILTIN, LOW);
  i++;
}