#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Arduino.h"

double max(double arr[], int arr_length);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
#define IR_SENSOR PA4

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const double freq = 10000;
const double period = 1 / freq;
const int size = 1000;
const double dt = period / size;
double reference1k[size];
const int measuredSize = 100;
double measured[measuredSize];

volatile int loopCounter = 0;

void setup()
{
  pinMode(IR_SENSOR, INPUT_ANALOG);
  pinMode(LED_BUILTIN, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  for (int x = 0; x < size; x++)
  {
    reference1k[x] = sin(2 * PI * freq * x * dt);
  }
  Serial.begin(9600);
}

void loop()
{
  double voltage = analogRead(IR_SENSOR) / 1024.0 * 3.3;
  measured[loopCounter] = voltage;

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("Voltage: ");
  display.println(voltage);
  display.println(loopCounter);

  display.display();

  loopCounter++;

  if (loopCounter >= measuredSize)
  {
    double convolved[measuredSize + size - 2]; // Size of convolved array
    int start = -measuredSize + 1;
    double value;
    for (int x = 0; x < measuredSize + size - 2; x++)
    {
      value = 0;
      for (int y = start; y < start + measuredSize; y++)
      {
        if (y >= 0 && y < size)
          value += measured[y - start] * reference1k[y];
      }
      convolved[x] = value;
      start++;
    }
    Serial.println(max(convolved, measuredSize + size - 2));
    loopCounter = 0;
  }
}

double max(double arr[], int arr_length)
{
  double max = arr[0];
  for (int x = 0; x < arr_length; x++)
  {
    if (arr[x] > max)
      max = arr[x];
  }
  return max;
}
double sampling_dt(){
  i
  while(true){

  }
}