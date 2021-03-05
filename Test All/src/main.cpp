#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Arduino.h"
#include <Servo.h>
#include <NewPing.h>
#include "pitches.h"

// OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
// DC Motors
#define DC_MOTOR_L1 PA_6
#define DC_MOTOR_L2 PA_7
#define DC_MOTOR_R1 PB_1
#define DC_MOTOR_R2 PB_0
// Servos
#define SWEEP_SERVO PB8
#define ARM_SERVO PB9
#define GATE_SERVO PA8
// Sonar
#define SONAR_ECHO PB10
#define SONAR_TRIGGER PB11
#define MAX_DISTANCE 100 // Max detection distance for sonar, in cm
// Tape Sensors
#define TAPE_LEFT PA5
#define TAPE_RIGHT PA2
#define TAPE_MIDDLE PA3
// Rear button
#define REAR_BUTTON PB5
// Turnpot
#define TURNPOT PA1
#define SPEAKER PB4
// Other constants

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
NewPing sonar(SONAR_TRIGGER, SONAR_ECHO, MAX_DISTANCE);
void runMotors(int speed, char ch = 'b'); // "l" left, "r" right, "b" both
void pickup();
void search();
void calibrateTape();
void tapeFollow(bool drive = true);
void sweepAll();
int getDistance();
void spinSearch();
bool checkRearButton();
void buttonInterrupt();
void controlGate(char ch); // 'o' = open, 'c' = closed
void deposit();
void readTape();
void cycleMode(int newMode = 0);
void entertain();
void oledDisplay();
void playSong(int melody[], int songLength);
void buttonInterrupt();

// Servos
Servo armServo;
Servo gateServo;
Servo sweepServo;

void setup()
{
  pinMode(DC_MOTOR_L1, OUTPUT);
  pinMode(DC_MOTOR_L2, OUTPUT);
  pinMode(DC_MOTOR_R1, OUTPUT);
  pinMode(DC_MOTOR_R2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TURNPOT, INPUT_ANALOG);
  pinMode(REAR_BUTTON, INPUT_PULLUP);

  // Tape sensors
  pinMode(TAPE_LEFT, INPUT_ANALOG);
  pinMode(TAPE_MIDDLE, INPUT_ANALOG);
  pinMode(TAPE_RIGHT, INPUT_ANALOG);

  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // calibrateTape();
  // while(true){
  //   entertain();
  // }
}

// Global variables
// DC Motors
const int wheelBalanceForwards = 36; // >0 if left is slower, <0 if right
const int wheelBalanceBackwards = 0; // >0 if left is slower, <0 if right

// 1 - idle, 2 - search, 3 - return, 4 - deposit, 5 - entertainment
int mode = 1;
unsigned long start_time = 0; // Competition start time
const int maxReturnTime = 36000;
int returnTime = maxReturnTime;  // Return after this number of seconds

// Display
int displayFreq = 50; // Update display every 50 loops

// Search for cans
const int searchSpeed = 340;
int distance = 0;
bool turning;
int numCans = 0;
const int maxNumCans = 100;

const int turnInterval = 1600; // Time between turns
const int spinInterval = 1400; // Time between spins
const int spinDuration = 3000; // Spin duration
const int spinSpeed = 360;
const bool startRight = false; // true if start with right turn, false if start with left turn
bool turnDir = 1;

int prevDist = MAX_DISTANCE;
unsigned long lastTurn = 0;
unsigned long lastSpin = 0;
unsigned long spin_start = 0;

// Entertainment
bool keepPlaying = true;

// // Tape following
bool foundTape = false;
int left_vals[] = {0, 0, 0};
int right_vals[] = {0, 0, 0};
int mid_vals[] = {0, 0, 0};
int tape_index = 0;
const int tape_speed = -440;
int tape_left = 0;
int tape_right = 0;
int tape_middle = 0;
int prev_tape[] = {0, 0, 0};
int prevSide = 0; // left: -1, right: 1
int error = 0;
int derivative = 0;
int integral = 0;
int prevError = 0;
int tape_thresholds[] = {445, 560, 554}; // Calibrated threshold values
int tape_max[3];
bool tape_detected[] = {0, 0, 0};
double tape_pid[] = {18, 150, 0}; // kp, kd, ki
int speedInc = 0;
int toTune = 1;

void loop()
{
  // If button is pressed, go to next mode
  if (checkRearButton())
  {
    if (mode == 2) // Reset robot
    { // Go idle if interrupted
      mode = 1;
      runMotors(0);
      returnTime -= (millis() - start_time + 6000);
      if (returnTime < 0 )
        returnTime = 0;
    }
    else
      cycleMode();
  }

  switch (mode)
  {
  case 1: // IDLE
    // Do nothing
    break;
  case 2: // SEARCH
    search();
    if (numCans >= maxNumCans || millis() - start_time > returnTime)
    {
      cycleMode();
    }
    break;
  case 3: // RETURN
    while (!foundTape)
    {
      readTape();
      runMotors(searchSpeed);
      delay(5);
      if (tape_middle > tape_thresholds[1])
      {
        foundTape = true;
        runMotors(0);
        delay(10);
      }
    }
    tapeFollow();
    break;
  case 4: // DEPOSIT
    deposit();
    delay(1000);
    cycleMode();
    break;
  case 5: // ENTERTAINMENT
    entertain();
  default:
    break;
  }

  // oledDisplay();
}

void tapeFollow(bool drive)
{
  // Read tape values - running average of last 3 values
  readTape();

  tape_detected[0] = tape_left > tape_thresholds[0];
  tape_detected[1] = tape_middle > tape_thresholds[1];
  tape_detected[2] = tape_right > tape_thresholds[2];

  // To left -> negative, to right -> postive
  if ((tape_detected[0] && tape_detected[1] && tape_detected[2]) ||
      (!tape_detected[0] && tape_detected[1] && !tape_detected[2]))
  { // middle only or all on
    prevSide = 0;
    error = 0;
  }
  else if (tape_detected[0] || tape_detected[1] || tape_detected[2]) // something on
  {
    error = tape_detected[0] * -4 + tape_detected[2] * 4;
    if (prevSide == 0 && tape_detected[0])
      prevSide = -1;
    if (prevSide == 0 && tape_detected[2])
      prevSide = 1;

    if (tape_detected[1])
    {
      prevSide = 0;
      error /= 2;
    }
  }
  else // all off
  {
    error = prevSide * 10;
  }

  if (error > 10)
    error = 10;
  else if (error < -10)
    error = -10;

  derivative = error - prevError;
  integral = error + integral;

  // Windup
  if (integral > 50)
    integral = 50;
  else if (integral < -50)
    integral = -50;

  prevError = error;
  prev_tape[0] = tape_left;
  prev_tape[1] = tape_middle;
  prev_tape[2] = tape_right;

  speedInc = error * tape_pid[0] + derivative * tape_pid[1] + integral * tape_pid[2];

  // Avoid switching directions!!
  if (speedInc > -tape_speed)
    speedInc = -tape_speed;
  else if (speedInc < tape_speed)
    speedInc = tape_speed;

  if (drive)
  {
    runMotors(tape_speed - speedInc, 'l');
    runMotors(tape_speed + speedInc, 'r');
  }
}
void pickup()
{
  static const int sweep_final_pos = 115;
  static const int arm_final_pos = 30;

  sweepServo.attach(SWEEP_SERVO);
  // Close sweeper
  sweepServo.write(sweep_final_pos);

  // Wait
  delay(200);
  // Raise arm
  armServo.attach(ARM_SERVO);
  armServo.write(arm_final_pos);
  delay(1000);
  armServo.write(arm_final_pos + 30);
  delay(50);
  armServo.detach();
  sweepServo.write(0);
  delay(50);
  sweepServo.detach();
  delay(500);
}
void runMotors(int speed, char ch)
{
  if (speed > 0)
  {
    if (ch == 'l' || ch == 'b')
    {
      pwm_start(DC_MOTOR_L2, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(DC_MOTOR_L1, 512, speed + wheelBalanceForwards, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (ch == 'r' || ch == 'b')
    {
      pwm_start(DC_MOTOR_R2, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(DC_MOTOR_R1, 512, speed - wheelBalanceForwards, RESOLUTION_10B_COMPARE_FORMAT);
    }
  }
  else if (speed < 0)
  {
    if (ch == 'l' || ch == 'b')
    {
      pwm_start(DC_MOTOR_L1, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(DC_MOTOR_L2, 512, -speed - wheelBalanceBackwards, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (ch == 'r' || ch == 'b')
    {
      pwm_start(DC_MOTOR_R1, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(DC_MOTOR_R2, 512, -speed + wheelBalanceBackwards, RESOLUTION_10B_COMPARE_FORMAT);
    }
  }
  else // Speed is 0
  {
    if (ch == 'l' || ch == 'b')
    {
      pwm_start(DC_MOTOR_L1, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(DC_MOTOR_L2, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (ch == 'r' || ch == 'b')
    {
      pwm_start(DC_MOTOR_R1, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(DC_MOTOR_R2, 512, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
  }
};
int getDistance()
{
  delay(20);                      // 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  return sonar.convert_cm(uS);
}
void calibrateTape()
{
  // Sensors off tape
  while (!checkRearButton())
  {
    readTape();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Tape Calibration");
    display.println("OFF TAPE");
    display.print("Left: ");
    display.println(tape_left);
    display.print("Middle: ");
    display.println(tape_middle);
    display.print("Right: ");
    display.println(tape_right);
    display.display();
  }
  int off_left = tape_left;
  int off_right = tape_right;
  int off_mid = tape_middle;

  // On tape
  while (!checkRearButton())
  {
    readTape();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Tape Calibration");
    display.println("ON TAPE");
    display.print("Left: ");
    display.println(tape_left);
    display.print("Middle: ");
    display.println(tape_middle);
    display.print("Right: ");
    display.println(tape_right);
    display.display();
  }
  tape_max[0] = tape_left;
  tape_max[1] = tape_middle;
  tape_max[2] = tape_right;
  tape_thresholds[0] = off_left + (tape_left - off_left) / 2;
  tape_thresholds[1] = off_mid + (tape_middle - off_mid) / 2;
  tape_thresholds[2] = off_right + (tape_right - off_right) / 2;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Tape Calibration");
  display.println("Thresholds");
  display.print("Left: ");
  display.println(tape_thresholds[0]);
  display.print("Middle: ");
  display.println(tape_thresholds[1]);
  display.print("Right: ");
  display.println(tape_thresholds[2]);
  display.println("Press to exit calibration...");
  display.display();
  while (!checkRearButton())
  {
    delay(5);
  }
}
bool checkRearButton()
{
  static unsigned long last_pressed = millis();
  static unsigned long curr_time;
  static bool prev_state = false;

  if (!digitalRead(REAR_BUTTON))
  {
    Serial.println("Rear pressed!");
    curr_time = millis();
    if (!prev_state && curr_time - last_pressed > 200)
    { // if previously not pressed and button held long enough
      prev_state = true;
      return true;
    }
    last_pressed = curr_time;
  }
  prev_state = false;
  return false;
}
void search()
{
  distance = getDistance();

  // Turn around if on tape
  if (((int)analogRead(TAPE_MIDDLE)) > tape_thresholds[1] && millis() - lastTurn > turnInterval) // wait turnInterval until next turn
  {
    turning = true;
    runMotors(0);
    delay(20);
    runMotors(-610, 'r');
    runMotors(-600, 'l');
    delay(400);
    runMotors(0, 'l');
    runMotors(-630, 'r');
    delay(800);
    runMotors(0);
    delay(20);
    runMotors(searchSpeed * 2);
    delay(620);
    lastTurn = millis();
    lastSpin = millis();
    prevDist = MAX_DISTANCE; // Reset prevDist
    return;
  }
  turning = false;

  // Spin until a reading is given
  if ((distance == 0 || distance < prevDist) && millis() - lastSpin > spinInterval)
  {
    spinSearch();
    // // Turned for longer than this
    // if (lastSpin - spin_start > 300)
    //   lastTurn = millis() - turnInterval + 500; // Don't turn for half a second after spinning
  }
  if (distance != 0 && distance < 15)
  {               // If object within 15cm
    runMotors(0); // stop and pickup
    delay(20);
    pickup();
    numCans++;
    delay(1000);
    lastSpin = 0;            // Spin immediately after pickup if distance is 0
    prevDist = MAX_DISTANCE; // Reset prev distance
  }
  else // continue straight
  {
    runMotors(searchSpeed);
  }
}
void spinSearch()
{
  bool timeRemaining = 1;

  runMotors(0);
  delay(10);
  spin_start = millis();

  if (prevDist >= MAX_DISTANCE)
    turnDir = !turnDir; // Change direction

  while ((distance > prevDist || distance == 0) && timeRemaining) // Turn duration
  {
    if (distance <= 50 && distance > 0)
      break;

    if (((int)analogRead(TAPE_MIDDLE)) > tape_thresholds[1])
    {
      // If on tape

      // If partly done the spin, finish spin and go straight
      if (millis() - spin_start > spinDuration / 5.0)
      {
        if (turnDir)
        {
          runMotors(0, 'l');
          runMotors(500, 'r');
        }
        else
        {
          runMotors(0, 'r');
          runMotors(500, 'l');
        }
        delay((1 - (millis() - spin_start) * 1.0 / spinDuration) * 1000);
        runMotors(searchSpeed * 2);
        delay(500);
        lastTurn = millis();
      }
      else
      { // Less than quarter done turn - reset lastTurn time and turn immediately
        lastTurn = 0;
        runMotors(0);
      }
      break; // Exit loop if on tape
    }
    distance = getDistance();

    // Alternate turn directions
    if (turnDir)
    {
      runMotors(0, 'l');
      runMotors(spinSpeed, 'r');
    }
    else
    {
      runMotors(0, 'r');
      runMotors(spinSpeed, 'l');
    }

    delay(10);
    timeRemaining = millis() - spin_start < spinDuration;
  }
  lastSpin = millis();

  if (timeRemaining)
  { // Reset prev distance if closer object not found
    prevDist = distance;
  }
  else
  {
    prevDist = MAX_DISTANCE;
  }
}
void controlGate(char ch)
{
  gateServo.attach(GATE_SERVO);
  delay(500);
  gateServo.write(ch == 'o' ? 180 : 0);
  delay(100);
  gateServo.detach();
}
void deposit() // Open and close gate
{
  delay(300);
  controlGate('o');
  delay(2000);
  controlGate('c');
}
void oledDisplay()
{
  static int loopCounter = 0;
  // Display
  if (loopCounter++ == displayFreq) // Update display every displayFreq loops
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Mode: ");
    display.println(mode);

    if (mode == 1)
    {
      display.println("IDLE");
    }
    else if (mode == 2)
    {
      display.print("On tape? ");
      display.println(turning);
      display.print("Distance: ");
      display.println(distance);
    }
    else if (mode == 3)
    {
      display.print("K: ");
      display.print(toTune);
      display.print(" ");
      display.println(tape_pid[toTune]);
      display.print("Thrshs: ");
      display.print(tape_thresholds[0]);
      display.print(" ");
      display.print(tape_thresholds[1]);
      display.print(" ");
      display.println(tape_thresholds[2]);
      display.print("Tape: ");
      display.print(tape_left);
      display.print(" ");
      display.print(tape_middle);
      display.print(" ");
      display.println(tape_right);
      display.print("Detected: ");
      display.print(tape_detected[0]);
      display.print(" ");
      display.print(tape_detected[1]);
      display.print(" ");
      display.println(tape_detected[2]);
      display.print("Prev side: ");
      display.println(prevSide == -1 ? "L" : "R");
      display.print("Error: ");
      display.println(error);
      display.print("Speed Inc: ");
      display.println(speedInc);
    }
    display.display();
    loopCounter = 0;
  }
}
void readTape()
{
  left_vals[tape_index] = analogRead(TAPE_LEFT);
  mid_vals[tape_index] = analogRead(TAPE_MIDDLE);
  right_vals[tape_index] = analogRead(TAPE_RIGHT);
  tape_index = (tape_index + 1) % 3;
  tape_left = (left_vals[0] + left_vals[1] + left_vals[2]) / 3;
  tape_middle = (mid_vals[0] + mid_vals[1] + mid_vals[2]) / 3;
  tape_right = (right_vals[0] + right_vals[1] + right_vals[2]) / 3;
}
void cycleMode(int newMode) // Cycle through modes-  these actions are only performed once
{
  runMotors(0);        // Stop motors
  mode = mode % 5 + 1; // Cycle through modes
  if (newMode >= 1 && newMode <= 4)
    mode = newMode;

  if (mode == 1)
  {
    detachInterrupt(digitalPinToInterrupt(REAR_BUTTON));
    displayFreq = 1000; // Change display frequency
  }
  else if (mode == 2)
  {
    start_time = millis();
    numCans = 0; // reset can count
    turnDir = startRight ? 1 : 0;
    displayFreq = 5;
    runMotors(searchSpeed * 2);
    delay(1600); // Drive forward before searching
    runMotors(0);
    delay(10);
  }
  else if (mode == 3)
  {
    // Reset values
    mid_vals[0] = 0;
    mid_vals[1] = 0;
    mid_vals[2] = 0;
    foundTape = false;
    displayFreq = 50;
    returnTime = maxReturnTime;
  }
}
void entertain()
{
  static int takeOnMe[] = {

      // Take on me, by A-ha
      // Score available at https://musescore.com/user/190926/scores/181370
      // Arranged by Edward Truong

      NOTE_FS5,
      8,
      NOTE_FS5,
      8,
      NOTE_D5,
      8,
      NOTE_B4,
      8,
      REST,
      8,
      NOTE_B4,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      NOTE_GS5,
      8,
      NOTE_GS5,
      8,
      NOTE_A5,
      8,
      NOTE_B5,
      8,
      NOTE_A5,
      8,
      NOTE_A5,
      8,
      NOTE_A5,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_D5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      NOTE_E5,
      8,
      NOTE_E5,
      8,
      NOTE_FS5,
      8,
      NOTE_E5,
      8,
      NOTE_FS5,
      8,
      NOTE_FS5,
      8,
      NOTE_D5,
      8,
      NOTE_B4,
      8,
      REST,
      8,
      NOTE_B4,
      8,
      REST,
      8,
      NOTE_E5,
      8,

      REST,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      NOTE_GS5,
      8,
      NOTE_GS5,
      8,
      NOTE_A5,
      8,
      NOTE_B5,
      8,
      NOTE_A5,
      8,
      NOTE_A5,
      8,
      NOTE_A5,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_D5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      NOTE_E5,
      8,
      NOTE_E5,
      8,
      NOTE_FS5,
      8,
      NOTE_E5,
      8,
      NOTE_FS5,
      8,
      NOTE_FS5,
      8,
      NOTE_D5,
      8,
      NOTE_B4,
      8,
      REST,
      8,
      NOTE_B4,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_E5,
      8,
      NOTE_GS5,
      8,
      NOTE_GS5,
      8,
      NOTE_A5,
      8,
      NOTE_B5,
      8,

      NOTE_A5,
      8,
      NOTE_A5,
      8,
      NOTE_A5,
      8,
      NOTE_E5,
      8,
      REST,
      8,
      NOTE_D5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      REST,
      8,
      NOTE_FS5,
      8,
      NOTE_E5,
      8,
      NOTE_E5,
      8,
      NOTE_FS5,
      8,
      NOTE_E5,
      8,

  };
  static int mii[] = {

      // Mii Channel theme
      // Score available at https://musescore.com/user/16403456/scores/4984153
      // Uploaded by Catalina Andrade

      NOTE_FS4, 8, REST, 8, NOTE_A4, 8, NOTE_CS5, 8, REST, 8, NOTE_A4, 8, REST, 8, NOTE_FS4, 8, //1
      NOTE_D4, 8, NOTE_D4, 8, NOTE_D4, 8, REST, 8, REST, 4, REST, 8, NOTE_CS4, 8,
      NOTE_D4, 8, NOTE_FS4, 8, NOTE_A4, 8, NOTE_CS5, 8, REST, 8, NOTE_A4, 8, REST, 8, NOTE_F4, 8,
      NOTE_E5, -4, NOTE_DS5, 8, NOTE_D5, 8, REST, 8, REST, 4,

      NOTE_GS4, 8, REST, 8, NOTE_CS5, 8, NOTE_FS4, 8, REST, 8, NOTE_CS5, 8, REST, 8, NOTE_GS4, 8, //5
      REST, 8, NOTE_CS5, 8, NOTE_G4, 8, NOTE_FS4, 8, REST, 8, NOTE_E4, 8, REST, 8,
      NOTE_E4, 8, NOTE_E4, 8, NOTE_E4, 8, REST, 8, REST, 4, NOTE_E4, 8, NOTE_E4, 8,
      NOTE_E4, 8, REST, 8, REST, 4, NOTE_DS4, 8, NOTE_D4, 8,

      NOTE_CS4, 8, REST, 8, NOTE_A4, 8, NOTE_CS5, 8, REST, 8, NOTE_A4, 8, REST, 8, NOTE_FS4, 8, //9
      NOTE_D4, 8, NOTE_D4, 8, NOTE_D4, 8, REST, 8, NOTE_E5, 8, NOTE_E5, 8, NOTE_E5, 8, REST, 8,
      REST, 8, NOTE_FS4, 8, NOTE_A4, 8, NOTE_CS5, 8, REST, 8, NOTE_A4, 8, REST, 8, NOTE_F4, 8,
      NOTE_E5, 2, NOTE_D5, 8, REST, 8, REST, 4,

      NOTE_B4, 8, NOTE_G4, 8, NOTE_D4, 8, NOTE_CS4, 4, NOTE_B4, 8, NOTE_G4, 8, NOTE_CS4, 8, //13
      NOTE_A4, 8, NOTE_FS4, 8, NOTE_C4, 8, NOTE_B3, 4, NOTE_F4, 8, NOTE_D4, 8, NOTE_B3, 8,
      NOTE_E4, 8, NOTE_E4, 8, NOTE_E4, 8, REST, 4, REST, 4, NOTE_AS4, 4,
      NOTE_CS5, 8, NOTE_D5, 8, NOTE_FS5, 8, NOTE_A5, 8, REST, 8, REST, 4,

      REST, 2, NOTE_A3, 4, NOTE_AS3, 4, //17
      NOTE_A3, -4, NOTE_A3, 8, NOTE_A3, 2,
      REST, 4, NOTE_A3, 8, NOTE_AS3, 8, NOTE_A3, 8, NOTE_F4, 4, NOTE_C4, 8,
      NOTE_A3, -4, NOTE_A3, 8, NOTE_A3, 2,

      REST, 2, NOTE_B3, 4, NOTE_C4, 4, //21
      NOTE_CS4, -4, NOTE_C4, 8, NOTE_CS4, 2,
      REST, 4, NOTE_CS4, 8, NOTE_C4, 8, NOTE_CS4, 8, NOTE_GS4, 4, NOTE_DS4, 8,
      NOTE_CS4, -4, NOTE_DS4, 8, NOTE_B3, 1,

      NOTE_E4, 4, NOTE_E4, 4, NOTE_E4, 4, REST, 8, //25

      //finishes with 26
      NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_FS4,8
  };
  static int cantina[] = {

      // Cantina BAnd - Star wars
      // Score available at https://musescore.com/user/6795541/scores/1606876
      NOTE_B4, -4, NOTE_E5, -4, NOTE_B4, -4, NOTE_E5, -4,
      NOTE_B4, 8, NOTE_E5, -4, NOTE_B4, 8, REST, 8, NOTE_AS4, 8, NOTE_B4, 8,
      NOTE_B4, 8, NOTE_AS4, 8, NOTE_B4, 8, NOTE_A4, 8, REST, 8, NOTE_GS4, 8, NOTE_A4, 8, NOTE_G4, 8,
      NOTE_G4, 4, NOTE_E4, -2,
      NOTE_B4, -4, NOTE_E5, -4, NOTE_B4, -4, NOTE_E5, -4,
      NOTE_B4, 8, NOTE_E5, -4, NOTE_B4, 8, REST, 8, NOTE_AS4, 8, NOTE_B4, 8,

      NOTE_A4, -4, NOTE_A4, -4, NOTE_GS4, 8, NOTE_A4, -4,
      NOTE_D5, 8, NOTE_C5, -4, NOTE_B4, -4, NOTE_A4, -4,
      NOTE_B4, -4, NOTE_E5, -4, NOTE_B4, -4, NOTE_E5, -4,
      NOTE_B4, 8, NOTE_E5, -4, NOTE_B4, 8, REST, 8, NOTE_AS4, 8, NOTE_B4, 8,
      NOTE_D5, 4, NOTE_D5, -4, NOTE_B4, 8, NOTE_A4, -4,
      NOTE_G4, -4, NOTE_E4, -2,
      NOTE_E4, 2, NOTE_G4, 2,
      NOTE_B4, 2, NOTE_D5, 2,

      NOTE_F5, -4, NOTE_E5, -4, NOTE_AS4, 8, NOTE_AS4, 8, NOTE_B4, 4, NOTE_G4, 4};

  keepPlaying = true;

  delay(1000);
  attachInterrupt(digitalPinToInterrupt(REAR_BUTTON), buttonInterrupt, FALLING);

  while (keepPlaying)
  {
    playSong(takeOnMe, sizeof(takeOnMe) / sizeof(takeOnMe[0]) / 2);
    delay(500);
    playSong(mii, sizeof(mii) / sizeof(mii[0]) / 2);
    delay(500);
    playSong(cantina, sizeof(cantina) / sizeof(cantina[0]) / 2);
    delay(500);
  }
  cycleMode();
}
void playSong(int melody[], int songLength)
{
  static const int tempo = 140;
  int divider = 0, noteDuration = 0;
  int wholenote = (60000 * 4) / tempo;
  for (int thisNote = 0; thisNote < songLength * 2 && keepPlaying; thisNote = thisNote + 2)
  {
    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0)
    {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    }
    else if (divider < 0)
    {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(SPEAKER, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(SPEAKER);
  }
}
void buttonInterrupt()
{
  Serial.println("Rear pressed!");
  // Debounce
  static unsigned long last_interrupt_time = 0;
  static unsigned long interrupt_time = millis();
  unsigned int interrupt_interval = 200; // amount of time between interrupts

  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time >= interrupt_interval)
  {
    keepPlaying = false;
  }

  last_interrupt_time = interrupt_time;
}