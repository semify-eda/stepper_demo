/*
  This code demonstrates the practical use of the EX-MotorShield8874.
  It was tested on a STM32 Nucleo F4464RE Microcontroller, 
  driving a 17HS4401 bipolar stepper motor - https://www.cytron.io/p-nema-17hs4401-bipolar-stepper-motor.

  Note: The stepper motor draws 1.7A per phase, requiring a power supply of at least 9V and 5A for stable operation.
*/

#include <Arduino.h>    // If used in VSCode / PlatformIO - include the Arduino library
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Function declarations
void initializeDisplay();
void initializeMotor();
void moveStepper(int targetPosition, int speed);
float avgCurrent();
void displayData(int position, int amps);
void debugInfo(int position, float amps);

#define MotorInterfaceType  2     // Define the AccelStepper interface type to be Full-2-Wire
#define SCREEN_WIDTH        128   // OLED display width, in pixels
#define SCREEN_HEIGHT       32    // OLED display hight, in pixels
#define OLED_RESET          -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS      0x3C  // I2C Address of 128x32 OLED display

// EX-MotorShield8874 Pin Assignment  Default - Alternative
#define pwmPinA             3         // D3   -   D2    
#define pwmPinB             11        // D11  -   D5
#define dirPinA             12        // D12  -   D10
#define dirPinB             13        // D13  -   D4
#define brakePinA           9         // D9   -   D7
#define brakePinB           8         // D8   -   D6
#define currentSenseA       0         // A0   -   A2
#define currentSenseB       1         // A1   -   A3
// FAULT_A                            // A4   -   D0
// FAULT_B                            // A5   -   D1

const double mVamp = 1.1;   // Scaling factor 1100mV/A  - https://www.pololu.com/product/4035 
const double senseFactor = ((5.0 / 1024) / mVamp) * 1000; // Used to convert the measured voltage on the Arduino's ADC input to mA

AccelStepper stepper(MotorInterfaceType, dirPinA, dirPinB);               // Create a new instance of the AccelStepper class
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Initialise the OLED display

void setup() {
  Serial.begin(9600);
  initializeDisplay();
  initializeMotor();
  stepper.setMaxSpeed(600);
}

void loop() {
  moveStepper(200, 60);  // Move forward
  moveStepper(-200, -60);  // Move backward
}

// Initialize the OLED display
void initializeDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true); // Halt execution
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
}

// Initialize motor control pins
void initializeMotor() {
  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  digitalWrite(pwmPinA, HIGH);
  digitalWrite(pwmPinB, HIGH);
  digitalWrite(brakePinA, LOW);
  digitalWrite(brakePinB, LOW);
}

// Move the stepper motor to a target position at a specified speed
void moveStepper(int targetPosition, int speed) {
  stepper.setCurrentPosition(0);
  while (stepper.currentPosition() != targetPosition) {
    stepper.setSpeed(speed);
    stepper.runSpeed();
    int stepPos = stepper.currentPosition();
    double amps = avgCurrent();
    displayData(stepPos, amps);
    debugInfo(stepPos, amps);
  }
  stepper.stop();
  delay(1000);
}

// Calculate average current drawn by the stepper, using the Arduino's ADC
float avgCurrent() {
  int samples = 150;
  float totalCurrent = 0.0;
  for (int i = 0; i < samples; i++) {
    double senseA = analogRead(currentSenseA);
    double senseB = analogRead(currentSenseB);
    totalCurrent += senseA + senseB;
  }
  float avgAmp = (totalCurrent / samples) * senseFactor;
  return avgAmp;
}

// Display motor position and current on OLED display
void displayData(int position, int amps) {
  display.clearDisplay();
  display.setFont();
  display.setCursor(5, 0);
  display.println("EX-MotorShield8874");
  display.setCursor(5, 10);
  display.println("Position:");
  display.setCursor(65, 10);
  display.println(position);
  display.setCursor(5, 20);
  display.println("Current:");
  display.setCursor(65, 20);
  display.println(amps);
  display.setCursor(85, 20);
  display.println(" mA");
  display.display();
}

// Print motor position and current to Serial for debugging
void debugInfo(int position, float amps) {
  Serial.print("Motor Position = ");
  Serial.println(position);
  Serial.print("Current (mA) = ");
  Serial.println(amps);
  Serial.println();
}