/*
  This code demonstrates the practical use of the EX-MotorShield8874.
  It was tested on an STM32 Nucleo F4464RE Microcontroller, 
  driving a 17HS4401 bipolar stepper motor - https://www.cytron.io/p-nema-17hs4401-bipolar-stepper-motor.
  Note: The stepper motor draws 1.7A per phase, requiring a power supply of at least 9V and 5A for stable operation.

  If the stepper motor is used with SmartWave to send data via I2C, the following example data can be used.

  The following set of commands changes the position of the stepper motor with an increment of 50 at a constant speed and direction.
  Position  Speed   Direction
    0x32     0x64     0x00
    0x64     0x64     0x00
    0x96     0x64     0x00
    0xC8     0x64     0x00

  The following set of commands changes the speed, position, and direction of the stepper motor.
  Position  Speed   Direction
    0xC8     0x0A     0x00    // Do a full revolution clockwise at low speed.
    0x00     0x0A     0x01    // Do a full revolution counterclockwise at low speed.
    0xC8     0xFA     0x00    // Do a full revolution clockwise at high speed
    0x00     0xFA     0x01    // Do a full revolution counterclockwise at high speed.
*/

#include <Arduino.h>    // If used in VSCode / PlatformIO - include the Arduino library
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_ADDRESS         0x8   // I2C address of this device - Used for SmartWave
#define MotorInterfaceType  2     // Define the AccelStepper interface type to be Full-2-Wire
#define SCREEN_WIDTH        128   // OLED display width, in pixels
#define SCREEN_HEIGHT       32    // OLED display height, in pixels
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
// FAULT_A pin                        // A4   -   D0
// FAULT_B pin                        // A5   -   D1

// Function declarations
void initializeDisplay(void);
void initializeMotor(void);
void moveStepper(void);  
float avgCurrent(void);
void displayData(int position, float amps);
void debugInfo(int position, float amps);
void I2C_RxHandler(int byteCount);

// Global Variables
int stepSpeed;
int stepPos;
int currentPos;
double amps;
int byteCount;

const double mVamp = 1.1;   // Scaling factor 1100mV/A  - https://www.pololu.com/product/4035 
const double senseFactor = ((5.0 / 1024) / mVamp) * 1000; // Used to convert the measured voltage on the Arduino's ADC input to mA

// Create a new instance of the AccelStepper class to control the stepper motor
AccelStepper stepper(MotorInterfaceType, dirPinA, dirPinB);              

// Initialise the OLED display as an I2C peripheral
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
/*
Create a second Wire object to receive I2C data from SmartWave.
Ensure that the selected pins are not used by default on the EX-MotorShield8874
The following pins can be used on the STM32 Nucleo F4464RE Microcontroller	 
*/
TwoWire Wire2(PC9, PA8); 	// PC9 - I2C3_SDA / PA8 - I2C3_SCL

void setup() 
{
  Serial.begin(9600);
  Wire.setSDA(D14);	
  Wire.setSCL(D15);
  Wire2.begin(0x8);
  Wire2.onReceive(I2C_RxHandler);
  initializeDisplay();
  initializeMotor();
  stepper.setMaxSpeed(2000);
  stepper.setCurrentPosition(0);
}

void loop() {
  moveStepper();
}


// Initialize the OLED display
void initializeDisplay(void) 
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true); // Halt execution
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
  Serial.println("Display was successfully initialized.");
}

// Initialize motor control pins
void initializeMotor(void) 
{
  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  digitalWrite(pwmPinA, LOW);
  digitalWrite(pwmPinB, LOW);
  digitalWrite(brakePinA, LOW);
  digitalWrite(brakePinB, LOW);
  Serial.println("Motor was successfully initialized.");
}

// Control the stepper motor using received I2C commands
void moveStepper() 
{ 
  Serial.println("Calling Stepper function");   

  if(stepSpeed == 0){
    Serial.println(F("The speed is set to zero, the stepper won't move."));
    Serial.println(F("Waiting for new command"));
  }

  if(stepPos == currentPos) {
    Serial.println(F("Stepper is already at the given position, the stepper won't move"));
    Serial.println(F("Waiting for new command"));
  }

  if (stepPos > 200){
    Serial.println(F("Invalid position."));
    Serial.println(F("The possible step position is between 0 and 200."));
  }
  else {
    // If new position is less than the current position 
    if (stepPos < currentPos){
      digitalWrite(pwmPinA, HIGH);
      digitalWrite(pwmPinB, HIGH);

      // If direction is set for clockwise
      if (stepSpeed > 0){
        while (stepper.currentPosition() != stepPos) {
          if (currentPos == 200){
            stepper.setCurrentPosition(0);
          }
          stepper.setSpeed(stepSpeed);
          stepper.runSpeed();
          currentPos = stepper.currentPosition();
          amps = avgCurrent();
          displayData(currentPos, amps);
          debugInfo(currentPos, amps);
        }
      }
      // If direction is set for counterclockwise
      else if (stepSpeed < 0){
        while (stepper.currentPosition() != stepPos) {
          if (currentPos == 0){
            stepper.setCurrentPosition(200);
          }
          stepper.setSpeed(stepSpeed);
          stepper.runSpeed();
          currentPos = stepper.currentPosition();
          amps = avgCurrent();
          displayData(currentPos, amps);
          debugInfo(currentPos, amps);
        }
      }
    }
    // If new position is greater than the current position
    else if (stepPos > currentPos){ 
      digitalWrite(pwmPinA, HIGH);
      digitalWrite(pwmPinB, HIGH);

      // If direction is set for counterclockwise
      if (stepSpeed < 0){
        while (stepper.currentPosition() != stepPos) {
          if (currentPos == 0){
            stepper.setCurrentPosition(200);
            if (stepPos == 200){
              displayData(stepPos, amps);
              debugInfo(currentPos, amps);
              break;
            }
          }
          stepper.setSpeed(stepSpeed);
          stepper.runSpeed();
          currentPos = stepper.currentPosition();
          amps = avgCurrent();
          displayData(currentPos, amps);
          debugInfo(currentPos, amps);
        }
      }  
      // If direction is set for clockwise 
      else {
        while (stepper.currentPosition() != stepPos) {
          if (currentPos == 200){
            stepper.setCurrentPosition(0);
          }
          stepper.setSpeed(stepSpeed);
          stepper.runSpeed();
          currentPos = stepper.currentPosition();
          amps = avgCurrent();
          displayData(currentPos, amps);
          debugInfo(currentPos, amps);
        }
      }
    }
    digitalWrite(pwmPinA, LOW);
    digitalWrite(pwmPinB, LOW);
    displayData(currentPos, avgCurrent());
  }
}


// Function to handle I2C data reception
void I2C_RxHandler(int byteCount)
{
  Serial.println();
  Serial.println("I2C data reception");
  byte pos;
  byte speed;
  byte direction;

  while(Wire2.available())
  {
    pos = Wire2.read();     
    speed = Wire2.read();     
    direction = Wire2.read();
  }

  stepPos = pos;
  if(direction == 0){
    stepSpeed = speed;
  }
  else if(direction == 1){
    stepSpeed = speed * (-1) ;
  }
  else {
    Serial.println(F("The received command is not recognized."));
  }

  Serial.print("Received Position = ");
  Serial.println(stepPos);
  Serial.print("Received Speed = ");
  Serial.println(stepSpeed);
  Serial.println();
}

// Calculate the average current drawn by the stepper, using the Arduino's ADC
float avgCurrent(void) {
  int samples = 300;
  float totalCurrent = 0.0;
  for (int i = 0; i < samples; i++) {
    double senseA = analogRead(currentSenseA);
    double senseB = analogRead(currentSenseB);
    totalCurrent += senseA + senseB;
  }
  float avgAmp = (totalCurrent / samples) * senseFactor;
  return avgAmp;
}

// Display motor position and current consumption on OLED display
void displayData(int position, float amps) {
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
  display.println((amps/1000),3);
  display.setCursor(85, 20);
  display.println("  A");
  display.display();
}

// Print motor position and current to Serial for debugging
void debugInfo(int position, float amps) {
  Serial.print("Motor Position = ");
  Serial.println(position);
  Serial.print("Current (mA) = ");
  Serial.println(amps);
}
