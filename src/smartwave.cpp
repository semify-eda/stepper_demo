/*
  This code demonstrates the practical use of the EX-MotorShield8874.
  It was tested on a STM32 Nucleo F4464RE Microcontroller, 
  driving a 17HS4401 bipolar stepper motor - https://www.cytron.io/p-nema-17hs4401-bipolar-stepper-motor.

  Note: The stepper motor draws 1.7A per phase, requiring a power supply of at least 9V and 5A for stable operation.
*/

//#include <Arduino.h>    // If used in VSCode / PlatformIO - include the Arduino library
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_ADDRESS         0X8   // I2C address of this device
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
// FAULT_A pin                        // A4   -   D0
// FAULT_B pin                        // A5   -   D1

// Function declarations
void initializeDisplay(void);
void initializeMotor(void);
void wait(int time);
void moveStepper(void);  
float avgCurrent(void);
void displayData(int position, int amps);
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

// Create a new instance of the AccelStepper class
AccelStepper stepper(MotorInterfaceType, dirPinA, dirPinB);              
// Initialise the OLED display 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

TwoWire Wire2(PC9, PA8); 

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
  wait(100);
}

// Custom wait with display
void wait(int time){
  for (int i = 0; i < time; i++){
    displayData(currentPos, avgCurrent());
  }
}

// Initialize the OLED display
void initializeDisplay(void) {
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
void initializeMotor(void) 
{
  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  digitalWrite(pwmPinA, HIGH);
  digitalWrite(pwmPinB, HIGH);
  digitalWrite(brakePinA, LOW);
  digitalWrite(brakePinB, LOW);
}

void moveStepper() 
{ 
    if(stepPos == 0xC8  | currentPos == 200) {
      stepper.stop();
      Serial.println("Reset stepper");
      Serial.println("Waiting for new command");
    }

  else
    Serial.println("Calling Stepper function"); 
    while (stepper.currentPosition() != stepPos) {
      stepper.setSpeed(stepSpeed);
      stepper.runSpeed();
      currentPos = stepper.currentPosition();
      amps = avgCurrent();
      displayData(currentPos, amps);
      debugInfo(currentPos, amps);
    }
}

// Function to handle I2C data reception
void I2C_RxHandler(int byteCount)
{
  Serial.println();
  Serial.println("I2C data reception");
  byte command;
  byte pos;
  byte speed;
  byte direction;
  char str[20];

  while(Wire2.available()){
    command   = Wire2.read();     // Read the I2C command Position or Speed change
    pos       = Wire2.read();         // Read the data associated with the command
	  speed     = Wire2.read();       // for position and speed
    direction = Wire2.read();
  }

  // Process the received command, position and speed
  // 0x50 / 80 - position control command
  if(command == 80){
    strcpy(str, "Position");
    stepPos = pos;
    if(direction == 0){
      stepSpeed = speed;
      }else
      stepSpeed = speed * (-1) ;
  } 
  // 0x46 / 70  - speed control command
  else if(command == 70){    
    strcpy(str, "Speed");         
    stepPos = pos;
    if(direction == 0){
      stepSpeed = speed;
      }else
      stepSpeed = speed * (-1) ;
  }
  else
  Serial.println(F("The received command is not recognised."));

  Serial.print("Received Command = ");
  Serial.println(str);
  Serial.print("Received Position = ");
  Serial.println(stepPos);
  Serial.print("Received Speed = ");
  Serial.println(stepSpeed);
  Serial.println();
}

// Calculate average current drawn by the stepper, using the Arduino's ADC
float avgCurrent(void) {
  int samples = 100;
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
}