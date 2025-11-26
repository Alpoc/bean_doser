#include <Arduino.h>
#include "HX711.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <BfButton.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pins
const int nemaDirPin = 2;
const int nemaStepPin = 3;
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;
const int encoderBtnPin = 6;
const int encoderDT = 7;
const int encoderCLK = 8;

const int nemaStepsPerRevolution = 200;
const int eepromAddress = 0;

// Boot stability
const int NUM_STABLE_READINGS = 5;  // How many consecutive readings must be stable
const int STABILITY_TOLERANCE = 0.1;

// Rolling average
const int NUM_READINGS = 5;
float readings[NUM_READINGS];
int readIndex = 0;

// Encoder
int clkState;
int clkLastState;

int lowerWeight = 0; // slow motor movemments once above this weight
int setWeight = 0;
bool doseBeans = false;
int lastDoseTime = 0;

HX711 scale;
BfButton btn(BfButton::STANDALONE_DIGITAL, encoderBtnPin, true, LOW);

void setup() {
  Serial.begin(57600);
  Serial.println("Beginning Oled setup");
  // OLED display section
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1.5);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("Good Morning");
  oled.display();
  
  // Encoder
  Serial.println("Enabling Switch");
  pinMode(encoderCLK, INPUT_PULLUP);
  pinMode(encoderDT, INPUT_PULLUP);
  clkLastState = digitalRead(encoderCLK);

  btn.onPress(buttonPressHandler)
  .onDoublePress(buttonPressHandler) // default timeout
  .onPressFor(buttonPressHandler, 1000); // custom timeout for 1 second

  EEPROM.get(eepromAddress, setWeight);
  if (setWeight == 0) {
    setWeight = 16;
  }
  Serial.println(setWeight); 
  lowerWeight = setWeight - 5;

  // Scale section
  Serial.println("Initializing load cell");
  writeToOled("Initializing load cell");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  while (!scale.is_ready()) {
    // This code runs repeatedly as long as the scale is NOT ready
    Serial.println("HX711 not found or disconnected. Waiting...");
    writeToOled("HX711 not found.");
    delay(1000); // Wait 1 second before checking again
  }
            
  scale.set_scale(-1063.1);  // this value is obtained by calibrating the scale with known weights;
  scale.tare();

  Serial.println("Checking for stability");
  // while (!isStable()) {
  //   oled.clearDisplay();
  //   oled.setTextSize(1);
  //   oled.setCursor(0, 0);
  //   oled.println("Stabilizing...");
  //   oled.display();
  //   // Optional: Give visual feedback that the system is waiting
  //   Serial.print(".");
  //   // Optional: Display a "Stabilizing..." message on the OLED
    
  //   delay(500); // Wait half a second before checking again
  // }

  // rolling average code
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = 0.0;
  }
  Serial.println("Setup Complete");

  // Motor setup
  pinMode(nemaStepPin, OUTPUT);
  pinMode(nemaDirPin, OUTPUT);
}


void loop() {
  float rawWeight = scale.get_units(5);
  float filteredWeight = rawWeight;
  // readings[readIndex] = rawWeight;
  // readIndex = (readIndex + 1) % NUM_READINGS;
  // float total = 0;
  // for (int i = 0; i < NUM_READINGS; i++) {
  //   total += readings[i];
  // }
  // float filteredWeight = total / NUM_READINGS;

  // if (filteredWeight < 0) {
  //   filteredWeight = 0;
  // }

  // Serial.println(filteredWeight);
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(String("Coffee Time!"));

  oled.setTextSize(3);
  oled.setCursor(0, 10);
  oled.print(String(filteredWeight));
  oled.display();

  if (doseBeans && filteredWeight < lowerWeight) {
    // Set the spinning direction counterclockwise:
    digitalWrite(nemaDirPin, LOW);
    Serial.println("below 9g");
    // Spin the stepper motor 1 revolution quickly:
    for (int i = 0; i < nemaStepsPerRevolution / 4; i++) {
      // These four lines result in 1 step:
      digitalWrite(nemaStepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(nemaStepPin, LOW);
      delayMicroseconds(1000);
    }
    lastDoseTime = millis();
  }
  else if (doseBeans && filteredWeight >= lowerWeight && filteredWeight < setWeight && millis() - lastDoseTime > 1000) {
    // Set the spinning direction counterclockwise:
    digitalWrite(nemaDirPin, HIGH);
    Serial.println("above 9g");
    // Spin the stepper motor 1 revolution quickly:
    for (int i = 0; i < nemaStepsPerRevolution / 4; i++) {
      // These four lines result in 1 step:
      digitalWrite(nemaStepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(nemaStepPin, LOW);
      delayMicroseconds(1000);
    }
    lastDoseTime = millis();
  }
  else if (filteredWeight >= setWeight) {
    doseBeans = false;
  }

}

void buttonPressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      Serial.println("Single push");
      doseBeans = !doseBeans;
      break;
      
    case BfButton::DOUBLE_PRESS:
      Serial.println("Double push");
      scale.tare(2);
      break;
      
    case BfButton::LONG_PRESS:
      Serial.println("Long push");
      if (setWeight != EEPROM.read(eepromAddress)) {
        EEPROM.put(eepromAddress, setWeight);
      }
      break;
  }
}

bool isStable() {
  float tempReadings[NUM_STABLE_READINGS]; // Local array for stability check
  float total = 0;
  
  // 1. Take all readings and store them locally
  for (int i = 0; i < NUM_STABLE_READINGS; i++) {
    tempReadings[i] = scale.get_units(1); // Take individual readings
    total += tempReadings[i];
    delay(50); // Pause between samples
  }
  float avg = total / NUM_STABLE_READINGS;

  // 2. Compare the local readings to the new local average
  for (int i = 0; i < NUM_STABLE_READINGS; i++) {
    if (fabs(tempReadings[i] - avg) > STABILITY_TOLERANCE) {
      // If any single reading is outside the tolerance range of the average, it's not stable
      return false;
    }
  }

  return true; // All readings were close to the average
}

void writeToOled(String message) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(message);
  oled.display();
}