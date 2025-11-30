#include <Arduino.h>
#include "HX711.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <BfButton.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Pins
const int nemaDirPin = 2;
const int nemaStepPin = 3;
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;
const int encoderCLK = 6;
const int encoderDT = 7;
const int encoderBtnPin = 8;

const int nemaStepsPerRevolution = 200;
const int eepromAddress = 0;

// Boot stability
const int NUM_STABLE_READINGS = 5;  // How many consecutive readings must be stable
const int STABILITY_TOLERANCE = 0.1;

// Rolling average
const int NUM_READINGS = 5;
float readings[NUM_READINGS];
int readIndex = 0;

float fastWeightLimit = 0; // slow motor movemments once above this weight
float setWeight = 0;
bool doseBeans = false;
int lastDoseTime = 0;

HX711 scale;
BfButton btn(BfButton::STANDALONE_DIGITAL, encoderBtnPin, true, LOW);


// --- Motor Control Vars ---
unsigned long stepInterval = 1000;      // Time between steps in microseconds (1000 µs = 1ms)
unsigned long lastStepTime = 0;         // Time the last step occurred
volatile int stepsRemaining = 0;        // Counter for remaining steps to take in the current dose

// --- Encoder Debouncing Vars ---
unsigned long encoderDebounceDelay = 2; // 5ms debounce
unsigned long lastEncoderTime = 0;      // Last time an encoder event was processed
int clkLastState;                       // Previous state of the CLK pin
int clkState;
bool btnReceived;
bool setWeightMode = false;
unsigned long longPressTimeout = 300;         // Long press is read twice.
unsigned long lastLongPress = 0;         // Long press is read twice.

// --- Display Vars ---
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
unsigned long lastDisplayTime = 0;
unsigned long displayDelay = 100;

// --- Load Cell Vars -- 
unsigned long lastWeighTime = 0; 
unsigned long weighDelay = 100;
float filteredWeight = 0;

unsigned long currentMillis = 0;

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
  if (setWeight <= 0.0 || isnan(setWeight)) {
    setWeight = 16.0;
  }
  Serial.println(setWeight); 
  fastWeightLimit = setWeight - 5;

  // Scale section
  Serial.println("Initializing load cell");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  while (!scale.is_ready()) {
    // This code runs repeatedly as long as the scale is NOT ready
    Serial.println("HX711 not found or disconnected. Waiting...");
    // writeToOled("HX711 not found.");
    delay(1000); // Wait 1 second before checking again
  }


            
  scale.set_scale(-1063.1);  // this value is obtained by calibrating the scale with known weights;
  scale.tare();
  delay(3000);
  filteredWeight = scale.get_units(10);

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

  // Motor setup
  pinMode(nemaStepPin, OUTPUT);
  pinMode(nemaDirPin, OUTPUT);

  writeSetWeight();
  Serial.println("Setup Complete");
}


void loop() {
  currentMillis = millis();
  btn.read();
  // --- load cell ---
  if (currentMillis - lastWeighTime > weighDelay) {
    float rawWeight = scale.get_units(1); // Reduced average to speed up reading
    filteredWeight = rawWeight;     // Note: Consider actual filtering for stability
  }
  
  
  btn.read();

  if (currentMillis - lastDisplayTime > displayDelay) {
    // Only update the changing weight.
    oled.fillRect(0, 15, 128, 24, BLACK);
    oled.setTextSize(3);
    oled.setCursor(0, 15);
    oled.print(filteredWeight, 1); // Display filtered weight with 1 decimal
    oled.display();
    displayDelay = currentMillis;
  }

  
  // Push button reading (assuming 'btn' is a library object, e.g., Bounce2)
  btn.read(); 
  // Example: if (btn.fell()) { doseBeans = true; } 

  // --- Stepper Motor Control (Non-Blocking) ---
  // Check if we need to start a new dose sequence
  if (doseBeans && filteredWeight < setWeight && stepsRemaining == 0) {
      // Determine if it's the main dose or the slower/refine dose
      if (filteredWeight < fastWeightLimit) {
          // Fast Dose: Set initial steps and direction
          stepsRemaining = nemaStepsPerRevolution / 4; 
          digitalWrite(nemaDirPin, LOW); 
          stepInterval = 1000; // Fast speed (1ms between steps)
          Serial.println("Starting FAST dose.");
      } else if (filteredWeight >= fastWeightLimit && filteredWeight < setWeight) {
          // Refine Dose: Set initial steps, change direction, and slow down
          stepsRemaining = nemaStepsPerRevolution / 8; // Smaller quarter-dose
          digitalWrite(nemaDirPin, HIGH); // Use the other direction for refinement
          stepInterval = 3000; // Slower speed (3ms between steps)
          Serial.println("Starting SLOW refine dose.");
      }
  }

  // Check if a step needs to be executed
  if (stepsRemaining > 0 && micros() - lastStepTime >= stepInterval) {
      // Execute the step
      lastStepTime = micros(); // Use micros() for high-resolution timing
      digitalWrite(nemaStepPin, !digitalRead(nemaStepPin)); // Toggle step pin

      // Only decrement the counter on the LOW-to-HIGH transition (or HIGH-to-LOW)
      if (digitalRead(nemaStepPin) == HIGH) {
          stepsRemaining--;
      }
  }

  // --- End Condition ---
  if (filteredWeight >= setWeight) {
    doseBeans = false;
    stepsRemaining = 0; // Stop any ongoing steps immediately
    Serial.print("Done dosing beans");
  }
}

void buttonPressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      if (doseBeans) {
        Serial.println("Stopping Dose");
      }
      else {
        Serial.println("Starting Dose");
      }
      doseBeans = !doseBeans;
      break;
      
    case BfButton::DOUBLE_PRESS:
      Serial.println("Double push");
      scale.tare();
      break;
      
    case BfButton::LONG_PRESS:
      currentMillis = millis();
      if (currentMillis - lastLongPress > longPressTimeout) {
        lastLongPress = currentMillis;
        Serial.println("Long push");
        setWeightMode = !setWeightMode;
        if (setWeightMode) {
          setWeightLoop();
        }
        else if (setWeight != EEPROM.read(eepromAddress)) {
          EEPROM.put(eepromAddress, setWeight);
        }
        writeSetWeight();
        break;
      }
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

void writeSetWeight() {
  // Write the first line of the display only as needed.
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("Coffee Time! Set "); 
  int xPos = oled.width() - (String(setWeight, 1).length() * 6);
  oled.setCursor(xPos, 0); 
  oled.print(setWeight, 1);
}

void writeToOled(String message1="", String message2="") {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 1);
  oled.print(message1);

  oled.setCursor(0, 15);
  oled.print(message2);
  oled.display();
}

void setWeightLoop() {
  oled.clearDisplay();
  writeToOled("Set weight mode", String(setWeight));
  while (setWeightMode) {
    
    int clkState = digitalRead(encoderCLK);
    
    // Check for rotation change
    if (clkState != clkLastState) { 

      if (digitalRead(encoderDT) != clkState) { 
        setWeight += 0.05;
      } else {
        setWeight -= 0.05;
      }

      if (setWeight < 0.0) setWeight = 0.0;
      
      Serial.println(setWeight);
      oled.clearDisplay();
      writeToOled("Set weight mode", String(setWeight));
    }
    clkLastState = clkState;
    btn.read();
  }
}