// DEBUG FIREBEETLE 
#include <Arduino.h>
#include <HX711-multi.h>

#define CLK 18
#define DOUT1 25
#define DOUT2 26
#define DOUT3 0
#define DOUT4 14

#define AIN1 36    // A0
#define AIN2 39    // A1
#define AIN3 15    // A4
#define AIN4 35    // A3

#define PIEZO_COUNT 4
#define CHANNEL_COUNT 4
#define TARE_TIMEOUT_SECONDS 4

const int piezoPins[PIEZO_COUNT] = {AIN1, AIN2, AIN3, AIN4};
byte DOUTS[CHANNEL_COUNT] = {DOUT1, DOUT2, DOUT3, DOUT4};
HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

uint8_t strainGaugeData[CHANNEL_COUNT];
uint8_t piezoData[PIEZO_COUNT];

void setup() {
  Serial.begin(115200);
  
  // Initialize strain gauge sensors
  tare();

  // Set piezo pins as INPUT
  for (int i = 0; i < PIEZO_COUNT; i++) {
    pinMode(piezoPins[i], INPUT);
  }
  
  Serial.println("Setup complete");
}

void tare() {
  bool tareSuccessful = false;
  unsigned long tareStartTime = millis();
  
  while (!tareSuccessful && millis() < (tareStartTime + TARE_TIMEOUT_SECONDS * 1000)) {
    tareSuccessful = scales.tare(20, 10000);
    Serial.println(tareSuccessful ? "Tare successful" : "Tare failed, retrying...");
  }
  
  if (!tareSuccessful) {
    Serial.println("Tare operation failed after timeout!");
  }
}

void readStrainGauges() {
  long results[CHANNEL_COUNT];
  scales.read(results);
  
  Serial.print("Strain: (");
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    if (results[i] < 0) {
      results[i] = 0;
    }
    strainGaugeData[i] = static_cast<uint8_t>((results[i]/842)*255/2500);
    Serial.print(strainGaugeData[i]);
    if (i < CHANNEL_COUNT - 1) {
      Serial.print(",");
    }
  }
  Serial.println(")");
}

void readPiezo() {
  Serial.print("Piezo: ->");
  for (int i = 0; i < PIEZO_COUNT; i++) {
    int reading = analogRead(piezoPins[i]);
    piezoData[i] = map(reading, 0, 4095, 0, 65535);
    Serial.print(piezoData[i]);
    if (i < PIEZO_COUNT - 1) {
      Serial.print(",");
    }
  }
  Serial.println("<-");
}

void loop() {
  static unsigned long lastStrainReadTime = 0;
  static unsigned long lastPiezoReadTime = 0;
  unsigned long currentTime = millis();

  // Lecture des jauges de contrainte (10 fois par seconde)
  if (currentTime - lastStrainReadTime >= 1000) {
    readStrainGauges();
    lastStrainReadTime = currentTime;
  }

  // // Lecture des données piézoélectriques (50 fois par seconde)
  // if (currentTime - lastPiezoReadTime >= 50) {
  //   readPiezo();
  //   lastPiezoReadTime = currentTime;
  // }
}

// -------------------------------------------------------------------------------------
// DEBUG ATMega
#include "ADCTouch.h"
#include <Arduino.h>

#ifndef A15
#define A15 69
#endif

const int numPins = 16;
int analogPins[numPins] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
int refValues[numPins];
int values[numPins];

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing capacitive sensors...");
    
    // Calibration
    for (int i = 0; i < numPins; ++i) {
        refValues[i] = ADCTouch.read(analogPins[i], 20);
        if ( i == 0 ){
          refValues[i] += 27 ;
        }
        if ( i == 1 ){
          refValues[i] += 10 ; 
        }
        Serial.print("Reference value for pin A");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(refValues[i]);
        delay(1);
    }
    Serial.println("Initialization complete.");
}

void loop() {
    // Lecture et envoi des données
    Serial.print("<");
    for (int i = 0; i < numPins; ++i) {
        values[i] = ADCTouch.read(analogPins[i]) - refValues[i];
        if (values[i] < 0 ){
          values[i] = 0 ;
        }

        Serial.print(values[i]);
        if (i < numPins - 1) {
            Serial.print(",");
        }
    }
    Serial.println(">");
    
    delay(1000); // Attente de 100ms avant la prochaine lecture
}