#include "ADCTouch.h"
#include <Arduino.h>

#ifndef A15
    #define A15 69
#endif

const int numPins = 16; // Number of analog pins
int analogPins[numPins] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
int refValues[numPins]; // Array to store reference values for the offset 
int values[numPins]; // Array to store current ADC values

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 100;  // Intervalle d'envoi en millisecondes

bool waitingAck = false;
unsigned long lastAckTime = 0;
const unsigned long ACK_TIMEOUT = 50;  // Timeout pour l'ACK en millisecondes

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1);

  Serial.println("Initializing capacitive sensors...");

  // Calibrate the capacitive sensors
  for (int i = 0; i < numPins; ++i) {
    refValues[i] = ADCTouch.read(analogPins[i], 20);
    Serial.print("Reference value for pin A");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(refValues[i]);
    delay(1);
  }

  Serial.println("Initialization complete. Starting synchronized data transmission...");
}

void checkAck() {
  if (Serial2.available()) {
    char ack = Serial2.read();
    if (ack == 'A') {  // ACK reçu
      waitingAck = false;
      lastAckTime = millis();
    }
  }
  
  // Gestion du timeout
  if (waitingAck && (millis() - lastAckTime > ACK_TIMEOUT)) {
    waitingAck = false;  // Reset si pas de réponse
    Serial.println("ACK timeout");
  }
}

void sendData() {
  if (!waitingAck) {  // Envoyer seulement si pas en attente d'un ACK
    Serial.println("Sending data:");
    Serial.print("<");
    Serial2.print("<");
    
    for (int i = 0; i < numPins; ++i) {
      values[i] = ADCTouch.read(analogPins[i]) - refValues[i];
      
      Serial.print(values[i]);
      Serial2.print(values[i]);
      
      if (i < numPins - 1) {
        Serial.print(",");
        Serial2.print(",");
      }
    }
    
    Serial.println(">");
    Serial2.println(">");
    
    waitingAck = true;
    lastAckTime = millis();
    Serial.println("Data sent, waiting for ACK...");
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  checkAck();  // Vérifier si ACK reçu
  
  if (currentTime - lastSendTime >= SEND_INTERVAL && !waitingAck) {
    sendData();
    lastSendTime = currentTime;
  }
}