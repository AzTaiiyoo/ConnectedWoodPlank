#include <Arduino.h>
#include <ArduinoBLE.h>
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

const int numCapacitivePins = 16;
const int numStrainGauges = CHANNEL_COUNT;
int capacitiveData[numCapacitivePins];
uint8_t strainGaugeData[numStrainGauges];
uint16_t piezoData[PIEZO_COUNT];

BLEService sensorService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
BLECharacteristic capacitiveCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLENotify, numCapacitivePins * 2 + 2);
BLECharacteristic strainGaugeCharacteristic("cc54f4ce-1037-4b73-9e5a-cdcd53e85145", BLERead | BLENotify, numStrainGauges + 2);
BLECharacteristic piezoCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a9", BLERead | BLENotify, 10); 

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX, TX

  // Initialize strain gauge sensors
  tare();

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("ESP32_Multi_Sensor");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(capacitiveCharacteristic);
  sensorService.addCharacteristic(strainGaugeCharacteristic);
  sensorService.addCharacteristic(piezoCharacteristic);
  BLE.addService(sensorService);

  // Set the UUID of the service to be advertised
  BLE.setAdvertisedServiceUuid(sensorService.uuid());

  // Set piezo pins as INPUT
  for (int i = 0; i < PIEZO_COUNT; i++) {
    pinMode(piezoPins[i], INPUT);
  }

  BLE.advertise();
  Serial.println("BLE Multi-Sensor Beacon started");
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

void readCapacitiveSensors() {
  static String buffer;  // Buffer pour accumuler les données
  
  while (Serial2.available()) {
    char c = Serial2.read();
    buffer += c;
    
    // Chercher une trame complète
    if (c == '>') {
      if (buffer.startsWith("<")) {
        // Trame complète trouvée
        String data = buffer.substring(1, buffer.length() - 1);  // Enlever < et >
        
        int index = 0;
        char* token = strtok(&data[0], ",");
        while (token != NULL && index < numCapacitivePins) {
          capacitiveData[index++] = atoi(token);
          token = strtok(NULL, ",");
        }
        
        // Envoyer ACK seulement si toutes les données sont reçues correctement
        if (index == numCapacitivePins) {
          Serial2.write('A');  // Envoyer ACK
          Serial.println("Capacitive data received and ACK sent");
        }
      }
      buffer = "";  // Reset buffer après traitement
    }
    
    // Protection contre débordement du buffer
    if (buffer.length() > 100) {  // Taille arbitraire de sécurité
      buffer = "";
    }
  }
}

void readStrainGauges() {
  long results[CHANNEL_COUNT];
  scales.read(results);
  
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    if (results[i] < 0) {
      results[i] = 0; // If the value is negative, set to 0
    }
    // Apply the approximation
    strainGaugeData[i] = static_cast<uint8_t>((results[i]/842)*255/2500);
  }
  
  Serial.println("Strain gauge data read and approximated");
}

void readPiezo(){
  for (int i = 0; i < PIEZO_COUNT; i++) {
    int reading = analogRead(piezoPins[i]);
    // Convertir la lecture ADC 12-bit (0-4095) en valeur 16-bit (0-65535)
    piezoData[i] = map(reading, 0, 4095, 0, 65535);
  }

  Serial.println("Piezo data read");
}

void updateBLEData(char sensorType) {
  switch(sensorType) {
    case 'CS': // Capacitive and Strain Gauge
      {
        const uint8_t CAPACITIVE_START = 0x3C; // '<' in hex
        const uint8_t CAPACITIVE_END = 0x3E; // '>' in hex
        const uint8_t STRAIN_START = 0x28; // '(' in hex
        const uint8_t STRAIN_END = 0x29; // ')' in hex

        // Capacitive data
        uint8_t capacitiveDataBytes[numCapacitivePins * 2 + 2];
        capacitiveDataBytes[0] = CAPACITIVE_START; // '<'
        for (int i = 0; i < numCapacitivePins; i++) {
          capacitiveDataBytes[i*2 + 1] = capacitiveData[i] & 0xFF;
          capacitiveDataBytes[i*2 + 2] = (capacitiveData[i] >> 8) & 0xFF;
        }
        capacitiveDataBytes[numCapacitivePins * 2 + 1] = CAPACITIVE_END; // '>'
        capacitiveCharacteristic.writeValue(capacitiveDataBytes, sizeof(capacitiveDataBytes));

        // Print capacitive data
        Serial.print("Capacitive: ");
        for (int i = 0; i < sizeof(capacitiveDataBytes); i++) {
          Serial.print(capacitiveDataBytes[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Strain Gauge data
        uint8_t strainGaugeDataBytes[numStrainGauges + 2];
        strainGaugeDataBytes[0] = STRAIN_START; // '('
        memcpy(&strainGaugeDataBytes[1], strainGaugeData, numStrainGauges);
        strainGaugeDataBytes[numStrainGauges + 1] = STRAIN_END; // ')'
        strainGaugeCharacteristic.writeValue(strainGaugeDataBytes, sizeof(strainGaugeDataBytes));

        // Print strain gauge data
        Serial.print("Strain: ");
        for (int i = 0; i < sizeof(strainGaugeDataBytes); i++) {
          Serial.print(strainGaugeDataBytes[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
      break;
    case 'P': // Piezo
      {
        uint8_t piezoPacket[10];
        piezoPacket[0] = 0x2D; // '-'
        piezoPacket[1] = 0x3E; // '>'
        for (int i = 0; i < PIEZO_COUNT; i++) {
          piezoPacket[2 + i * 2] = (piezoData[i] >> 8) & 0xFF;
          piezoPacket[3 + i * 2] = piezoData[i] & 0xFF;
        }
        piezoPacket[8] = 0x3C; // '<'
        piezoPacket[9] = 0x2D; // '-'
        piezoCharacteristic.writeValue(piezoPacket, 10);

        // Print piezo data
        Serial.print("Piezo: ");
        for (int i = 0; i < 10; i++) {
          Serial.print(piezoPacket[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
      break;
  }
}

void loop() {
  static unsigned long lastCSReadTime = 0;
  static unsigned long lastPiezoReadTime = 0;

  unsigned long currentTime = millis();

  // Lecture et envoi des données capacitives et de jauge de contrainte (10 fois par seconde)
  if (currentTime - lastCSReadTime >= 100) {
    readCapacitiveSensors();
    readStrainGauges();
    updateBLEData('CS');
    lastCSReadTime = currentTime;
  }

  // Lecture et envoi des données piézoélectriques (50 fois par seconde)
  if (currentTime - lastPiezoReadTime >= 20) {
    readPiezo();
    updateBLEData('P');
    lastPiezoReadTime = currentTime;
  }

  BLE.poll();
}