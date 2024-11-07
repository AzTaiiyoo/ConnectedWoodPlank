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
  Serial.begin(115200);  // Pour le débogage via USB
  Serial2.begin(115200, SERIAL_8N1, 17, 16); // RX, TX pour la communication inter-contrôleurs

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
  static String buffer;
  
  while (Serial2.available()) {
    char c = Serial2.read();
    buffer += c;
    
    if (c == '>') {
      if (buffer.startsWith("<")) {
        String data = buffer.substring(1, buffer.length() - 1);
        
        int index = 0;
        char* token = strtok(&data[0], ",");
        while (token != NULL && index < numCapacitivePins) {
          capacitiveData[index++] = atoi(token);
          token = strtok(NULL, ",");
        }
        
        if (index == numCapacitivePins) {
          Serial2.write('A');
          
          // Nouvel affichage uniformisé
          Serial.println("\n--- Capacitive Sensor Data ---");
          for (int i = 0; i < numCapacitivePins; i++) {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(" - Raw: ");
            Serial.print(capacitiveData[i]);
            Serial.print(", Scaled: ");  // Dans ce cas, Raw = Scaled car pas de conversion
            Serial.println(capacitiveData[i]);
          }
        }
      }
      buffer = "";
    }
    
    if (buffer.length() > 100) {
      buffer = "";
    }
  }
}

void readStrainGauges() {
  long results[CHANNEL_COUNT];
  scales.read(results);
  
  Serial.println("\n--- Strain Gauge Data ---");
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    if (results[i] < 0) {
      results[i] = 0;
    }
    strainGaugeData[i] = static_cast<uint8_t>((results[i]/842)*255/2500);
    
    Serial.print("Gauge ");
    Serial.print(i);
    Serial.print(" - Raw: ");
    Serial.print(results[i]);
    Serial.print(", Scaled: ");
    Serial.println(strainGaugeData[i]);
  }
}

void readPiezo() {
  Serial.println("\n--- Piezo Sensor Data ---");
  for (int i = 0; i < PIEZO_COUNT; i++) {
    int reading = analogRead(piezoPins[i]);
    piezoData[i] = map(reading, 0, 4095, 0, 65535);
    
    Serial.print("Piezo ");
    Serial.print(i);
    Serial.print(" - Raw: ");
    Serial.print(reading);
    Serial.print(", Mapped: ");
    Serial.println(piezoData[i]);
  }
}

void updateBLEData(char sensorType) {
  switch(sensorType) {
    case 'C':  // Capacitive sensors only
      {
        const uint8_t CAPACITIVE_START = 0x3C;
        const uint8_t CAPACITIVE_END = 0x3E;

        uint8_t capacitiveDataBytes[numCapacitivePins * 2 + 2];
        capacitiveDataBytes[0] = CAPACITIVE_START;
        for (int i = 0; i < numCapacitivePins; i++) {
          capacitiveDataBytes[i*2 + 1] = capacitiveData[i] & 0xFF;
          capacitiveDataBytes[i*2 + 2] = (capacitiveData[i] >> 8) & 0xFF;
        }
        capacitiveDataBytes[numCapacitivePins * 2 + 1] = CAPACITIVE_END;
        capacitiveCharacteristic.writeValue(capacitiveDataBytes, sizeof(capacitiveDataBytes));

        // Print BLE packet data
        Serial.print("\nCapacitive BLE packet: ");
        for (int i = 0; i < sizeof(capacitiveDataBytes); i++) {
          Serial.print(capacitiveDataBytes[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
      break;

    case 'S':  // Strain Gauges only
      {
        const uint8_t STRAIN_START = 0x28;
        const uint8_t STRAIN_END = 0x29;

        uint8_t strainGaugeDataBytes[numStrainGauges + 2];
        strainGaugeDataBytes[0] = STRAIN_START;
        memcpy(&strainGaugeDataBytes[1], strainGaugeData, numStrainGauges);
        strainGaugeDataBytes[numStrainGauges + 1] = STRAIN_END;
        strainGaugeCharacteristic.writeValue(strainGaugeDataBytes, sizeof(strainGaugeDataBytes));

        Serial.print("Strain Gauge BLE packet: ");
        for (int i = 0; i < sizeof(strainGaugeDataBytes); i++) {
          Serial.print(strainGaugeDataBytes[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
      break;

    case 'P':  // Piezo sensors
      {
        uint8_t piezoPacket[10];
        piezoPacket[0] = 0x2D;
        piezoPacket[1] = 0x3E;
        for (int i = 0; i < PIEZO_COUNT; i++) {
          piezoPacket[2 + i * 2] = (piezoData[i] >> 8) & 0xFF;
          piezoPacket[3 + i * 2] = piezoData[i] & 0xFF;
        }
        piezoPacket[8] = 0x3C;
        piezoPacket[9] = 0x2D;
        piezoCharacteristic.writeValue(piezoPacket, 10);

        Serial.print("Piezo BLE packet: ");
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
  static unsigned long lastCapacitiveStrainReadTime = 0;
  static unsigned long lastPiezoReadTime = 0;

  unsigned long currentTime = millis();

  // Read and send capacitive sensor data (every 100ms)
  if (currentTime - lastCapacitiveStrainReadTime >= 100) {
    readCapacitiveSensors();
    updateBLEData('C');

  // Read and send strain gauge data (every 100ms)
    readStrainGauges();
    updateBLEData('S');
    lastCapacitiveStrainReadTime = currentTime;
  }

  // Read and send piezo data (every 20ms)
  if (currentTime - lastPiezoReadTime >= 20) {
    readPiezo();
    updateBLEData('P');
    lastPiezoReadTime = currentTime;
  }

  BLE.poll();
}