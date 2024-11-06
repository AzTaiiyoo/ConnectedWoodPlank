// Starting of Firebeetle --------------------------------------------------
#include "HX711-multi.h"
#include "ArduinoBLE.h"
#include "BLEDevice.h"
#include <vector>
#include <iostream>
#include <cstring>
#include <cstdlib>

// Contain the time start of a little recording session 
unsigned long start_record;

// Used for the reception of the capacities from the Atmega 2560 
boolean newData = false;
const byte numChars = 250;
int receivedData = 0; 

// Define pins for the clock and data outputs of the load cell amplifiers
#define CLK 18     
#define DOUT1 25    // D2
#define DOUT2 26    // D3
#define DOUT3 0     // D5
#define DOUT4 14    // D6
#define AIN1 36    // A0
#define AIN2 39    // A1
#define AIN3 15    // A4
#define AIN4 35    // A3
#define TARE_TIMEOUT_SECONDS 200 // Timeout duration for the tare operation in seconds
byte DOUTS[4] = {DOUT1,DOUT2,DOUT3,DOUT4}; // Array to hold the data pin numbers for each channel
#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)  // Calculate the total number of channels based on the size of the DOUTS array
long int results[CHANNEL_COUNT];  // Declare an array to store the results from each channel
HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK); // Initialize the HX711MULTI object with the number of channels and the corresponding pins


// Piezo sensors 
const int numPiezos = 4; // Number of piezos
const uint8_t piezoPins[numPiezos] = {36,39,15,35}; // A0, A1, A4, A3 | Analog pin connected to the piezoelectric disc
int piezo_reads[numPiezos]; 
uint32_t lastPizeoRead = 0;
uint32_t piezoReadInterval = 10; // Interval between piezo readings in milliseconds
uint32_t lastStrainCapRead = 0;
uint32_t strainCapReadInterval = 200; // Interval between strain gauge readings in milliseconds

// Piezo Sensor digital filter parameters 5 (USELESS AT THE MOMENT)
const float alpha = 0.3;  // Smoothing factor
const int numReadings = 8; // Number of readings to average
int readings[numPiezos][numReadings];      // The readings from the analog input
int readIndex = 0;              // The index of the current reading
int total[numPiezos];                  // The running total
float average[numPiezos];              // The average
const float gain = 1.2;
const int threshold = 10;
float filteredValue[numPiezos];

//buffers 
std::vector<std::vector<uint8_t>> bufferPiezo;
std::vector<uint8_t> BufferCapacities;
std::vector<std::vector<uint8_t>> bufferStrainGauge;
uint8_t BLEBuffer[17]; //The only buffer used for BLE Advertising, will be loaded with all the group of datas one by one 

// IDs used for the reception device, Those index are always at the end of BLEBuffer
uint8_t Strain_Gauge_ID_start = 15 ;
uint8_t Piezo_ID = 25 ;
uint8_t Capacities_ID = 35 ;
uint8_t Strain_Gauge_ID_end = 45 ;
uint8_t Size_ID = 55 ;


int iteration = 0; // To implement a different delay for the strain gauges function and piezos function in the loop, THIS IS A LOOP COUNTER USED FOR DECIDING THE SAMPLING RATE

// UUIDs for the service and characteristic
BLEService customService("5b6898c3-db5e-4d5c-b01d-fb96bcc58892"); // I don't know what the string in parameters stands for 
// ----------------------------------------- IMPORTANT UNDER --------------------------------------------------
// I don't know what the parameters stands for. If the 14 is the maximum data lenghts, all the process could be revised to just send one big buffer instead of a lot of small ones.
BLECharacteristic customCharacteristic("5c77ce0e-0d91-43c6-9678-95604083112f", BLERead | BLEWrite, 14); 

void ClearBuffer(){
  // Clear the buffers 
  bufferStrainGauge.clear();
  bufferPiezo.clear();
  BufferCapacities.clear();
  memset(BLEBuffer, 0, sizeof(BLEBuffer));
}

void SendPacket(uint8_t* dataset, int size, int sensor){
  // Send data using the indicate characteristic
  // dataset is an array of 16 values 
  // sensor is the Snesor ID (15,25,35,45,55)

  BLEAdvertisingData advData;
  dataset[16] = sensor;
  for (size_t i=0 ; i<17 ; i++){
    Serial.print(dataset[i]);
    Serial.print(",");
  }
  advData.setLocalName("DATASENT");
  // Set parameters for advertising packet
  advData.setManufacturerData(0x9A4, dataset, size);
  // Copy set parameters in the actual advertising packet
  BLE.setAdvertisingData(advData);
  // advertise during 250 milliseconds
  BLE.advertise(); 
  Serial.println("advertising");
  delay(250);
  BLE.stopAdvertise();
  Serial.println("stop advertising");
}

void Insert_time(int iteration, unsigned long time){
  // Insert time into the load cells data sending 
  Serial.println(time);
  Serial.println(start_record);
  if (iteration == 2){ //First time is iteration == 2 so I put the start of the record of this cycle 
    BLEBuffer[12]=(uint8_t)(start_record>>24);
    BLEBuffer[13]=(uint8_t)(start_record>>16);
    BLEBuffer[14]=(uint8_t)(start_record>>8);
    BLEBuffer[15]=(uint8_t)start_record;
    SendPacket(BLEBuffer,sizeof(BLEBuffer),Strain_Gauge_ID_start); // Directly send the data
  }
  else { // Next time this function is entered is anything elkse but 2 so I put the end time of the recording for this cycle
    BLEBuffer[12]=(uint8_t)(time>>24);
    BLEBuffer[13]=(uint8_t)(time>>16);
    BLEBuffer[14]=(uint8_t)(time>>8);
    BLEBuffer[15]=(uint8_t)time;
    SendPacket(BLEBuffer,sizeof(BLEBuffer),Strain_Gauge_ID_end); // Directly send the data
  }
}

void CreatePacket(){
  size_t index = 0;
  unsigned long end_record = millis();
  
  // Send number of readings for each type of sensor 

  BLEBuffer[0]=bufferStrainGauge.size();
  if (bufferPiezo.size() > 255){
    BLEBuffer[1]=(uint8_t)(bufferPiezo.size()>>8);
    BLEBuffer[2]=(uint8_t)bufferPiezo.size();
  }
  else {
    BLEBuffer[2]=bufferPiezo.size();
  }
  
  if (BufferCapacities.size() > 255){
    BLEBuffer[3]=(uint8_t)(BufferCapacities.size()>>8);
    BLEBuffer[4]=(uint8_t)BufferCapacities.size();
  }
  else {
    BLEBuffer[4]=BufferCapacities.size();
  }
  SendPacket(BLEBuffer, sizeof(BLEBuffer), Size_ID);

  // Send data of the Strain Gauge 

  for (size_t i = 0; i < bufferStrainGauge.size(); ++i) {
    for (size_t j = 0; j < bufferStrainGauge[i].size(); ++j) {
      BLEBuffer[index++] = bufferStrainGauge[i][j];
    }
    if (i % 3 == 2 || i == bufferStrainGauge.size() - 1) {
      Insert_time(i, end_record);
      index = 0;
    }
  }

  //  Send data of the Piezo
  for (size_t i = 0; i < bufferPiezo.size(); ++i) {
    for (size_t j = 0; j < bufferPiezo[i].size(); ++j) {
      BLEBuffer[index++] = bufferPiezo[i][j];
    }
    if (i % 4 == 3 || i == bufferPiezo.size() -1) { //The last buffer of each cycle may contain values from the buffer second to last since I'm not clearing it each time I put new values in it 
      index = 0;
      SendPacket(BLEBuffer, sizeof(BLEBuffer), Piezo_ID);
    }
  }
  
  // Send data of the capacities 
  if (BufferCapacities.empty()){
    Serial.println("No data received from the Capacities");
  }
  else {
    for (size_t i=0 ; i<BufferCapacities.size();i++){
      if (i % 16 == 0 && i != 0){
        index++;
        Serial.println();
        SendPacket(BLEBuffer,sizeof(BLEBuffer),Capacities_ID);
        memset(BLEBuffer,0,sizeof(BLEBuffer));
      }
      BLEBuffer[i % 16] = BufferCapacities[i];
    }
    SendPacket(BLEBuffer,sizeof(BLEBuffer),Capacities_ID);
    memset(BLEBuffer,0,sizeof(BLEBuffer));
  }
  
  Serial.print("Sending done ");

  // Debut of the new cycle 
  start_record = millis();

  // Clear the Buffers 
  ClearBuffer();
}

void Capacities() {
    // This function is the receiving end of the UART connection between the ESP32 and the ATmega2560
    static boolean recvInProgress = false;

    // String array that will temporarily contain Capacities data as char
    char receiverCapacities[numChars];
    static byte ndx = 0;

    //Used to Recognize the start and the end of the trnasmission 
    char startMarker = '<';
    char endMarker = '>';

    char rc;

    // Receive data from The UART connection
    while (Serial2.available() > 0 && newData == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receiverCapacities[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receiverCapacities[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
  if (newData == true) {
    //Serial.print("This just in ... ");
    //Serial.println(receiverCapacities);
    newData = false;
  }

  // This part was madse by Claude, it converts the temporary array of String in a Buffer of Numerical values
  char* token = std::strtok(receiverCapacities, ","); // Divide the array in tokens 

  while (token != nullptr) {
    int number = std::atoi(token); // Convert the token in int 
    if (number >= 0 && number <= 255) { // Verify that the number is an uint_8
      BufferCapacities.push_back(static_cast<uint8_t>(number)); // Add the number to the buffer
    }
    else if (number >255){
      BufferCapacities.push_back(static_cast<uint8_t>(255));
    }
    else {
      BufferCapacities.push_back(static_cast<uint8_t>(0));
    }
    token = std::strtok(nullptr, ","); // Next Token
  }

  // Print the content of the buffer to ensure the result of the conversion.
  for (uint8_t value : BufferCapacities) {
    std::cout << static_cast<int>(value) << " ";
  }
  std::cout << std::endl;
  
  receivedData = 0; // receivedData do nothing ?
}

int read_piezos(int index) {
  // THIS WHOLE FUNCTION IS USELESS BECAUSE THE DIGITAL FILTER IS BAD. TO READ PIEZO, YOU CAN KEEP ONLY LINE 270 AND ADD THE ANALOG FILTER.
  
  // Subtract the last reading
  total[index] = total[index] - readings[index][readIndex];

  // Read the new value
  readings[index][readIndex] = analogRead(piezoPins[index]);

  // Add the new reading to the total
  total[index] = total[index] + readings[index][readIndex];

  // Calculate the average
  average[index] = total[index] / numReadings;

  // Apply the low-pass filter
  filteredValue[index] = round(alpha * average[index] + (1 - alpha) * filteredValue[index]);
    
  // Limite the value mesured at the range of the ADC (0-1023) (Actually the ADC can go a little bit further I believe I saw it in the doc)
  if (filteredValue[index] > 1023) {
      filteredValue[index] = 1023;
  }

  if (readings[index][readIndex] > threshold + filteredValue[index]){
	  // Amplify peak if its superior to the threshold
    filteredValue[index] = gain * filteredValue[index];
    if (filteredValue[index] > 1023) {
      filteredValue[index] = 1023;
    }
  }
  // Advance to the next position in the array
  if (index == 3){
    readIndex++;
  }
  // If we're at the end of the array, wrap around to the beginning
  if (readIndex >= numReadings && index == 3) {
    readIndex = 0;
  }
  delay(10);
  return filteredValue[index];
}

void tare() {
  // First calibration of the load cell sensors (Find the 0)
  bool tareSuccessful = false; // Flag to check if the tare operation was successful
  
  unsigned long tareStartTime = millis(); // Record the start time of the tare operation
  
  // Wait for the tare operation to succeed or until the timeout period expires
  while (!tareSuccessful && millis() < (tareStartTime + TARE_TIMEOUT_SECONDS * 1000)) {
    tareSuccessful = scales.tare(20, 10000); // Attempt to perform the tare operation with specified parameters
    Serial.println(tareSuccessful);
  }
}

void Load_Cells() {
  scales.read(results);
  // Temporary buffer with the readings
  std::vector<uint8_t> row;
  for (int i = 0; i < scales.get_count(); i++) {
    if (results[i] < 0) {
      results[i] = 0; // If the value is negative, get 0 instead
    }
    // This is an approximate mean of converting the readings from long to a uint8_t
    uint8_t value = static_cast<uint8_t>((results[i]/842)*255/2500); 
    row.push_back(value);
    Serial.print(value);
    Serial.print(",");
  }
  // Put the temporary buffer (row) into the actual buffer
  bufferStrainGauge.push_back(row);
  Serial.println("Load Cells read");
}

void setup() {
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("failed to initialize BLE!");
    while (1);
  }
  
  // Set advertised local name and service
  BLE.setLocalName("ConnectedWoodPlank");
  BLE.setAdvertisedService(customService);

  // Add characteristic to service
  customService.addCharacteristic(customCharacteristic);
  BLE.setLocalName("ConnectedWoodPlank");
  BLE.addService(customService);

  // Set all Piezo pins at Input (Reduce their Volatge to 0 to see current from the Piezos coming in)
  pinMode(AIN1, INPUT);
  pinMode(AIN2, INPUT);
  pinMode(AIN3, INPUT);
  pinMode(AIN4, INPUT);

  // Initialisation of the UART port through USB for debuging 
  Serial.begin(115200);

  // Initialisation of the UART connection between the firebeetle and the ATmega2560
  Serial2.begin(115200,SERIAL_8N1, 17,16); 

  // Calibrate the Load cell
  tare();

  // Initialize the filtered value with the first reading for the digital filter of the Piezos 
  for (int i = 0; i<4; i++){
    filteredValue[i] =0;
    for (int j = 0; j < numReadings; j++) {
      readings[i][j] = 0;
    }
  }
  Serial.println("FireBeetle ESP32 is ready!");
}

void loop(){
  unsigned long currentTime = millis();

  if (currentTime - lastPizeoRead >= piezoReadInterval){
    std::vector<uint8_t> row;
    for(int i = 0; i< 4 ; i++){
      uint8_t value = analogRead(piezoPins[i]) / 4; // Divide by 4 to have it in a uint8_t
      row.push_back(value);
      Serial.print(value);
      Serial.print(",");
    }
    bufferPiezo.push_back(row);
    Serial.println();
    lastPizeoRead = currentTime;
  }

  if (currentTime - lastStrainCapRead >= strainCapReadInterval){
    scales.power_up();
    Load_Cells();
    scales.power_down();
    
    Serial2.print("SendData\n");
    Serial.println("SYNC sent to the second microcontroller.");
    delay(10);
    if (Serial2.available() > 0) {
      Capacities();
    }

    CreatePacket();

    lastStrainCapRead = currentTime;
  }

  BLE.poll();
  delay(1);
}
// void loop() {
//   // Every 10 iterration, enter the code for the Load Cell 
//   // 10 Iteration with a delay of 10 milliseconds = Every 100 milliseconds 
//   if (iteration > 9 && iteration %10 == 0){
//     scales.power_up();
//     Load_Cells((iteration/10) -1); // For example, if iteration == 10 : 10/10 -1 = 0 -> Index 0 in the buffer
//     scales.power_down();

//     // Every 50 iteration, enter the code asking for Capacities daat from the UART and sending data through BLE
//     // 50 Iteration with a delay of 10 milliseconds = Every 500 milliseconds 
//     if (iteration > 49){
//       Serial2.print("SendData\n"); // Send Synchronisation message for the ATmega
//       Serial.println("SYNC sent to the second microcontroller."); // 
//       if (Serial2.available() > 0) { // Check if datas are available
//         Capacities(); // Gather data
//       }
//       CreatePacket(); // Send all the datas
//       iteration =0; // New Cycle
//     }
//   }

//   // PIEZO Readings
//   std::vector<uint8_t> row;
//   for (int i = 0; i < 4; i++) {
//     uint8_t value = read_piezos(i) / 4; // Divide by 4 to have it in a uint8_t
//     row.push_back(value);
//     Serial.print(value);
//     Serial.print(",");
//   }
//   bufferPiezo.push_back(row);
//   Serial.println();

//   // Wait for a short time before taking the next reading
//   delay(10);

//   // Increment 
//   iteration +=1;
// }

// End of Firebeetle --------------------------------------------------
// Starting of Atmega --------------------------------------------------
#include "ADCTouch.h"
#include <Arduino.h>

#ifndef A15
    #define A15 69
#endif

const int numPins = 16; // Number of analog pins
int analogPins[numPins] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15}; // Array to store analog pin numbers
int refValues[numPins]; // Array to store reference values for the offset 
int values[numPins]; // Array to store current ADC values

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial2.begin(115200, SERIAL_8N1); // Initialize serial for UART communication with ESP32

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

  Serial.println("Initialization complete. Waiting for 'SendData' command...");
}

void loop() {
  if (Serial2.available() > 0) {
    String message = Serial2.readStringUntil('\n');
    if (message == "SendData") {
      Serial.println("'SendData' command received. Sending data...");
      sendData();
    }
  }
}

void sendData() {
  Serial.println("Sending data:");
  Serial.print("<");
  Serial2.print("<");
  
  for (int i = 0; i < numPins; ++i) {
    values[i] = ADCTouch.read(analogPins[i]) - refValues[i]; // Read ADC values and remove offset
    
    Serial.print(values[i]);
    Serial2.print(values[i]);
    
    if (i < numPins - 1) {
      Serial.print(",");
      Serial2.print(",");
    }
  }
  
  Serial.println(">");
  Serial2.println(">");
  
  Serial.println("Data sent.");
}

// End of Atmega --------------------------------------------------