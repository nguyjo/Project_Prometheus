#include <Arduino.h>
#include <LittleFS.h>

// Automatically connects to the bottom pads!
LittleFS_QSPIFlash flash; 

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000);

  Serial.println("\n--- Testing QSPI Flash (W25Q128JV) ---");

  // 1. Initialize the Flash Chip
  if (!flash.begin()) {
    Serial.println("ERROR: QSPI Flash chip not found!");
    Serial.println("Check solder joints on the bottom of the Teensy.");
    while (1) { delay(100); }
  }
  Serial.println("QSPI Flash initialized successfully!");

  // 2. Format the drive (Warning: This wipes all data!)
  // In a real flight computer, we put this behind a Serial command so it doesn't 
  // accidentally erase data every time you turn the rocket on.
  Serial.println("Formatting LittleFS... (This takes a few seconds)");
  flash.format();
  Serial.println("Format complete!");

  // 3. Write a test file
  Serial.println("Writing test file...");
  File dataFile = flash.open("flight_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Timestamp,Altitude,AccelZ");
    dataFile.println("0.1,120.5,5.2");
    dataFile.println("0.2,125.1,5.1");
    dataFile.close();
    Serial.println("File written and saved!");
  } else {
    Serial.println("ERROR: Failed to open file for writing.");
  }

  // 4. Read the test file back
  Serial.println("\n--- Reading Data Back ---");
  File readFile = flash.open("flight_log.csv", FILE_READ);
  if (readFile) {
    while (readFile.available()) {
      Serial.write(readFile.read());
    }
    readFile.close();
    Serial.println("\n-------------------------");
    Serial.println("FLASH MEMORY TEST: PASS!");
  } else {
    Serial.println("ERROR: Failed to open file for reading.");
  }
}

void loop() {
  // Nothing to do here for the test
}