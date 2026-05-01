#pragma once
#include <Arduino.h>
#include <SPI.h>

class FlashMemory {
  public:
    FlashMemory(int csPin);
    
    bool begin();
    void readJEDEC(uint8_t &manufacturerID, uint16_t &deviceID);

  private:
    int _cs;
    
    // Standard SPI Flash Opcodes
    const uint8_t CMD_JEDEC_ID = 0x9F;
};