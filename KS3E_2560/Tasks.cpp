#include "Tasker.h"

class HandleCan : public TaskClass {
public:
    void execute() override {
      int packetSize = CAN.parsePacket();

      if (packetSize) {
        // received a packet
        showText("rxd");
    
        if (CAN.packetExtended()) {
          
        }
    
        if (CAN.packetRtr()) {
          // Remote transmission request, packet contains no data
          
        }
    
    
        if (CAN.packetRtr()) {
          
        } else {
          
    
          // only print packet data for non-RTR packets
          while (CAN.available()) {
//            Serial.print((char)CAN.read());
          }
        }
      }
    }

private:
    volatile unsigned int getExecutionDelay() override { return 1000000; }
};

class PrintLad : public TaskClass {
public:
    void execute() override {
//        Serial.println("lad");
    }

private:
    volatile unsigned int getExecutionDelay() override { return 6000100; }
};
