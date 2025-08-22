#include <CAN.h>
#include <LED.h>

LED Led;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  CAN.begin(500000);
  Led.Add(LED1);
  Led.Off(LED1);
}

void loop() {
  int packetSize = CAN.parsePacket();
  if (packetSize && CAN.packetId() == 0x01) { // Only process packets for our CAN ID
    if (packetSize >= 2) {
      uint8_t cmd = CAN.read();
      uint8_t val = CAN.read();
      if (cmd == 0xF3) { // Enable/disable command
        if (val == 0x01) {
          Led.On(LED1); // Enable: turn LED on
        } else if (val == 0x00) {
          Led.Off(LED1); // Disable: turn LED off
        }
      }
    }
  }
}