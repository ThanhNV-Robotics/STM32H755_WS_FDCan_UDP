/**
 * serial_bridge_m7.ino - Portenta H7 M7 core
 * Boots the M4 core and forwards everything the M4 prints through SerialRPC
 * to the USB Serial port. Leaves FDCAN1 untouched (owned by the M4).
 */

#include <Arduino.h>
#include <RPC.h>
#include <SerialRPC.h>

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    if (!SerialRPC.begin()) {   // also boots the M4
        Serial.println("[M7] RPC init failed");
    }
    Serial.println("[M7] serial bridge running");
}

void loop() {
    // Buffer so lines from the M4 are not interleaved
    String buffer;
    while (SerialRPC.available()) buffer += (char)SerialRPC.read();
    if (buffer.length() > 0) Serial.print(buffer);
}
