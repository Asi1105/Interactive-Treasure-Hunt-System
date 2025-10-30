#include <Arduino.h>      // Required for all Arduino sketches
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

/*
 * ================================================
 *  💎 TREASURE BOX (BLE BEACON TRANSMITTER)
 * ================================================
 * This code configures the ESP32-E board as a BLE
 * transmitter (beacon). It continuously broadcasts
 * a BLE signal named "TREASURE_05" for the Treasure
 * Detector to locate.
 */

void setup() {
  // Initialize BLE device with the name "TREASURE_05"
  BLEDevice::init("TREASURE_05");

  // Set maximum transmit power (+9 dBm)
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  // Create BLE server and advertising instance
  BLEServer *pServer = BLEDevice::createServer();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  // Prepare the advertisement data packet
  BLEAdvertisementData adv;
  adv.setName("TREASURE_05");
  adv.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

  // Apply advertisement data and start broadcasting
  pAdvertising->setAdvertisementData(adv);
  pAdvertising->start();

  // Initialize serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Treasure broadcasting...");
}

void loop() {
  // Periodically print status message for verification
  Serial.println("Still broadcasting...");
  delay(2000);   // Print every 2 seconds
}
