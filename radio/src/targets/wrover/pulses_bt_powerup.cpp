/*
 * Copyright (C) OpenTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */ 
 
#include "opentx.h"
#include "Arduino.h"
#include "BLEDevice.h"
//#include "BLEScan.h"
#include "pulses_esp32.h"

#define STICK_MAX_VALUE (1024)
#define STICK_MIN_VALUE (-1024)
#define THR_STICK_CHANNEL 2  // channel 3
#define RDR_STICK_CHANNEL 3  // channel 4

// The remote service we wish to connect to.
static BLEUUID PWUserviceUUID("86C3810E-F171-40D9-A117-26B300768CD6");
static BLEUUID BATserviceUUID("180F");
// The characteristic of the remote service we are interested in.
static BLEUUID THRcharUUID("86C3810E-0010-40D9-A117-26B300768CD6");
static BLEUUID RDRcharUUID("86C3810E-0021-40D9-A117-26B300768CD6");
static BLEUUID BATcharUUID("2A19");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pThrottle;
static BLERemoteCharacteristic* pRudder;
static BLERemoteCharacteristic* pBattLevel;
static BLEAdvertisedDevice* myDevice;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println(*pData);
}

static void scanCompleteCB(BLEScanResults result) {
  Serial.println("Scan completed");
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pPWUService = pClient->getService(PWUserviceUUID);
    BLERemoteService* pBattService = pClient->getService(BATserviceUUID);
    if ((pPWUService == nullptr) || (pBattService == nullptr)) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(PWUserviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    std::map<std::string, BLERemoteCharacteristic*>* map = pPWUService->getCharacteristics();

    std::map<std::string, BLERemoteCharacteristic*>::iterator itr;
    Serial.println("----- BLERemoteCharacteristic Map: -----");
    for(itr=map->begin(); itr!=map->end(); itr++) {
      Serial.printf(" --- %s %s\n", itr->first.c_str(), (char *)itr->second->toString().c_str());
    }
  
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pThrottle = pPWUService->getCharacteristic(THRcharUUID);
    pRudder = pPWUService->getCharacteristic(RDRcharUUID);
    pBattLevel = pBattService->getCharacteristic(BATcharUUID);
    if ((pThrottle == nullptr) || (pRudder == nullptr) || (pBattLevel == nullptr)) {
      Serial.print("Failed to find our characteristic\n");
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pBattLevel->canNotify())
      pBattLevel->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(PWUserviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

static void* BtPowerUPInit(uint8_t module)
{
  (void)module;
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(500);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, scanCompleteCB, false);
  return 0;
}

static void BtPowerUPDeInit(void* context)
{
  BLEDevice::getScan()->stop();
  if (connected) {
    pThrottle->writeValue(0);
    pRudder->writeValue(0);
  }
}

static void BtPowerUPSetupPulses(void* context, int16_t* channels, uint8_t nChannels)
{
  // nothing to do
}

static void BtPowerUPSendPulses(void* context)
{
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    static int prevThr = 0;
    static int prevRdr = 0;
    int thr = (channelOutputs[THR_STICK_CHANNEL] + ((STICK_MAX_VALUE - STICK_MIN_VALUE) / 2)) * 254 /
        (STICK_MAX_VALUE - STICK_MIN_VALUE);
    int rdr = (0 <= channelOutputs[RDR_STICK_CHANNEL]) ?
        channelOutputs[RDR_STICK_CHANNEL] * (-128) / STICK_MAX_VALUE :
        channelOutputs[RDR_STICK_CHANNEL] * 127 / STICK_MIN_VALUE;
    
    static uint32_t thrTick = 0;
    static uint32_t rdrTick = 0;

    uint32_t now = RTOS_GET_MS();
    if (prevThr != thr) {
      prevThr = thr;
      pThrottle->writeValue(thr);
    } else {
      // TODO: seems the module would power down motor if the value was not sent or changed for a while. So do some kind of dithering here
      if (now - thrTick > 1000) {
        if (254 == thr) {
          pThrottle->writeValue(thr + ((esp_random() > (UINT32_MAX / 2)) ? 0 : -1));
        } else if (2 < thr) {
          pThrottle->writeValue(thr + ((esp_random() > (UINT32_MAX / 2)) ? 1 : -1));
        }
        thrTick = now;
      }
    }
    if (prevRdr != rdr) {
      prevRdr = rdr;
      pRudder->writeValue(rdr);
    } else {
      // TODO: seems the module would power down motor if the value was not sent or changed for a while. So do some kind of dithering here
      if (now - rdrTick > 1000) {
        if (127 == rdr) {
          pRudder->writeValue(rdr + ((esp_random() > (UINT32_MAX / 2)) ? 0 : -1));
        } else if (-128 == rdr) {
          pRudder->writeValue(rdr + ((esp_random() > (UINT32_MAX / 2)) ? 1 : 0));
        } else if (2 < abs(rdr)) {
          pRudder->writeValue(rdr + ((esp_random() > (UINT32_MAX / 2)) ? 1 : -1));
        }
        rdrTick = now;
      }
    }
  }else if(doScan){
    BLEDevice::getScan()->start(0, scanCompleteCB, false);
    doScan = false;
  }
}

static int BtPowerUPGetByte(void* context, uint8_t* data)
{
return 0;
}

static void BtPowerUPProcessData(void* context, uint8_t data, uint8_t* buffer, uint8_t* len)
{
}

#include "hal/module_driver.h"

const etx_module_driver_t BtPowerUPDriver = {
  .protocol = PROTOCOL_CHANNELS_ESPNOW,
  .init = BtPowerUPInit,
  .deinit = BtPowerUPDeInit,
  .setupPulses = BtPowerUPSetupPulses,
  .sendPulses = BtPowerUPSendPulses,
  .getByte = BtPowerUPGetByte,
  .processData = BtPowerUPProcessData,
};