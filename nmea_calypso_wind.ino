
//  An Arduino/ESP32 program to read BLE data from a Calypso wireless wind meter
//  and re-transmit it over NMEA2000.


#include <Arduino.h>

#define USE_ELEGANT_OTA 0
#define USE_BLE_OTA 1

#define DO_BMS 1

#if USE_ELEGANT_OTA
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include <DNSServer.h>
#include <ElegantOTA.h>
#endif

// Used by both NMEA2000 and BLE
const char *MANUFACTURER = "dougbraun.com";
const char *MODEL_NAME = "Doug Calypso BLE wind monitor";
const char *MODEL_VERSION = "1.0";
const char *FIRMWARE_VERSION = __DATE__ " " __TIME__;

#define ESP32_CAN_TX_PIN GPIO_NUM_3  // Set CAN TX port   This is the pin labeled D2 on the Seeed Xiao ESP32S3 board
#define ESP32_CAN_RX_PIN GPIO_NUM_2  // Set CAN RX port  This is the pin labeled D on the Seeed Xiao ESP32S3 board

#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>


#include <HardwareSerial.h>
#include <Preferences.h>

#include "esp_mac.h"

#if USE_BLE_OTA
/*
  Add a BLE OTA service that implement https://components.espressif.com/components/espressif/ble_ota without security
  The service advertises itself as: 00008018-0000-1000-8000-00805f9b34fb
  If any of this is defined:
  MODEL
  SERIAL_NUM 
  FW_VERSION  
  HW_VERSION 
  MANUFACTURER 
  the DIS service is added

  The flow of creating the BLE server is:
  1. Create a BLE Server
  2. Add a BLE OTA Service
  3. Add a DIS Service
  4. Start the service.
  5. Start advertising.
  6. Process the update
*/

#include "NimBLEOTA.h"

NimBLEOTAClass BLEOTA;
#else
#include <NimBLEDevice.h>
#endif

// Persistent Data

Preferences prefs;

class PersistentData {
public:
  bool data_valid;
  int node_address;  // For NMEA2K, default is 34

  void begin() {
    prefs.begin("calypso_prefs");
    prefs.getBytes("calypso", this, sizeof(*this));
  }

  void init() {
    // Call commit() after this!

    data_valid = true;
    node_address = 34;
  }


  void commit() {
    prefs.putBytes("calypso", this, sizeof(*this));
  }
};

PersistentData persistentData;



tNMEA0183* NMEA0183 = nullptr;


// List here the N2K messages we will transmit.
// Apparent Wind and DC/Battery Status messages
const unsigned long transmitMessages[] PROGMEM = { 127506L, 127508L, 130306L, 0 }; 



void OnN2kOpen() {
  // Nothing to do
}




// *****************************************************************************
void SendN2kWind(double windSpeed, double windAngle) {

  digitalWrite(LED_BUILTIN, LOW);  // Indicate NMEA data transmission

  tN2kMsg N2kMsg;

  Serial.printf("Transmitting NMEA data: AWS %2.1f  AWD %3.0f\n", windSpeed, windAngle);

  SetN2kWindSpeed(N2kMsg, 1, windSpeed, DegToRad(windAngle), N2kWind_Apprent);
  NMEA2000.SendMsg(N2kMsg);

  tNMEA0183Msg NMEA0183Msg;
  if (NMEA0183SetMWV(NMEA0183Msg, DegToRad(windAngle), NMEA0183Wind_Apparent, windSpeed)) {
    NMEA0183->SendMessage(NMEA0183Msg);
  }

  delay(5);  //allow LED to blink and the cpu to switch to other tasks
  digitalWrite(LED_BUILTIN, HIGH);
}

// PGN 127506
// *****************************************************************************
void SendN2kBatteryLevel(int batteryIdx, int batteryLevel, int batteryHealth = 100) {

  tN2kMsg N2kMsg;

  Serial.printf("Transmitting NMEA data: Battery %d Level %d\n", batteryIdx, batteryLevel);

  SetN2kDCStatus(N2kMsg, 1, batteryIdx, N2kDCt_Battery, batteryLevel, batteryHealth, 999.9 /* Remaining time */);
  NMEA2000.SendMsg(N2kMsg);
}


// PGN 127508: Temperature in Celsius!
// *****************************************************************************
void SendN2kBatteryStatus(int batteryIdx, double voltage, double current=N2kDoubleNA, double temperature=N2kDoubleNA) {

  tN2kMsg N2kMsg;

  Serial.printf("Transmitting NMEA data: Battery %d voltage %g current %g temperature %g\n", batteryIdx,
                voltage, current, temperature);

  SetN2kDCBatStatus(N2kMsg, batteryIdx, voltage, current, CToKelvin(temperature));
  NMEA2000.SendMsg(N2kMsg);
}



// Calypso-defined values
static NimBLEUUID CALYPSO_DATA_SERVICE("180D");      // Unfortunately the same as the standard heart rate service.
static NimBLEUUID WIND_DATA_CHARACTERISTIC("2A39");  // Unfortunately the same as the standard heart rate control point
static NimBLEUUID DATA_RATE_CHARACTERISTIC("A002");  // We ignore this, but we need to expose it for the clients to work.
static NimBLEUUID PITCH_CHARACTERISTIC("A003");  // We ignore this, but we need to expose it for the clients to work.
static NimBLEUUID CALYPSO_MYSTERY_SERVICE("8EC90001-F315-4F60-9FB8-838830DAEA50");  // The Calypso device advertises this.


// These services and characteristics provide the same wind/battery data, in a more user-friendly way.
static NimBLEUUID WIND_SERVICE("181A");        // Industry standard
static NimBLEUUID AWS_CHARACTERISTIC("2A72");  // Industry standard
static NimBLEUUID AWD_CHARACTERISTIC("2A73");  // Industry standard

static NimBLEUUID BATTERY_SERVICE("180F");               // Standard
static NimBLEUUID BATTERY_LEVEL_CHARACTERISTIC("2a19");  // Standard

#if DO_BMS
static NimBLEUUID BMS_DATA_SERVICE("FFE0");
static NimBLEUUID BMS_DATA_CHARACTERISTIC("FFE1");  
#endif



static boolean doingOta = false;

static int numConnectedClients = 0;

static uint32_t serial_num;  // Derived from BLE MAC address


static NimBLERemoteCharacteristic* pWindDataCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pAwsCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pAwdCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pBatteryLevelCharacteristic = nullptr;

NimBLEClient* pCalypsoClient = nullptr;

static NimBLECharacteristic* pWindDataServerCharacteristic = nullptr;
static NimBLECharacteristic* pAwsServerCharacteristic = nullptr;
static NimBLECharacteristic* pAwdServerCharacteristic = nullptr;
static NimBLECharacteristic* pBatteryServerCharacteristic = nullptr;

#if DO_BMS
static NimBLERemoteCharacteristic* pBMSCharacteristic = nullptr;
NimBLEClient* pBMSClient = nullptr;
#endif


static bool notified = false;

// Convert from integral hundredths to a proper floating-point value.
double convertWindValue(const uint8_t* data) {

  return ((data[1] * 256) + data[0]) / 100.0;
}


static void windDataNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  Serial.printf("Notify callback for Calypso wind data, data length %d ", length);
  assert(length >= 10);

  notified = true;

  double AWS = (pData[1] << 8 | pData[0]) * 0.01;
  double AWD  = (pData[3] << 8 | pData[2]); 
	//uint8_t level = 10 * pData[4];  // pData[4] seems to be a 0-10 battery level, but pData[5] is always 100

  Serial.printf("values : AWS %2.1f  AWD: %3.0f\n", AWS, AWD);

  // Send the data to NMEA2000
  SendN2kWind(AWS, AWD);

  // If our relay server is running, pass on the entire data array
  if (pWindDataServerCharacteristic) {
    pWindDataServerCharacteristic->setValue(pData, length);
    pWindDataServerCharacteristic->notify();
  }

  // A convenient place to periodically print a message...
  Serial.printf("%d connected clients\n", numConnectedClients);

}



static void awsNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  Serial.printf("Notify callback for AWS, data length %d ", length);
  double AWS = convertWindValue(pData);
  Serial.printf("value : %2.1f\n", AWS);

  // If our relay server is running, pass on the new value
  if (pAwsServerCharacteristic) {
    pAwsServerCharacteristic->setValue(pData, length);
    pAwsServerCharacteristic->notify();
  }
}



static void awdNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  Serial.printf("Notify callback for AWD, data length %d ", length);
  double AWD = convertWindValue(pData);
  Serial.printf("value : %3.0f\n", AWD);

  
  // If our relay server is running, pass on the new value
  if (pAwdServerCharacteristic) {
    pAwdServerCharacteristic->setValue(pData, length);
    pAwdServerCharacteristic->notify();
  }
}

static void batteryLevelNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  Serial.printf("Notify callback for battery, data length %d ", length);
  int batteryLevel = pData[0];
  Serial.printf("value : %d\n", batteryLevel);

  // If our relay server is running, pass on the new value
  if (pBatteryServerCharacteristic) {
    pBatteryServerCharacteristic->setValue(pData, length);
    pBatteryServerCharacteristic->notify();
  }
}


class MyCalypsoClientCallbacks : public BLEClientCallbacks {
  void onConnect(NimBLEClient* client) {
    assert(client == pCalypsoClient);
  }

  void onDisconnect(NimBLEClient* client) {
    assert(client == pCalypsoClient);
    Serial.println("Calypso onDisconnect");
  }
};

MyCalypsoClientCallbacks calypsoClientCallbacks;


#if DO_BMS
static void bmsNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  Serial.printf("Notify callback for BMS, data length %d ", length);
  
}



uint8_t crc(const uint8_t data[], const uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc + data[i];
  }
  return crc;
}

void process_cell_data_frame(const std::vector<uint8_t> &data) {
  // Packet format: https://github.com/syssi/esphome-jk-bms/blob/main/docs/protocol-design-ble.md

  auto jk_get_16bit = [&](size_t i) -> uint16_t { return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0); };
  auto jk_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(jk_get_16bit(i + 2)) << 16) | (uint32_t(jk_get_16bit(i + 0)) << 0);
  };

  assert(data[4] == 2);    // Cell data packet

  Serial.printf("Cell info frame (JK02_32S, %zu bytes) received:  ", data.size());
  //Serial.printf("  ");
  //print_hex_pretty(&data.front(), 150);
  //Serial.printf("\n  ");
  //print_hex_pretty(&data.front()+150, data.size()-150); 
  //Serial.printf("\n");

  float voltage = (float) ((int32_t) jk_get_32bit(150)) * 0.001f;
  float current = (float) ((int32_t) jk_get_32bit(158)) * 0.001f;
  float temperature = (float) ((int16_t) jk_get_16bit(162)) * 0.1f;   // Second sensor is offset 164

  Serial.printf("SOC: %d  Voltage: %g  Current: %g  Temperature: %g SOH: %d\n", data[173], voltage, current, temperature, data[190]);

  SendN2kBatteryLevel(1, data[173], data[190]);
  SendN2kBatteryStatus(1, voltage, current, temperature);
}

static const uint16_t MIN_RESPONSE_SIZE = 300;
static const uint16_t MAX_RESPONSE_SIZE = 384 + 16;

std::vector<uint8_t> frame_buffer;
unsigned long last_bms_n2k_time = 0;  // In ms


void assemble(const uint8_t *data, uint16_t length) {

  if (length <= 4) {
    frame_buffer.clear();
    return;  // Ignore the 4-byte AT/r/n frames that are constantly sent
  }

  if (frame_buffer.size() > MAX_RESPONSE_SIZE) {
    Serial.printf("Frame dropped because of excessive length\n");
    frame_buffer.clear();
    return;
  }

  // Flush buffer on every preamble
  if (length >= 4 && data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
    frame_buffer.clear();
  }

  frame_buffer.insert(frame_buffer.end(), data, data + length);

  if (frame_buffer.size() < MIN_RESPONSE_SIZE) {
    return;  // Need more data
  }

  // We have assembled a hopefully valid frame.

  // The BMS streams about 2 frames per second.  We can
  // throw away most of them because we only need to send
  // the battery level to N2K every 2 or so seconds.
  unsigned long cur_time = millis();
  if (cur_time - last_bms_n2k_time < 2000) {
    frame_buffer.clear();
    return;
  } 
  
    
  // Make a copy so we can immediately clear the main buffer
  std::vector<uint8_t> frame(frame_buffer.begin(), frame_buffer.end());
  frame_buffer.clear();

  // Even if the frame is 320 bytes long the CRC is at position 300 in front of 0xAA 0x55 0x90 0xEB
  const uint16_t frame_size = 300;  // frame_buffer.size();
  uint8_t computed_crc = crc(&frame[0], frame_size - 1);
  uint8_t remote_crc = frame[frame_size - 1];
  if (computed_crc != remote_crc) {
    Serial.printf("CRC check failed! 0x%02X != 0x%02X\n", computed_crc, remote_crc);
    return;
  }
  
  if (frame[4] == 2) {
  // Cell data frame
    last_bms_n2k_time = cur_time;
    process_cell_data_frame(frame);
  }

}



static void BMSDataNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  assemble(pData, length);
}




class MyBMSClientCallbacks : public BLEClientCallbacks {
  void onConnect(NimBLEClient* client) {
    assert(client == pBMSClient);
  }

  void onDisconnect(NimBLEClient* client) {
    assert(client == pBMSClient);
    Serial.println("BMS onDisconnect");
  }
};

MyBMSClientCallbacks bmsClientCallbacks;

#endif

unsigned long last_wind_batt_n2k_time = 0;  // In ms

bool connectToCalypsoServer(const NimBLEAdvertisedDevice* device) {
  Serial.print("Forming a connection to ");
  Serial.println(device->getAddress().toString().c_str());

  assert(pCalypsoClient && !pCalypsoClient->isConnected());
  
  // Connect to the remote BLE Server.
  pCalypsoClient->connect(device); 

  Serial.printf("Connected to Calypso server %s\n", device->getAddress().toString().c_str());
  Serial.printf("MTU: %d\n", pCalypsoClient->getMTU()); 

  // Obtain a reference to the service we are after in the remote BLE server.
  NimBLERemoteService* pCalypsoService = pCalypsoClient->getService(CALYPSO_DATA_SERVICE);
  if (pCalypsoService == nullptr) {
    Serial.print("Failed to find data service UUID: ");
    Serial.println(CALYPSO_DATA_SERVICE.toString().c_str());
    pCalypsoClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain the data characteristic in the service of the remote BLE server.
  pWindDataCharacteristic = pCalypsoService->getCharacteristic(WIND_DATA_CHARACTERISTIC);
  if (!pWindDataCharacteristic) {
    Serial.print("Failed to find wind data characteristic UUID: ");
    Serial.println(WIND_DATA_CHARACTERISTIC.toString().c_str());
    pCalypsoClient->disconnect();
    return false;
  }
  assert(pWindDataCharacteristic->canNotify());
  pWindDataCharacteristic->subscribe(true, windDataNotifyCallback);
  Serial.println(" - Found wind data characteristic");

  // Obtain references to other interesting characteristics in the service of the remote BLE server.
  // These are not needed for NMEA2000 or relay support, just user convenience.
  NimBLERemoteService* pWindService = pCalypsoClient->getService(WIND_SERVICE);

  pAwsCharacteristic = pWindService->getCharacteristic(AWS_CHARACTERISTIC);
  if (pAwsCharacteristic == nullptr) {
    Serial.print("Failed to find AWS characteristic UUID: ");
    Serial.println(AWS_CHARACTERISTIC.toString().c_str());
    disconnectFromCalypsoServer();
    return false;
  }
  Serial.println(" - Found AWS characteristic");

  assert(pAwsCharacteristic->canNotify());
  pAwsCharacteristic->subscribe(true, awsNotifyCallback);


  pAwdCharacteristic = pWindService->getCharacteristic(AWD_CHARACTERISTIC);
  if (pAwdCharacteristic == nullptr) {
    Serial.print("Failed to find AWD characteristic UUID: ");
    Serial.println(AWD_CHARACTERISTIC.toString().c_str());
    disconnectFromCalypsoServer();
    return false;
  }
  Serial.println(" - Found AWD characteristic");

  assert(pAwdCharacteristic->canNotify());
  pAwdCharacteristic->subscribe(true, awdNotifyCallback);

  // Obtain a reference to the battery service and characteristic.
  // Tolerate a missing battery service.
  // BTW, the notification for this data seem to be broken.

  NimBLERemoteService* pBatteryService = pCalypsoClient->getService(BATTERY_SERVICE);
  if (pBatteryService) {
    Serial.println(" - Found our battery service");
    pBatteryLevelCharacteristic = pBatteryService->getCharacteristic(BATTERY_LEVEL_CHARACTERISTIC);
    if (pBatteryLevelCharacteristic) {
      assert(pBatteryLevelCharacteristic->canNotify());

      Serial.println(" - Found Battery Level characteristic");
      pBatteryLevelCharacteristic->subscribe(true, batteryLevelNotifyCallback);
    } else {
      Serial.print("Failed to find battery level characteristic UUID: ");
      Serial.println(BATTERY_LEVEL_CHARACTERISTIC.toString().c_str());
    }
  } else {
    Serial.print("Failed to find battery service UUID: ");
    Serial.println(BATTERY_SERVICE.toString().c_str());
  }

  last_wind_batt_n2k_time = millis();

  return true;
}

bool disconnectFromCalypsoServer() {
  assert(pCalypsoClient);
  pCalypsoClient->disconnect();
  Serial.println(" - Disconnected from Calypso server");
  
  return true;
}


#if DO_BMS

unsigned long last_bms_command_time = 0;  // In ms


bool connectToBMSServer(const NimBLEAdvertisedDevice* device) {
  Serial.print("Forming a connection to ");
  Serial.println(device->getAddress().toString().c_str());

  assert(pBMSClient && !pBMSClient->isConnected());
  
  // Connect to the remote BLE Server.
  pBMSClient->connect(device);  
  
  Serial.printf("Connected to BMS server: %s\n", device->getAddress().toString().c_str());
  Serial.printf("MTU: %d\n", pBMSClient->getMTU()); 

  // Obtain a reference to the service we are after in the remote BMS server.
  NimBLERemoteService* pBMSService = pBMSClient->getService(BMS_DATA_SERVICE);
  if (pBMSService == nullptr) {
    Serial.print("Failed to find BMS data service UUID: ");
    Serial.println(BMS_DATA_SERVICE.toString().c_str());
    pBMSClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain the data characteristic in the service of the remote BLE server.
  pBMSCharacteristic = pBMSService->getCharacteristic(BMS_DATA_CHARACTERISTIC);
  if (!pBMSCharacteristic) {
    Serial.print("Failed to find BMS data characteristic UUID: ");
    Serial.println(BMS_DATA_CHARACTERISTIC.toString().c_str());
    pBMSClient->disconnect();
    return false;
  }

  // Request notifications from it.
  assert(pBMSCharacteristic->canNotify());
  pBMSCharacteristic->subscribe(true, BMSDataNotifyCallback);
  Serial.println(" - Found BMS data characteristic");

  last_bms_command_time = millis();
  last_bms_n2k_time = millis();


  return true;
}

bool disconnectFromBMSServer() {
  assert(pBMSClient);
  pBMSClient->disconnect();
  Serial.println(" - Disconnected from BMS server");
  
  return true;
}

#endif


static const NimBLEAdvertisedDevice* advertisedCalypsoDevice = nullptr;
#if DO_BMS
static const NimBLEAdvertisedDevice* advertisedBMSDevice = nullptr;
#endif


/**
 * Scan for BLE servers and find the first one(s) that advertises the services we are looking for.
 */
class MyScanCallbacks : public NimBLEScanCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    // We have found a device, let us now see if it is a Calypso wind meter.
    // We look for the mystery service to distinguish a real Calypso device from another device running this program.
    if (!pCalypsoClient->isConnected() && !advertisedCalypsoDevice) {
      if (advertisedDevice->getName() == "ULTRASONIC" && advertisedDevice->isAdvertisingService(WIND_SERVICE) &&
          advertisedDevice->isAdvertisingService(CALYPSO_MYSTERY_SERVICE)) {
        Serial.println("Found Calypso device.");
        advertisedCalypsoDevice = advertisedDevice;
      }
    }

#if DO_BMS
    // let us now see if it is a BMS device.
    if (!pBMSClient->isConnected() && !advertisedBMSDevice) {
      if (advertisedDevice->getName() == "40906430730" && advertisedDevice->isAdvertisingService(BMS_DATA_SERVICE)) {
        Serial.println("Found BMS device");
        advertisedBMSDevice = advertisedDevice;
      }  
    }
#endif

#if DO_BMS
    if (advertisedCalypsoDevice && advertisedBMSDevice) {
      NimBLEDevice::getScan()->stop();
    }
#else
    if (advertisedCalypsoDevice) {
      NimBLEDevice::getScan()->stop();
    }
#endif
  }    // onResult
};

MyScanCallbacks scanCallbacks;


void startScan() {
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 10 seconds.
  assert(!NimBLEDevice::getScan()->isScanning());

  Serial.println("Starting scan...");
  NimBLEScan* pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(&scanCallbacks);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10000);  // 10000 milliseconds
}


// Only the onDisconnect() callback is really needed.
class MyServerCallbacks : public NimBLEServerCallbacks {

  void onConnect(NimBLEServer* pWindServer, NimBLEConnInfo& connInfo) override {
    Serial.println("Server: Client connected");
    numConnectedClients++;
    if (numConnectedClients <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
      // Unfortunately the ESP32S3 crashes after the second client connects...
      //NimBLEDevice::startAdvertising();   // Restart advertising to allow multiple connections
    }
  };

  void onDisconnect(NimBLEServer* pWindServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.println("Server: Client disconnected");
    numConnectedClients--;
    NimBLEDevice::startAdvertising();
  };

  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
    Serial.printf("Server: MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  };
};

MyServerCallbacks serverCallbacks;


#if USE_BLE_OTA
class myOTACallbacks : public BLEOTACallbacks {
public:
    void beforeStartOTA() override {
        Serial.println("beforeStartOTA called!\n");
        doingOta = true;
    }
};

myOTACallbacks otaCallbacks;
#endif

void startBLEServer() {
  // Server set-up

  NimBLEServer* pWindServer = NimBLEDevice::createServer();
  pWindServer->setCallbacks(&serverCallbacks);

  NimBLEService* pCalypsoDataService = pWindServer->createService(CALYPSO_DATA_SERVICE);

  // Our characterics are all read-only, so no callbacks (e.g. onWrite()) are needed.
  pWindDataServerCharacteristic = pCalypsoDataService->createCharacteristic(WIND_DATA_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pWindDataServerCharacteristic->setValue("\0\0\0\0\0\0\0\0\0\0");
  pWindDataServerCharacteristic->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Principal");

  // Create two r/w characteristics that the clients expect to see, but we ignore
  NimBLECharacteristic *chr = pCalypsoDataService->createCharacteristic(DATA_RATE_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  chr->setValue(uint8_t(1));
  chr->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Data rate");

  chr = pCalypsoDataService->createCharacteristic(PITCH_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  chr->setValue(uint8_t(0));
  chr->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Activate clinometer and eCompass");



  // We relay the wind service for user convenience - it is redundant with the calypso data service.
  NimBLEService* pWindService = pWindServer->createService(WIND_SERVICE);

  // Our characterics are all read-only, so no callbacks (e.g. onWrite()) are needed.
  pAwsServerCharacteristic = pWindService->createCharacteristic(AWS_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pAwsServerCharacteristic->setValue("\0\0");
  pAwsServerCharacteristic->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Wind speed");

  pAwdServerCharacteristic = pWindService->createCharacteristic(AWD_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pAwdServerCharacteristic->setValue("\0\0");
  pAwdServerCharacteristic->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Wind direction");


  NimBLEService* pBatteryService = pWindServer->createService(BATTERY_SERVICE);

  pBatteryServerCharacteristic = pBatteryService->createCharacteristic(BATTERY_LEVEL_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pBatteryServerCharacteristic->setValue("\0");

#if USE_BLE_OTA
  // Add OTA and Device Information Service
  BLEOTA.begin(pWindServer);
  BLEOTA.setCallbacks(&otaCallbacks);

  BLEOTA.setModel(MODEL_NAME);
  BLEOTA.setSerialNumber(String(serial_num));
  BLEOTA.setFWVersion(FIRMWARE_VERSION);
  BLEOTA.setHWVersion(MODEL_VERSION);
  BLEOTA.setManufactuer(MANUFACTURER);

  BLEOTA.init();
#endif

  Serial.println("Server advertising starts");

  // We have to spoof a real Calypso Portable Mini, so that apps will recognize and connect to us.
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("ULTRASONIC");
  pAdvertising->addServiceUUID(pWindService->getUUID());
  
#if USE_BLE_OTA  
  pAdvertising->addServiceUUID(BLEOTA.getBLEOTAuuid());
#endif
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();
}



#if USE_ELEGANT_OTA
const char* ssid = "calypso";
const char* password = "1234567890";

const char* hostname = "calypso";

DNSServer dnsServer;

AsyncWebServer webServer(80);

bool wifiRunning = false;
#endif


#if DO_BMS

std::array<uint8_t, 20> build_frame(uint8_t address, uint32_t value, uint8_t length) {
  std::array<uint8_t, 20> frame{};
  frame[0] = 0xAA;     // start sequence
  frame[1] = 0x55;     // start sequence
  frame[2] = 0x90;     // start sequence
  frame[3] = 0xEB;     // start sequence
  frame[4] = address;  // holding register
  frame[5] = length;   // size of the value in byte
  frame[6] = value >> 0;
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;
  frame[19] = crc(frame.data(), frame.size() - 1);
  return frame;
}

bool send_bms_command(uint8_t opcode) {
  if (!pBMSClient->isConnected()) {
    return false;
  } else {
    Serial.printf("Sending BMS command 0x%x\n", opcode);
    auto frame = build_frame(opcode, 0x00000000, 0x00);
    pBMSCharacteristic->writeValue(frame, frame.size());
    return true;
  }
}


#endif

void setup() {


  Serial.begin(115200);
  delay(1000);

  Serial.print("setup 0\n");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);




  // Note: The ESP32 EEPROM library is used differently than the official Arduino version.
  persistentData.begin();

  if (!persistentData.data_valid) {
    // First-time initialization of persistent data
    persistentData.init();
    persistentData.commit();
    Serial.println("Initialized persistent data");
  }

  delay(1000);

  // Generate unique serial number from chip id
  uint8_t chipid[6];
  esp_efuse_mac_get_default(chipid);

  serial_num = 0;
  for (int i = 0; i < 6; i++) serial_num += (chipid[i] << (7 * i));

#if USE_ELEGANT_OTA
  // Set WiFi AP Mode
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);  // Usually 192.168.4.1


  WiFi.softAPsetHostname(hostname);


  // Connect to Wi-Fi network with SSID and password
  // Remove the password parameter, if you want the AP (Access Point) to be open


  Serial.print("AP SSID: ");
  Serial.println(WiFi.softAPSSID());

  Serial.print("hostname: ");
  Serial.println(WiFi.softAPgetHostname());

  dnsServer.start(53, "*", WiFi.softAPIP());

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Hi! I am ESP32 Calypso.");
  });

  webServer.begin();
  Serial.println("Web server started");

  ElegantOTA.begin(&webServer);  // Start ElegantOTA

  wifiRunning = true;
#endif

  // For NMEA0183 output on ESP32
  Serial2.begin(4800, SERIAL_8N1, 16 /*Rx pin*/, 17 /*Tx pin*/, true /*invert*/);

  NMEA0183 = new tNMEA0183(&Serial2);
  NMEA0183->Open();

  Serial.print("setup 1\n");

  // Reserve enough buffer for sending all messages.
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  

  // Set product information
  NMEA2000.SetProductInformation(String(serial_num).c_str(),           // Manufacturer's Model serial code
                                 1,                   // Manufacturer's product code  (made up)
                                 MODEL_NAME,       // Manufacturer's Model ID
                                 FIRMWARE_VERSION,             // Manufacturer's Software version code
                                 MODEL_VERSION,                // Manufacturer's Model version
                                 1                     // Load Equivalency  (units of 50mA)
                                 );

  // Set device information
  NMEA2000.SetDeviceInformation(serial_num,   // Unique number. Use e.g. Serial number.
                                130,  // Device function=Atmospheric. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85,   // Device class=External Environment. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2006  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );


  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  Serial.printf("NMEA2000 device address initialized to 0x%x\n", persistentData.node_address);
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, persistentData.node_address);  // Read stored last NodeAddress, default 34
  NMEA2000.EnableForward(false);
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(transmitMessages);
  Serial.print("setup 2\n");

  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);

  if (!NMEA2000.Open()) {
    Serial.println("NMEA2000.Open failed");
  }

  NimBLEDevice::init("RELAY");

  pCalypsoClient = NimBLEDevice::createClient();
  pCalypsoClient->setClientCallbacks(&calypsoClientCallbacks);
  Serial.println(" - Created Calypso client");

#if DO_BMS
  pBMSClient = NimBLEDevice::createClient();
  pBMSClient->setClientCallbacks(&bmsClientCallbacks);
  Serial.println(" - Created BMS client");

#endif

  startBLEServer();
}






// *****************************************************************************
void loop() {


  if (doingOta) {
    // Stop doing everything else
    if (pCalypsoClient->isConnected()) {
      disconnectFromCalypsoServer();
    }
#if DO_BMS
    if (pBMSClient->isConnected()) {
      disconnectFromBMSServer();
    }
#endif

    NimBLEDevice::getScan()->stop();
  } else {
    // Normal flow
  
    bool need_scan = false;

    if (!pCalypsoClient->isConnected()) {
      if (advertisedCalypsoDevice) {
        if (connectToCalypsoServer(advertisedCalypsoDevice)) {
          Serial.println("We are now connected to the Calypso Server.");
        } else {
          Serial.println("We have failed to connect to the Calypso server.");
        }
        advertisedCalypsoDevice = nullptr;
      } else {
        need_scan = true;
      }
    }

#if DO_BMS
   if (!pBMSClient->isConnected()) {
      if (advertisedBMSDevice) {
        if (connectToBMSServer(advertisedBMSDevice)) {
          Serial.println("We are now connected to the JK-BMS Server.");
        } else {
          Serial.println("We have failed to connect to the BMS server.");
        }
        advertisedBMSDevice = nullptr;
      } else {
        need_scan = true;
      }
    }
#endif

    if (need_scan && !NimBLEDevice::getScan()->isScanning()) {
      startScan();
    }


    NMEA2000.ParseMessages();

    // Check if SourceAddress has changed (due to address conflict on bus)
    if (NMEA2000.ReadResetAddressChanged()) {
      // Save potentially changed Source Address to NVS memory
      persistentData.node_address = NMEA2000.GetN2kSource();
      persistentData.commit();
      Serial.printf("NMEA2000 device address changed to 0x%x\n", persistentData.node_address);
    }
  

    if (notified) {   // This flag gets set about once per second.
      notified = false;
    
      if (pBatteryLevelCharacteristic) {
        // Read the battery level characteristic  (it has more resolution than the byte in the main data string)
        uint8_t level = pBatteryLevelCharacteristic->readValue()[0];

        if (pBatteryServerCharacteristic) {
          pBatteryServerCharacteristic->setValue(&level, sizeof(level));
        }

        // We only need to send the battery level to N2K every 2 or so seconds.
        unsigned long cur_time = millis();
        if (cur_time - last_wind_batt_n2k_time >= 2000) {
          last_wind_batt_n2k_time = cur_time;
          SendN2kBatteryLevel(2, level);
        } 
      }
    }
  }

#if USE_BLE_OTA
  BLEOTA.process();
#endif

//Serial.printf("wifi %d\n", wifiRunning);
#if USE_ELEGANT_OTA
  if (wifiRunning) {
    ElegantOTA.loop();
    dnsServer.processNextRequest();
  }


  // If the Wifi AP has been running for a minute or so and nobody has connected to it,
  // turn it off.
  if (wifiRunning && millis() > 60000 && WiFi.softAPgetStationNum() == 0) {
    WiFi.softAPdisconnect(true);
    Serial.printf("Wifi AP turned off.\n");
    wifiRunning = false;
  }
#endif

#if DO_BMS

  unsigned long cur_time = millis();

  static int state = 0;

  if (!pBMSClient->isConnected()) {
    state = 0;  // If the connection is dropped and re-established, we must re-do the command sequence.
  } else {
    if (cur_time - last_bms_command_time > 2000) {
      last_bms_command_time = cur_time;

      if (state == 0) {
        send_bms_command(0x97);
        state = 1;
      } else if (state == 1) {
        send_bms_command(0x96);
        state = 2;
      }
    }
  }

#endif

}
