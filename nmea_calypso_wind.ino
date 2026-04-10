
//  An Arduino/ESP32 program to read BLE data from a Calypso wireless wind meter
//  and re-transmit it over NMEA2000.


#include <Arduino.h>

#define USE_ELEGANT_OTA 1
#undef USE_BLE_OTA

#if USE_ELEGANT_OTA
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include <DNSServer.h>
#include <ElegantOTA.h>
#endif



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

#include <NimBLEDevice.h>

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
const unsigned long transmitMessages[] PROGMEM = { 130306L, 127506L, 0 };  // Apparent Wind and DC Status messages



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


// *****************************************************************************
void SendN2kBatteryLevel(int batteryLevel) {

  tN2kMsg N2kMsg;

  Serial.printf("Transmitting NMEA data: Battery Level %d\n", batteryLevel);

  SetN2kDCStatus(N2kMsg, 1, 1, N2kDCt_Battery, batteryLevel, 100 /* state of health % */, 999.9 /* Remaining time */);
  NMEA2000.SendMsg(N2kMsg);
}




// Calypso-defined values
static NimBLEUUID CALYPSO_DATA_SERVICE("180D");      // Unfortunately the same as the standard heart rate service.
static NimBLEUUID WIND_DATA_CHARACTERISTIC("2A39");  // Unfortunately the same as the standard heart rate control point
static NimBLEUUID DATA_RATE_CHARACTERISTIC("A002");  // We ignore this, but we need to expose it for the clients to work.
static NimBLEUUID PITCH_CHARACTERISTIC("A003");  // We ignore this, but we need to expose it for the clients to work.

// These services and characteristics provide the same wind/battery data, in a more user-friendly way.
static NimBLEUUID WIND_SERVICE("181A");        // Industry standard
static NimBLEUUID AWS_CHARACTERISTIC("2A72");  // Industry standard
static NimBLEUUID AWD_CHARACTERISTIC("2A73");  // Industry standard

static NimBLEUUID BATTERY_SERVICE("180F");               // Standard
static NimBLEUUID BATTERY_LEVEL_CHARACTERISTIC("2a19");  // Standard

// The Calypso advertises this service, but does not seem to implement it.
static NimBLEUUID CALYPSO_MYSTERY_SERVICE("8ec90001-f315-4f60-9fb8-838830daea50");



static boolean doConnect = false;
static boolean connected = false;

static int numConnectedClients = 0;

static NimBLERemoteCharacteristic* pWindDataCharacteristic = nullptr;

static NimBLERemoteCharacteristic* pAwsCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pAwdCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pBatteryLevelCharacteristic = nullptr;

static const NimBLEAdvertisedDevice* myDevice;

static NimBLECharacteristic* pWindDataServerCharacteristic = nullptr;
static NimBLECharacteristic* pAwsServerCharacteristic = nullptr;
static NimBLECharacteristic* pAwdServerCharacteristic = nullptr;
static NimBLECharacteristic* pBatteryServerCharacteristic = nullptr;


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

  double AWS = (pData[1] << 8 | pData[0]) * 0.01;
  double AWD  = (pData[3] << 8 | pData[2]); 
	uint8_t batt = pData[5];  // pData[4] seems to be a 0-10 battery level.

  Serial.printf("values : AWS %2.1f  AED: %3.0f  batt  %d\n", AWS, AWD, batt);

  
  // Send the data to NMEA2000
  SendN2kWind(AWS, AWD);
  SendN2kBatteryLevel(batt);

  // If our relay server is running, pass on the new value
  if (pWindDataServerCharacteristic) {
    pWindDataServerCharacteristic->setValue(pData, length);
    pWindDataServerCharacteristic->notify();
  }

  if (pBatteryServerCharacteristic) {
    pBatteryServerCharacteristic->setValue(&batt, sizeof(batt));
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


class MyBLEClientCallback : public BLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
    connected = true;
  }

  void onDisconnect(NimBLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

MyBLEClientCallback clientCallbacks;



bool connectToBLEServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  NimBLEClient* pClient = NimBLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(&clientCallbacks);

  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  // Doug: needed? pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

  Serial.printf("Connecting to: %s\n", myDevice->getAddress().toString().c_str());

  // Obtain a reference to the service we are after in the remote BLE server.
  NimBLERemoteService* pCalypsoService = pClient->getService(CALYPSO_DATA_SERVICE);
  if (pCalypsoService == nullptr) {
    Serial.print("Failed to find data service UUID: ");
    Serial.println(CALYPSO_DATA_SERVICE.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain the data characteristic in the service of the remote BLE server.
  pWindDataCharacteristic = pCalypsoService->getCharacteristic(WIND_DATA_CHARACTERISTIC);
  if (!pWindDataCharacteristic) {
    Serial.print("Failed to find wind data characteristic UUID: ");
    Serial.println(WIND_DATA_CHARACTERISTIC.toString().c_str());
    pClient->disconnect();
    return false;
  }
  assert(pWindDataCharacteristic->canNotify());
  pWindDataCharacteristic->subscribe(true, windDataNotifyCallback);
  Serial.println(" - Found wind data characteristic");



  // Obtain references to other interesting characteristics in the service of the remote BLE server.
  // These are not needed for NMEA2000 or relay support, just user convenience.
  NimBLERemoteService* pWindService = pClient->getService(WIND_SERVICE);

  pAwsCharacteristic = pWindService->getCharacteristic(AWS_CHARACTERISTIC);
  if (pAwsCharacteristic == nullptr) {
    Serial.print("Failed to find AWS characteristic UUID: ");
    Serial.println(AWS_CHARACTERISTIC.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found AWS characteristic");

  assert(pAwsCharacteristic->canNotify());
  pAwsCharacteristic->subscribe(true, awsNotifyCallback);


  pAwdCharacteristic = pWindService->getCharacteristic(AWD_CHARACTERISTIC);
  if (pAwdCharacteristic == nullptr) {
    Serial.print("Failed to find AWD characteristic UUID: ");
    Serial.println(AWD_CHARACTERISTIC.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found AWD characteristic");

  assert(pAwdCharacteristic->canNotify());
  pAwdCharacteristic->subscribe(true, awdNotifyCallback);

  // Obtain a reference to the battery service and characteristic.
  // Tolerate a missing battery service.
  // BTW, the notification for this data seem to be broken.

  NimBLERemoteService* pBatteryService = pClient->getService(BATTERY_SERVICE);
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


  connected = true;
  return true;
}


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyScanCallbacks : public NimBLEScanCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    // We have found a device, let us now see if it is a Calypso wind meter.
    if (advertisedDevice->getName() == "ULTRASONIC" && advertisedDevice->isAdvertisingService(WIND_SERVICE)) {

      Serial.println("Found device, stopping scan...");

      NimBLEDevice::getScan()->stop();
      myDevice = advertisedDevice;
      doConnect = true;

    }  // Found our server
  }    // onResult
};

MyScanCallbacks scanCallbacks;


void startScan() {
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 10 seconds.
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

  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.println("Server: Client connected");
    numConnectedClients++;
    if (numConnectedClients <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
      NimBLEDevice::startAdvertising();   // Restart advertising to allow multiple connections
    }
  };

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.println("Server: Client disconnected");
    numConnectedClients--;
    NimBLEDevice::startAdvertising();
  };

  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
    Serial.printf("Server: MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  };
};

MyServerCallbacks serverCallbacks;


void startBLEServer() {
  // Server set-up

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  NimBLEService* pCalypsoDataService = pServer->createService(CALYPSO_DATA_SERVICE);

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
  NimBLEService* pWindService = pServer->createService(WIND_SERVICE);

  // Our characterics are all read-only, so no callbacks (e.g. onWrite()) are needed.
  pAwsServerCharacteristic = pWindService->createCharacteristic(AWS_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pAwsServerCharacteristic->setValue("\0\0");
  pAwsServerCharacteristic->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Wind speed");

  pAwdServerCharacteristic = pWindService->createCharacteristic(AWD_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pAwdServerCharacteristic->setValue("\0\0");
  pAwdServerCharacteristic->createDescriptor(NimBLEUUID("2901"), NIMBLE_PROPERTY::READ)->setValue("Wind direction");


  NimBLEService* pBatteryService = pServer->createService(BATTERY_SERVICE);

  pBatteryServerCharacteristic = pBatteryService->createCharacteristic(BATTERY_LEVEL_CHARACTERISTIC, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pBatteryServerCharacteristic->setValue("\0");

#if USE_BLE_OTA
  // Add OTA and Device Information Service
  BLEOTA.begin(pServer);

  //BLEOTA.setModel(MODEL_NAME);
  //BLEOTA.setSerialNumber(SERIAL_NUM);
  //BLEOTA.setFWVersion(FIRMWARE_VERSION);
  //BLEOTA.setHWVersion(MODEL_VERSION);
  //BLEOTA.setManufactuer(MANUFACTURER);

  BLEOTA.init();
#endif

  Serial.println("Server advertising starts");

  // We have to spoof a real Calypso Portable Mini, so that apps will recognize and connect to us.
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("ULTRASONIC");
  pAdvertising->addServiceUUID(pWindService->getUUID());
  pAdvertising->addServiceUUID(CALYPSO_MYSTERY_SERVICE);
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

  // Generate unique number from chip id
  uint8_t chipid[6];
  esp_efuse_mac_get_default(chipid);

  uint32_t id = 0;
  for (int i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  // Set product information
  NMEA2000.SetProductInformation("002",                            // Manufacturer's Model serial code
                                 1,                                // Manufacturer's product code
                                 "Doug Calypso BLE wind monitor",  // Manufacturer's Model ID
                                 __DATE__,                         // Manufacturer's Software version code
                                 "1.0",                            // Manufacturer's Model version
                                 1                                 // Load Equivalency  (units of 50mA)
  );

  // Set device information
  NMEA2000.SetDeviceInformation(id,   // Unique number. Use e.g. Serial number.
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

  Serial.print("setup 3\n");

  NimBLEDevice::init("RELAY");

  startBLEServer();
}






// *****************************************************************************
void loop() {

  if (!connected && !doConnect && !NimBLEDevice::getScan()->isScanning()) {
    startScan();
  }


  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToBLEServer()) {
      Serial.println("We are now connected to the Calyposo BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  }

#if USE_BLE_OTA
  BLEOTA.process();
#endif

  NMEA2000.ParseMessages();

  // Check if SourceAddress has changed (due to address conflict on bus)
  if (NMEA2000.ReadResetAddressChanged()) {
    // Save potentially changed Source Address to NVS memory
    persistentData.node_address = NMEA2000.GetN2kSource();
    persistentData.commit();
    Serial.printf("NMEA2000 device address changed to 0x%x\n", persistentData.node_address);
  }


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
}
