//#include <NMEA2000_esp32.h>


/***********************************************************************//**
  An Arduino/ESP32 program to read BLE data from a Calypso wireless wind meter
  and re-transmit it over NMEA2000.
*/

#include <Arduino.h>
//#define N2k_SPI_CS_PIN 53    // If you use mcp_can and CS pin is not 53, uncomment this and modify definition to match your CS pin.
//#define N2k_CAN_INT_PIN 21   // If you use mcp_can and interrupt pin is not 21, uncomment this and modify definition to match your interrupt pin.
//#define USE_MCP_CAN_CLOCK_SET 8  // If you use mcp_can and your mcp_can shield has 8MHz chrystal, uncomment this.
//#define ESP32_CAN_TX_PIN GPIO_NUM_16 // If you use ESP32 and do not have TX on default IO 16, uncomment this and and modify definition to match your CAN TX pin.
//#define ESP32_CAN_RX_PIN GPIO_NUM_17 // If you use ESP32 and do not have RX on default IO 4, uncomment this and and modify definition to match your CAN TX pin.
//#define NMEA2000_ARDUINO_DUE_CAN_BUS tNMEA2000_due::CANDevice1  // If you use Arduino DUE and want to use CAN bus 1 instead of 0, uncomment this.
//#define NMEA2000_TEENSY_CAN_BUS 1 // If you use Teensy 3.5 or 3.6 and want to use second CAN bus, uncomment this.

#define ESP32_CAN_TX_PIN GPIO_NUM_3  // Set CAN TX port   This is the pin labeled D2 on the Seeed Xiao ESP32S3 board
#define ESP32_CAN_RX_PIN GPIO_NUM_2   // Set CAN RX port  This is the pin labeled D on the Seeed Xiao ESP32S3 board

#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>


#include <HardwareSerial.h>
#include <Preferences.h>

#include "esp_mac.h"

#include <BLEDevice.h>


tNMEA0183* NMEA0183 = nullptr;


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




// List here messages your device will transmit.
const unsigned long transmitMessages[] PROGMEM={130306L,0};   // Apparent wind message





// Calypso-defined values
const char *CALYPSO_DATA_SERVICE = "181A";    // Industry standard
const char *AWS_CHARACTERISTIC = "2A72";   // Industry standard
const char *AWD_CHARACTERISTIC = "2A73";   // Industry standard

const char *CALYPSO_BATTERY_SERVICE = "180F";
const char *BATTERY_LEVEL_CHARACTERISTIC =   "2a19";  // Standard

// The remote service we wish to connect to.
static BLEUUID serviceUUID(CALYPSO_DATA_SERVICE);
// The characteristic of the remote service we are interested in.
static BLEUUID    awsUUID(AWS_CHARACTERISTIC);
static BLEUUID    awdUUID(AWD_CHARACTERISTIC);


static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pAwsCharacteristic;
static BLERemoteCharacteristic* pAwdCharacteristic;

static BLEAdvertisedDevice* myDevice;



double convertValue(const uint8_t* data) {

  return ((data[1]*256)+data[0])/100.0;
}


double lastAWS = 0.0;   


static void awsNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.print(length);

    lastAWS = convertValue(pData);
    Serial.printf("  Value : %2.1f\n", lastAWS);
}



static void awdNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.print(length);

    double lastAWD = convertValue(pData);

    Serial.printf("  Value : %3.0f\n", lastAWD);

    // Send this AWD and the most recent AWS to NMEA2000
    // Note that the Calypso sends the AWD notification immediately after an AWS notification.
    SendN2kWind(lastAWS, lastAWD);
}




class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
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

    // Connect to the remote BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

    Serial.printf("Connecting to: %s\n", myDevice->getAddress().toString().c_str());

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain references to the characteristics in the service of the remote BLE server.
    pAwsCharacteristic = pRemoteService->getCharacteristic(awsUUID);
    if (pAwsCharacteristic == nullptr) {
      Serial.print("Failed to find AWS characteristic UUID: ");
      Serial.println(awsUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found AWS characteristic");   

    assert(pAwsCharacteristic->canNotify());
    pAwsCharacteristic->registerForNotify(awsNotifyCallback);

  
    pAwdCharacteristic = pRemoteService->getCharacteristic(awdUUID);
    if (pAwdCharacteristic == nullptr) {
      Serial.print("Failed to find AWD characteristic UUID: ");
      Serial.println(awdUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found AWD characteristic");

    
    assert(pAwdCharacteristic->canNotify());
    pAwdCharacteristic->registerForNotify(awdNotifyCallback);

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
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void startScan() {
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 10 seconds.
  Serial.println("Starting scan...");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false); 
}


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
  NMEA2000.SetProductInformation("002",                // Manufacturer's Model serial code
                                  1,                   // Manufacturer's product code
                                 "Doug Calypso BLE wind monitor",       // Manufacturer's Model ID
                                 __DATE__,             // Manufacturer's Software version code
                                 "1.0",                // Manufacturer's Model version
                                 1                     // Load Equivalency  (units of 50mA)
                                 );

   // Set device information
  NMEA2000.SetDeviceInformation(id,   // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2006  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
			      );


  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  Serial.printf("NMEA2000 device address initialized to 0x%x\n", persistentData.node_address);
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, persistentData.node_address);   // Read stored last NodeAddress, default 34
  NMEA2000.EnableForward(false);
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(transmitMessages);
  Serial.print("setup 2\n");

  if (!NMEA2000.Open()) {
    Serial.println("NMEA2000.Open failed");
  }

  Serial.print("setup 3\n");

  BLEDevice::init("");

}






// *****************************************************************************
void loop() {

  if(!connected && !doConnect && !BLEDevice::getScan()->isScanning()){
     startScan();
  }

  
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the Calyposo BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  }

  
  
  NMEA2000.ParseMessages();

  // Check if SourceAddress has changed (due to address conflict on bus)
  if (NMEA2000.ReadResetAddressChanged()) {
    // Save potentially changed Source Address to NVS memory
    persistentData.node_address = NMEA2000.GetN2kSource();
    persistentData.commit();
    Serial.printf("NMEA2000 device address changed to 0x%x\n", persistentData.node_address);
  }


  delay(1000); // Delay a second between loops.

}


// *****************************************************************************
void SendN2kWind(double windSpeed, double windAngle) {

  digitalWrite(LED_BUILTIN, LOW);  // Indicate NMEA data transmission

  tN2kMsg N2kMsg;
 
  Serial.printf("Transmitting NMEA data: AWS %2.1f  AWD %3.0f\n", windSpeed, windAngle);

  SetN2kWindSpeed(N2kMsg, 1, windSpeed, DegToRad(windAngle), N2kWind_Apprent);
  NMEA2000.SendMsg(N2kMsg);
  
  tNMEA0183Msg NMEA0183Msg;
  if ( NMEA0183SetMWV(NMEA0183Msg, DegToRad(windAngle), NMEA0183Wind_Apparent, windSpeed) ) {
    NMEA0183->SendMessage(NMEA0183Msg);
  }
  
  delay(5); //allow LED to blink and the cpu to switch to other tasks
  digitalWrite(LED_BUILTIN, HIGH);

}
