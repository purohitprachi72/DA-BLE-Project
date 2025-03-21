#include "BLEDevice.h"
#include <Wire.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>

//Default Temperature is in Celsius
//Comment the next line for Temperature in Fahrenheit
#define temperatureCelsius

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "Nordic_Peripheral"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID nrfServiceUUID("0x00001523-0x1212-0xefde-0x1523-0x785feabcd123");

// BLE Characteristics

//Temperature Celsius Characteristic
static BLEUUID temperatureCharacteristicUUID("0e869c86-1148-4a3e-9882-37b66dc4d073");

// Humidity Characteristic
static BLEUUID humidityCharacteristicUUID("ca73b3ba-39f6-4ab3-91ae-186dc9577d99");

// Lux Characteristic
static BLEUUID luxCharacteristicUUID("3487a81a-a563-4034-ac91-c19a9dd2f3be");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

// /******************************modified****************/
// static bool bleConnected = false;

// class MyBLEClientCallbacks : public BLEClientCallbacks {
//     void onConnect(BLEClient* pClient) {
//         bleConnected = true;
//     }

//     void onDisconnect(BLEClient* pClient) {
//         bleConnected = false;
//     }
// };


// /******************************modified****************/

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* temperatureCharacteristic;
static BLERemoteCharacteristic* humidityCharacteristic;
static BLERemoteCharacteristic* luxCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//Variables to store temperature and humidity
char* temperatureChar;
char* humidityChar;
char* luxChar;

//Flags to check whether new temperature and humidity readings are available
boolean newTemperature = false;
boolean newHumidity = false;
boolean newLux = false;

// BLEService nrfService("0x00001523-0x1212-0xefde-0x1523-0x785feabcd123");
//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress *pRemoteService) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remote BLE Server.
  pClient->connect(nrfServiceUUID);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(nrfServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(nrfServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  temperatureCharacteristic = pRemoteService->getCharacteristic(temperatureCharacteristicUUID);
  humidityCharacteristic = pRemoteService->getCharacteristic(humidityCharacteristicUUID);
  luxCharacteristic = pRemoteService->getCharacteristic(luxCharacteristicUUID);

  if (temperatureCharacteristic == nullptr || humidityCharacteristic == nullptr || luxCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  temperatureCharacteristic->registerForNotify(temperatureNotifyCallback);
  humidityCharacteristic->registerForNotify(humidityNotifyCallback);
  luxCharacteristic->registerForNotify(luxNotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 
//When the BLE Server sends a new temperature reading with the notify property
static void temperatureNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store temperature value
  temperatureChar = (char*)pData;
  newTemperature = true;
}

//When the BLE Server sends a new humidity reading with the notify property
static void humidityNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  //store humidity value
  humidityChar = (char*)pData;
  newHumidity = true;
  Serial.print(newHumidity);
}

//When the BLE Server sends a new lux reading with the notify property
static void luxNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  //store lux value
  luxChar = (char*)pData;
  newLux = true;
  Serial.print(newLux);
}

void setup() {
  //Start serial communication
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  //Init BLE device
  BLEDevice::init("");
 
// //modified
//   // Set up BLE client callbacks
//   BLEClient::setClientCallbacks(new MyBLEClientCallbacks());
// //modified

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.

  // if (bleConnected) {
  //     Serial.println("Connected to intended BLE device.");
  //       // You can perform additional actions here once connected
  // } else {
  //     Serial.println("Not connected to intended BLE device.");
  //       // You can handle the case when not connected here
  // }

  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of each Characteristic
      temperatureCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      humidityCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      luxCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  //if new temperature readings are available, print in the OLED
  if (newTemperature && newHumidity && newLux){
    newTemperature = false;
    newHumidity = false;
    newLux = false;
    // printReadings();
    Serial.print("Temperature: ");
    Serial.println(newTemperature);
    Serial.print("Humidity: ");
    Serial.println(newHumidity);
    Serial.print("Lux: ");
    Serial.println(newLux);
    
  }
  delay(1000); // Delay a second between loops.
}