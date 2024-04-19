//nrf // server //peripheral //advertising

#include <Arduino.h>
#include <bluefruit.h>
#include <Wire.h>
#include <hp_BH1750.h>
#include <HTU2xD_SHT2x_Si70xx.h>

// #define bleServerName "Nordic_Perihperal"

hp_BH1750 BH1750;       //  create the sensor
float htValue; //to store T/RH result
HTU2xD_SHT2x_SI70xx ht2x(HTU2xD_SENSOR, HUMD_12BIT_TEMP_14BIT); //sensor type, resolution

float temp;
float hum;
float lux;

// Timer variables
// unsigned long lastTime = 0;
// unsigned long timerDelay = 30000;

bool deviceConnected = false;

// #define SERVICE_UUID "0x00001523-0x1212-0xefde-0x1523-0x785feabcd123" //nrf uuid

BLEService nrfService("0x00001523-0x1212-0xefde-0x1523-0x785feabcd123");

// Temp Characteristic and Descriptor
// BLEService temperatureService("0e869c86-1148-4a3e-9882-37b66dc4d073");
BLECharacteristic temperatureCharacteristic("0e869c86-1148-4a3e-9882-37b66dc4d073", BLERead | BLENotify, 4);

// Hum Characteristic and Descriptor
// BLEService humidityService("ca73b3ba-39f6-4ab3-91ae-186dc9577d99");
BLECharacteristic humidityCharacteristic("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLERead | BLENotify, 4);

// Lux Characteristic and Descriptor
// BLEService luxService("3487a81a-a563-4034-ac91-c19a9dd2f3be");
BLECharacteristic luxCharacteristic("3487a81a-a563-4034-ac91-c19a9dd2f3be", BLERead | BLENotify, 4);

void setup() {
  Serial.begin(115200);
  
  while (ht2x.begin() != true) //reset sensor, set heater off, set resolution, check power (sensor doesn't operate correctly if VDD < +2.25v)
  {
    Serial.println(F("HTU2xD/SHT2x not connected, fail or VDD < +2.25v")); //(F()) save string to flash & keeps dynamic memory free

    delay(5000);
  }

  Serial.println(F("HTU2xD/SHT2x OK"));

  // delay(500);

  bool avail = BH1750.begin(BH1750_TO_GROUND);// init the sensor with address pin connetcted to ground
  // BH1750.calibrateTiming(); //  Calibrate for fastest conversion time
  // result (bool) wil be be "false" if no sensor found
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
   while (true) {};                                        
  }

  BH1750.start(); 

  //nrfService.addCharacteristic(temperatureCharacteristic);

  // BLECharacteristic temperatureCharacteristic = BLECharacteristic(nrfService, "0e869c86-1148-4a3e-9882-37b66dc4d073", BLERead | BLENotify, 4);
  // BLECharacteristic humidityCharacteristic = BLECharacteristic(nrfService, "ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLERead | BLENotify, 4);
  // BLECharacteristic luxCharacteristic = BLECharacteristic(nrfService, "3487a81a-a563-4034-ac91-c19a9dd2f3be", BLERead | BLENotify, 4);
  
  // Set permissions for the service
  nrfService.setPermission(SECMODE_OPEN, SECMODE_OPEN);

  // Set permissions, properties, and other configurations for each characteristic
  temperatureCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  humidityCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  luxCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);

  Bluefruit.begin();  
  nrfService.begin();
  Bluefruit.setName("Nordic_Peripheral");
  
  Bluefruit.setTxPower(4);    // Check your power requirements and adjust accordingly
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);


  temperatureCharacteristic.begin();
  // humidityService.begin();
  humidityCharacteristic.begin();
  // luxService.begin();
  luxCharacteristic.begin();

  startAdv(); //setupAdv();
  Bluefruit.Advertising.start();
}

void startAdv() {
  
  // Bluefruit.Advertising.addName("Nordic_Peripheral");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE | BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addService(nrfService);
  
  // Bluefruit.Advertising.addService(temperatureService);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds 

  // Bluefruit.Advertising.addService(BLEService& temperatureService, BLEService& humidityService, BLEService& luxService);
}


void loop() {
  // Your code here
  lux = BH1750.getLux();
  htValue = ht2x.readTemperature();

  if(Bluefruit.connected()){
    // Serial.println("HK HK KK HH HR HR RR HH");

    // Serial.print(F("Lux.......................: "));
    // Serial.println(lux);
    // Serial.print(F("Temperature...............: "));
    // Serial.println(htValue);

    temperatureCharacteristic.write(&htValue, sizeof(htValue));
    humidityCharacteristic.write(&hum, sizeof(hum));
    luxCharacteristic.write(&lux, sizeof(lux));

    delay(5000);
  }
}

void connect_callback(uint16_t conn_handle) {
  // Handle connection event
  deviceConnected = true;

}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  // Handle disconnection event
  deviceConnected = false;
}
