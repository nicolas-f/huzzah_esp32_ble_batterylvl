

/*Program to use GATT service on ESP32 to send Battery Level
 * ESP32 works as server - Mobile as client
 * Program by: B.Aswinth Raj
 * Dated on: 13-10-2018
 * Website: www.circuitdigest.com
 */
#define led 13
int sensorPin = A13;    // select the input pin for the potentiometer
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h> //Library to use BLE as server
#include <BLE2902.h> 
#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP_HIGH  60        //Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP_MEDIUM  300
#define TIME_TO_SLEEP_LOW  1800
#define TIME_CLIENT_WAITING 15

bool _BLEClientConnected = false;

#define BatteryService BLEUUID((uint16_t)0x180F) 
BLECharacteristic BatteryLevelCharacteristic(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true; 
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
      digitalWrite(led, LOW);
    }
};

void InitBLE() {
  BLEDevice::init("UMRAE Noise Monitoring");

  // push power
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL3, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL4, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL5, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL6, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL8, ESP_PWR_LVL_P9);
  //  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);

  
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pBattery = pServer->createService(BatteryService);

  pBattery->addCharacteristic(&BatteryLevelCharacteristic);
  BatteryLevelDescriptor.setValue("Battery level");
  BatteryLevelCharacteristic.addDescriptor(&BatteryLevelDescriptor);
  BatteryLevelCharacteristic.addDescriptor(new BLE2902());

  pServer->getAdvertising()->addServiceUUID(BatteryService);

  pBattery->start();


  
  // Start advertising
  pServer->getAdvertising()->start();
}

void stopBle() {  
    BLEDevice::deinit(true);
}
const float bat_min = 3.2; // volts - cut off voltage
const float bat_max = 4.2;  // volts - max voltage
void ble_status_led(void *pvParameters) {
  int lastLedStatus = LOW;
  while(1) {
    if(_BLEClientConnected) {
      digitalWrite(led, HIGH);
    } else {
      lastLedStatus = lastLedStatus == LOW ? HIGH : LOW;
      digitalWrite(led, lastLedStatus);
    }
    delay(500);
  }
}

void setup() {
  pinMode(led, OUTPUT);

  
  Serial.begin(115200);
  
  Serial.println("Battery Level Indicator - BLE");
  
  InitBLE();
  
  // Create a task to blink led
  xTaskCreatePinnedToCore(ble_status_led, "ble led", 2048, NULL, 1, NULL, 1);
  unsigned long start = millis();
  uint8_t level = 100;
  while(_BLEClientConnected || millis() - start < TIME_CLIENT_WAITING * 1000) {    
    float sensorValue = (analogRead(sensorPin) / 4095.0)*2.0*3.3*1.1;
    level = (uint8_t)min(100, max(0, (int)map((int)(sensorValue * 100), (int)(bat_min * 100), (int)(bat_max * 100), 0, 100))); 
    BatteryLevelCharacteristic.setValue(&level, 1);
    BatteryLevelCharacteristic.notify();
    Serial.print("Battery Level");
    Serial.print(sensorValue);
    Serial.print(" - ");
    Serial.println(level);
    delay(5000);
  }
  
  stopBle();


  int sleeptime;
  if(level < 30) {
    sleeptime = TIME_TO_SLEEP_LOW;
  } else if(level < 50) {
    sleeptime = TIME_TO_SLEEP_MEDIUM;
  } else {
    sleeptime = TIME_TO_SLEEP_HIGH;
  }
  esp_sleep_enable_timer_wakeup(sleeptime * uS_TO_S_FACTOR);
  //Go to sleep now
  esp_deep_sleep_start();
}

void loop() { }
