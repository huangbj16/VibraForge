#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "f10016f6-542b-460a-ac8b-bbb0b2010599"
#define CHARACTERISTIC_UUID "f22535de-5375-44bd-8ca9-d0ea9ff9e410"
//#define CURRENTSENSING_UUID "640b8bf5-3c88-44f6-95e0-f5813b390d73"
BLECharacteristic *csCharacteristic;

bool deviceConnected = false;

// ---------------------------------------------------------------------------
// PIN MAP: QT Py ESP32-S3  ->  XIAO ESP32-S3 (same physical pads)
//   QT Py A0 = GPIO18  ->  XIAO D0 = GPIO1
//   QT Py A1 = GPIO17  ->  XIAO D1 = GPIO2
//   QT Py A2 = GPIO9   ->  XIAO D2 = GPIO3   (NOTE: GPIO3 is a strapping pin)
//   QT Py A3 = GPIO8   ->  XIAO D3 = GPIO4
// GPIO numbers are used directly so this compiles under either the
// "XIAO_ESP32S3" or a generic "ESP32S3 Dev Module" board selection.
// ---------------------------------------------------------------------------
const int subchain_pins[4] = {1, 2, 3, 4};  // physical pads D0, D1, D2, D3
const int subchain_num = 4;
int global_counter = 0;

// The XIAO ESP32-S3 has a single onboard user LED on GPIO21 (active LOW).
// It has NO addressable RGB NeoPixel, so the QT Py NeoPixel code is removed
// and replaced with this plain status LED.
#define STATUS_LED 21

EspSoftwareSerial::UART serial_group[4];

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      if (value.length() % 3 == 0) {  // Ensure the length is a multiple of 3 bytes
          unsigned long timestamp = millis(); // Get current time in milliseconds
          Serial.print("Timestamp: ");
          Serial.print(timestamp);
          Serial.print(" ms, Data = ");
          Serial.print(value.length());
          Serial.print(" bytes, # = ");
          Serial.println(++global_counter);
  
          for (int i = 0; i < value.length(); i += 3) {
              uint8_t byte1 = value[i];
              uint8_t byte2 = value[i+1];
              uint8_t byte3 = value[i+2];
  
              if (byte1 == 0xFF) continue; // Skip if the first byte of the command is 0xFF
  
              int serial_group_number = (byte1 >> 2) & 0x0F;
              int is_start = byte1 & 0x01;
              int addr = byte2 & 0x3F;
              int duty = (byte3 >> 3) & 0x0F;
              int freq = (byte3 >> 1) & 0x03;
              int wave = byte3 & 0x01;
  
              // Print received values for debugging
              // Uncomment the following lines if you want to output the values for debugging purposes
              
              Serial.print("Received: ");
              Serial.print("SG: "); Serial.print(serial_group_number);
              Serial.print(", Mode: "); Serial.print(is_start);
              Serial.print(", Addr: "); Serial.print(addr);
              Serial.print(", Duty: "); Serial.print(duty);
              Serial.print(", Freq: "); Serial.print(freq);
              Serial.print(", Wave: "); Serial.println(wave);
              
              
              sendCommand(serial_group_number, addr, is_start, duty, freq, wave);
          }
      }
      else{
          unsigned long timestamp = millis(); // Get current time in milliseconds
          Serial.print("Timestamp: ");
          Serial.print(timestamp);
          Serial.print(" ms, Data = ");
          Serial.print(value.length());
          Serial.print(", WRONG LENGTH!!!!!!!!!!!!!!!!");
      }
  }


    /* command format
        command = {
            'addr':motor_addr,
            'mode':start_or_stop,
            'duty':3, # default
            'freq':2, # default
            'wave':0, # default
        }
    */
    void sendCommand(int serial_group_number, int motor_addr, int is_start, int duty, int freq, int wave) {
//      uint8_t message = motor_addr;
//      serial_group[serial_group_number].write(message);
//      return;
      if (is_start == 1) { // Start command, two bytes
        uint8_t message[2];
        message[0] = (motor_addr << 1) | is_start;
        message[1] = 0x80 | (duty << 3) | (freq << 1) | wave;
        serial_group[serial_group_number].write(message, 2);
      } else { // Stop command, only one byte
        uint8_t message = (motor_addr << 1) | is_start;
        serial_group[serial_group_number].write(&message, 1);
      }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param){
        Serial.println("connected!");
        Serial.println(BLEDevice::toString().c_str());
        char bda_str[18];
        sprintf(bda_str, "%02X:%02X:%02X:%02X:%02X:%02X", param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2], param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        Serial.println("Device connected with Address: " + String(bda_str));
        pServer->updateConnParams(param->connect.remote_bda, 0, 0, 0, 100);
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("disconnected!");
      delay(500);
      deviceConnected = false;
      BLEDevice::startAdvertising();
    }
};


void setup() {
  Serial.begin(500000);//even parity check
  //  pinMode(8, OUTPUT);
  //  pinMode(7, INPUT);
  //  Serial1.begin(115200, SERIAL_8E1);// Hardware Serial
  Serial.print("number of hardware serial available: ");
  Serial.println(SOC_UART_NUM);
  for (int i = 0; i < subchain_num; ++i) {
    Serial.print("initialize uart on GPIO");
    Serial.println(subchain_pins[i]);
    serial_group[i].begin(115200, SWSERIAL_8E1, -1, subchain_pins[i], false);
    serial_group[i].enableIntTx(false);
    if (!serial_group[i]) { // If the object did not initialize, then its configuration is invalid
      Serial.println("Invalid EspSoftwareSerial pin configuration, check config");
    }
    delay(200);
  }
  Serial.println("Starting BLE work!");

  // setup status LED (XIAO user LED on GPIO21 is active LOW: LOW = on)
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);   // turn LED on to indicate the board is ready

  //BLE setup
  BLEDevice::init("XIAO ESP32-S3");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEDevice::setMTU(128);
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setValue("0");
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
//  csCharacteristic = pService->createCharacteristic(
//                       CURRENTSENSING_UUID,
//                       BLECharacteristic::PROPERTY_READ
//                     );
//  csCharacteristic->setValue("0");

  pService->start();

  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
//  pAdvertising->setMinPreferred(0x00);
//  pAdvertising->setMaxPreferred(0x00);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  //actuator pin setup
  //  for (int i=0; i<subchain_num; ++i){
  //    pinMode(subchain_pins[i], OUTPUT);
  //    digitalWrite(subchain_pins[i], LOW);
  //  }
//  pinMode(39, INPUT);   // NOTE: GPIO39 is not broken out on the XIAO base;
                          // pick a XIAO ADC pad (e.g. D0-D3 -> GPIO1-4) if re-enabling current sensing
}

void loop() {
  // put your main code here, to run repeatedly:
//  csCharacteristic->setValue(String(analogRead(39)).c_str());
//  delay(1000);
}
