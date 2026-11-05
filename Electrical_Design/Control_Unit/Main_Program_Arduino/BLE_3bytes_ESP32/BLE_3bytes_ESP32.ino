/*
 * BLE_3bytes_ESP32.ino
 * ESP32-S3 BLE Peripheral for Haptic Motor Control
 * Protocol: 3-byte commands (duty5=0..31, freq3=0..7, wave in byte1)
 * 
 * PC (BLE) --> ESP32 --> PIC16F18313 (via SoftwareSerial)
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <string.h>

// ==================== BLE UUIDs ====================
#define SERVICE_UUID        "f10016f6-542b-460a-ac8b-bbb0b2010599"
#define CHARACTERISTIC_UUID "f22535de-5375-44bd-8ca9-d0ea9ff9e410"

// ==================== Timing ====================
#define UART_BAUD       115200
#define MIN_IFG_US      120
#define INTER_STOP_US   150
#define STOP_RETRIES    2

// ==================== Modes ====================
#define MODE_STOP       0b00
#define MODE_START      0b01
#define MODE_SOFTSTOP   0b10

// ==================== Hardware ====================
Adafruit_NeoPixel strip(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

const int subchain_pins[4] = {18, 17, 9, 8};
const int subchain_num = 4;

EspSoftwareSerial::UART serial_group[4];

// ==================== State ====================
bool deviceConnected = false;
static uint32_t last_tx_us[4] = {0, 0, 0, 0};
static uint32_t global_counter = 0;

// ==================== Inter-Frame Guard ====================
static inline void guard_ifg(int group) {
    if (group < 0 || group >= subchain_num) return;
    uint32_t now = micros();
    uint32_t dt = now - last_tx_us[group];
    if ((int32_t)dt < (int32_t)MIN_IFG_US) {
        delayMicroseconds(MIN_IFG_US - dt);
    }
    last_tx_us[group] = micros();
}

// ==================== PIC Commands ====================
// STOP: 1 byte [addr6 << 1 | 0]
static inline void sendStopPIC(int group, int addr6) {
    if (group < 0 || group >= subchain_num) return;
    uint8_t b = (uint8_t)(((addr6 & 0x3F) << 1) | 0x00);
    for (int k = 0; k < STOP_RETRIES; ++k) {
        guard_ifg(group);
        serial_group[group].write(&b, 1);
        serial_group[group].flush();
        delayMicroseconds(INTER_STOP_US);
    }
}

// START: 3 bytes
//   msg[0] = [addr6 << 1 | 1]           (MSB=0, address byte)
//   msg[1] = [1][duty5]                 (MSB=1, data1)
//   msg[2] = [1][wave][0][0][0][freq3]  (MSB=1, data2)
static inline void sendStartPIC(int group, int addr6, uint8_t duty5, uint8_t freq3, uint8_t wave) {
    if (group < 0 || group >= subchain_num) return;
    uint8_t msg[3];
    msg[0] = (uint8_t)(((addr6 & 0x3F) << 1) | 0x01);
    msg[1] = (uint8_t)(0x80 | (duty5 & 0x1F));
    msg[2] = (uint8_t)(0x80 | ((wave & 0x01) << 3) | (freq3 & 0x07));
    guard_ifg(group);
    serial_group[group].write(msg, 3);
    serial_group[group].flush();
}

// ==================== Frame Processing ====================
/*
 * 3-byte protocol (PC --> ESP32):
 *   Byte1: [W][0][G3:G0][M1:M0]  - Wave, Group(0-15), Mode
 *   Byte2: [0][0][A5:A0]         - Address (0-63)
 *   Byte3: [D4:D0][F2:F0]        - Duty5 (0-31), Freq3 (0-7)
 */
static inline void processFrame3(const uint8_t* f) {
    const uint8_t b1 = f[0], b2 = f[1], b3 = f[2];
    
    // Skip padding frames
    if (b1 == 0xFF && b2 == 0xFF && b3 == 0xFF) return;
    
    // Decode
    const uint8_t wave  = (b1 >> 7) & 0x01;
    const uint8_t mode  =  b1       & 0x03;
    const int     group = (b1 >> 2) & 0x0F;
    const int     addr6 =  b2       & 0x3F;
    const uint8_t duty5 = (b3 >> 3) & 0x1F;
    const uint8_t freq3 =  b3       & 0x07;
    
    // Debug output
    Serial.printf("CMD: G%d A%d M%d D%d F%d W%d\n", 
                  group, addr6, mode, duty5, freq3, wave);
    
    // Route to PIC
    if (mode == MODE_STOP || mode == MODE_SOFTSTOP) {
        sendStopPIC(group, addr6);
    } else {
        sendStartPIC(group, addr6, duty5, freq3, wave);
    }
}

// ==================== BLE Callbacks ====================
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        size_t len = value.length();
        
        if (len == 0) return;
        
        unsigned long timestamp = millis();
        
        if (len % 3 != 0) {
            Serial.printf("[%lu ms] ERROR: %d bytes (not multiple of 3)\n", timestamp, len);
            return;
        }
        
        Serial.printf("[%lu ms] RX %d bytes, #%u\n", timestamp, len, ++global_counter);
        
        // Process each 3-byte frame
        for (size_t i = 0; i < len; i += 3) {
            uint8_t frame[3] = {
                (uint8_t)value[i],
                (uint8_t)value[i + 1],
                (uint8_t)value[i + 2]
            };
            processFrame3(frame);
        }
    }
};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Connected");
        
        // LED: Blue when connected
        strip.setPixelColor(0, strip.Color(0, 0, 255));
        strip.show();
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Disconnected");
        
        // LED: Green when advertising
        strip.setPixelColor(0, strip.Color(0, 255, 0));
        strip.show();
        
        delay(100);
        BLEDevice::startAdvertising();
    }
};

// ==================== Setup ====================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== BLE Haptic Controller (3-byte) ===");
    Serial.printf("Subchains: %d (pins: %d, %d, %d, %d)\n",
                  subchain_num,
                  subchain_pins[0], subchain_pins[1],
                  subchain_pins[2], subchain_pins[3]);
    
    // Initialize SoftwareSerial for each subchain
    for (int i = 0; i < subchain_num; ++i) {
        serial_group[i].begin(UART_BAUD, SWSERIAL_8N1, -1, subchain_pins[i], false);
        serial_group[i].enableIntTx(true);
        Serial.printf("  UART[%d] on pin %d: OK\n", i, subchain_pins[i]);
        delay(30);
    }
    
    // Initialize NeoPixel
    strip.begin();
    strip.setBrightness(20);
    strip.setPixelColor(0, strip.Color(255, 128, 0));  // Orange during init
    strip.show();
    
    // Initialize BLE
    BLEDevice::init("VibraForge-BLE");
    BLEDevice::setMTU(128);
    
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR  // Write without response for speed
    );
    pCharacteristic->setValue("0");
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    
    pService->start();
    
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    
    // LED: Green = ready & advertising
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();
    
    Serial.println("BLE Advertising started. Waiting for connection...");
    Serial.printf("Service UUID: %s\n", SERVICE_UUID);
    Serial.printf("Char UUID:    %s\n", CHARACTERISTIC_UUID);
}

// ==================== Loop ====================
void loop() {
    // BLE events handled via callbacks
    delay(10);
}
