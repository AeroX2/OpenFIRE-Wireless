// by SATANASSI Alessandro

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <SPI.h>

#include "TinyUSB_Devices.h"

// Here we define the Manufacturer Name, Device Name, and Vendor ID of the gun as will be displayed by the operating
// system. For multiplayer, different guns need different IDs! If unsure, just leave these at their defaults, as Product
// ID is determined by what's saved in local storage, or Player Number as a fallback. For compatibility with some
// whitelists' support for OpenFIRE, Manufacturer Name and Device VID must be kept at the default values
// "OpenFIRE"/OxF143
#define MANUFACTURER_NAME "OpenFIRE"
#define DEVICE_NAME "FIRECon"  // MAX 15 char
#define DEVICE_VID 0xF143

Adafruit_USBD_HID usbHid;

#ifdef OPENFIRE_WIRELESS_DEVICE_ESPNOW
    #include <ESP32_NOW_Serial.h>
    #include <MacAddress.h>
    #include <WiFi.h>
// #include "ESP_NOW_Serial_Wrapper.h"

    #define ESPNOW_WIFI_CHANNEL PLAYER_NUMBER

    // WiFi interface configuration
    #define ESPNOW_WIFI_MODE_STATION 1
    #if ESPNOW_WIFI_MODE_STATION
        #define ESPNOW_WIFI_MODE WIFI_STA
        #define ESPNOW_WIFI_IF WIFI_IF_STA
    #else
        #define ESPNOW_WIFI_MODE WIFI_AP
        #define ESPNOW_WIFI_IF WIFI_IF_AP
    #endif

    #define GUN_MAC_ADDRESS {0xB4, 0x3A, 0x45, 0x89, 0xF5, 0xD0}
    #define PEDAL_MAC_ADDRESS {0xB4, 0x3A, 0x45, 0x89, 0xF5, 0x34}

const MacAddress gun_peer_mac(GUN_MAC_ADDRESS);
const MacAddress pedal_peer_mac(PEDAL_MAC_ADDRESS);

ESP_NOW_Serial_Class NowSerialGun(gun_peer_mac, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);
ESP_NOW_Serial_Class NowSerialPedal(pedal_peer_mac, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);

// ESP_NOW_Serial_Wrapper NowSerialGun(NowSerialGunRaw);
// ESP_NOW_Serial_Wrapper NowSerialPedal(NowSerialPedalRaw);

// Store both gun and pedal mouse data for merging
hid_abs_mouse_report_t gunMouseReport = {0};
hid_abs_mouse_report_t pedalMouseReport = {0};
bool hasGunMouseData = false;
bool hasPedalMouseData = false;

// Function to merge gun and pedal mouse data and send combined report
void sendMergedMouseReport() {
    hid_abs_mouse_report_t mergedReport = {0};

    if (hasGunMouseData) {
        // Use gun's position data (x, y, wheel)
        mergedReport.x = gunMouseReport.x;
        mergedReport.y = gunMouseReport.y;
        mergedReport.wheel = gunMouseReport.wheel;
        mergedReport.buttons |= gunMouseReport.buttons;
    }

    if (hasPedalMouseData) {
        // Merge pedal buttons
        mergedReport.buttons |= pedalMouseReport.buttons;
    }

    // Send the merged report
    usbHid.sendReport(HID_RID_e::HID_RID_MOUSE, (uint8_t *)&mergedReport, sizeof(hid_abs_mouse_report_t));
}
#endif

#ifdef USES_DISPLAY
    #include "OpenFIRE_logo.h"
    #ifdef USE_LOVYAN_GFX
        #define LGFX_USE_V1
        #include <LovyanGFX.hpp>

        #include "LGFX_096_ST7735S_80x160.hpp"
LGFX tft;
        #define WHITE TFT_WHITE
        #define BLACK TFT_BLACK
        #define BLUE TFT_BLUE
        #define RED TFT_RED
        #define MAGENTA TFT_MAGENTA
        #define GREEN TFT_GREEN
        #define CYAN TFT_CYAN
        #define YELLOW TFT_YELLOW
        #define BROWN TFT_BROWN
        #define GRAY TFT_LIGHTGRAY
        #define ORANGE TFT_ORANGE
    #else
        #include <Adafruit_ST7735.h>
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
        #define WHITE ST7735_WHITE
        #define BLACK ST7735_BLACK
        #define BLUE ST7735_BLUE
        #define RED ST7735_RED
        #define MAGENTA ST7735_MAGENTA
        #define GREEN ST7735_GREEN
        #define CYAN ST7735_CYAN
        #define YELLOW ST7735_YELLOW
        #define BROWN 0XBC40
        #define GRAY 0X8430
        #define ORANGE ST7735_ORANGE
    #endif  // USE_LOVYAN_GFX
#endif      // USES_DISPLAY

bool enablePedal = false;
bool lastButtonState = HIGH;  // Button is pulled up, so HIGH when not pressed
bool buttonState = HIGH;      // Current debounced state
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50ms debounce delay
uint8_t buffer[RX_FRAME_CAP];

// Function to handle all packet types from a serial connection
void handlePacketData(ESP_NOW_Serial_Class &serial) {
    uint8_t packetType = serial.read();
    // Serial.printf("Received packet type: %d\n", packetType);

    size_t bytesRead;
    hid_keyboard_report_t keyboardReport;
    hid_abs_mouse_report_t mouseReport;
    hid_gamepad16_report_t gamepadReport;

    switch (packetType) {
        case PACKET_SERIAL: {
            while (!serial.available()) {
                delay(1);
            }
            size_t size = serial.read();
            uint8_t buf[size];
            serial.readBytes(buf, size);
            Serial.write(buf, size);
            Serial.flush();
            break;
        }
        case PACKET_MOUSE:
            // Read mouse data
            bytesRead = serial.readBytes((uint8_t *)&mouseReport, sizeof(hid_abs_mouse_report_t));

            // Store the data based on source
            if (&serial == &NowSerialGun) {
                gunMouseReport = mouseReport;
                hasGunMouseData = true;
            } else if (&serial == &NowSerialPedal) {
                pedalMouseReport = mouseReport;
                hasPedalMouseData = true;
            }

            // Always send merged report
            sendMergedMouseReport();
            break;
        case PACKET_KEYBOARD:
            // Read keyboard data and forward to USB HID
            bytesRead = serial.readBytes((uint8_t *)&keyboardReport, sizeof(hid_keyboard_report_t));
            usbHid.sendReport(HID_RID_e::HID_RID_KEYBOARD, (uint8_t *)&keyboardReport, sizeof(hid_keyboard_report_t));
            break;
        case PACKET_GAMEPAD:
            // Read gamepad data and forward to USB HID
            bytesRead = serial.readBytes((uint8_t *)&gamepadReport, sizeof(hid_gamepad16_report_t));
            usbHid.sendReport(HID_RID_e::HID_RID_GAMEPAD, (uint8_t *)&gamepadReport, sizeof(hid_gamepad16_report_t));
            break;
    }
}

void redrawDisplay() {
    MacAddress macAddress = WiFi.macAddress();

    tft.fillScreen(BLACK);
    tft.drawBitmap(40, 0, customSplashBanner, CUSTSPLASHBANN_WIDTH, CUSTSPLASHBANN_HEIGHT, BLUE);
    tft.setTextSize(1);
    tft.setCursor(0, 26);
    tft.setTextColor(RED);
    tft.println(macAddress.toString());
    tft.setTextColor(RED);
    tft.setCursor(0, 40);
    tft.printf("Player: %d", PLAYER_NUMBER);
    tft.setCursor(0, 50);
    tft.setTextColor(GRAY);
    tft.printf("Channel: %d", ESPNOW_WIFI_CHANNEL);
    tft.setCursor(0, 60);
    tft.setTextColor(GRAY);
    tft.printf("Pedal: %s", enablePedal ? "enabled" : "disabled");
}

// The main show!
void setup() {
    pinMode(0, INPUT_PULLUP);  // Button to toggle pedal enable/disable

#ifdef USES_DISPLAY
    #ifdef USE_LOVYAN_GFX
    tft.init();
    tft.setSwapBytes(true);
    tft.setColorDepth(16);
        // tft.setBrightness(255);
    #else
    tft.initR(INITR_MINI160x80_PLUGIN);  // Init ST7735S mini display
    pinMode(TFT_PIN_BL, OUTPUT);
    digitalWrite(TFT_PIN_BL, 0);  // turns on display backlight
    #endif  // USE_LOVYAN_GFX

    tft.setRotation(3);
#endif  // USES_DISPLAY

    // ====== wireless connection management ====================
    WiFi.mode(ESPNOW_WIFI_MODE);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    while (!(WiFi.STA.started() || WiFi.AP.started())) {
        delay(100);
    }

    NowSerialGun.begin(115200);
    NowSerialPedal.begin(115200);
    // ====== end wireless management .. continues only after the device has paired =======

    // ====== USB connection ====== sets VID and PID as passed by the gun ===============
    if (!TinyUSBDevice.isInitialized()) {  // added ..it worked the same, but this is safer .. definitely needed for
                                           // Esp32 with library not integrated in the core
        TinyUSBDevice.begin(0);
    }

    TinyUSBDevice.setManufacturerDescriptor(MANUFACTURER_NAME);
    TinyUSBDevice.setProductDescriptor(DEVICE_NAME);
    TinyUSBDevice.setID(DEVICE_VID, PLAYER_NUMBER);

    // Initializing the USB devices chunk.
    TinyUSBDevices.begin(1);
    Serial.begin(9600);
    Serial.setTimeout(0);
    Serial.setTxTimeoutMs(0);
    // ====== end USB connection ==========================================================================

#ifdef USES_DISPLAY
    redrawDisplay();
#endif  // USES_DISPLAY
}

void loop() {
    // Read button with proper debouncing
    bool reading = digitalRead(0);

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // if the button state has changed:
        if (reading != buttonState) {
            buttonState = reading;

            // only toggle if the new button state is LOW (pressed)
            if (buttonState == LOW) {
                enablePedal = !enablePedal;
                Serial.printf("Button pressed! Pedal now: %s\n", enablePedal ? "enabled" : "disabled");
#ifdef USES_DISPLAY
                redrawDisplay();
#endif
            }
        }
    }

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState = reading;

    // Read from gun
    if (NowSerialGun.available()) {
        handlePacketData(NowSerialGun);
    }

    // Read from pedal
    if (enablePedal && NowSerialPedal.available()) {
        handlePacketData(NowSerialPedal);
    }

    // Handle USB serial to gun & pedal communication (forward host bytes over ESP-NOW framed as PACKET_SERIAL)
    if (Serial.available()) {
        size_t avail = Serial.available();
        if (avail) {
            if (avail > RX_FRAME_CAP)
                avail = RX_FRAME_CAP;
            size_t n = Serial.readBytes(buffer, avail);
            if (n != 0) {
                auto sendFramed = [&](ESP_NOW_Serial_Class &dest) {
                    if (!dest.availableForWrite())
                        return;  // skip if can't write now
                    dest.write(PACKET_SERIAL);
                    dest.write((uint8_t)n);
                    dest.write(buffer, n);
                    dest.flush();
                };

                sendFramed(NowSerialGun);
                if (enablePedal) {
                    sendFramed(NowSerialPedal);
                }
            }
        }
    }

    delay(1);
}