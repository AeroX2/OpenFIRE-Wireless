#include <Arduino.h>
#include <ESP32_NOW_Serial.h>
#include <MacAddress.h>
#include <WiFi.h>

#include "TinyUSB_Devices.h"

// Button configuration - only pedal
#define PEDAL_PIN 20  // D9 for SeeedStudio ESP32-C6

// Channel to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 1

// WiFi interface configuration
#define ESPNOW_WIFI_MODE_STATION 1
#if ESPNOW_WIFI_MODE_STATION
    #define ESPNOW_WIFI_MODE WIFI_STA
    #define ESPNOW_WIFI_IF WIFI_IF_STA
#else
    #define ESPNOW_WIFI_MODE WIFI_AP
    #define ESPNOW_WIFI_IF WIFI_IF_AP
#endif

// Dongle: 30:ED:A0:06:5E:74
const MacAddress dongle_peer_mac({0x30, 0xED, 0xA0, 0x06, 0x5E, 0x74});

ESP_NOW_Serial_Class NowSerialDongle(dongle_peer_mac, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);

// Pedal state tracking
bool pedalPressed = false;
bool lastPedalPressed = false;

// Pedal mode enum
enum class PedalMode : uint8_t {
    SEPARATE = 0,     // Separate button
    RELOAD = 1,       // Reload button
    MIDDLE_MOUSE = 2  // Middle mouse button
};

// Pedal mode tracking
PedalMode pedalMode = PedalMode::SEPARATE;

// Button state tracking
uint8_t currentButtonState = 0;
uint8_t lastButtonState = 0;

// Process serial commands (simplified version - only pedal functionality)
void processSerialCommand(char command) {
    Serial.printf("Received command: %c\n", command);
    switch (command) {
        case 'S':  // Start signal
            Serial.println("SERIALREAD: Start command received");
            break;

        case 'M':  // Mode setting
        {
            char subCommand = NowSerialDongle.read();
            char subSubCommand = NowSerialDongle.read();

            switch (subCommand) {
                case '2':  // Pedal functionality
                {
                    char pedalSubCommand = NowSerialDongle.read();
                    switch (pedalSubCommand) {
                        case '0':  // Separate button
                            pedalMode = PedalMode::SEPARATE;
                            Serial.println("Pedal: Separate button");
                            break;
                        case '1':  // Make reload button
                            pedalMode = PedalMode::RELOAD;
                            Serial.println("Pedal: Reload button");
                            break;
                        case '2':  // Make middle mouse button
                            pedalMode = PedalMode::MIDDLE_MOUSE;
                            Serial.println("Pedal: Middle mouse button");
                            break;
                    }
                } break;
            }
        } break;
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("OpenFIRE Simple - Pedal Only - Starting up...");

    // Initialize pedal button pin
    pinMode(PEDAL_PIN, INPUT_PULLUP);
    Serial.println("Pedal button initialized");

    WiFi.mode(ESPNOW_WIFI_MODE);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    while (!(WiFi.STA.started() || WiFi.AP.started())) {
        delay(100);
    }

    TinyUSBDevices.begin(-1);
    NowSerialDongle.begin(115200);

    Serial.println("Setup complete!");
}

void loop() {
    // Check for serial commands
    if (NowSerialDongle.available()) {
        char s = NowSerialDongle.read();
        if (s == 0x01) {  // TODO: make this a constant, also support SERIAL_WITH_SIZE
            char command = NowSerialDongle.read();
            processSerialCommand(command);
        } else {
            Serial.printf("Received packet type: %d\n", s);
        }
    }

    // Read pedal button state
    pedalPressed = !digitalRead(PEDAL_PIN);  // Inverted because of INPUT_PULLUP

    // Print pedal press/release events
    if (pedalPressed && !lastPedalPressed) {
        Serial.println("Pedal pressed");
    }
    if (!pedalPressed && lastPedalPressed) {
        Serial.println("Pedal released");
    }

    // Handle pedal mode logic and set appropriate mouse button
    currentButtonState = 0;  // Default to no buttons pressed

    if (pedalPressed) {  // Pedal is pressed
        switch (pedalMode) {
            case PedalMode::SEPARATE:  // Separate button (default) - MOUSE_BUTTON4
                currentButtonState = MOUSE_BUTTON4;
                break;
            case PedalMode::RELOAD:  // Remap to A button - MOUSE_RIGHT
                currentButtonState = MOUSE_RIGHT;
                break;
            case PedalMode::MIDDLE_MOUSE:  // Remap to B button - MOUSE_MIDDLE
                currentButtonState = MOUSE_MIDDLE;
                break;
        }
    }

    // Only update buttons if state changed
    if (currentButtonState != lastButtonState) {
        AbsMouse5.buttons(currentButtonState);
        lastButtonState = currentButtonState;
        Serial.printf("Button state changed to: 0x%02X\n", currentButtonState);
    }

    lastPedalPressed = pedalPressed;

    // Small delay to prevent overwhelming the system
    delay(10);
}