#include <Arduino.h>
#include <WiFi.h>

#include "OpenFIRE_Packet.h"
#include "OpenFIRE_Wireless.h"
#include "TinyUSB_Devices.h"

// Button configuration - only pedal
#define PEDAL_PIN 20  // D9 for SeeedStudio ESP32-C6

// Pedal state tracking
bool pedalPressed = false;
bool lastPedalPressed = false;

// Connection state tracking
bool wasConnected = false;

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
    switch (command) {
        case 'S':  // Start signal
            Serial.println("SERIALREAD: Start command received");
            break;

        case 'M':  // Mode setting
        {
            char subCommand = Serial.read();
            char subSubCommand = Serial.read();

            switch (subCommand) {
                case '2':  // Pedal functionality
                {
                    char pedalSubCommand = Serial.read();
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
    Serial.begin(115200);
    Serial.println("OpenFIRE Simple - Pedal Only - Starting up...");

    // Initialize pedal button pin
    pinMode(PEDAL_PIN, INPUT_PULLUP);
    Serial.println("Pedal button initialized");

    // Initialize wireless
    SerialWireless.begin();
    Serial.println("Wireless initialized");

// Set up USB data for wireless communication
#if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
    strncpy(usb_data_wireless.deviceManufacturer, "OpenFIRE", sizeof(usb_data_wireless.deviceManufacturer));
    strncpy(usb_data_wireless.deviceName, "Pedal", sizeof(usb_data_wireless.deviceName));
    usb_data_wireless.deviceVID = 0x1234;  // Example VID
    usb_data_wireless.devicePID = PLAYER_NUMBER;
    usb_data_wireless.devicePlayer = PLAYER_NUMBER;
    usb_data_wireless.channel = espnow_wifi_channel;
    usb_data_wireless.deviceType = 'P';  // Pedal device type
#endif

    // Try to connect
    SerialWireless.connection_gun();

    Serial.println("Setup complete!");
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        char command = Serial.read();
        processSerialCommand(command);
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

    // Print connection status changes
    if (SerialWireless.wireless_connection_state == DEVICES_CONNECTED && !wasConnected) {
        wasConnected = true;
        Serial.println("*** CONNECTED ***");
    } else if (SerialWireless.wireless_connection_state != DEVICES_CONNECTED && wasConnected) {
        wasConnected = false;
        Serial.println("*** DISCONNECTED ***");
    }

    // Small delay to prevent overwhelming the system
    delay(10);
}