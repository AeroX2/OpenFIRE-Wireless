/*
 * This module simulates the standard Arduino "Mouse.h" and
 * "Keyboard.h" API for use with the TinyUSB HID API. Instead of doing
 *  #include <HID.h>
 *  #include <Mouse.h>
 *  #include <Keyboard.h>
 *
 *  Simply do
 *
 *  #include <TinyUSB_Mouse_Keyboard.h>
 *
 *  and this module will automatically select whether or not to use the
 *  standard Arduino mouse and keyboard API or the TinyUSB API. We had to
 *  combine them into a single library because of the way TinyUSB handles
 *  descriptors.
 *
 *  For details on Arduino Mouse.h see
 *   https://www.arduino.cc/reference/en/language/functions/usb/mouse/
 *  For details on Arduino Keyboard.h see
 *   https://www.arduino.cc/reference/en/language/functions/usb/keyboard/
 *
 *  NOTE: This code is derived from the standard Arduino Mouse.h, Mouse.cpp,
 *    Keyboard.h, and Keyboard.cpp code. The copyright on that original code
 *    is as follows.
 *
 *  Copyright (c) 2015, Arduino LLC
 *  Original code (pre-library): Copyright (c) 2011, Peter Barrett
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#pragma once

#if defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)
    #include <Adafruit_TinyUSB.h>  // needed for some headers
    #include <Arduino.h>

    #include "../OpenFIRE_Packet/OpenFIRE_Packet.h"
    #include "TinyUSB_Devices.h"

    #if defined(ARDUINO_ARCH_ESP32)
        // #include <WiFi.h>
        #include <esp_mac.h>  // For the MAC2STR and MACSTR macros
        #include <esp_now.h>
        #include <esp_wifi.h>  // needed to read mac address

        #include "MacAddress.h"
        #include "esp_timer.h"
        #include "freertos/semphr.h"
    #elif defined(ARDUINO_ARCH_RP2040)
    // let's see
    #endif

/*****************************
 *   ESP_NOW
 *****************************/

extern uint8_t espnow_wifi_channel;
extern uint8_t espnow_wifi_power;
extern uint8_t lastDongleAddress[6];
extern uint8_t peerAddress[6];
extern bool lastDongleSave;

enum WIRELESS_MODE {
    NONE_WIRELESS = 0,
    ENABLE_BLUETOOTH_TO_PC,         // 1
    ENABLE_BLUETOOTH_TO_DONGLE,     // 2
    ENABLE_ESP_NOW_TO_DONGLE,       // 3
    ENABLE_WIFI_TO_DONGLE,          // 4
    ENABLE_NRF_24L01PLUS_TO_DONGLE  // 5
};

enum PACKET_TX {
    SERIAL_TX = 1,  // SERIAL DATA
    KEYBOARD_TX,
    MOUSE_TX,
    GAMEPAD_TX,
    CONNECTION,                    // CONNECTION AND ASSOCIATION DONGLE WITH GUN
    CHECK_CONNECTION_LAST_DONGLE,  // VERIFY WIRELESS CONNECTION BETWEEN GUN AND DONGLE AND VICE VERSA
    // ========= TO EVALUATE
    RESET_DATA_USB,
    REBOOT,           // DONGLE REBOOT
    TX_DATA_USB,      // USB DATA TRANSMISSION FROM GUN TO DONGLE
    CHANGE_DATA_USB,  // CHANGE OF USB DATA, SO ?????
    CONN_REQ_TO_GUN,  // REQUEST SENT FROM DONGLE TOWARDS GUN (SENDS DONGLE MAC ADDRESS AND THE MAC ADDRESS TO WHICH THE
                      // REQUEST WAS SENT (CAN BE BROADCAST))
    CONN_REQ_TO_DONGLE,  // REQUEST SENT FROM GUN TO DONGLE, TO ACCEPT CONNECTION REQUEST (SENDS GUN MAC ADDRESS AND MAC
                         // ADDRESS TO WHICH REQUEST WAS SENT)
    CONN_ACCEPT,  // THE DONGLE HAS ACCEPTED CONNECTION (ALSO SENDS DONGLE MAC ADDRESS AND GUN MAC ADDRESS TO WHICH THE
                  // REQUEST WAS ACCEPTED)
};

enum CONNECTION_STATE {
    NONE_CONNECTION = 0,
    TX_DONGLE_SEARCH_GUN_BROADCAST,  // 1
    TX_GUN_TO_DONGLE_PRESENCE,       // 2
    TX_DONGLE_TO_GUN_ACCEPT,         // 3
    TX_GUN_TO_DONGLE_CONFIRM,        // 4
    DEVICES_CONNECTED,               // 5
    // =============================
    // === for connection control
    NONE_CONNECTION_DONGLE,
    TX_CHECK_CONNECTION_LAST_DONGLE,     // 6  GUN -> DONGLE
    TX_CONFIRM_CONNECTION_LAST_DONGLE,   // 7 DONGLE -> GUN
    DEVICES_CONNECTED_WITH_LAST_DONGLE,  // 8
};

typedef struct __attribute__((packed)) {
    char deviceManufacturer[21];
    char deviceName[16];
    char deviceType;
    uint16_t deviceVID;
    uint16_t devicePID;
    uint8_t devicePlayer;
    uint8_t channel;  // from 0 to 13
} USB_Data_GUN_Wireless;

extern USB_Data_GUN_Wireless usb_data_wireless;

    /*****************************
     *   SERIAL WIRELESS SECTION
     *****************************/
    #ifndef _SERIAL_STREAM_H_
        #define _SERIAL_STREAM_H_

        // Maximum number of simultaneous connections (players)
        #define MAX_CONNECTIONS 4

// Structure to track connected devices
typedef struct {
    uint8_t mac_address[6];
    uint8_t player_number;
    bool is_connected;
    USB_Data_GUN_Wireless device_data;
} ConnectedDevice;

class SerialWireless_ : public Stream {
   public:
    uint8_t mac_esp_interface[6];
    uint8_t mac_esp_another_card[6];
    uint8_t wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;  // 0;

    // Multiple connection support
    ConnectedDevice connected_devices[MAX_CONNECTIONS];
    uint8_t current_player_number;
    uint8_t num_connected_devices;

        // ======= for SERIAL FIFO ===============
        // ===== for write === linear buffer ====
        #define FIFO_SIZE_WRITE_SERIAL 128
        #define TIME_OUT_SERIAL_WRITE 3                   // later remove or redetermine in microseconds
        #define TIMER_HANDLE_SERIAL_DURATION_MICROS 3000  // 3000 microseconds or 3 milliseconds
    uint8_t bufferSerialWrite[FIFO_SIZE_WRITE_SERIAL];
    unsigned long startTimeSerialWrite = 0;  // = millis(); // then convert to microseconds // later remove
    volatile uint16_t lenBufferSerialWrite = 0;
    esp_timer_handle_t timer_handle_serial;
        // ====== for read ====== circular buffer =====
        #define FIFO_SIZE_READ_SERIAL 200
    uint8_t bufferSerialRead[FIFO_SIZE_READ_SERIAL];
    volatile uint16_t lenBufferSerialRead = 0;
    volatile uint16_t _writerSerialRead = 0;
    volatile uint16_t _readerSerialRead = 0;
    bool _overflow_bufferSerialRead = false;
    void write_on_rx_serialBuffer(const uint8_t *data, int len);
    // ============================================

    Packet packet;

    // for read buffer
    volatile uint16_t _readLen = 0;
    volatile uint16_t _writer = 0;
    volatile uint16_t _reader = 0;
        #define FIFO_SIZE_READ 1024  // read buffer
    uint8_t _queue[FIFO_SIZE_READ];
    bool _overflow_read = false;
    // end for read buffer

    // for write buffer
    volatile uint16_t writeIndex = 0;
    volatile uint16_t readIndex = 0;
    volatile uint16_t _writeLen = 0;
        #define BUFFER_SIZE 1024  // write buffer
    uint8_t buffer[BUFFER_SIZE];
    bool _overflow_write = false;
    // end for write buffer

    SerialWireless_() : Stream() {}
    ~SerialWireless_() {}

    void begin();
    bool end();

    // override from ::Stream
    int peek() override;
    int read() override;
    int available() override;
    // end override from ::Stream

    // override from ::Print
    int availableForWrite() override;
    void flush() override;
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *data, size_t len) override;
    using Print::write;
    // ==== end override from :: Print

    operator bool();

    // insert from me for output buffer management
    size_t writeBin(uint8_t c);
    size_t writeBin(const uint8_t *data, size_t len);
    void flushBin();
    int availableForWriteBin();
    bool flush_sem();
    // insert from me for input buffer management
    int peekBin();
    int readBin();
    int availableBin();

    // inserted for packet management
    volatile uint16_t numAvailablePacket = 0;
    int availablePacket();

    // ======== generiche ============
    void SendData();  // we also use flush
    void SendData_sem();
    void SendPacket(const uint8_t *data, const uint8_t &len, const uint8_t &packetID);  // I don't think we'll use this

    bool checkForRxPacket();  // will go in the main loop .. placed in callback

    bool connection_dongle();
    bool connection_gun();
    bool connection_gun_at_last_dongle();

    // Multiple connection management functions
    bool add_connected_device(uint8_t mac[6], uint8_t player_number, USB_Data_GUN_Wireless &device_data);
    bool remove_connected_device(uint8_t mac[6]);
    ConnectedDevice *get_connected_device(uint8_t player_number);
    void print_connected_devices();

    // ===============================
    // ===== for timers ================

    // void setupTimer(uint64_t duration_us);
    void setupTimer();
    void stopTimer_serial();
    void resetTimer_serial(uint64_t duration_us);

   private:
};
extern SerialWireless_ SerialWireless;

    #endif  // _SERIAL_STREAM_H_

#endif  // OPENFIRE_WIRELESS_ENABLE
