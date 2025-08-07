
#if defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)

    // #include <WiFi.h>
    #include "OpenFIRE_Wireless.h"

    #ifdef DONGLE
        #ifdef USES_DISPLAY
            #ifdef USE_LOVYAN_GFX
                #include <LovyanGFX.hpp>

                #include "..\..\src\LGFX_096_ST7735S_80x160.hpp"
extern LGFX tft;
            #else
                #include <Adafruit_ST7735.h>
extern Adafruit_ST7735 tft;
            #endif  // USE_LOVYAN_GFX
        #endif      // USES_DISPLAY
    #endif          // DONGLE

    #define ESPNOW_WIFI_CHANNEL_DEFAULT 12

    // transmission power can range from 8 to 84, where 84 is the maximum value corresponding to 20 db
    #define ESPNOW_WIFI_POWER_DEFAULT 84

uint8_t espnow_wifi_channel = ESPNOW_WIFI_CHANNEL_DEFAULT;  // MADE VARIABLE FOR FUTURE CONFIGURATION VIA APP OR OLED
uint8_t espnow_wifi_power = ESPNOW_WIFI_POWER_DEFAULT;      // MADE VARIABLE FOR FUTURE CONFIGURATION VIA APP OR OLED

USB_Data_GUN_Wireless usb_data_wireless = {
    "OpenFIRE_DONGLE",  // MANUFACTURER
    "FIRECon",          // NAME
    #if defined(DONGLE)
    'D',
    #elif defined(GUN)
    'G',
    #elif defined(PEDAL)
    'P',
    #endif
    0xF143,         // VID
    0x1998,         // 0x0001   // PID
    PLAYER_NUMBER,  // PLAYER
    espnow_wifi_channel
    // ESPNOW_WIFI_CHANNEL_DEFAULT // CHANNEL
    //,""               // ????
};

SerialWireless_ SerialWireless;
    #ifndef WIRELESS_ONLY
extern Adafruit_USBD_HID usbHid;
    #endif
SemaphoreHandle_t tx_sem = NULL;  // used for espnow transmission end callback

SemaphoreHandle_t mutex_tx_serial = NULL;   // used to transmit serial buffer
SemaphoreHandle_t mutex_writer_bin = NULL;  // used to transmit serial buffer

uint8_t buffer_espnow[ESP_NOW_MAX_DATA_LEN];
esp_now_peer_info_t
    peerInfo;  // must stay outside functions from functions -- global --utility variable for configuration

static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);     // esp_now callback
static void _esp_now_tx_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);  // esp_now callback

void packet_callback_read_dongle();  // packet callback
void packet_callback_read_gun();     // packet callback

///////////////////////////////////////////////////////////////////
// #define SIZE_BASE_AUX 13
uint8_t aux_buffer[13 + sizeof(usb_data_wireless)];
// 1 BYTE INDICATES THE PROGRESS OF CONNECTION ESTABLISHMENT, AT WHAT POINT WE ARE
// 6 BYTE THE MAC ADDRESS OF THE DEVICE THAT TRANSMITS
// 6 BYTE THE MAC ADDRESS OF THE DEVICE TO WHICH THE PACKET IS ADDRESSED
// AT THE LAST SEND THE GUN ALSO SENDS THE DATA OF THE USB_DATA_WIRELESS STRUCTURE WHICH CONTAINS VID, PID, ETC. OF THE
// GUN SO THAT THE DONGLE INITIALIZES THE USB SESSION WITH THAT DATA

// uint8_t mac_esp_another_device[6];
// uint8_t wireless_connection_state = 0;
uint8_t connection_type = 0;

// connection state

// in case of DONGLE
// 0 = not established
// 1 = broadcast request sent
// 2 = someone responded to the connection and asked to be connected
// 3 = connection confirmation sent;

// in case of GUN
// 0 = not established
// 1 = broadcast connection request received
// 2 = accepted and connection request sent
// 3 = connection acceptance received

uint8_t peerAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // broadcast

// in case the gun turns off and we have stored the last connected dongle, try to reconnect immediately
uint8_t lastDongleAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool lastDongleSave = false;  // if true means we have an address of the last dongle otherwise false

const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    #if defined(DONGLE)
const functionPtr callbackArr[] = {packet_callback_read_dongle};
    #elif defined(GUN) || defined(PEDAL)
const functionPtr callbackArr[] = {packet_callback_read_gun};
    #endif
///////////////////////////////////////////////////////////////////

/*****************************
 *   SERIAL WIRELESS SECTION
 *****************************/

SerialWireless_::operator bool() {
    return true;
}

int SerialWireless_::peek() {
    if (lenBufferSerialRead) {
        return bufferSerialRead[_readerSerialRead];
    }
    return -1;
}

int SerialWireless_::peekBin() {
    if (_readLen) {
        return _queue[_reader];
    }
    return -1;
}

int SerialWireless_::read() {
    if (lenBufferSerialRead) {
        uint8_t ret = bufferSerialRead[_readerSerialRead];
        lenBufferSerialRead--;
        _readerSerialRead++;
        if (_readerSerialRead >= FIFO_SIZE_READ_SERIAL) {
            _readerSerialRead -= FIFO_SIZE_READ_SERIAL;
        }
        return ret;
    }
    return -1;
}

bool SerialWireless_::checkForRxPacket() {
    uint16_t numAvailableBin = _readLen;
    uint8_t data;
    for (uint16_t i = 0; i < numAvailableBin; i++) {
        data = (uint8_t)readBin();
        if (data == START_BYTE)
            packet.reset();  // reset packet start - // check data .. if it's equal to packet::start_byte ..
                             // clear everything and start over otherwise
        packet.parse(data, true);
    }
    return true;
}

int SerialWireless_::readBin() {
    if (_readLen) {
        uint8_t ret = _queue[_reader];
        _readLen--;
        _reader++;
        if (_reader >= FIFO_SIZE_READ) {
            _reader -= FIFO_SIZE_READ;
        }
        return ret;
    }
    return -1;
}

int SerialWireless_::available() {
    return lenBufferSerialRead;
}

int SerialWireless_::availableBin() {
    return _readLen;
}

int SerialWireless_::availableForWrite() {
    return FIFO_SIZE_WRITE_SERIAL - lenBufferSerialWrite;
}

int SerialWireless_::availableForWriteBin() {
    return BUFFER_SIZE - _writeLen;
}

// blocking until the serial buffer is empty (non-blocking until the data has actually been sent)
void SerialWireless_::flush() {
    esp_timer_stop(timer_handle_serial);             // turn off timer
    xSemaphoreTake(mutex_tx_serial, portMAX_DELAY);  // take semaphore / wait until it's free
    while (lenBufferSerialWrite) {
        flush_sem();
        taskYIELD();  // yield();
    }

    xSemaphoreGive(mutex_tx_serial);  // Release the semaphore after the callback
}

// decide whether to check if the buffer_bin is also empty
bool SerialWireless_::flush_sem() {  // it's blocking and doesn't exit until the output buffer is completely empty
                                     // // in virtual com with TinyUISB it's not blocking .. sends the largest packet
                                     // it can and returns
    if (lenBufferSerialWrite) {
        if ((BUFFER_SIZE - _writeLen) > (lenBufferSerialWrite + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
            memcpy(&packet.txBuff[PREAMBLE_SIZE], bufferSerialWrite, lenBufferSerialWrite);
            packet.constructPacket(lenBufferSerialWrite, PACKET_TX::SERIAL_TX);
            writeBin(packet.txBuff, lenBufferSerialWrite + PREAMBLE_SIZE + POSTAMBLE_SIZE);
            lenBufferSerialWrite = 0;
            SendData();  // transition completed leave only this
            return true;
        } else
            SendData();
    }
    return false;
}

void SerialWireless_::flushBin() {  // never used
    SendData();
}

void SerialWireless_::SendData() {
    if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
        xSemaphoreTake(mutex_writer_bin, portMAX_DELAY);
        if (_writeLen > 0) {
            SendData_sem();
        } else
            xSemaphoreGive(tx_sem);
        xSemaphoreGive(mutex_writer_bin);
    }
}

void SerialWireless_::SendData_sem() {
    uint16_t len_tx = _writeLen > ESP_NOW_MAX_DATA_LEN ? ESP_NOW_MAX_DATA_LEN : _writeLen;
    uint16_t bytesToSendEnd = len_tx > (BUFFER_SIZE - readIndex) ? BUFFER_SIZE - readIndex : len_tx;
    memcpy(buffer_espnow, &buffer[readIndex], bytesToSendEnd);
    if (len_tx > bytesToSendEnd) {
        memcpy(&buffer_espnow[bytesToSendEnd], &buffer[0], len_tx - bytesToSendEnd);
    }
    esp_err_t result = esp_now_send(peerAddress, buffer_espnow, len_tx);
    if (result == ESP_OK) {  // verificare se esistono casi in cui seppur non risporta ESP_OK vengono trasmessi lo steso
        readIndex += len_tx;
        if (readIndex >= BUFFER_SIZE) {
            readIndex -= BUFFER_SIZE;
        }
        _writeLen -= len_tx;
    }
}

size_t SerialWireless_::write(uint8_t c) {
    return write(&c, 1);
}

size_t SerialWireless_::writeBin(uint8_t c) {
    return writeBin(&c, 1);
}

size_t SerialWireless_::write(const uint8_t *data, size_t len) {
    size_t len_remainer = len;
    size_t pos_remainer = 0;

    xSemaphoreTake(mutex_tx_serial, portMAX_DELAY);
    while (len_remainer > 0) {
        size_t available_space = FIFO_SIZE_WRITE_SERIAL - lenBufferSerialWrite;
        if (available_space >= len_remainer) {
            memcpy(&bufferSerialWrite[lenBufferSerialWrite], &data[pos_remainer], len_remainer);
            if (lenBufferSerialWrite == 0) {
                esp_timer_start_once(timer_handle_serial, TIMER_HANDLE_SERIAL_DURATION_MICROS);
            }
            lenBufferSerialWrite += len_remainer;
            len_remainer = 0;
        } else {
            if (len_remainer == len)
                esp_timer_stop(timer_handle_serial);
            if (available_space) {
                memcpy(&bufferSerialWrite[lenBufferSerialWrite], &data[pos_remainer], available_space);
                lenBufferSerialWrite = FIFO_SIZE_WRITE_SERIAL;
                pos_remainer += available_space;
                len_remainer -= available_space;
            }
            flush_sem();
        }
        taskYIELD();
    }
    xSemaphoreGive(mutex_tx_serial);
    return len;
}

size_t SerialWireless_::writeBin(const uint8_t *data, size_t len) {
    xSemaphoreTake(mutex_writer_bin, portMAX_DELAY);

    if ((BUFFER_SIZE - _writeLen) >= len) {
        size_t firstChunk = BUFFER_SIZE - writeIndex;
        if (firstChunk < len) {
            memcpy(&buffer[writeIndex], data, firstChunk);
            writeIndex = len - firstChunk;
            memcpy(&buffer[0], data + firstChunk, writeIndex);
        } else {
            memcpy(&buffer[writeIndex], data, len);
            writeIndex += len;
            if (writeIndex == BUFFER_SIZE)
                writeIndex = 0;
        }
        _writeLen += len;
        xSemaphoreGive(mutex_writer_bin);
        return len;
    } else {
        _overflow_write = true;
        // TODO: handle overflow
        // Serial.println("Overflow in writing to the WRITE BUFFER");
        xSemaphoreGive(mutex_writer_bin);
        return 0;
    }
}

void SerialWireless_::SendPacket(const uint8_t *data, const uint8_t &len, const uint8_t &packetID) {
    /*
    TODO: handle separate fifos for control ??? evaluate
    switch (packetID)
    {
    case PACKET_TX::MOUSE_TX:
      // handle a separate fifo ?
      break;
    case PACKET_TX::KEYBOARD_TX:
      //
      break;
    case PACKET_TX::GAMEPADE_TX:
      //
      break;
    default:
      break;
    }
    */
    if ((BUFFER_SIZE - _writeLen) > (len + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
        memcpy(&packet.txBuff[PREAMBLE_SIZE], data, len);
        packet.constructPacket(len, packetID);
        writeBin(packet.txBuff, len + PREAMBLE_SIZE + POSTAMBLE_SIZE);
        SendData();  // try_Send
    }
}

void SerialWireless_::write_on_rx_serialBuffer(const uint8_t *data, int len) {
    if ((FIFO_SIZE_READ_SERIAL - lenBufferSerialRead) >= len) {
        size_t firstChunk = FIFO_SIZE_READ_SERIAL - _writerSerialRead;
        if (firstChunk < len) {
            memcpy(&bufferSerialRead[_writerSerialRead], data, firstChunk);
            _writerSerialRead = len - firstChunk;
            memcpy(&bufferSerialRead[0], data + firstChunk, _writerSerialRead);
        } else {
            memcpy(&bufferSerialRead[_writerSerialRead], data, len);
            _writerSerialRead += len;
            if (_writerSerialRead == FIFO_SIZE_READ_SERIAL)
                _writerSerialRead = 0;
        }
        lenBufferSerialRead += len;
    } else {
        _overflow_bufferSerialRead = true;
        // TODO: handle overflow
        // Serial.println("Overflow in writing the SERIAL BUFFER read buffer");
    }
}

int SerialWireless_::availablePacket() {
    return numAvailablePacket;
}

void SerialWireless_::begin() {
    configST myConfig;  // utility variable for configuration
    // ============ semaphore initialization =============
    tx_sem = xSemaphoreCreateBinary();
    mutex_tx_serial = xSemaphoreCreateMutex();
    mutex_writer_bin = xSemaphoreCreateMutex();

    xSemaphoreGive(tx_sem);
    xSemaphoreGive(mutex_tx_serial);
    xSemaphoreGive(mutex_writer_bin);
    // ==== end semaphore initialization

    // WiFi.mode(WIFI_STA);
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        Serial.printf("WiFi not started! 0x%x)", err);
        return;
    }

    // WiFi.macAddress(mac_esp_interface); // register the mac address of the card

    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac_esp_interface);
    if (err != ESP_OK) {
        Serial.println("Failed to read MAC address");
        return;
    }

    err = esp_wifi_set_channel(espnow_wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("esp_wifi_set_channel failed! 0x%x", err);
        return;
    }

    err = esp_wifi_set_max_tx_power(espnow_wifi_power);  // from 8 to 84 corresponding to 2dbm to 20 dbm);
    if (err != ESP_OK) {
        Serial.printf("esp_wifi_set_max_tx_power failed! 0x%x", err);
        return;
    }

    // WiFi.disconnect();  // ???
    esp_wifi_disconnect();

    vTaskDelay(pdMS_TO_TICKS(1000));  // delay(1000);

    err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("esp_now_init failed! 0x%x", err);
        return;
    }

    if (lastDongleSave) {
        memcpy(peerAddress, lastDongleAddress, 6);
        wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION_DONGLE;
    } else {
        memcpy(peerAddress, BROADCAST_ADDR, 6);
        wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;
    }

    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = espnow_wifi_channel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Errore nell'aggiunta del peer");
        return;
    }

    err = esp_now_register_recv_cb(_esp_now_rx_cb);
    if (err != ESP_OK) {
        Serial.printf("esp_now_register_recv_cb failed! 0x%x", err);
        return;
    }

    err = esp_now_register_send_cb(_esp_now_tx_cb);
    if (err != ESP_OK) {
        Serial.printf("esp_now_register_send_cb failed! 0x%x", err);
        return;
    }

    myConfig.port =
        &Serial;  // this will be removed - remaining only for accounting =========================================
    myConfig.debug = false;  // true; //false; //true;
    myConfig.debugPort = &Serial;
    myConfig.timeout = DEFAULT_TIMEOUT;  // 50ms
    myConfig.callbacks = callbackArr;
    myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);

    packet.begin(myConfig);
    TinyUSBDevices.wireless_mode = WIRELESS_MODE::ENABLE_ESP_NOW_TO_DONGLE;

    setupTimer();  // create timers .. timer for serial data transmission
}

bool SerialWireless_::end() {
    esp_now_del_peer(peerAddress);
    esp_err_t err = esp_now_deinit();
    if (err != ESP_OK) {
        Serial.printf("esp_now_deinit failed! 0x%x", err);
        return false;
    }
    // WiFi.disconnect(true);
    // WiFi.mode(WIFI_OFF);  // Disable WiFi
    esp_wifi_disconnect();
    esp_wifi_deinit();
    esp_timer_delete(timer_handle_serial);
    return true;
}

bool SerialWireless_::connection_dongle() {
    uint8_t channel = espnow_wifi_channel;  // between 1 and 13 (14 should be reserved)
    #define TIMEOUT_TX_PACKET 500           // in milliseconds
    #define TIMEOUT_CHANGE_CHANNEL 2000     // in milliseconds - change channel every
    #define TIMEOUT_DIALOGUE 6000           // in milliseconds - maximum time to complete pairing operation
    unsigned long lastMillis_tx_packet = millis();
    unsigned long lastMillis_change_channel = millis();
    unsigned long lastMillis_start_dialogue = millis();
    uint8_t aux_buffer_tx[13];

    wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;
    aux_buffer_tx[0] = CONNECTION_STATE::TX_DONGLE_SEARCH_GUN_BROADCAST;
    memcpy(&aux_buffer_tx[1], SerialWireless.mac_esp_interface, 6);
    memcpy(&aux_buffer_tx[7], peerAddress, 6);

    #ifdef DONGLE
        #ifdef USES_DISPLAY
    tft.fillRect(95, 60, 50, 20, 0 /*BLACK*/);
    tft.setCursor(95, 60);
    tft.printf("%2d", channel);
        #endif  // USES_DISPLAY
    #endif      // DONGLE

    while (wireless_connection_state != CONNECTION_STATE::DEVICES_CONNECTED) {
        if (wireless_connection_state == CONNECTION_STATE::NONE_CONNECTION) {
            if (((millis() - lastMillis_change_channel) > TIMEOUT_CHANGE_CHANNEL) &&
                ((millis() - (lastMillis_tx_packet - 50))) >
                    TIMEOUT_TX_PACKET) {  // added set 50 ms as margin, to avoid that when sending
                                          // packet change immediately channel without giving the possibility to answer
                channel++;
                if (channel > 13)
                    channel = 1;

    #ifdef DONGLE
        #ifdef USES_DISPLAY
                tft.fillRect(95, 60, 50, 20, 0 /*BLACK*/);
                tft.setCursor(95, 60);
                tft.printf("%2d", channel);
        #endif  // USES_DISPLAY
    #endif      // DONGLE

                if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
                    Serial.printf("DONGLE - esp_wifi_set_channel failed!");
                }
                peerInfo.channel = channel;
                if (esp_now_mod_peer(&peerInfo) != ESP_OK) {  // modify the peer channel
                    Serial.println("DONGLE - Error in channel modification");
                }
                lastMillis_change_channel = millis();
                lastMillis_tx_packet = 0;  // to make it send a packet immediately on the new channel
            }
            if ((millis() - lastMillis_tx_packet) > TIMEOUT_TX_PACKET) {
                SerialWireless.SendPacket((const uint8_t *)aux_buffer_tx, 13, PACKET_TX::CONNECTION);
                Serial.print("DONGLE - sent broadcast packet on channel: ");
                Serial.println(channel);
                lastMillis_tx_packet = millis();
            }
            lastMillis_start_dialogue = millis();
        } else {
            if (((millis() - lastMillis_start_dialogue) > TIMEOUT_DIALOGUE) &&
                wireless_connection_state != CONNECTION_STATE::DEVICES_CONNECTED) {
                wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;
                Serial.println("DONGLE - Negotiation between DONGLE/GUN was not completed and we start over");
                lastMillis_change_channel = millis();
            }
        }
        yield();  // waiting for connection establishment
    }

    // Serial.println("DONGLE - Negotiation completed - association of GUN/DONGLE devices");
    if (esp_now_del_peer(peerAddress) != ESP_OK) {  // delete the broadcast from peers
        Serial.println("DONGLE - Error in broadcast peer deletion");
    }
    memcpy(peerAddress, mac_esp_another_card, 6);
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    // peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    // peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // inserts the dongle in peers
        Serial.println("DONGLE - Error adding new GUN peer");
    }
    TinyUSBDevices.onBattery = true;
    return true;
}

bool SerialWireless_::connection_gun_at_last_dongle() {
    #define TIMEOUT_TX_PACKET_LAST_DONGLE \
        300  // in milliseconds - packet sending time every milliseconds so 4-5 packets
    #define TIMEOUT_DIALOGUE_LAST_DONGLE 2000  // in milliseconds - maximum time for last dongle search
    unsigned long lastMillis_tx_packet_last_dongle = 0;
    unsigned long lastMillis_start_dialogue_last_dongle = millis();

    uint8_t aux_buffer_tx[13];

    wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION_DONGLE;
    aux_buffer_tx[0] = CONNECTION_STATE::TX_CHECK_CONNECTION_LAST_DONGLE;
    memcpy(&aux_buffer_tx[1], SerialWireless.mac_esp_interface, 6);
    memcpy(&aux_buffer_tx[7], peerAddress, 6);
    lastMillis_tx_packet_last_dongle = 0;
    lastMillis_start_dialogue_last_dongle = millis();
    while (
    #ifndef WIRELESS_ONLY
        !TinyUSBDevice.mounted() &&
    #endif
        (wireless_connection_state != CONNECTION_STATE::DEVICES_CONNECTED_WITH_LAST_DONGLE) &&
        ((millis() - lastMillis_start_dialogue_last_dongle) < TIMEOUT_DIALOGUE_LAST_DONGLE)) {
        if ((millis() - lastMillis_tx_packet_last_dongle) > TIMEOUT_TX_PACKET_LAST_DONGLE) {
            SerialWireless.SendPacket((const uint8_t *)aux_buffer_tx, 13, PACKET_TX::CHECK_CONNECTION_LAST_DONGLE);
            lastMillis_tx_packet_last_dongle = millis();
        }
        yield();
    }
    if (wireless_connection_state == CONNECTION_STATE::DEVICES_CONNECTED_WITH_LAST_DONGLE) {
        wireless_connection_state = CONNECTION_STATE::DEVICES_CONNECTED;
        TinyUSBDevices.onBattery = true;
        return true;
    } else {
        wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;
        return false;
    }
}

bool SerialWireless_::connection_gun() {
    #define TIMEOUT_GUN_DIALOGUE 1000  // in milliseconds
    unsigned long lastMillis_start_dialogue = millis();

    lastMillis_start_dialogue = millis();
    wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;
    // SET THE PEER BOARCAST HERE ======================
    if (esp_now_del_peer(peerAddress) != ESP_OK) {  // delete the broadcast from peers
        Serial.println("Error in peer deletion");
    }
    memcpy(peerAddress, BROADCAST_ADDR, 6);
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    // peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    // peerInfo.encrypt = false;
    Serial.println("Add peer");
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // insert the dongle in peers
        Serial.println("Error in peer addition");
    }
    // ====================================================

    while (
    #ifndef WIRELESS_ONLY
        !TinyUSBDevice.mounted() &&
    #endif
        wireless_connection_state != CONNECTION_STATE::TX_GUN_TO_DONGLE_CONFIRM) {
        if (wireless_connection_state == CONNECTION_STATE::NONE_CONNECTION) {
            lastMillis_start_dialogue = millis();
        }

        if (((millis() - lastMillis_start_dialogue) > TIMEOUT_GUN_DIALOGUE) &&
            wireless_connection_state != CONNECTION_STATE::TX_GUN_TO_DONGLE_CONFIRM) {
            wireless_connection_state = CONNECTION_STATE::NONE_CONNECTION;
        }
    }

    if (wireless_connection_state == CONNECTION_STATE::TX_GUN_TO_DONGLE_CONFIRM) {
        if (esp_now_del_peer(peerAddress) != ESP_OK) {  // delete the broadcast from peers
                                                        // Serial.println("Error in peer deletion");
        }

        memcpy(peerAddress, mac_esp_another_card, 6);
        memcpy(peerInfo.peer_addr, peerAddress, 6);
        // peerInfo.channel = ESPNOW_WIFI_CHANNEL;
        // peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // adds the dongle to peers
                                                      // Serial.println("Error adding new peer");
        }

        TinyUSBDevices.onBattery = true;
        return true;
    }
    return false;
}

void packet_callback_read_dongle() {
    switch (SerialWireless.packet.currentPacketID()) {
        case PACKET_TX::SERIAL_TX:
            Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
            Serial.flush();  // ????
            break;
    #ifndef WIRELESS_ONLY
        case PACKET_TX::MOUSE_TX:
            usbHid.sendReport(HID_RID_e::HID_RID_MOUSE, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE],
                              SerialWireless.packet.bytesRead);
            break;
        case PACKET_TX::GAMEPADE_TX:
            usbHid.sendReport(HID_RID_e::HID_RID_GAMEPAD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE],
                              SerialWireless.packet.bytesRead);
            break;
        case PACKET_TX::KEYBOARD_TX:
            usbHid.sendReport(HID_RID_e::HID_RID_KEYBOARD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE],
                              SerialWireless.packet.bytesRead);
            break;
    #endif
        case PACKET_TX::CHECK_CONNECTION_LAST_DONGLE:
            // CODICE
            memcpy(aux_buffer, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
            switch (aux_buffer[0]) {
                case CONNECTION_STATE::TX_CHECK_CONNECTION_LAST_DONGLE:
                    if ((memcmp(&aux_buffer[1], peerAddress, 6) == 0) &&
                        (memcmp(&aux_buffer[7], SerialWireless.mac_esp_interface, 6) == 0) &&
                        SerialWireless.wireless_connection_state == CONNECTION_STATE::DEVICES_CONNECTED) {
                        aux_buffer[0] = CONNECTION_STATE::TX_CONFIRM_CONNECTION_LAST_DONGLE;
                        memcpy(&aux_buffer[1], SerialWireless.mac_esp_interface, 6);
                        memcpy(&aux_buffer[7], peerAddress, 6);
                        // evaluate whether to send the packet a couple of times or just once
                        // sends it 3 times - once every 70ms
                        for (uint8_t i = 0; i < 3; i++) {
                            if (i > 0) {
                                // vTaskDelay(pdMS_TO_TICKS(1000)); // equivalent to delay(1000) but not blocking on
                                // esp32
                                unsigned long lastMillis_tx_packet_connection_last_dongle = millis();
                                while ((millis() - lastMillis_tx_packet_connection_last_dongle) < 70)
                                    yield();
                            }
                            SerialWireless.SendPacket((const uint8_t *)aux_buffer, 13,
                                                      PACKET_TX::CHECK_CONNECTION_LAST_DONGLE);
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case PACKET_TX::CONNECTION:
            memcpy(aux_buffer, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE],
                   SerialWireless.packet.bytesRead);  // sizeof(aux_buffer));
            // Serial.println("DONGLE - connection request arrived");
            switch (aux_buffer[0]) {
                case CONNECTION_STATE::TX_GUN_TO_DONGLE_PRESENCE:
                    if ((memcmp(&aux_buffer[7], SerialWireless.mac_esp_interface, 6) == 0) &&
                        SerialWireless.wireless_connection_state == CONNECTION_STATE::NONE_CONNECTION) {
                        memcpy(SerialWireless.mac_esp_another_card, &aux_buffer[1], 6);
                        aux_buffer[0] = CONNECTION_STATE::TX_DONGLE_TO_GUN_ACCEPT;
                        memcpy(&aux_buffer[1], SerialWireless.mac_esp_interface, 6);
                        memcpy(&aux_buffer[7], SerialWireless.mac_esp_another_card, 6);
                        SerialWireless.SendPacket((const uint8_t *)aux_buffer, 13, PACKET_TX::CONNECTION);
                        // Serial.println("DONGLE - sent connection confirmation packet");

                        // ensure that the data has been sent

                        SerialWireless.wireless_connection_state = CONNECTION_STATE::TX_DONGLE_TO_GUN_ACCEPT;
                    }
                    break;
                case CONNECTION_STATE::TX_GUN_TO_DONGLE_CONFIRM:
                    if ((memcmp(&aux_buffer[1], SerialWireless.mac_esp_another_card, 6) == 0) &&
                        (memcmp(&aux_buffer[7], SerialWireless.mac_esp_interface, 6) == 0) &&
                        SerialWireless.wireless_connection_state == CONNECTION_STATE::TX_DONGLE_TO_GUN_ACCEPT) {
                        // SAVE THE DATA RELATING TO THE GUN VID, PID, PLAYER , ETC.
                        memcpy(&usb_data_wireless, &aux_buffer[13], sizeof(usb_data_wireless));
                        SerialWireless.wireless_connection_state = CONNECTION_STATE::DEVICES_CONNECTED;
                        // Serial.println("DONGLE - received connection packet with GUN data");
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void packet_callback_read_gun() {
    Serial.println("Packet callback read gun");
    Serial.println(SerialWireless.packet.currentPacketID());
    switch (SerialWireless.packet.currentPacketID()) {
        case PACKET_TX::SERIAL_TX:
            SerialWireless.write_on_rx_serialBuffer(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE],
                                                    SerialWireless.packet.bytesRead);
            break;
        case PACKET_TX::MOUSE_TX:
            /* code */
            break;
        case PACKET_TX::GAMEPAD_TX:
            /* code */
            break;
        case PACKET_TX::KEYBOARD_TX:
            /* code */
            break;
        case PACKET_TX::CHECK_CONNECTION_LAST_DONGLE:
            // CODE TO EVALUATE WHETHER TO DO A CHECK IF THE GUN IS STILL CONNECTED .. IF NOT RESTART THE CARD ?
            Serial.println("Check connection last dongle");
            memcpy(aux_buffer, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
            Serial.println("Copy aux buffer");
            switch (aux_buffer[0]) {
                case CONNECTION_STATE::TX_CONFIRM_CONNECTION_LAST_DONGLE:
                    if ((memcmp(&aux_buffer[1], peerAddress, 6) == 0) &&
                        (memcmp(&aux_buffer[7], SerialWireless.mac_esp_interface, 6) == 0) &&
                        SerialWireless.wireless_connection_state == CONNECTION_STATE::NONE_CONNECTION_DONGLE) {
                        SerialWireless.wireless_connection_state = CONNECTION_STATE::DEVICES_CONNECTED_WITH_LAST_DONGLE;
                    }
                    break;
                default:
                    break;
            }
            break;
        case PACKET_TX::CONNECTION:
            Serial.println("Connection");
            memcpy(
                aux_buffer, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE],
                SerialWireless.packet.bytesRead);  // 13); //sizeof(aux_buffer)); // it's also fine to copy 13 as data
            Serial.println("Copy aux buffer 2");
            switch (aux_buffer[0]) {
                case CONNECTION_STATE::TX_DONGLE_SEARCH_GUN_BROADCAST:
                    if (SerialWireless.wireless_connection_state ==
                        CONNECTION_STATE::NONE_CONNECTION) {  // takes the first available dongle
                        memcpy(SerialWireless.mac_esp_another_card, &aux_buffer[1], 6);
                        // send connection request
                        aux_buffer[0] = CONNECTION_STATE::TX_GUN_TO_DONGLE_PRESENCE;
                        memcpy(&aux_buffer[1], SerialWireless.mac_esp_interface, 6);
                        memcpy(&aux_buffer[7], SerialWireless.mac_esp_another_card, 6);
                        SerialWireless.SendPacket((const uint8_t *)aux_buffer, 13, PACKET_TX::CONNECTION);
                        SerialWireless.wireless_connection_state = CONNECTION_STATE::TX_GUN_TO_DONGLE_PRESENCE;
                        // ensure that the data has been sent
                    }
                    break;
                case 2:
                    /* code */
                    break;
                case CONNECTION_STATE::TX_DONGLE_TO_GUN_ACCEPT:
                    if ((memcmp(&aux_buffer[1], SerialWireless.mac_esp_another_card, 6) == 0) &&
                        (memcmp(&aux_buffer[7], SerialWireless.mac_esp_interface, 6) == 0) &&
                        SerialWireless.wireless_connection_state == CONNECTION_STATE::TX_GUN_TO_DONGLE_PRESENCE) {
                        aux_buffer[0] = CONNECTION_STATE::TX_GUN_TO_DONGLE_CONFIRM;
                        memcpy(&aux_buffer[1], SerialWireless.mac_esp_interface, 6);
                        memcpy(&aux_buffer[7], SerialWireless.mac_esp_another_card, 6);
                        // ALSO SEND DATA RELATING TO VID, PID, ETC, ETC, OF THE GUN
                        memcpy(&aux_buffer[13], &usb_data_wireless, sizeof(usb_data_wireless));

                        // =========================================================
                        // SEND THE FINAL PACKET MULTIPLE TIMES
                        // =========================================================
                        for (uint8_t i = 0; i < 3; i++) {
                            if (i > 0) {
                                // vTaskDelay(pdMS_TO_TICKS(1000)); // equivalente a delay(1000) ma non bloccante su
                                // esp32
                                unsigned long lastMillis_tx_packet_gun_to_dongle_conferm = millis();
                                while ((millis() - lastMillis_tx_packet_gun_to_dongle_conferm) < 70)
                                    yield();
                            }
                            SerialWireless.SendPacket((const uint8_t *)aux_buffer, sizeof(aux_buffer),
                                                      PACKET_TX::CONNECTION);
                        }
                        SerialWireless.wireless_connection_state = CONNECTION_STATE::TX_GUN_TO_DONGLE_CONFIRM;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if ((FIFO_SIZE_READ - SerialWireless._readLen) >= len) {
        size_t firstChunk = FIFO_SIZE_READ - SerialWireless._writer;
        if (firstChunk < len) {
            memcpy(&SerialWireless._queue[SerialWireless._writer], data, firstChunk);
            SerialWireless._writer = len - firstChunk;
            memcpy(&SerialWireless._queue[0], data + firstChunk, SerialWireless._writer);
        } else {
            memcpy(&SerialWireless._queue[SerialWireless._writer], data, len);
            SerialWireless._writer += len;
            if (SerialWireless._writer == FIFO_SIZE_READ)
                SerialWireless._writer = 0;
        }
        SerialWireless._readLen += len;
    } else {
        SerialWireless._overflow_read = true;
        // Serial.println("Overflow in writing the READ BUFFER");
    }
    SerialWireless.checkForRxPacket();
}

static void _esp_now_tx_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    xSemaphoreTake(mutex_writer_bin, portMAX_DELAY);

    if (SerialWireless._writeLen > 0) {
        SerialWireless.SendData_sem();
    } else
        xSemaphoreGive(tx_sem);

    xSemaphoreGive(mutex_writer_bin);
}

// ================================== TIMER ===================================
// CALLBACK
void timer_callback_serial(void *arg) {
    // Serial.println("Timer expired!");

    xSemaphoreTake(mutex_tx_serial, portMAX_DELAY);

    while (SerialWireless.lenBufferSerialWrite) {
        SerialWireless.flush_sem();
        taskYIELD();  // yield();
    }

    xSemaphoreGive(mutex_tx_serial);  // Release the semaphore after the callback
}
// ===============================================================================

void SerialWireless_::setupTimer() {
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback_serial, .arg = NULL, .dispatch_method = ESP_TIMER_TASK, .name = "timer_serial"};

    esp_timer_create(&timer_args, &timer_handle_serial);
}

void SerialWireless_::stopTimer_serial() {
    esp_timer_stop(timer_handle_serial);
}

void SerialWireless_::resetTimer_serial(uint64_t duration_us) {
    esp_timer_stop(timer_handle_serial);
    esp_timer_start_once(timer_handle_serial, duration_us);
}

// ==========================   FINE TIMER ====================================

#endif  // OPENFIRE_WIRELESS_ENABLE