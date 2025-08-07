// by SATANASSI Alessandro

#include <Arduino.h>
#include <SPI.h>

#include "TinyUSB_Devices.h"

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

#endif  // USES_DISPLAY

#include "OpenFIRE-DONGLE-version.h"

// ================== DUAL CORE MANAGEMENT ============================
#if defined(DUAL_CORE) && defined(ESP_PLATFORM) && false
void setup1();
void loop1();
TaskHandle_t task_loop1;
void esploop1(void* pvParameters) {
    setup1();
    for (;;)
        loop1();
}
#endif  // DUAL_CORE
// ========================= END DUAL CORE MANAGEMENT ======================================

// The main show!
void setup() {
// =========================== X DUAL CORE MANAGEMENT ===============================
#if defined(DUAL_CORE) && defined(ESP_PLATFORM) && false
    xTaskCreatePinnedToCore(esploop1,               /* Task function. */
                            "loop1",                /* name of task. */
                            10000,                  /* Stack size of task */
                            NULL,                   /* parameter of the task */
                            1,                      /* priority of the task */
                            &task_loop1,            /* Task handle to keep track of created task */
                            !ARDUINO_RUNNING_CORE); /* pin task to core 0 */
#endif
    // ======================== END X DUAL CORE MANAGEMENT =================================

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

    tft.fillScreen(BLACK);
    #ifdef USE_LOVYAN_GFX
    tft.pushImage(10, 1, LOGO_RGB_ALPHA_OPEN_WIDTH, LOGO_RGB_ALPHA_OPEN_HEIGHT, (uint16_t*)logo_rgb_alpha_open);
    #else
    tft.drawRGBBitmap(10, 1, (uint16_t*)logo_rgb_alpha_open, LOGO_RGB_ALPHA_OPEN_WIDTH, LOGO_RGB_ALPHA_OPEN_HEIGHT);
    #endif  // USE_LOVYAN_GFX

    delay(3000);

    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("..SEARCHING..");
    tft.drawBitmap(10, 25, customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT, BLUE);
    tft.setTextSize(2);
    tft.setCursor(65, 30);
    tft.setTextColor(GRAY);
    tft.println("Channel");
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
#endif  // USES_DISPLAY

    // ====== wireless connection management ====================
    SerialWireless.begin();
    SerialWireless.connection_dongle();
    // ====== end wireless management .. continues only after the device has paired =======

    // ====== USB connection ====== sets VID and PID as passed by the gun ===============
    if (!TinyUSBDevice.isInitialized()) {  // added ..it worked the same, but this is safer .. definitely needed for
                                           // Esp32 with library not integrated in the core
        TinyUSBDevice.begin(0);
    }

    TinyUSBDevice.setManufacturerDescriptor(usb_data_wireless.deviceManufacturer);
    TinyUSBDevice.setProductDescriptor(usb_data_wireless.deviceName);
    TinyUSBDevice.setID(usb_data_wireless.deviceVID, usb_data_wireless.devicePID);

    // Initializing the USB devices chunk.
    TinyUSBDevices.begin(1);
    Serial.begin(9600);
    Serial.setTimeout(0);
    Serial.setTxTimeoutMs(0);
    // ====== end USB connection ==========================================================================

#ifdef USES_DISPLAY
    tft.fillScreen(BLACK);
    tft.drawBitmap(40, 0, customSplashBanner, CUSTSPLASHBANN_WIDTH, CUSTSPLASHBANN_HEIGHT, BLUE);
    tft.setTextSize(2);
    tft.setCursor(0, 20);
    tft.setTextColor(RED);
    tft.println(usb_data_wireless.deviceName);
    tft.setTextColor(RED);
    tft.setCursor(0, 40);
    tft.printf("Player: %d", usb_data_wireless.devicePlayer);
    tft.setTextSize(2);
    tft.setCursor(0, 60);
    tft.setTextColor(GRAY);
    tft.printf("Channel: %d", usb_data_wireless.channel);

#endif  // USES_DISPLAY
}

#define FIFO_SIZE_READ_SER 200  // the original was 32
#define TIME_OUT_AVALAIBLE 2
#define TIME_OUT_SERIAL_MICRO 1000  // 1000 microseconds = 1 millisecond

int rx_available = 0;
unsigned long startTime = 0;  // = millis();

uint64_t timer_serial_micro = 0;

// uint16_t len_aux;
uint8_t buffer_aux[FIFO_SIZE_READ_SER];
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1));
    rx_available = Serial.available();
    if (rx_available > FIFO_SIZE_READ_SER)
        rx_available = FIFO_SIZE_READ_SER;
    if (rx_available) {
        Serial.readBytes(buffer_aux, rx_available);
        SerialWireless.write(buffer_aux, rx_available);
        SerialWireless.flush();  // try to remove
    }
}