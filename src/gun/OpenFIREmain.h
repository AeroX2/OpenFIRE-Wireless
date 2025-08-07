/*!
 * @file OpenFIREmain.h
 * @brief OpenFIRE main control program.
 *
 * @copyright Samco, https://github.com/samuelballantyne, June 2020
 * @copyright Mike Lynch, July 2021
 * @copyright That One Seong, https://github.com/SeongGino, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @author Mike Lynch
 * @author [That One Seong](SeongsSeongs@gmail.com)
 * @date 2025
 */

#ifndef _OPENFIREMAIN_H_
#define _OPENFIREMAIN_H_

#include <Arduino.h>
#include <Wire.h>
// include TinyUSB or HID depending on USB stack option
#if defined(USE_TINYUSB)
    #include <Adafruit_TinyUSB.h>
#elif defined(CFG_TUSB_MCU)
    #error Incompatible USB stack. Use Adafruit TinyUSB.
#endif

#include <DFRobotIRPositionEx.h>
#include <OpenFIREBoard.h>

#include "OpenFIREDefines.h"
#include "OpenFIREcommon.h"

#ifdef ARDUINO_ARCH_RP2040
    #include <hardware/irq.h>
    #include <hardware/pwm.h>
// declare PWM ISR
void rp2040pwmIrq(void);
#elif defined(ARDUINO_ARCH_ESP32)  // 696969 for ESP32
hw_timer_t *My_timer = NULL;
void ARDUINO_ISR_ATTR esp32s3pwmIrq(void);
#endif

#define POLL_RATE 1

// TinyUSB devices interface object that's initialized in MainCoreSetup
// TinyUSBDevices_ TUSBDeviceSetup; // 696969 removed, not needed

// Selector for which profile in the profile selector of the simple pause menu you're picking.
uint8_t profileModeSelection = 0;
// Flag to tell if we're in the profile selector submenu of the simple pause menu.
bool pauseModeSelectingProfile = false;

// Timestamp of when we started holding a buttons combo.
unsigned long pauseHoldStartstamp;
bool pauseHoldStarted = false;
bool pauseExitHoldStarted = false;

// Timestamp of last USB packet update.
unsigned long lastUSBpoll = 0;

uint32_t fifoData = 0;

// ============ VARIABLES and CONSTANTS ADDED BY ME ========= 696969

Stream *Serial_OpenFIRE_Stream;  // SERVES TO MANAGE WIRELESS SERIAL

// == for Analog Stick management ==
#define ANALOG_STICK_MIN_X 0        // not needed at the moment
#define ANALOG_STICK_MAX_X 4095     // not needed at the moment
#define ANALOG_STICK_MIN_Y 0        // not needed at the moment
#define ANALOG_STICK_MAX_Y 4095     // not needed at the moment
#define ANALOG_STICK_CENTER_X 2048  // serves to send data to the simulated joystick value between 0 to 4095
#define ANALOG_STICK_CENTER_Y 2048  // serves to send data to the simulated joystick value between 0 to 4095
#if defined(ARDUINO_ARCH_RP2040)
uint16_t ANALOG_STICK_DEADZONE_X_MIN = 1900;  // set to the same original OpenFIRE values
uint16_t ANALOG_STICK_DEADZONE_X_MAX = 2200;  // set to the same original OpenFIRE values
uint16_t ANALOG_STICK_DEADZONE_Y_MIN = 1900;  // set to the same original OpenFIRE values
uint16_t ANALOG_STICK_DEADZONE_Y_MAX = 2200;  // set to the same original OpenFIRE values
uint16_t ANALOG_STICK_DEADZONE_X_CENTER =
    ANALOG_STICK_CENTER_X;  // serves to send data to the simulated joystick value between 0 to 4095
uint16_t ANALOG_STICK_DEADZONE_Y_CENTER =
    ANALOG_STICK_CENTER_Y;  // serves to send data to the simulated joystick value between 0 to 4095
#elif defined(ARDUINO_ARCH_ESP32)
uint16_t ANALOG_STICK_DEADZONE_X_MIN =
    ANALOG_STICK_MAX_X;  // serves to read values from the joystick connected to the micro (deadzone relative to center,
                         // calculates it in setup phase)
uint16_t ANALOG_STICK_DEADZONE_X_MAX =
    ANALOG_STICK_MIN_X;  // serves to read values from the joystick connected to the micro (deadzone relative to center,
                         // calculates it in setup phase)
uint16_t ANALOG_STICK_DEADZONE_Y_MIN =
    ANALOG_STICK_MAX_Y;  // serves to read values from the joystick connected to the micro (deadzone relative to center,
                         // calculates it in setup phase)
uint16_t ANALOG_STICK_DEADZONE_Y_MAX =
    ANALOG_STICK_MIN_Y;  // serves to read values from the joystick connected to the micro (deadzone relative to center,
                         // calculates it in setup phase)
uint16_t ANALOG_STICK_DEADZONE_X_CENTER =
    ANALOG_STICK_CENTER_X;  // serves to send data to the simulated joystick value between 0 to 4095
uint16_t ANALOG_STICK_DEADZONE_Y_CENTER =
    ANALOG_STICK_CENTER_Y;  // serves to send data to the simulated joystick value between 0 to 4095
#endif
// == END for Analog Stick management ==

// ============ FUNCTION DEFINITIONS ================== 696969 for platformio - in arduino IDE they can also not be
// defined first
void startIrCamTimer(const int &frequencyHz);
void ExecGunModeDocked();
void TriggerFire();
void TriggerNotFire();
void TriggerFireSimple();
void TriggerNotFireSimple();
void AnalogStickPoll();
void SendEscapeKey();
void SetProfileSelection(const bool &isIncrement);
void SetPauseModeSelection(const bool &isIncrement);
void RumbleToggle();
void SolenoidToggle();
void IncreaseIrSensitivity(const uint32_t &sens);
void DecreaseIrSensitivity(const uint32_t &sens);
void OffscreenToggle();
void AutofireSpeedToggle();
void SelectCalProfileFromBtnMask(const uint32_t &mask);
void ExecRunMode();

#ifdef ARDUINO_ARCH_RP2040
void rp2040EnablePWMTimer(const unsigned int &slice_num, const unsigned int &frequency);
#endif

// ==========================================================

#endif  // _SAMCOENHANCED_H_