#ifndef SIMPLE_BUTTON_DEBOUNCER_H
#define SIMPLE_BUTTON_DEBOUNCER_H

#include <Arduino.h>

class SimpleButtonDebouncer {
   private:
    uint8_t pin;
    uint32_t debounceTime;
    bool lastState;
    bool currentState;
    unsigned long lastChangeTime;
    bool isInitialized;

   public:
    SimpleButtonDebouncer();

    // Initialize the button with pin and debounce time
    void begin(uint8_t buttonPin, uint32_t debounceMs = 50);

    // Poll the button state - call this regularly
    bool poll();

    // Get current stable state
    bool getState() const {
        return currentState;
    }

    // Check if button was just pressed (rising edge)
    bool wasPressed();

    // Check if button was just released (falling edge)
    bool wasReleased();

    // Get the pin number
    uint8_t getPin() const {
        return pin;
    }

    // Check if button is initialized
    bool initialized() const {
        return isInitialized;
    }

   private:
    bool pressed;
    bool released;
};

#endif  // SIMPLE_BUTTON_DEBOUNCER_H