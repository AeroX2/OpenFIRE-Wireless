#include "SimpleButtonDebouncer.h"

SimpleButtonDebouncer::SimpleButtonDebouncer() {
    pin = 0;
    debounceTime = 50;
    lastState = false;
    currentState = false;
    lastChangeTime = 0;
    isInitialized = false;
    pressed = false;
    released = false;
}

void SimpleButtonDebouncer::begin(uint8_t buttonPin, uint32_t debounceMs) {
    pin = buttonPin;
    debounceTime = debounceMs;
    lastState = false;
    currentState = false;
    lastChangeTime = 0;
    pressed = false;
    released = false;

    // Set pin as input with internal pull-up
    pinMode(pin, INPUT_PULLUP);

    // Read initial state
    lastState = digitalRead(pin);
    currentState = lastState;

    isInitialized = true;
}

bool SimpleButtonDebouncer::poll() {
    if (!isInitialized) {
        return false;
    }

    bool rawState = digitalRead(pin);
    unsigned long currentTime = millis();

    // Reset edge detection flags
    pressed = false;
    released = false;

    // If state changed
    if (rawState != lastState) {
        lastChangeTime = currentTime;
        lastState = rawState;
    }

    // If enough time has passed since last change, update stable state
    if (currentTime - lastChangeTime >= debounceTime) {
        if (rawState != currentState) {
            // State has changed after debounce period
            if (rawState && !currentState) {
                // Button was pressed (rising edge)
                pressed = true;
            } else if (!rawState && currentState) {
                // Button was released (falling edge)
                released = true;
            }
            currentState = rawState;
        }
    }

    return currentState;
}

bool SimpleButtonDebouncer::wasPressed() {
    return pressed;
}

bool SimpleButtonDebouncer::wasReleased() {
    return released;
}