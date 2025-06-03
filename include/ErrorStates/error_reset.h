#ifndef ERROR_RESET_H
#define ERROR_RESET_H

#include <Arduino.h>
#include "ErrorStates/GENERAL_FUNCTIONS.h"

// Error timing constants
const unsigned long STANDARD_ERROR_BLINK_INTERVAL = 250;  // ms - Standard error LED blink rate
const unsigned long SUCTION_ERROR_BLINK_INTERVAL = 1500;  // ms - Suction error LED blink rate (slower)

// Function declarations for error reset state functionality
void handleErrorResetState();

#endif // ERROR_RESET_H 