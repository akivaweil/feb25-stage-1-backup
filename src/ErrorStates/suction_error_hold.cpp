#include "ErrorStates/Suction_Error_Hold.h"
#include "ErrorStates/Error_Reset.h"  // For error timing constants
#include "StateMachine/StateManager.h"
#include <Bounce2.h>

// External references to functions from main.cpp (LED functions only)
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOff();
extern void turnGreenLedOff();
extern void turnBlueLedOff();

//* ************************************************************************
//* ********************* SUCTION ERROR HOLD *******************************
//* ************************************************************************
// Handles waiting for user to reset a wood suction error via cycle switch.
// This state is entered from CUTTING (Step 1) if the WOOD_SUCTION_CONFIRM_SENSOR indicates an error (LOW).
// Step 1: Slowly blink the red LED using defined suction error timing interval.
// Step 2: Ensure yellow, green, and blue LEDs are off.
// Step 3: Monitor the start cycle switch.
// Step 4: If the start cycle switch shows a rising edge (OFF to ON transition):
//          - Print a message about resetting from suction error.
//          - Turn off the red LED.
//          - Set continuousModeActive to false.
//          - Set startSwitchSafe to false (requires user to cycle switch again for a new start).
//          - Transition to HOMING state to re-initialize the system.
void handleSuctionErrorHoldState() {
    static unsigned long lastSuctionErrorBlinkTime = 0;
    static bool suctionErrorBlinkState = false;

    // Blink STATUS_LED_RED using defined suction error timing interval
    if (millis() - lastSuctionErrorBlinkTime >= SUCTION_ERROR_BLINK_INTERVAL) {
        lastSuctionErrorBlinkTime = millis();
        suctionErrorBlinkState = !suctionErrorBlinkState;
        if(suctionErrorBlinkState) turnRedLedOn(); else turnRedLedOff();
    }
    
    // Ensure other LEDs are off
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();

    // Use StateManager to access switches instead of global variables
    if (stateManager.getStartCycleSwitch()->rose()) { // Check for start switch OFF to ON transition
        Serial.println("Start cycle switch toggled ON. Resetting from suction error. Transitioning to HOMING.");
        turnRedLedOff();   // Turn off error LED explicitly before changing state
        
        stateManager.setContinuousModeActive(false); // Ensure continuous mode is off
        stateManager.setStartSwitchSafe(false);      // Require user to cycle switch OFF then ON for a new actual start
        
        stateManager.changeState(HOMING);        // Go to HOMING to re-initialize using proper StateManager method
    }
} 