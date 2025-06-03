#include "ErrorStates/General_Error.h"
#include "ErrorStates/Error_Reset.h"  // For error timing constants
#include "StateMachine/StateManager.h"

// External references to functions from main.cpp (LED and motor functions only)
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOn();
extern void turnYellowLedOff();
extern void stopCutMotor();
extern void stopFeedMotor();

//* ************************************************************************
//* *********************** GENERAL ERROR **********************************
//* ************************************************************************
// Handles general system error states.
// Step 1: Blink red and yellow LEDs to indicate a general system error at standard rate.
// Step 2: Ensure cut and feed motors are stopped.
// Step 3: Wait for the reload switch to be pressed (rising edge) to acknowledge the error.
// Step 4: Once error is acknowledged, transition to ERROR_RESET state.
void handleGeneralErrorState() {
    // Blink error LEDs using StateManager timing
    if (millis() - stateManager.getLastErrorBlinkTime() > STANDARD_ERROR_BLINK_INTERVAL) {
        bool newBlinkState = !stateManager.getErrorBlinkState();
        stateManager.setErrorBlinkState(newBlinkState);
        
        if(newBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!newBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        
        stateManager.setLastErrorBlinkTime(millis());
    }
    
    // Keep motors stopped
    stopCutMotor();
    stopFeedMotor();
    
    // Wait for reload switch to acknowledge error using StateManager
    if (stateManager.getErrorAcknowledged()) {
        stateManager.changeState(ERROR_RESET);
        Serial.println("General error acknowledged. Transitioning to ERROR_RESET.");
    }
} 