#include "ErrorStates/STANDARD_ERROR.h"
#include "ErrorStates/ERROR_RESET.h"  // For error timing constants
#include "StateMachine/StateManager.h"

// External references to functions from main.cpp (LED and motor functions only)
extern void turnRedLedOn();
extern void turnRedLedOff();
extern void turnYellowLedOn();
extern void turnYellowLedOff();
extern void stopCutMotor();
extern void stopFeedMotor();

//* ************************************************************************
//* ***************************** ERROR ************************************
//* ************************************************************************
// Handles system error states.
// Step 1: Blink red and yellow LEDs to indicate an error at standard rate.
// Step 2: Ensure cut and feed motors are stopped.
// Step 3: Wait for the reload switch to be pressed (rising edge) to acknowledge the error.
// Step 4: Once error is acknowledged, transition to ERROR_RESET state.
void handleStandardErrorState() {
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
        Serial.println("Error acknowledged in standard error state. Transitioning to ERROR_RESET.");
    }
} 