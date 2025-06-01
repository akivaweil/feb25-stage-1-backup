#include "StateMachine/StateManager.h"
#include "StateMachine/00_STARTUP.h"
#include "StateMachine/01_HOMING.h"
#include "StateMachine/02_IDLE.h"
#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/07_FEED_WOOD_FWD_ONE.h"
#include "StateMachine/03_CUTTING.h"
#include "StateMachine/04_Yes_2x4.h"
#include "StateMachine/05_No_2x4.h"
#include "ErrorStates/standard_error.h"
#include "ErrorStates/error_reset.h"
#include "ErrorStates/suction_error_hold.h"
#include <memory>

//* ************************************************************************
//* ************************* STATE MANAGER *******************************
//* ************************************************************************
// Central state manager implementation that coordinates all state operations.

// Global state manager instance
StateManager stateManager;

// State instances
static StartupState startupState;
static HomingState homingState;
static IdleState idleState;
static FeedFirstCutState feedFirstCutState;
static FeedWoodFwdOneState feedWoodFwdOneState;
static CuttingState cuttingState;
static Yes2x4State yes2x4State;
static No2x4State no2x4State;

StateManager::StateManager() : previousState(ERROR_RESET) {
    // Constructor - previousState initialized to different state to ensure first print
}

void StateManager::execute() {
    handleCommonOperations();
    
    // Update error LED blinking for ERROR state
    if (currentState == ERROR) {
        handleErrorLedBlink();
    }
    
    switch (currentState) {
        case STARTUP:
            startupState.execute(*this);
            break;
        case HOMING:
            homingState.execute(*this);
            break;
        case IDLE:
            idleState.execute(*this);
            break;
        case FEED_FIRST_CUT:
            feedFirstCutState.execute(*this);
            break;
        case FEED_WOOD_FWD_ONE:
            feedWoodFwdOneState.execute(*this);
            break;
        case CUTTING:
            cuttingState.execute(*this);
            break;
        case Yes_2x4:
            yes2x4State.execute(*this);
            break;
        case No_2x4:
            no2x4State.execute(*this);
            break;
        case ERROR:
            handleStandardErrorState();
            break;
        case ERROR_RESET:
            handleErrorResetState();
            break;
        case SUCTION_ERROR_HOLD:
            handleSuctionErrorHoldState();
            break;
    }
}

void StateManager::changeState(SystemState newState) {
    if (currentState != newState) {
        // Call onExit for the current state before changing
        switch (currentState) {
            case STARTUP: startupState.onExit(*this); break;
            case HOMING: homingState.onExit(*this); break;
            case IDLE: idleState.onExit(*this); break;
            case FEED_FIRST_CUT: feedFirstCutState.onExit(*this); break;
            case FEED_WOOD_FWD_ONE: feedWoodFwdOneState.onExit(*this); break;
            case CUTTING: cuttingState.onExit(*this); break;
            case Yes_2x4: yes2x4State.onExit(*this); break;
            case No_2x4: no2x4State.onExit(*this); break;
            // Error states don't have onExit handlers
            default: break;
        }
        
        previousState = currentState;
        currentState = newState;
        
        // Call onEnter for the new state after changing
        switch (newState) {
            case STARTUP: startupState.onEnter(*this); break;
            case HOMING: homingState.onEnter(*this); break;
            case IDLE: idleState.onEnter(*this); break;
            case FEED_FIRST_CUT: feedFirstCutState.onEnter(*this); break;
            case FEED_WOOD_FWD_ONE: feedWoodFwdOneState.onEnter(*this); break;
            case CUTTING: cuttingState.onEnter(*this); break;
            case Yes_2x4: yes2x4State.onEnter(*this); break;
            case No_2x4: no2x4State.onEnter(*this); break;
            // Error states don't have onEnter handlers
            default: break;
        }
        
        // Debug print for state transitions
        switch (newState) {
            case STARTUP: Serial.println("STARTUP"); break;
            case HOMING: Serial.println("HOMING"); break;
            case IDLE: Serial.println("IDLE"); break;
            case FEED_FIRST_CUT: Serial.println("FEED_FIRST_CUT"); break;
            case FEED_WOOD_FWD_ONE: Serial.println("FEED_WOOD_FWD_ONE"); break;
            case CUTTING: Serial.println("CUTTING"); break;
            case Yes_2x4: Serial.println("Yes_2x4"); break;
            case No_2x4: Serial.println("No_2x4"); break;
            case ERROR: Serial.println("ERROR"); break;
            case ERROR_RESET: Serial.println("ERROR_RESET"); break;
            case SUCTION_ERROR_HOLD: Serial.println("SUCTION_ERROR_HOLD"); break;
        }
    }
}

void StateManager::printStateChange() {
    if (currentState != previousState) {
        Serial.print("Current State: ");
        switch (currentState) {
            case STARTUP: Serial.println("STARTUP"); break;
            case HOMING: Serial.println("HOMING"); break;
            case IDLE: Serial.println("IDLE"); break;
            case FEED_FIRST_CUT: Serial.println("FEED_FIRST_CUT"); break;
            case FEED_WOOD_FWD_ONE: Serial.println("FEED_WOOD_FWD_ONE"); break;
            case CUTTING: Serial.println("CUTTING"); break;
            case Yes_2x4: Serial.println("Yes_2x4"); break;
            case No_2x4: Serial.println("No_2x4"); break;
            case ERROR: Serial.println("ERROR"); break;
            case ERROR_RESET: Serial.println("ERROR_RESET"); break;
            case SUCTION_ERROR_HOLD: Serial.println("SUCTION_ERROR_HOLD"); break;
            default: Serial.println("UNKNOWN"); break;
        }
        previousState = currentState;
    }
}

void StateManager::updateSwitches() {
    // Update all debounced switches - moved from main loop
    cutHomingSwitch.update();
    positionHomingSwitch.update();
    reloadSwitch.update();
    startCycleSwitch.update();
    pushwoodForwardSwitch.update();
    fixPositionSwitch.update();
}

void StateManager::handleCommonOperations() {
    // Update all switches first
    updateSwitches();
    
    // Check for cut motor hitting home sensor during Yes_2x4 return
    extern bool cutMotorInYes2x4Return; // This global flag is still in main.cpp
    if (cutMotorInYes2x4Return && cutMotor && cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
        Serial.println("Cut motor hit homing sensor during Yes_2x4 return - stopping immediately!");
        cutMotor->forceStopAndNewPosition(0);  // Stop immediately and set position to 0
    }

    // Handle rotation servo return after hold duration at active position AND when WAS_WOOD_SUCTIONED_SENSOR reads HIGH
    if (rotationServoIsActiveAndTiming && millis() - rotationServoActiveStartTime >= ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS) {
        extern const int WOOD_SUCTION_CONFIRM_SENSOR; // This is in main.cpp
        if (digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == HIGH) {
            // Return rotation servo to home position
            rotationServo.write(ROTATION_SERVO_HOME_POSITION);
            Serial.println("Servo timing completed AND WAS_WOOD_SUCTIONED_SENSOR is HIGH, returning rotation servo to home.");
            rotationServoIsActiveAndTiming = false; // Clear flag
        } else {
            Serial.println("Waiting for WAS_WOOD_SUCTIONED_SENSOR to read HIGH before returning rotation servo...");
        }
    }

    // Handle Rotation Clamp retraction after 1 second
    if (rotationClampIsExtended && (millis() - rotationClampExtendTime >= ROTATION_CLAMP_EXTEND_DURATION_MS)) {
        retractRotationClamp();
        Serial.println("Rotation Clamp retracted after 1 second.");
    }

    // 2x4 sensor - Update global _2x4Present flag
    extern const int _2x4_PRESENT_SENSOR; // This is in main.cpp
    _2x4Present = (digitalRead(_2x4_PRESENT_SENSOR) == LOW);
    
    // Handle start switch safety check
    if (!startSwitchSafe && startCycleSwitch.fell()) {
        startSwitchSafe = true;
    }
    
    // Handle error acknowledgment separately
    if (reloadSwitch.rose() && currentState == ERROR) {
        changeState(ERROR_RESET);
        errorAcknowledged = true;
    }
    
    // Check for continuous mode activation/deactivation - modified to include safety check
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
    }
    
    // Handle TA signal timeout after TA_SIGNAL_DURATION
    if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
        extern const int TRANSFER_ARM_SIGNAL_PIN; // This is in main.cpp
        digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
        signalTAActive = false;
        Serial.println("Signal to Transfer Arm (TA) timed out and reset to LOW"); 
    }
} 