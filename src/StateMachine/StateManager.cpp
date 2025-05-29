#include "StateMachine/StateManager.h"
#include "StateMachine/StartupState.h"
#include "StateMachine/HomingState.h"
#include "StateMachine/ReadyState.h"
#include "StateMachine/IdleState.h"
#include "StateMachine/CuttingState.h"
#include "StateMachine/NoWoodState.h"
#include "StateMachine/YesWoodState.h"
#include "StateMachine/ReturningState.h"
#include "StateMachine/PositioningState.h"
#include "ErrorStates/standard_error.h"
#include "ErrorStates/error_reset.h"
#include "ErrorStates/suction_error_hold.h"
#include "ErrorStates/fix_cut_motor_position.h"
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
static ReadyState readyState;
static IdleState idleState;
static CuttingState cuttingState;
static NoWoodState noWoodState;
static YesWoodState yesWoodState;
static ReturningState returningState;
static PositioningState positioningState;

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
        case READY:
            readyState.execute(*this);
            break;
        case IDLE:
            idleState.execute(*this);
            break;
        case CUTTING:
            cuttingState.execute(*this);
            break;
        case NOWOOD:
            noWoodState.execute(*this);
            break;
        case YESWOOD:
            yesWoodState.execute(*this);
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
        case FIX_CUT_MOTOR_POSITION:
            handleFixCutMotorPositionState();
            break;
    }
}

void StateManager::changeState(SystemState newState) {
    if (currentState != newState) {
        previousState = currentState;
        currentState = newState;
        
        // Debug print for state transitions
        switch (newState) {
            case STARTUP: Serial.println("STARTUP"); break;
            case HOMING: Serial.println("HOMING"); break;
            case READY: Serial.println("READY"); break;
            case IDLE: Serial.println("IDLE"); break;
            case CUTTING: Serial.println("CUTTING"); break;
            case NOWOOD: Serial.println("NOWOOD"); break;
            case YESWOOD: Serial.println("YESWOOD"); break;
            case ERROR: Serial.println("ERROR"); break;
            case ERROR_RESET: Serial.println("ERROR_RESET"); break;
            case SUCTION_ERROR_HOLD: Serial.println("SUCTION_ERROR_HOLD"); break;
            case FIX_CUT_MOTOR_POSITION: Serial.println("FIX_CUT_MOTOR_POSITION"); break;
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
            case CUTTING: Serial.println("CUTTING"); break;
            case NOWOOD: Serial.println("NOWOOD"); break;
            case YESWOOD: Serial.println("YESWOOD"); break;
            case RETURNING: Serial.println("RETURNING"); break;
            case POSITIONING: Serial.println("POSITIONING"); break;
            case ERROR: Serial.println("ERROR"); break;
            case ERROR_RESET: Serial.println("ERROR_RESET"); break;
            case SUCTION_ERROR_HOLD: Serial.println("SUCTION_ERROR_HOLD"); break;
            case FIX_CUT_MOTOR_POSITION: Serial.println("FIX_CUT_MOTOR_POSITION"); break;
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
    fixPositionButton.update();
}

void StateManager::handleCommonOperations() {
    // Update all switches first
    updateSwitches();
    
    // Check for cut motor hitting home sensor during YES_WOOD return
    extern bool cutMotorInYesWoodReturn; // This global flag is still in main.cpp
    if (cutMotorInYesWoodReturn && cutMotor && cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
        Serial.println("Cut motor hit homing sensor during YES_WOOD return - stopping immediately!");
        cutMotor->forceStopAndNewPosition(0);  // Stop immediately and set position to 0
    }

    // Handle servo return after hold duration at active position AND when WAS_WOOD_SUCTIONED_SENSOR reads HIGH
    if (catcherServoIsActiveAndTiming && (millis() - catcherServoActiveStartTime >= CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS)) {
        extern const int WAS_WOOD_SUCTIONED_SENSOR; // This is in main.cpp
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == HIGH) {
            catcherServoIsActiveAndTiming = false;
            handleCatcherServoReturn(); // Call function to return catcher servo to home position
            Serial.println("Servo timing completed AND WAS_WOOD_SUCTIONED_SENSOR is HIGH, returning catcher servo to home.");
        } else {
            // Only log this message periodically to avoid flooding the serial monitor
            static unsigned long lastLogTime = 0;
            if (millis() - lastLogTime >= 500) { // Log every 500ms
                Serial.println("Waiting for WAS_WOOD_SUCTIONED_SENSOR to read HIGH before returning catcher servo...");
                lastLogTime = millis();
            }
        }
    }

    // Handle Catcher Clamp disengagement after 1 second
    if (catcherClampIsEngaged && (millis() - catcherClampEngageTime >= CATCHER_CLAMP_ENGAGE_DURATION_MS)) {
        retractCatcherClamp();
        Serial.println("Catcher Clamp disengaged after 1 second.");
    }

    // Read wood sensor (active LOW per explanation)
    extern const int WOOD_SENSOR; // This is in main.cpp
    woodPresent = (digitalRead(WOOD_SENSOR) == LOW);
    
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
    
    // Handle signal timing independently of other operations
    if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
        extern const int TA_SIGNAL_OUT_PIN; // This is in main.cpp
        digitalWrite(TA_SIGNAL_OUT_PIN, LOW); // Return to inactive state (LOW)
        signalTAActive = false;
        Serial.println("Signal to Stage 1 to TA completed");
    }
    
    // Check for fix position button press
    extern Bounce fixPositionButton; // This is in main.cpp
    if (fixPositionButton.rose() && currentState == IDLE) {
        changeState(FIX_CUT_MOTOR_POSITION);
        initFixCutMotorPosition(); // Reset step
        allLedsOff();
        turnBlueLedOn(); // Indicate FIX_CUT_MOTOR_POSITION active
        Serial.println("FIX_CUT_MOTOR_POSITION button pressed in IDLE state. Starting cut motor home position verification.");
    }
} 