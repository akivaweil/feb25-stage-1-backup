#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
// This state manages a multi-step cutting process.
// It includes logic for normal cutting, deciding between a YES_WOOD Sequence and a NO_WOOD Sequence, and error handling.

void CuttingState::onEnter(StateManager& stateManager) {
    // Reset all step counters when entering cutting state
    resetSteps();
}

void CuttingState::onExit(StateManager& stateManager) {
    // Reset all step counters when exiting cutting state
    resetSteps();
}

void CuttingState::execute(StateManager& stateManager) {
    // Throttle state logging to every 2 seconds
    static unsigned long lastStateLogTime = 0;
    if (millis() - lastStateLogTime >= 2000) {
        Serial.println("Current State: CUTTING");
        lastStateLogTime = millis();
    }
    
    if (homePositionErrorDetected) {
        handleHomePositionError(stateManager);
        return;
    }
    
    // Handle signal timing independently of motor movements
    if (signalActive && millis() - signalStartTime >= 2000) { // Original was 2000ms
        signalActive = false;
    }

    switch (cuttingStep) {
        case 0: 
            handleCuttingStep0(stateManager);
            break;
        case 1: 
            handleCuttingStep1(stateManager);
            break;
        case 2: 
            handleCuttingStep2(stateManager);
            break;
        case 3: 
            handleCuttingStep3(stateManager);
            break;
        case 4: 
            handleCuttingStep4(stateManager);
            break;
        case 5: 
            handleCuttingStep5(stateManager);
            break;
        case 8:
            handleCuttingStep8_PositionMotorHomingSequence(stateManager);
            break;
    }
}

void CuttingState::handleCuttingStep0(StateManager& stateManager) {
    Serial.println("Cutting Step 0: Starting cut motion."); 
    moveCutMotorToCut();
    catcherClampActivatedThisCycle = false; // Reset for this cut cycle
    cuttingStep = 1;
}

void CuttingState::handleCuttingStep1(StateManager& stateManager) {
    // CUTTING (Step 1): Delay 1000ms, activate catcher servo for 3 seconds, then check WAS_WOOD_SUCTIONED_SENSOR
    extern const int WOOD_SUCTION_CONFIRM_SENSOR; // From main.cpp
    extern const unsigned long CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS; // From main.cpp
    
    if (stepStartTime == 0) {
        stepStartTime = millis();
        Serial.println("Cutting Step 1: Delaying 1000ms then activating catcher servo.");
    }

    if (millis() - stepStartTime >= 1000 && !catcherServoIsActiveAndTiming) {
        activateCatcherServo(); // The function sets timing flags
        Serial.println("Cutting Step 1: Catcher servo activated for 3 seconds.");
    }

    if (catcherServoIsActiveAndTiming && millis() - stepStartTime >= 1000 + CATCHER_SERVO_ACTIVE_HOLD_DURATION_MS) {
        if (digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == LOW) { // LOW means NO SUCTION (Error condition)
            Serial.println("Cutting Step 1: WAS_WOOD_SUCTIONED_SENSOR is LOW (No Suction). Error detected. Transitioning to SUCTION_ERROR_HOLD state.");
            stateManager.changeState(SUCTION_ERROR_HOLD);
            resetSteps();
            return;
        } else {
            Serial.println("Cutting Step 1: WAS_WOOD_SUCTIONED_SENSOR is HIGH (Suction OK). Advancing to Step 2.");
            Serial.println("Cutting Step 1: Starting cut motor toward cut position.");
            configureCutMotorForCutting();
            moveCutMotorToCut();
            cuttingStep = 2;
            stepStartTime = 0; // Reset for next step
        }
    }
}

void CuttingState::handleCuttingStep2(StateManager& stateManager) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const int WOOD_PRESENT_SENSOR; // From main.cpp
    
    // Check if motor finished moving to cut position
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cutting Step 2: Cut fully complete."); 
        sendSignalToTA(); // Signal to Transfer Arm
        configureCutMotorForReturn();

        int sensorValue = digitalRead(WOOD_PRESENT_SENSOR);
        bool noWoodDetected = (sensorValue == HIGH);
        
        if (noWoodDetected) {
            Serial.println("Cutting Step 2: NO_WOOD state - Wood sensor reads HIGH. Transitioning to NOWOOD state.");
            stateManager.changeState(NOWOOD);
            resetSteps();
        } else {
            Serial.println("Cutting Step 2: YES_WOOD state - Wood sensor reads LOW. Transitioning to YESWOOD state.");
            stateManager.changeState(YESWOOD);
            resetSteps();
        }
    }
}

void CuttingState::handleCuttingStep3(StateManager& stateManager) {
    Serial.println("Cutting Step 3: (Should be bypassed for wood path) Initial position move complete.");
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    if (positionMotor && !positionMotor->isRunning()) {
        retractPositionClamp();
        retractWoodSecureClamp();
        Serial.println("Position clamp and wood secure clamp disengaged.");

        configurePositionMotorForReturn();
        movePositionMotorToHome();
        Serial.println("Position motor moving to home (0).");
    }
}

void CuttingState::handleCuttingStep4(StateManager& stateManager) {
    Serial.println("Cutting Step 4: (Logic moved to Step 7 for wood path) Position motor at home (0).");
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES; // From main.cpp
    extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES; // From main.cpp
    extern const int CUT_MOTOR_STEPS_PER_INCH; // From main.cpp
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    
    if (positionMotor && !positionMotor->isRunning()) {
        retractPositionClamp();
        Serial.println("Position clamp disengaged.");

        if (cutMotor && !cutMotor->isRunning()) {
            Serial.println("Cut motor also at home. Checking cut motor position switch.");
            bool sensorDetectedHome = false;
            for (int i = 0; i < 3; i++) {
                delay(30);
                stateManager.getCutHomingSwitch()->update();
                Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(stateManager.getCutHomingSwitch()->read());
                if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                    sensorDetectedHome = true;
                    if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                    Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                    break;
                }
            }
            if (!sensorDetectedHome) {
                Serial.println("ERROR: Cut motor position switch did not detect home after return attempt.");
                if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
                    Serial.print("Attempting incremental move. Total moved: ");
                    Serial.print(cutMotorIncrementalMoveTotalInches);
                    Serial.println(" inches.");
                    cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                    cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                    // Stay in cuttingStep 4 to re-check sensor after move
                } else {
                    Serial.println("ERROR: Cut motor position switch did not detect home after MAX incremental moves!");
                    stopCutMotor();
                    stopPositionMotor();
                    extendPositionClamp();
                    turnRedLedOn();
                    turnYellowLedOff();
                    stateManager.changeState(ERROR);
                    stateManager.setErrorStartTime(millis());
                    resetSteps();
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset for next attempt
                    Serial.println("Transitioning to ERROR state due to cut motor homing failure after cut.");
                }
            } else {
                Serial.println("Cut motor position switch confirmed home. Moving position motor to final position.");
                cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                cuttingStep = 5; 
            }
        }
    }
}

void CuttingState::handleCuttingStep5(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    if (positionMotor && !positionMotor->isRunning()) {
        Serial.println("Cutting Step 5: Position motor at final position. Starting end-of-cycle position motor homing sequence."); 
        
        //! ************************************************************************
        //! STEP 1: RETRACT POSITION CLAMP AND START POSITION MOTOR HOMING SEQUENCE
        //! ************************************************************************
        retractPositionClamp();
        Serial.println("Position clamp retracted. Starting position motor homing sequence...");
        
        // Transition to new step 8 for position motor homing sequence
        cuttingStep = 8;
        cuttingSubStep8 = 0; // Initialize homing substep
        Serial.println("Transitioning to position motor homing sequence (Step 8)."); 
    }
}

void CuttingState::handleCuttingStep8_PositionMotorHomingSequence(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern const int POSITION_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking position motor homing sequence
    switch (cuttingSubStep8) {
        case 0: // Start homing - move toward home switch
            Serial.println("Position Motor Homing Step 8.0: Moving toward home switch.");
            if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
                positionMotor->moveTo(10000 * POSITION_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            cuttingSubStep8 = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getPositionHomingSwitch()->update();
            if (stateManager.getPositionHomingSwitch()->read() == HIGH) {
                Serial.println("Position Motor Homing Step 8.1: Home switch triggered. Stopping motor.");
                if (positionMotor) {
                    positionMotor->stopMove();
                    positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("Position motor hit home switch.");
                cuttingSubStep8 = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.2 inch from switch
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Position Motor Homing Step 8.2: Moving to -0.2 inch from home switch to establish working zero.");
                positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH - 0.1 * POSITION_MOTOR_STEPS_PER_INCH);
                cuttingSubStep8 = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Position Motor Homing Step 8.3: Setting new working zero position.");
                positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("Position motor homed: 0.2 inch from switch set as position 0.");
                
                configurePositionMotorForNormalOperation();
                cuttingSubStep8 = 4;
            }
            break;
            
        case 4: // Homing complete - check for continuous mode or finish cycle
            Serial.println("Position Motor Homing Step 8.4: Homing sequence complete.");
            extendWoodSecureClamp(); 
            Serial.println("Wood secure clamp engaged."); 
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Check if start cycle switch is active for continuous operation
            if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                Serial.println("Start cycle switch is active - continuing with another cut cycle.");
                // Prepare for next cycle
                extendPositionClamp();
                configureCutMotorForCutting(); // Ensure cut motor is set to proper cutting speed
                turnYellowLedOn();
                stateManager.setCuttingCycleInProgress(true);
                stateManager.changeState(CUTTING);
                resetSteps();
                Serial.println("Transitioning to CUTTING state for continuous operation.");
            } else {
                Serial.println("Cycle complete. Transitioning to IDLE state.");
                stateManager.changeState(IDLE);
                resetSteps();
            }
            break;
    }
}

void CuttingState::handleHomePositionError(StateManager& stateManager) {
    Serial.println("Home position error detected during cutting operation."); 
    unsigned long lastErrorBlinkTime = stateManager.getLastErrorBlinkTime();
    bool errorBlinkState = stateManager.getErrorBlinkState();
    
    if (millis() - lastErrorBlinkTime > 100) { 
        errorBlinkState = !errorBlinkState;
        stateManager.setErrorBlinkState(errorBlinkState);
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        stateManager.setLastErrorBlinkTime(millis());
    }
    
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
    if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
    
    extendPositionClamp();
    extendWoodSecureClamp();
    
    if (stateManager.getReloadSwitch()->rose()) {
        homePositionErrorDetected = false;
        stateManager.changeState(ERROR_RESET);
        stateManager.setErrorAcknowledged(true);
        Serial.println("Home position error acknowledged by reload switch."); 
    }
}

void CuttingState::resetSteps() {
    cuttingStep = 0;
    signalStartTime = 0;
    signalActive = false;
    homePositionErrorDetected = false;
    catcherClampActivatedThisCycle = false;
    catcherServoActivatedThisCycle = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    cuttingSubStep8 = 0; // Reset position motor homing substep
} 