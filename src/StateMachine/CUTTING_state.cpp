#include "StateMachine/CuttingState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

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
        case 6:
            handleCuttingStep6_NO_WOOD_Sequence(stateManager);
            break;
        case 7: 
            handleCuttingStep7_YES_WOOD_Sequence(stateManager);
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
    // Modified to use >= as per original file content for cutMotor->getCurrentPosition()
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const int CUT_MOTOR_STEPS_PER_INCH; // From main.cpp
    extern const int WAS_WOOD_SUCTIONED_SENSOR; // From main.cpp
    
    if (cutMotor && cutMotor->getCurrentPosition() >= (1.0 * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.println("Cutting Step 1: Cut motor at >= 1 inch, checking wood suction.");
        if (digitalRead(WAS_WOOD_SUCTIONED_SENSOR) == LOW) { // LOW means NO SUCTION (Error condition)
            Serial.println("Wood suction error detected! Waiting for cycle switch OFF then ON to reset.");
            // Stop motors
            if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            FastAccelStepper* positionMotor = stateManager.getPositionMotor();
            if (positionMotor) positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());

            stateManager.setCuttingCycleInProgress(false);
            resetSteps();

            // Set LEDs for suction error state
            turnRedLedOn();
            turnYellowLedOff();
            turnGreenLedOff();
            turnBlueLedOff();

            stateManager.changeState(SUCTION_ERROR_HOLD); // Transition to new error hold state
        } else { // Sensor is HIGH, suction is OK
            cuttingStep = 2;
            Serial.println("Cutting Step 1: Wood suction OK (or not present). Proceeding to step 2.");
        }
    }
}

void CuttingState::handleCuttingStep2(StateManager& stateManager) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const int CUT_MOTOR_STEPS_PER_INCH; // From main.cpp
    extern const float CUT_TRAVEL_DISTANCE; // From main.cpp
    extern const float CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    extern const float CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    extern const int WOOD_SENSOR; // From main.cpp
    
    // Early Catcher Clamp Activation
    if (!catcherClampActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - CATCHER_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.println("Cutting Step 2: Activating Catcher Clamp (early).");
        extendCatcherClamp(); // Engage Catcher Clamp
        catcherClampActivatedThisCycle = true;
    }

    // Early Catcher Servo Activation
    if (!catcherServoActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - CATCHER_SERVO_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.println("Cutting Step 2: Activating Catcher Servo (early).");
        extern Servo catcherServo; // From main.cpp
        extern unsigned long catcherServoActiveStartTime; // From main.cpp
        extern bool catcherServoIsActiveAndTiming; // From main.cpp
        extern const int CATCHER_SERVO_ACTIVE_POSITION; // From main.cpp
        catcherServo.write(CATCHER_SERVO_ACTIVE_POSITION);
        catcherServoActiveStartTime = millis();
        catcherServoIsActiveAndTiming = true;
        catcherServoActivatedThisCycle = true;
        Serial.print("Catcher servo moved to ");
        Serial.print(CATCHER_SERVO_ACTIVE_POSITION);
        Serial.println(" degrees (early activation).");
    }

    // Check for full cut completion
    if (cutMotor && !cutMotor->isRunning()) {
        Serial.println("Cutting Step 2: Cut fully complete."); 
        sendSignalToTA(); // Signal to Transfer Arm
        configureCutMotorForReturn();

        int sensorValue = digitalRead(WOOD_SENSOR);
        bool noWoodDetected = (sensorValue == HIGH);
        
        if (noWoodDetected) {
            Serial.println("Cutting Step 2: NO_WOOD state - Wood sensor reads HIGH. Entering NO_WOOD Sequence (handled in cuttingStep 6).");
            configureCutMotorForReturn();
            moveCutMotorToHome();
            configurePositionMotorForNormalOperation();
            cuttingStep = 6;
        } else {
            Serial.println("Cutting Step 2: YES_WOOD state - Wood sensor reads LOW. Entering YES_WOOD Sequence (simultaneous return, handled in cuttingStep 7)."); 
            configurePositionMotorForReturn();
            
            retractPositionClamp(); 
            retractWoodSecureClamp(); 
            Serial.println("Position and Wood Secure clamps disengaged for simultaneous return.");

            // Set flag to indicate we're in YES_WOOD return mode - enable homing sensor check
            extern bool cutMotorInYesWoodReturn; // From main.cpp
            cutMotorInYesWoodReturn = true;
            moveCutMotorToHome();
            movePositionMotorToYesWoodHome();
            
            cuttingStep = 7;
            cuttingSubStep7 = 0; // Reset sub-step for case 7
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

void CuttingState::handleCuttingStep6_NO_WOOD_Sequence(StateManager& stateManager) {
    // NO_WOOD sequence logic
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (noWoodStep == 0) { // First time entering this specific NO_WOOD logic path
        Serial.println("NO_WOOD Step 6.0: Initiating position motor to home & retracting wood secure clamp.");
        retractWoodSecureClamp();
        if (positionMotor) {
            if (positionMotor->getCurrentPosition() != 0 || positionMotor->isRunning()) {
                positionMotor->moveTo(0);
                Serial.println("NO_WOOD Step 6.0: Position motor commanded to home.");
            } else {
                Serial.println("NO_WOOD Step 6.0: Position motor already at home.");
            }
        }
        noWoodStep = 1;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        noWoodStep++; 
    }
    
    if (!waitingForCylinder) {
        handleNO_WOOD_Step(stateManager, noWoodStep);
    }
}

void CuttingState::handleNO_WOOD_Step(StateManager& stateManager, int step) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then engage position clamp
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.1 (new): Cut motor returned home. Engaging position clamp.");
                extendPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause noWoodStep to increment to 2 after delay
            }
            break;
            
        case 2: // Was original noWoodStep 1: wait for position motor, then retract position clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.2 (was 6.1): Position motor at home. Disengaging position clamp."); 
                retractPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original noWoodStep 2: move position motor to 2.0 inches
            Serial.println("NO_WOOD Step 6.3 (was 6.2): Moving position motor to 2.0 inches."); 
            configurePositionMotorForNormalOperation(); // Ensure correct config
            movePositionMotorToPosition(2.0);
            noWoodStep = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original noWoodStep 3: wait for position motor at 2.0, engage position clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.4 (was 6.3): Position motor at 2.0 inches. Engaging position clamp."); 
                extendPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 5
            }
            break;
            
        case 5: // Was original noWoodStep 4: move position motor to home
            Serial.println("NO_WOOD Step 6.5 (was 6.4): Moving position motor to home."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToHome();
            noWoodStep = 6; // Directly advance step
            break;
            
        case 6: // Was original noWoodStep 5: wait for position motor at home, retract position clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.6 (was 6.5): Position motor at home. Disengaging position clamp."); 
                retractPositionClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 7
            }
            break;
            
        case 7: // Was original noWoodStep 6: move position motor to final position
            Serial.println("NO_WOOD Step 6.7 (was 6.6): Moving position motor to final position (POSITION_TRAVEL_DISTANCE)."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
            noWoodStep = 8; // Directly advance step
            break;
            
        case 8: // Was original noWoodStep 7: wait for motor, check cut home, start position motor homing
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6.8 (was 6.7): Position motor at final position."); 
                bool sensorDetectedHome = false;
                Serial.println("NO_WOOD Step 6.8: Checking cut motor position switch."); 
                for (int i = 0; i < 3; i++) {
                    delay(30);  
                    stateManager.getCutHomingSwitch()->update();
                    bool sensorReading = stateManager.getCutHomingSwitch()->read();
                    Serial.print("Cut position switch read attempt "); 
                    Serial.print(i+1); 
                    Serial.print(" of 3: "); 
                    Serial.print(sensorReading ? "HIGH" : "LOW");
                    Serial.print(" (raw: ");
                    Serial.print(sensorReading);
                    Serial.println(")");
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        Serial.println("Cut motor position switch detected HIGH during NO_WOOD sequence completion."); 
                        break;  
                    }
                }
                
                // TEMPORARY FIX: Always proceed regardless of sensor to identify the issue
                if (!sensorDetectedHome) {
                    Serial.println("DIAGNOSTIC: Cut motor home sensor did NOT detect HIGH.");
                    Serial.println("DIAGNOSTIC: Proceeding anyway to test if sensor logic is inverted.");
                    Serial.println("DIAGNOSTIC: If cut motor is physically at home, sensor logic may need inversion.");
                } else {
                    Serial.println("DIAGNOSTIC: Cut motor home sensor successfully detected HIGH.");
                }

                Serial.println("NO_WOOD Step 6.8: Starting position motor homing sequence...");
                
                //! ************************************************************************
                //! START POSITION MOTOR HOMING SEQUENCE FOR NO_WOOD
                //! ************************************************************************
                retractPositionClamp();
                Serial.println("Position clamp retracted. Starting NO_WOOD position motor homing sequence...");
                
                // Transition to new NO_WOOD homing substeps
                noWoodStep = 9;
                noWoodHomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to NO_WOOD position motor homing sequence (Step 6.9)."); 
            }
            break;
            
        case 9: // NO_WOOD Position Motor Homing Sequence
            handleNO_WOOD_PositionMotorHoming(stateManager);
            break;
    }
}

void CuttingState::handleCuttingStep7_YES_WOOD_Sequence(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern bool cutMotorInYesWoodReturn; // From main.cpp
    
    // This step handles the completion of the "YES_WOOD Sequence".
    switch (cuttingSubStep7) {
        case 0: // Wait for position motor to home
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Cutting Step 7.0: Position motor has returned home. Engaging position clamp.");
                extendPositionClamp();
                Serial.println("Position clamp engaged (position motor home).");
                cuttingSubStep7 = 1;
            }
            break;

        case 1: // Wait for cut motor to home, then proceed with original logic
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("Cutting Step 7.1: Cut motor has returned home.");
                // Clear the YES_WOOD return flag since cut motor has stopped
                cutMotorInYesWoodReturn = false;

                // Check the cut motor homing switch. If not detected, transition to ERROR.
                bool sensorDetectedHome = false;
                Serial.println("Checking cut motor position switch after simultaneous return.");
                for (int i = 0; i < 3; i++) { 
                    delay(30);  
                    stateManager.getCutHomingSwitch()->update();
                    Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); Serial.println(stateManager.getCutHomingSwitch()->read());
                    if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                        sensorDetectedHome = true;
                        if (cutMotor) cutMotor->setCurrentPosition(0); 
                        Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                        break; 
                    }
                }

                if (!sensorDetectedHome) {
                    // Homing failed, transition to ERROR state.
                    Serial.println("ERROR: Cut motor position switch did not detect home after simultaneous return!");
                    stopCutMotor();
                    extendWoodSecureClamp(); 
                    turnRedLedOn();
                    turnYellowLedOff(); 
                    stateManager.changeState(ERROR);
                    stateManager.setErrorStartTime(millis());
                    resetSteps();
                } else {
                    // Homing successful, proceed with next steps.
                    retractWoodSecureClamp();
                    Serial.println("Wood secure clamp retracted after successful cut motor home detection.");

                    Serial.println("Cut motor position switch confirmed home. Proceeding to move position motor to final travel position.");
                    configurePositionMotorForNormalOperation();
                    movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                    cuttingStep = 5; // Transition to original cuttingStep 5
                    cuttingSubStep7 = 0; // Reset sub-step
                }
            }
            break;
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
                Serial.println("Cycle complete. Transitioning to READY state.");
                stateManager.changeState(READY);
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

void CuttingState::handleNO_WOOD_PositionMotorHoming(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern const int POSITION_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking position motor homing sequence for NO_WOOD
    switch (noWoodHomingSubStep) {
        case 0: // Start homing - move toward home switch
            Serial.println("NO_WOOD Position Motor Homing Step 6.9.0: Moving toward home switch.");
            if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
                positionMotor->moveTo(10000 * POSITION_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            noWoodHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getPositionHomingSwitch()->update();
            if (stateManager.getPositionHomingSwitch()->read() == HIGH) {
                Serial.println("NO_WOOD Position Motor Homing Step 6.9.1: Home switch triggered. Stopping motor.");
                if (positionMotor) {
                    positionMotor->stopMove();
                    positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("NO_WOOD: Position motor hit home switch.");
                noWoodHomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.1 inch from switch
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Position Motor Homing Step 6.9.2: Moving to -0.1 inch from home switch to establish working zero.");
                positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH - 0.1 * POSITION_MOTOR_STEPS_PER_INCH);
                noWoodHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Position Motor Homing Step 6.9.3: Setting new working zero position.");
                positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("NO_WOOD: Position motor homed: 0.1 inch from switch set as position 0.");
                
                configurePositionMotorForNormalOperation();
                noWoodHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - finish NO_WOOD sequence
            Serial.println("NO_WOOD Position Motor Homing Step 6.9.4: Homing sequence complete.");
            
            retractWoodSecureClamp(); 
            Serial.println("Wood secure clamp disengaged (final check in NO_WOOD)."); 
            extendWoodSecureClamp(); 
            Serial.println("Wood secure clamp engaged."); 
            turnYellowLedOff();
            turnBlueLedOn(); 

            resetSteps();
            stateManager.setCuttingCycleInProgress(false);
            stateManager.changeState(READY);
            stateManager.setContinuousModeActive(false);
            
            // Check if cycle switch is currently ON - if yes, require cycling
            if (stateManager.getStartCycleSwitch()->read() == HIGH) {
                stateManager.setStartSwitchSafe(false);
                Serial.println("Cycle switch is still ON - must be cycled OFF then ON for next cycle.");
            } else {
                Serial.println("Cycle switch is OFF - ready for next cycle.");
            }
            
            Serial.println("NO_WOOD sequence with position motor homing complete. Transitioning to READY state. Continuous mode OFF."); 
            break;
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
    cuttingSubStep7 = 0;
    cuttingSubStep8 = 0; // Reset position motor homing substep
    
    noWoodStep = 0;
    noWoodHomingSubStep = 0; // Reset NO_WOOD position motor homing substep
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 