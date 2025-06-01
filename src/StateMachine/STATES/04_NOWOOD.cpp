#include "StateMachine/04_NOWOOD.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************** NO WOOD STATE *******************************
//* ************************************************************************
// Handles the NO_WOOD cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

void NoWoodState::execute(StateManager& stateManager) {
    handleNO_WOOD_Sequence(stateManager);
}

void NoWoodState::onEnter(StateManager& stateManager) {
    Serial.println("Entering NO_WOOD state");
    
    // Initialize NO_WOOD sequence from CUTTING_state logic
    Serial.println("NO_WOOD state - Wood sensor reads HIGH. Starting NO_WOOD Sequence.");
    configureCutMotorForReturn();
    moveCutMotorToHome();
    configurePositionMotorForNormalOperation();
    
    // Initialize step tracking
    noWoodStep = 0;
    noWoodHomingSubStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
}

void NoWoodState::onExit(StateManager& stateManager) {
    Serial.println("Exiting NO_WOOD state");
    resetSteps();
}

void NoWoodState::handleNO_WOOD_Sequence(StateManager& stateManager) {
    // NO_WOOD sequence logic
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (noWoodStep == 0) { // First time entering this specific NO_WOOD logic path
        Serial.println("NO_WOOD Step 0: Initiating position motor to home & retracting wood secure clamp.");
        retractWoodSecureClamp();
        if (positionMotor) {
            if (positionMotor->getCurrentPosition() != 0 || positionMotor->isRunning()) {
                positionMotor->moveTo(0);
                Serial.println("NO_WOOD Step 0: Position motor commanded to home.");
            } else {
                Serial.println("NO_WOOD Step 0: Position motor already at home.");
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

void NoWoodState::handleNO_WOOD_Step(StateManager& stateManager, int step) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then engage feed clamp
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("NO_WOOD Step 1: Cut motor returned home. Engaging feed clamp.");
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause noWoodStep to increment to 2 after delay
            }
            break;
            
        case 2: // Was original noWoodStep 1: wait for position motor, then retract feed clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 2: Position motor at home. Disengaging feed clamp.");
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original noWoodStep 2: move position motor to 2.0 inches
            Serial.println("NO_WOOD Step 3: Moving position motor to 2.0 inches."); 
            configurePositionMotorForNormalOperation(); // Ensure correct config
            movePositionMotorToPosition(2.0);
            noWoodStep = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original noWoodStep 3: wait for position motor at 2.0, engage feed clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 4: Position motor at 2.0 inches. Engaging feed clamp.");
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 5
            }
            break;
            
        case 5: // Was original noWoodStep 4: move position motor to home
            Serial.println("NO_WOOD Step 5: Moving position motor to home."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToHome();
            noWoodStep = 6; // Directly advance step
            break;
            
        case 6: // Was original noWoodStep 5: wait for position motor at home, retract feed clamp
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 6: Position motor at home. Disengaging feed clamp.");
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 7
            }
            break;
            
        case 7: // Was original noWoodStep 6: move position motor to final position
            Serial.println("NO_WOOD Step 7: Moving position motor to final position (POSITION_TRAVEL_DISTANCE)."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
            noWoodStep = 8; // Directly advance step
            break;
            
        case 8: // Was original noWoodStep 7: wait for motor, check cut home, start position motor homing
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Step 8: Position motor at final position."); 
                bool sensorDetectedHome = false;
                Serial.println("NO_WOOD Step 8: Checking cut motor position switch."); 
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

                Serial.println("NO_WOOD Step 8: Starting position motor homing sequence...");
                
                //! ************************************************************************
                //! START POSITION MOTOR HOMING SEQUENCE FOR NO_WOOD
                //! ************************************************************************
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Starting NO_WOOD position motor homing sequence...");
                
                // Transition to new NO_WOOD homing substeps
                noWoodStep = 9;
                noWoodHomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to NO_WOOD position motor homing sequence (Step 9)."); 
            }
            break;
            
        case 9: // NO_WOOD Position Motor Homing Sequence
            handleNO_WOOD_PositionMotorHoming(stateManager);
            break;
    }
}

void NoWoodState::handleNO_WOOD_PositionMotorHoming(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern const int POSITION_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking position motor homing sequence for NO_WOOD
    switch (noWoodHomingSubStep) {
        case 0: // Start homing - move toward home switch
            Serial.println("NO_WOOD Position Motor Homing Step 9.0: Moving toward home switch.");
            if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
                positionMotor->moveTo(10000 * POSITION_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            noWoodHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getPositionHomingSwitch()->update();
            if (stateManager.getPositionHomingSwitch()->read() == HIGH) {
                Serial.println("NO_WOOD Position Motor Homing Step 9.1: Home switch triggered. Stopping motor.");
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
                Serial.println("NO_WOOD Position Motor Homing Step 9.2: Moving to -0.1 inch from home switch to establish working zero.");
                positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH - 0.1 * POSITION_MOTOR_STEPS_PER_INCH);
                noWoodHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("NO_WOOD Position Motor Homing Step 9.3: Setting new working zero position.");
                positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("NO_WOOD: Position motor homed: 0.1 inch from switch set as position 0.");
                
                configurePositionMotorForNormalOperation();
                noWoodHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - finish NO_WOOD sequence
            Serial.println("NO_WOOD Position Motor Homing Step 9.4: Homing sequence complete.");
            
            retractWoodSecureClamp(); 
            Serial.println("Wood secure clamp disengaged (final check in NO_WOOD)."); 
            extendWoodSecureClamp(); 
            Serial.println("Wood secure clamp engaged."); 
            turnYellowLedOff();
            turnBlueLedOn(); 

            resetSteps();
            stateManager.setCuttingCycleInProgress(false);
            stateManager.changeState(IDLE);
            
            // Check if cycle switch is currently ON - if yes, require cycling
            if (stateManager.getStartCycleSwitch()->read() == HIGH) {
                stateManager.setStartSwitchSafe(false);
                Serial.println("Cycle switch is still ON - must be cycled OFF then ON for next cycle.");
            } else {
                Serial.println("Cycle switch is OFF - ready for next cycle.");
            }
            
            Serial.println("NO_WOOD sequence with position motor homing complete. Transitioning to IDLE state. Continuous mode OFF."); 
            break;
    }
}

void NoWoodState::resetSteps() {
    noWoodStep = 0;
    noWoodHomingSubStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 