#include "StateMachine/05_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************** NO 2X4 STATE ********************************
//* ************************************************************************
// Handles the No_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

void No2x4State::execute(StateManager& stateManager) {
    handleNo2x4Sequence(stateManager);
}

void No2x4State::onEnter(StateManager& stateManager) {
    Serial.println("Entering No_2x4 state");
    
    // Initialize No_2x4 sequence from CUTTING_state logic
    Serial.println("No_2x4 state - Wood sensor reads HIGH. Starting No_2x4 Sequence.");
    configureCutMotorForReturn();
    moveCutMotorToHome();
    configurePositionMotorForNormalOperation();
    
    // Initialize step tracking
    no2x4Step = 0;
    no2x4HomingSubStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
}

void No2x4State::onExit(StateManager& stateManager) {
    Serial.println("Exiting No_2x4 state");
    resetSteps();
}

void No2x4State::handleNo2x4Sequence(StateManager& stateManager) {
    // No_2x4 sequence logic
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    const unsigned long CYLINDER_ACTION_DELAY_MS = 150;
    
    if (no2x4Step == 0) { // First time entering this specific No_2x4 logic path
        //! ************************************************************************
        //! STEP 0: INITIATE POSITION MOTOR TO HOME & RETRACT 2x4 SECURE CLAMP
        //! ************************************************************************
        Serial.println("No_2x4 Step 0: Initiating position motor to home & retracting 2x4 secure clamp.");
        retract2x4SecureClamp();
        if (positionMotor) {
            if (positionMotor->getCurrentPosition() != 0 || positionMotor->isRunning()) {
                positionMotor->moveTo(0);
                Serial.println("No_2x4 Step 0: Position motor commanded to home.");
            } else {
                Serial.println("No_2x4 Step 0: Position motor already at home.");
            }
        }
        no2x4Step = 1;
    }

    if (waitingForCylinder && (millis() - cylinderActionTime >= CYLINDER_ACTION_DELAY_MS)) {
        waitingForCylinder = false;
        no2x4Step++; 
    }
    
    if (!waitingForCylinder) {
        handleNo2x4Step(stateManager, no2x4Step);
    }
}

void No2x4State::handleNo2x4Step(StateManager& stateManager, int step) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    
    switch (step) { 
        case 1: // New Step: Wait for cut motor, then extend feed clamp
            //! ************************************************************************
            //! STEP 1: WAIT FOR CUT MOTOR AND EXTEND FEED CLAMP
            //! ************************************************************************
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("No_2x4 Step 1: Cut motor returned home. Extending feed clamp.");
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Will cause no2x4Step to increment to 2 after delay
            }
            break;
            
        case 2: // Was original no2x4Step 1: wait for position motor, then retract feed clamp
            //! ************************************************************************
            //! STEP 2: WAIT FOR POSITION MOTOR AND RETRACT FEED CLAMP
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("No_2x4 Step 2: Position motor at home. Disengaging feed clamp.");
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 3
            }
            break;
            
        case 3: // Was original no2x4Step 2: move position motor to 2.0 inches
            //! ************************************************************************
            //! STEP 3: MOVE POSITION MOTOR TO 2.0 INCHES
            //! ************************************************************************
            Serial.println("No_2x4 Step 3: Moving position motor to 2.0 inches."); 
            configurePositionMotorForNormalOperation(); // Ensure correct config
            movePositionMotorToPosition(2.0);
            no2x4Step = 4; // Directly advance step here as it's a command
            break;
            
        case 4: // Was original no2x4Step 3: wait for position motor at 2.0, extend feed clamp
            //! ************************************************************************
            //! STEP 4: WAIT FOR POSITION MOTOR AT 2.0 AND EXTEND FEED CLAMP
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("No_2x4 Step 4: Position motor at 2.0 inches. Extending feed clamp.");
                extendFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 5
            }
            break;
            
        case 5: // Was original no2x4Step 4: move position motor to home
            //! ************************************************************************
            //! STEP 5: MOVE POSITION MOTOR TO HOME
            //! ************************************************************************
            Serial.println("No_2x4 Step 5: Moving position motor to home."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToHome();
            no2x4Step = 6; // Directly advance step
            break;
            
        case 6: // Was original no2x4Step 5: wait for position motor at home, retract feed clamp
            //! ************************************************************************
            //! STEP 6: WAIT FOR POSITION MOTOR AT HOME AND RETRACT FEED CLAMP
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("No_2x4 Step 6: Position motor at home. Disengaging feed clamp.");
                retractFeedClamp();
                cylinderActionTime = millis();
                waitingForCylinder = true; // Increments to 7
            }
            break;
            
        case 7: // Was original no2x4Step 6: move position motor to final position
            //! ************************************************************************
            //! STEP 7: MOVE POSITION MOTOR TO FINAL POSITION
            //! ************************************************************************
            Serial.println("No_2x4 Step 7: Moving position motor to final position (FEED_TRAVEL_DISTANCE)."); 
            configurePositionMotorForNormalOperation();
            movePositionMotorToPosition(FEED_TRAVEL_DISTANCE);
            no2x4Step = 8; // Directly advance step
            break;
            
        case 8: // Was original no2x4Step 7: wait for motor, check cut home, start position motor homing
            //! ************************************************************************
            //! STEP 8: WAIT FOR MOTOR, CHECK CUT HOME, START POSITION MOTOR HOMING
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("No_2x4 Step 8: Position motor at final position."); 
                bool sensorDetectedHome = false;
                Serial.println("No_2x4 Step 8: Checking cut motor position switch."); 
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
                        Serial.println("Cut motor position switch detected HIGH during No_2x4 sequence completion."); 
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

                Serial.println("No_2x4 Step 8: Starting position motor homing sequence...");
                
                //! ************************************************************************
                //! START POSITION MOTOR HOMING SEQUENCE FOR No_2x4
                //! ************************************************************************
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Starting No_2x4 position motor homing sequence...");
                
                // Transition to new No_2x4 homing substeps
                no2x4Step = 9;
                no2x4HomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to No_2x4 position motor homing sequence (Step 9)."); 
            }
            break;
            
        case 9: // No_2x4 Position Motor Homing Sequence
            //! ************************************************************************
            //! STEP 9: No_2x4 POSITION MOTOR HOMING SEQUENCE
            //! ************************************************************************
            handleNo2x4PositionMotorHoming(stateManager);
            break;
    }
}

void No2x4State::handleNo2x4PositionMotorHoming(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern const int FEED_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking position motor homing sequence for No_2x4
    switch (no2x4HomingSubStep) {
        case 0: // Start homing - move toward home switch
            //! ************************************************************************
            //! STEP 9.0: START HOMING - MOVE TOWARD HOME SWITCH
            //! ************************************************************************
            Serial.println("No_2x4 Position Motor Homing Step 9.0: Moving toward home switch.");
            if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                positionMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            no2x4HomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            //! ************************************************************************
            //! STEP 9.1: WAIT FOR HOME SWITCH TO TRIGGER
            //! ************************************************************************
            stateManager.getPositionHomingSwitch()->update();
            if (stateManager.getPositionHomingSwitch()->read() == HIGH) {
                Serial.println("No_2x4 Position Motor Homing Step 9.1: Home switch triggered. Stopping motor.");
                if (positionMotor) {
                    positionMotor->stopMove();
                    positionMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("No_2x4: Position motor hit home switch.");
                no2x4HomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.1 inch from switch
            //! ************************************************************************
            //! STEP 9.2: MOVE TO -0.1 INCH FROM SWITCH TO ESTABLISH WORKING ZERO
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("No_2x4 Position Motor Homing Step 9.2: Moving to -0.1 inch from home switch to establish working zero.");
                positionMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                no2x4HomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            //! ************************************************************************
            //! STEP 9.3: SET NEW WORKING ZERO POSITION
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("No_2x4 Position Motor Homing Step 9.3: Setting new working zero position.");
                positionMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("No_2x4: Position motor homed: 0.1 inch from switch set as position 0.");
                
                configurePositionMotorForNormalOperation();
                no2x4HomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - finish No_2x4 sequence
            //! ************************************************************************
            //! STEP 9.4: HOMING COMPLETE - FINISH No_2x4 SEQUENCE
            //! ************************************************************************
            Serial.println("No_2x4 Position Motor Homing Step 9.4: Homing sequence complete.");
            
            retract2x4SecureClamp(); 
            Serial.println("2x4 secure clamp retracted (final check in No_2x4).");
            extend2x4SecureClamp();
            Serial.println("2x4 secure clamp extended.");
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
            
            Serial.println("No_2x4 sequence with position motor homing complete. Transitioning to IDLE state. Continuous mode OFF."); 
            break;
    }
}

void No2x4State::resetSteps() {
    no2x4Step = 0;
    no2x4HomingSubStep = 0;
    cylinderActionTime = 0;
    waitingForCylinder = false;
} 