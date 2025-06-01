#include "StateMachine/04_Yes_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************** YES 2X4 STATE *******************************
//* ************************************************************************
// Handles the Yes_2x4 cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

void Yes2x4State::execute(StateManager& stateManager) {
    handleYes2x4Sequence(stateManager);
}

void Yes2x4State::onEnter(StateManager& stateManager) {
    Serial.println("Entering Yes_2x4 state");
    
    // Initialize Yes_2x4 sequence from CUTTING_state logic
    Serial.println("Yes_2x4 state - Wood sensor reads LOW. Starting Yes_2x4 Sequence (simultaneous return).");
    configurePositionMotorForReturn();
    
    retractFeedClamp(); 
    retract2x4SecureClamp(); 
    Serial.println("Position and 2x4 Secure clamps retracted for simultaneous return.");

    // Set flag to indicate we're in Yes_2x4 return mode - enable homing sensor check
    extern bool cutMotorInYes2x4Return; // From main.cpp
    cutMotorInYes2x4Return = true;
    moveCutMotorToHome();
    movePositionMotorToYes2x4Home();
    
    // Initialize step tracking
    yes2x4SubStep = 0;
}

void Yes2x4State::onExit(StateManager& stateManager) {
    Serial.println("Exiting Yes_2x4 state");
    resetSteps();
}

void Yes2x4State::handleYes2x4Sequence(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern bool cutMotorInYes2x4Return; // From main.cpp
    
    // This step handles the completion of the "Yes_2x4 Sequence".
    switch (yes2x4SubStep) {
        case 0: // Wait for position motor to home
            //! ************************************************************************
            //! STEP 0: WAIT FOR POSITION MOTOR TO HOME AND EXTEND FEED CLAMP
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Yes_2x4 Step 0: Position motor has returned home. Extending feed clamp.");
                extendFeedClamp();
                Serial.println("Feed clamp extended (position motor home).");
                yes2x4SubStep = 1;
            }
            break;

        case 1: // Wait for cut motor to home, then proceed with original logic
            //! ************************************************************************
            //! STEP 1: WAIT FOR CUT MOTOR TO HOME AND VERIFY POSITION
            //! ************************************************************************
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("Yes_2x4 Step 1: Cut motor has returned home.");
                // Clear the Yes_2x4 return flag since cut motor has stopped
                cutMotorInYes2x4Return = false;

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
                    extend2x4SecureClamp(); 
                    turnRedLedOn();
                    turnYellowLedOff(); 
                    stateManager.changeState(ERROR);
                    stateManager.setErrorStartTime(millis());
                    resetSteps();
                } else {
                    // Homing successful, proceed with next steps.
                    retract2x4SecureClamp();
                    Serial.println("2x4 secure clamp retracted after successful cut motor home detection.");

                    Serial.println("Cut motor position switch confirmed home. Proceeding to move position motor to final travel position.");
                    configurePositionMotorForNormalOperation();
                    movePositionMotorToPosition(FEED_TRAVEL_DISTANCE);
                    yes2x4SubStep = 2; // Move to position motor homing sequence
                }
            }
            break;
            
        case 2: // Wait for position motor to reach final position, then start homing sequence
            //! ************************************************************************
            //! STEP 2: WAIT FOR POSITION MOTOR TO REACH FINAL POSITION
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Yes_2x4 Step 2: Position motor at final position. Starting end-of-cycle position motor homing sequence."); 
                
                //! ************************************************************************
                //! STEP: RETRACT FEED CLAMP AND START POSITION MOTOR HOMING SEQUENCE
                //! ************************************************************************
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Starting position motor homing sequence...");
                
                // Transition to position motor homing sequence
                yes2x4SubStep = 3;
                positionHomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to position motor homing sequence."); 
            }
            break;
            
        case 3: // Position Motor Homing Sequence
            //! ************************************************************************
            //! STEP 3: POSITION MOTOR HOMING SEQUENCE
            //! ************************************************************************
            handleYes2x4PositionMotorHoming(stateManager);
            break;
    }
}

void Yes2x4State::handleYes2x4PositionMotorHoming(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern const int FEED_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking position motor homing sequence
    switch (positionHomingSubStep) {
        case 0: // Start homing - move toward home switch
            //! ************************************************************************
            //! STEP 3.0: START HOMING - MOVE TOWARD HOME SWITCH
            //! ************************************************************************
            Serial.println("Yes_2x4 Position Motor Homing Step 0: Moving toward home switch.");
            if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                positionMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            positionHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            //! ************************************************************************
            //! STEP 3.1: WAIT FOR HOME SWITCH TO TRIGGER
            //! ************************************************************************
            stateManager.getPositionHomingSwitch()->update();
            if (stateManager.getPositionHomingSwitch()->read() == HIGH) {
                Serial.println("Yes_2x4 Position Motor Homing Step 1: Home switch triggered. Stopping motor.");
                if (positionMotor) {
                    positionMotor->stopMove();
                    positionMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("Position motor hit home switch.");
                positionHomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.2 inch from switch
            //! ************************************************************************
            //! STEP 3.2: MOVE TO -0.2 INCH FROM SWITCH TO ESTABLISH WORKING ZERO
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Yes_2x4 Position Motor Homing Step 2: Moving to -0.2 inch from home switch to establish working zero.");
                positionMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                positionHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            //! ************************************************************************
            //! STEP 3.3: SET NEW WORKING ZERO POSITION
            //! ************************************************************************
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("Yes_2x4 Position Motor Homing Step 3: Setting new working zero position.");
                positionMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("Position motor homed: 0.2 inch from switch set as position 0.");
                
                configurePositionMotorForNormalOperation();
                positionHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - check for continuous mode or finish cycle
            //! ************************************************************************
            //! STEP 3.4: HOMING COMPLETE - CHECK FOR CONTINUOUS MODE OR FINISH CYCLE
            //! ************************************************************************
            Serial.println("Yes_2x4 Position Motor Homing Step 4: Homing sequence complete.");
            extend2x4SecureClamp(); 
            Serial.println("2x4 secure clamp extended."); 
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Check if start cycle switch is active for continuous operation
            if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                Serial.println("Start cycle switch is active - continuing with another cut cycle.");
                // Prepare for next cycle
                extendFeedClamp();
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

void Yes2x4State::resetSteps() {
    yes2x4SubStep = 0;
    positionHomingSubStep = 0;
} 