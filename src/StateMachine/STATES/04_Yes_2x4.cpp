#include "StateMachine/04_Yes_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ RETURNING YES 2X4 STATE **********************
//* ************************************************************************
// Handles the RETURNING_YES_2x4 cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

void ReturningYes2x4State::execute(StateManager& stateManager) {
    handleReturningYes2x4Sequence(stateManager);
}

void ReturningYes2x4State::onEnter(StateManager& stateManager) {
    Serial.println("Entering RETURNING_YES_2x4 state");
    
    // Initialize RETURNING_YES_2x4 sequence from CUTTING_state logic
    Serial.println("RETURNING_YES_2x4 state - Wood sensor reads LOW. Starting RETURNING_YES_2x4 Sequence (simultaneous return).");
    configureFeedMotorForReturn();
    
    retractFeedClamp(); 
    retract2x4SecureClamp(); 
    Serial.println("Feed and 2x4 Secure clamps disengaged for simultaneous return.");

    // Set flag to indicate we're in RETURNING_YES_2x4 return mode - enable homing sensor check
    extern bool cutMotorInReturningYes2x4Return; // From main.cpp
    cutMotorInReturningYes2x4Return = true;
    moveCutMotorToHome();
    moveFeedMotorToHome();
    
    // Initialize step tracking
    returningYes2x4SubStep = 0;
}

void ReturningYes2x4State::onExit(StateManager& stateManager) {
    Serial.println("Exiting RETURNING_YES_2x4 state");
    resetSteps();
}

void ReturningYes2x4State::handleReturningYes2x4Sequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern bool cutMotorInReturningYes2x4Return; // From main.cpp
    
    // This step handles the completion of the "RETURNING_YES_2x4 Sequence".
    switch (returningYes2x4SubStep) {
        case 0: // Wait for feed motor to home
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Step 0: Feed motor has returned home. Engaging feed clamp.");
                extendFeedClamp();
                Serial.println("Feed clamp engaged (feed motor home).");
                returningYes2x4SubStep = 1;
            }
            break;

        case 1: // Wait for cut motor to home, then proceed with original logic
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Step 1: Cut motor has returned home.");
                // Clear the RETURNING_YES_2x4 return flag since cut motor has stopped
                cutMotorInReturningYes2x4Return = false;

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

                    Serial.println("Cut motor position switch confirmed home. Proceeding to move feed motor to final travel position.");
                    configureFeedMotorForNormalOperation();
                    moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                    returningYes2x4SubStep = 2; // Move to feed motor homing sequence
                }
            }
            break;
            
        case 2: // Wait for feed motor to reach final position, then start homing sequence
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Step 2: Feed motor at final position. Starting end-of-cycle feed motor homing sequence.");
                
                //! ************************************************************************
                //! STEP: RETRACT FEED CLAMP AND START FEED MOTOR HOMING SEQUENCE
                //! ************************************************************************
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Starting feed motor homing sequence...");
                
                // Transition to feed motor homing sequence
                returningYes2x4SubStep = 3;
                feedHomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to feed motor homing sequence."); 
            }
            break;
            
        case 3: // Feed Motor Homing Sequence
            handleReturningYes2x4FeedMotorHoming(stateManager);
            break;
    }
}

void ReturningYes2x4State::handleReturningYes2x4FeedMotorHoming(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern const int FEED_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking feed motor homing sequence
    switch (feedHomingSubStep) {
        case 0: // Start homing - move toward home switch
            Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 0: Moving toward home switch.");
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            feedHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getFeedHomingSwitch()->update();
            if (stateManager.getFeedHomingSwitch()->read() == HIGH) {
                Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 1: Home switch triggered. Stopping motor.");
                if (feedMotor) {
                    feedMotor->stopMove();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("Feed motor hit home switch.");
                feedHomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.2 inch from switch
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 2: Moving to -0.2 inch from home switch to establish working zero.");
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                feedHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 3: Setting new working zero position.");
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("Feed motor homed: 0.2 inch from switch set as position 0.");
                
                configureFeedMotorForNormalOperation();
                feedHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - check for continuous mode or finish cycle
            Serial.println("RETURNING_YES_2x4 Feed Motor Homing Step 4: Homing sequence complete.");
            extend2x4SecureClamp(); 
            Serial.println("2x4 secure clamp engaged."); 
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

void ReturningYes2x4State::resetSteps() {
    returningYes2x4SubStep = 0;
    feedHomingSubStep = 0;
} 