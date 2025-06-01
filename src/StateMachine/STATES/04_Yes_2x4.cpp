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
    
    // Configure motors for return operation
    configureCutMotorForReturn();
    configureFeedMotorForReturn();
    
    // Start cut motor return to home
    moveCutMotorToHome();
    Serial.println("Cut motor moving to home position");
    
    // Start feed motor return to post-cut home
    moveFeedMotorToPostCutHome();
    
    // Reset step counters
    returningYes2x4SubStep = 0;
}

void ReturningYes2x4State::onExit(StateManager& stateManager) {
    // Reset all step counters when exiting state
    resetSteps();
}

void ReturningYes2x4State::handleReturningYes2x4Sequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    
    // Check if both motors have finished their return movements
    bool cutMotorFinished = (cutMotor && !cutMotor->isRunning());
    bool feedMotorFinished = (feedMotor && !feedMotor->isRunning());
    
    switch (returningYes2x4SubStep) {
        case 0: // Wait for both motors to finish returning
            if (cutMotorFinished && feedMotorFinished) {
                Serial.println("Both motors finished returning. Starting feed motor homing sequence.");
                
                //! ************************************************************************
                //! STEP 1: RETRACT FEED CLAMP AND START FEED MOTOR HOMING SEQUENCE
                //! ************************************************************************
                retract2x4SecureClamp();
                Serial.println("Feed clamp retracted. Starting feed motor homing sequence...");
                
                // Transition to feed motor homing sequence
                returningYes2x4SubStep = 1;
                feedHomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to feed motor homing sequence.");
            }
            break;
            
        case 1: // Handle feed motor homing sequence
            handleReturningYes2x4FeedMotorHoming(stateManager);
            break;
            
        case 2: // Homing complete - check for continuous mode or finish cycle
            Serial.println("RETURNING_YES_2x4 sequence complete.");
            extend2x4SecureClamp();
            Serial.println("2x4 secure clamp extended."); 
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Check if start cycle switch is active for continuous operation
            if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                Serial.println("Start cycle switch is active - continuing with another cut cycle.");
                // Prepare for next cycle
                extend2x4SecureClamp();
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

void ReturningYes2x4State::handleReturningYes2x4FeedMotorHoming(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float FEED_TRAVEL_DISTANCE; // From main.cpp
    extern const int FEED_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking feed motor homing sequence
    switch (feedHomingSubStep) {
        case 0: // Start homing - move toward home switch
            Serial.println("Feed Motor Homing Step 0: Moving toward home switch.");
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            feedHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getFeedHomingSwitch()->update();
            if (stateManager.getFeedHomingSwitch()->read() == HIGH) {
                Serial.println("Feed Motor Homing Step 1: Home switch triggered. Stopping motor.");
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
                Serial.println("Feed Motor Homing Step 2: Moving to -0.2 inch from home switch to establish working zero.");
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                feedHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (feedMotor && !feedMotor->isRunning()) {
                Serial.println("Feed Motor Homing Step 3: Setting new working zero position.");
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("Feed motor homed: 0.2 inch from switch set as position 0.");
                
                configureFeedMotorForNormalOperation();
                feedHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete
            Serial.println("Feed Motor Homing Step 4: Homing sequence complete.");
            returningYes2x4SubStep = 2; // Move to completion step
            break;
    }
}

void ReturningYes2x4State::resetSteps() {
    returningYes2x4SubStep = 0;
    feedHomingSubStep = 0;
} 