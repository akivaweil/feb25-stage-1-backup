#include "StateMachine/05_YESWOOD.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"
#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************** YES WOOD STATE ******************************
//* ************************************************************************
// Handles the YES_WOOD cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

void YesWoodState::execute(StateManager& stateManager) {
    handleYES_WOOD_Sequence(stateManager);
}

void YesWoodState::onEnter(StateManager& stateManager) {
    Serial.println("Entering YES_WOOD state");
    
    // Initialize YES_WOOD sequence from CUTTING_state logic
    Serial.println("YES_WOOD state - Wood sensor reads LOW. Starting YES_WOOD Sequence (simultaneous return).");
    configurePositionMotorForReturn();
    
    retractFeedClamp(); 
    retractWoodSecureClamp(); 
    Serial.println("Position and Wood Secure clamps disengaged for simultaneous return.");

    // Set flag to indicate we're in YES_WOOD return mode - enable homing sensor check
    extern bool cutMotorInYesWoodReturn; // From main.cpp
    cutMotorInYesWoodReturn = true;
    moveCutMotorToHome();
    movePositionMotorToYesWoodHome();
    
    // Initialize step tracking
    yesWoodSubStep = 0;
}

void YesWoodState::onExit(StateManager& stateManager) {
    Serial.println("Exiting YES_WOOD state");
    resetSteps();
}

void YesWoodState::handleYES_WOOD_Sequence(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern bool cutMotorInYesWoodReturn; // From main.cpp
    
    // This step handles the completion of the "YES_WOOD Sequence".
    switch (yesWoodSubStep) {
        case 0: // Wait for position motor to home
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("YES_WOOD Step 0: Position motor has returned home. Engaging feed clamp.");
                extendFeedClamp();
                Serial.println("Feed clamp engaged (position motor home).");
                yesWoodSubStep = 1;
            }
            break;

        case 1: // Wait for cut motor to home, then proceed with original logic
            if (cutMotor && !cutMotor->isRunning()) {
                Serial.println("YES_WOOD Step 1: Cut motor has returned home.");
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
                    yesWoodSubStep = 2; // Move to position motor homing sequence
                }
            }
            break;
            
        case 2: // Wait for position motor to reach final position, then start homing sequence
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("YES_WOOD Step 2: Position motor at final position. Starting end-of-cycle position motor homing sequence."); 
                
                //! ************************************************************************
                //! STEP: RETRACT FEED CLAMP AND START POSITION MOTOR HOMING SEQUENCE
                //! ************************************************************************
                retractFeedClamp();
                Serial.println("Feed clamp retracted. Starting position motor homing sequence...");
                
                // Transition to position motor homing sequence
                yesWoodSubStep = 3;
                positionHomingSubStep = 0; // Initialize homing substep
                Serial.println("Transitioning to position motor homing sequence."); 
            }
            break;
            
        case 3: // Position Motor Homing Sequence
            handleYES_WOOD_PositionMotorHoming(stateManager);
            break;
    }
}

void YesWoodState::handleYES_WOOD_PositionMotorHoming(StateManager& stateManager) {
    FastAccelStepper* positionMotor = stateManager.getPositionMotor();
    extern const float POSITION_MOTOR_HOMING_SPEED; // From main.cpp
    extern const float POSITION_TRAVEL_DISTANCE; // From main.cpp
    extern const int POSITION_MOTOR_STEPS_PER_INCH; // From main.cpp
    
    // Non-blocking position motor homing sequence
    switch (positionHomingSubStep) {
        case 0: // Start homing - move toward home switch
            Serial.println("YES_WOOD Position Motor Homing Step 0: Moving toward home switch.");
            if (positionMotor) {
                positionMotor->setSpeedInHz((uint32_t)POSITION_MOTOR_HOMING_SPEED);
                positionMotor->moveTo(10000 * POSITION_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            positionHomingSubStep = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getPositionHomingSwitch()->update();
            if (stateManager.getPositionHomingSwitch()->read() == HIGH) {
                Serial.println("YES_WOOD Position Motor Homing Step 1: Home switch triggered. Stopping motor.");
                if (positionMotor) {
                    positionMotor->stopMove();
                    positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH);
                }
                Serial.println("Position motor hit home switch.");
                positionHomingSubStep = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.2 inch from switch
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("YES_WOOD Position Motor Homing Step 2: Moving to -0.2 inch from home switch to establish working zero.");
                positionMotor->moveTo(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH - 0.1 * POSITION_MOTOR_STEPS_PER_INCH);
                positionHomingSubStep = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (positionMotor && !positionMotor->isRunning()) {
                Serial.println("YES_WOOD Position Motor Homing Step 3: Setting new working zero position.");
                positionMotor->setCurrentPosition(POSITION_TRAVEL_DISTANCE * POSITION_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                Serial.println("Position motor homed: 0.2 inch from switch set as position 0.");
                
                configurePositionMotorForNormalOperation();
                positionHomingSubStep = 4;
            }
            break;
            
        case 4: // Homing complete - check for continuous mode or finish cycle
            Serial.println("YES_WOOD Position Motor Homing Step 4: Homing sequence complete.");
            extendWoodSecureClamp(); 
            Serial.println("Wood secure clamp engaged."); 
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

void YesWoodState::resetSteps() {
    yesWoodSubStep = 0;
    positionHomingSubStep = 0;
} 