#include "StateMachine/PushwoodForwardState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ********************* PUSHWOOD FORWARD STATE **************************
//* ************************************************************************
// Handles the pushwood forward sequence when fix position switch is pressed
// in idle state AND wood sensor reads LOW.
//
// Sequence:
// 1. Retract the position clamp
// 2. Move the position motor to 0
// 3. Extend the position clamp and retract the secure wood clamp
// 4. Wait 300ms
// 5. Move the position motor to POSITION_TRAVEL_DISTANCE

void PushwoodForwardState::execute(StateManager& stateManager) {
    executeStep(stateManager);
}

void PushwoodForwardState::onEnter(StateManager& stateManager) {
    currentStep = RETRACT_POSITION_CLAMP;
    stepStartTime = millis();
    Serial.println("PushwoodForward: Starting pushwood forward sequence");
    
    //! ************************************************************************
    //! STEP 1: RETRACT THE POSITION CLAMP
    //! ************************************************************************
    retractPositionClamp();
    Serial.println("PushwoodForward: Position clamp retracted");
}

void PushwoodForwardState::onExit(StateManager& stateManager) {
    // Clean up if needed
}

void PushwoodForwardState::executeStep(StateManager& stateManager) {
    switch (currentStep) {
        case RETRACT_POSITION_CLAMP:
            // Position clamp already retracted in onEnter, move to next step
            advanceToNextStep(stateManager);
            break;
            
        case MOVE_TO_ZERO:
            //! ************************************************************************
            //! STEP 2: MOVE THE POSITION MOTOR TO 0
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                movePositionMotorToPosition(0.0);
                Serial.println("PushwoodForward: Moving position motor to 0");
            }
            
            // Wait for motor to finish moving
            if (!stateManager.getPositionMotor()->isRunning()) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case EXTEND_POSITION_CLAMP_RETRACT_SECURE:
            //! ************************************************************************
            //! STEP 3: EXTEND POSITION CLAMP AND RETRACT SECURE WOOD CLAMP
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                extendPositionClamp();
                retractWoodSecureClamp();
                Serial.println("PushwoodForward: Position clamp extended, secure wood clamp retracted");
            }
            advanceToNextStep(stateManager);
            break;
            
        case WAIT_300MS:
            //! ************************************************************************
            //! STEP 4: WAIT 300MS
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                Serial.println("PushwoodForward: Waiting 300ms");
            }
            
            if (millis() - stepStartTime >= 300) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case MOVE_TO_TRAVEL_DISTANCE:
            //! ************************************************************************
            //! STEP 5: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                movePositionMotorToTravel();
                Serial.println("PushwoodForward: Moving position motor to travel distance");
            }
            
            // Wait for motor to finish moving
            if (!stateManager.getPositionMotor()->isRunning()) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case COMPLETE:
            //! ************************************************************************
            //! STEP 6: PUSHWOOD FORWARD SEQUENCE COMPLETE, RETURN TO IDLE
            //! ************************************************************************
            Serial.println("PushwoodForward: Sequence complete, returning to IDLE");
            stateManager.changeState(IDLE);
            break;
    }
}

void PushwoodForwardState::advanceToNextStep(StateManager& stateManager) {
    stepStartTime = 0; // Reset step timer
    
    switch (currentStep) {
        case RETRACT_POSITION_CLAMP:
            currentStep = MOVE_TO_ZERO;
            break;
        case MOVE_TO_ZERO:
            currentStep = EXTEND_POSITION_CLAMP_RETRACT_SECURE;
            break;
        case EXTEND_POSITION_CLAMP_RETRACT_SECURE:
            currentStep = WAIT_300MS;
            break;
        case WAIT_300MS:
            currentStep = MOVE_TO_TRAVEL_DISTANCE;
            break;
        case MOVE_TO_TRAVEL_DISTANCE:
            currentStep = COMPLETE;
            break;
        case COMPLETE:
            // Final step, no advancement needed
            break;
    }
} 