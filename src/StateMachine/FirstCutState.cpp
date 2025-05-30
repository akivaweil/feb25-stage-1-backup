#include "StateMachine/FirstCutState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ************************ FIRST CUT STATE ******************************
//* ************************************************************************
// Handles the first cut sequence when pushwood forward switch is pressed
// in idle state AND wood sensor reads high.
//
// Sequence:
// 1. Retract the position clamp
// 2. Move the position motor to -2
// 3. Extend the position clamp and retract the secure wood clamp
// 4. Wait 300ms
// 5. Move the position motor to POSITION_TRAVEL_DISTANCE
// 6. Start a cutting cycle by switching to cutting state

void FirstCutState::execute(StateManager& stateManager) {
    executeStep(stateManager);
}

void FirstCutState::onEnter(StateManager& stateManager) {
    currentStep = RETRACT_POSITION_CLAMP;
    stepStartTime = millis();
    Serial.println("FirstCut: Starting first cut sequence");
    
    //! ************************************************************************
    //! STEP 1: RETRACT THE POSITION CLAMP
    //! ************************************************************************
    retractPositionClamp();
    Serial.println("FirstCut: Position clamp retracted");
}

void FirstCutState::onExit(StateManager& stateManager) {
    // Clean up if needed
}

void FirstCutState::executeStep(StateManager& stateManager) {
    switch (currentStep) {
        case RETRACT_POSITION_CLAMP:
            // Position clamp already retracted in onEnter, move to next step
            advanceToNextStep(stateManager);
            break;
            
        case MOVE_TO_MINUS_ONE:
            //! ************************************************************************
            //! STEP 2: MOVE THE POSITION MOTOR TO -1
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                movePositionMotorToPosition(-2.0);
                Serial.println("FirstCut: Moving position motor to -1 inch");
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
                Serial.println("FirstCut: Position clamp extended, secure wood clamp retracted");
            }
            advanceToNextStep(stateManager);
            break;
            
        case WAIT_300MS:
            //! ************************************************************************
            //! STEP 4: WAIT 300MS
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                Serial.println("FirstCut: Waiting 300ms");
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
                Serial.println("FirstCut: Moving position motor to travel distance");
            }
            
            // Wait for motor to finish moving
            if (!stateManager.getPositionMotor()->isRunning()) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case TRANSITION_TO_CUTTING:
            //! ************************************************************************
            //! STEP 6: COMPLETE FIRST RUN, ADVANCE TO SECOND RUN
            //! ************************************************************************
            Serial.println("FirstCut: First run complete, starting second run");
            advanceToNextStep(stateManager);
            break;
            
        case RETRACT_POSITION_CLAMP_SECOND:
            //! ************************************************************************
            //! STEP 7: RETRACT THE POSITION CLAMP (SECOND RUN)
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                retractPositionClamp();
                Serial.println("FirstCut: Position clamp retracted (second run)");
            }
            advanceToNextStep(stateManager);
            break;
            
        case MOVE_TO_MINUS_ONE_SECOND:
            //! ************************************************************************
            //! STEP 8: MOVE THE POSITION MOTOR TO -2 (SECOND RUN)
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE - 2.75);
                Serial.println("FirstCut: Moving position motor to -2 inch (second run)");
            }
            
            // Wait for motor to finish moving
            if (!stateManager.getPositionMotor()->isRunning()) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case EXTEND_POSITION_CLAMP_RETRACT_SECURE_SECOND:
            //! ************************************************************************
            //! STEP 9: EXTEND POSITION CLAMP AND RETRACT SECURE WOOD CLAMP (SECOND RUN)
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                extendPositionClamp();
                retractWoodSecureClamp();
                Serial.println("FirstCut: Position clamp extended, secure wood clamp retracted (second run)");
            }
            advanceToNextStep(stateManager);
            break;
            
        case WAIT_300MS_SECOND:
            //! ************************************************************************
            //! STEP 10: WAIT 300MS (SECOND RUN)
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                Serial.println("FirstCut: Waiting 300ms (second run)");
            }
            
            if (millis() - stepStartTime >= 300) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case MOVE_TO_TRAVEL_DISTANCE_MINUS_275:
            //! ************************************************************************
            //! STEP 11: MOVE POSITION MOTOR TO POSITION_TRAVEL_DISTANCE - 2.75
            //! ************************************************************************
            if (stepStartTime == 0) {
                stepStartTime = millis();
                movePositionMotorToPosition(POSITION_TRAVEL_DISTANCE);
                Serial.println("FirstCut: Moving position motor to travel distance minus 2.75 inches");
            }
            
            // Wait for motor to finish moving
            if (!stateManager.getPositionMotor()->isRunning()) {
                advanceToNextStep(stateManager);
            }
            break;
            
        case TRANSITION_TO_CUTTING_FINAL:
            //! ************************************************************************
            //! STEP 12: START CUTTING CYCLE BY SWITCHING TO CUTTING STATE (FINAL)
            //! ************************************************************************
            Serial.println("FirstCut: Transitioning to CUTTING state (final)");
            stateManager.setCuttingCycleInProgress(true);
            configureCutMotorForCutting();
            turnGreenLedOff();
            turnYellowLedOn();
            stateManager.changeState(IDLE);
            break;
    }
}

void FirstCutState::advanceToNextStep(StateManager& stateManager) {
    stepStartTime = 0; // Reset step timer
    
    switch (currentStep) {
        case RETRACT_POSITION_CLAMP:
            currentStep = MOVE_TO_MINUS_ONE;
            break;
        case MOVE_TO_MINUS_ONE:
            currentStep = EXTEND_POSITION_CLAMP_RETRACT_SECURE;
            break;
        case EXTEND_POSITION_CLAMP_RETRACT_SECURE:
            currentStep = WAIT_300MS;
            break;
        case WAIT_300MS:
            currentStep = MOVE_TO_TRAVEL_DISTANCE;
            break;
        case MOVE_TO_TRAVEL_DISTANCE:
            currentStep = TRANSITION_TO_CUTTING;
            break;
        case TRANSITION_TO_CUTTING:
            currentStep = RETRACT_POSITION_CLAMP_SECOND;
            break;
        case RETRACT_POSITION_CLAMP_SECOND:
            currentStep = MOVE_TO_MINUS_ONE_SECOND;
            break;
        case MOVE_TO_MINUS_ONE_SECOND:
            currentStep = EXTEND_POSITION_CLAMP_RETRACT_SECURE_SECOND;
            break;
        case EXTEND_POSITION_CLAMP_RETRACT_SECURE_SECOND:
            currentStep = WAIT_300MS_SECOND;
            break;
        case WAIT_300MS_SECOND:
            currentStep = MOVE_TO_TRAVEL_DISTANCE_MINUS_275;
            break;
        case MOVE_TO_TRAVEL_DISTANCE_MINUS_275:
            currentStep = TRANSITION_TO_CUTTING_FINAL;
            break;
        case TRANSITION_TO_CUTTING_FINAL:
            // Final step, no advancement needed
            break;
    }
} 