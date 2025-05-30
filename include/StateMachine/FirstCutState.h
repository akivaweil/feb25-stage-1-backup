#ifndef FIRST_CUT_STATE_H
#define FIRST_CUT_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************ FIRST CUT STATE ******************************
//* ************************************************************************
// Handles the first cut sequence when pushwood forward switch is pressed.
// Manages position clamp retraction, motor movement, and transition to cutting.

class FirstCutState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return FIRSTCUT; }

private:
    enum FirstCutStep {
        RETRACT_POSITION_CLAMP,
        MOVE_TO_MINUS_ONE,
        EXTEND_POSITION_CLAMP_RETRACT_SECURE,
        WAIT_300MS,
        MOVE_TO_TRAVEL_DISTANCE,
        TRANSITION_TO_CUTTING
    };
    
    FirstCutStep currentStep = RETRACT_POSITION_CLAMP;
    unsigned long stepStartTime = 0;
    
    void executeStep(StateManager& stateManager);
    void advanceToNextStep(StateManager& stateManager);
};

#endif // FIRST_CUT_STATE_H 