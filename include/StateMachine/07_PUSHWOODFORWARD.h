#ifndef PUSHWOOD_FORWARD_STATE_H
#define PUSHWOOD_FORWARD_STATE_H

#include "BaseState.h"

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
// 4. Wait 200ms
// 5. Move the position motor to POSITION_TRAVEL_DISTANCE

class PushwoodForwardState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return PUSHWOOD_FORWARD; }

private:
    enum PushwoodForwardStep {
        RETRACT_POSITION_CLAMP,
        MOVE_TO_ZERO,
        EXTEND_POSITION_CLAMP_RETRACT_SECURE,
        WAIT_200MS,
        MOVE_TO_TRAVEL_DISTANCE,
        COMPLETE
    };
    
    PushwoodForwardStep currentStep = RETRACT_POSITION_CLAMP;
    unsigned long stepStartTime = 0;
    
    void executeStep(StateManager& stateManager);
    void advanceToNextStep(StateManager& stateManager);
};

#endif // PUSHWOOD_FORWARD_STATE_H 