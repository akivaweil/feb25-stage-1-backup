#ifndef YES_2X4_STATE_H
#define YES_2X4_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** YES 2X4 STATE *******************************
//* ************************************************************************
// Handles the Yes_2x4 cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

class Yes2x4State : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return Yes_2x4; }

private:
    // Yes_2x4 sequence tracking
    int yes2x4SubStep = 0;
    int positionHomingSubStep = 0; // For position motor homing sequence
    
    // Helper methods for Yes_2x4 sequence
    void handleYes2x4Sequence(StateManager& stateManager);
    void handleYes2x4PositionMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // YES_2X4_STATE_H 