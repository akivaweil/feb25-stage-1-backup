#ifndef YESWOOD_STATE_H
#define YESWOOD_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** YES WOOD STATE ******************************
//* ************************************************************************
// Handles the YES_WOOD cutting sequence when wood is detected.
// This state manages the simultaneous return process for wood that triggers the wood sensor.

class YesWoodState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return YESWOOD; }

private:
    // YES_WOOD sequence tracking
    int yesWoodSubStep = 0;
    int positionHomingSubStep = 0; // For position motor homing sequence
    
    // Helper methods for YES_WOOD sequence
    void handleYES_WOOD_Sequence(StateManager& stateManager);
    void handleYES_WOOD_PositionMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // YESWOOD_STATE_H 