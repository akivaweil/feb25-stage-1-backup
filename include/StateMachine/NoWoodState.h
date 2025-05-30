#ifndef NOWOOD_STATE_H
#define NOWOOD_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** NO WOOD STATE *******************************
//* ************************************************************************
// Handles the NO_WOOD cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

class NoWoodState : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return NOWOOD; }

private:
    // NO_WOOD sequence tracking
    int noWoodStep = 0;
    int noWoodHomingSubStep = 0; // For NO_WOOD position motor homing sequence
    unsigned long cylinderActionTime = 0;
    bool waitingForCylinder = false;
    
    // Helper methods for NO_WOOD sequence
    void handleNO_WOOD_Sequence(StateManager& stateManager);
    void handleNO_WOOD_Step(StateManager& stateManager, int step);
    void handleNO_WOOD_PositionMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // NOWOOD_STATE_H 