#ifndef NO_2X4_STATE_H
#define NO_2X4_STATE_H

#include "BaseState.h"

//* ************************************************************************
//* ************************** NO 2X4 STATE ********************************
//* ************************************************************************
// Handles the No_2x4 cutting sequence when no wood is detected.
// This state manages the multi-step process for handling material that doesn't trigger the wood sensor.

class No2x4State : public BaseState {
public:
    void execute(StateManager& stateManager) override;
    void onEnter(StateManager& stateManager) override;
    void onExit(StateManager& stateManager) override;
    SystemState getStateType() const override { return No_2x4; }

private:
    // No_2x4 sequence tracking
    int no2x4Step = 0;
    int no2x4HomingSubStep = 0; // For No_2x4 position motor homing sequence
    unsigned long cylinderActionTime = 0;
    bool waitingForCylinder = false;
    
    // Helper methods for No_2x4 sequence
    void handleNo2x4Sequence(StateManager& stateManager);
    void handleNo2x4Step(StateManager& stateManager, int step);
    void handleNo2x4PositionMotorHoming(StateManager& stateManager);
    
    // Reset all step counters
    void resetSteps();
};

#endif // NO_2X4_STATE_H 