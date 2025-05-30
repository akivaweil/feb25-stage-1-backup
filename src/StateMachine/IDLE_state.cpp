#include "StateMachine/IdleState.h"
#include "StateMachine/StateManager.h"
#include "Functions.h"

//* ************************************************************************
//* ************************** IDLE STATE **********************************
//* ************************************************************************
// Handles the idle state, awaiting user input or automatic cycle start.
// Maintains secure wood clamp extended and position clamp retracted.
// Checks for pushwood forward switch press to transition to FirstCut state.
// If not in reload mode:
//   Step 1: Turn on green LED to indicate system is idle.
//   Step 2: Check for pushwood forward switch press AND wood sensor high to start FirstCut.
//   Step 3: Check for start cycle conditions:
//           - Start switch just flipped ON (rising edge).
//           - OR Continuous mode active AND not already in a cutting cycle.
//           - AND Wood suction error is not present.
//           - AND Start switch is safe to use (wasn't ON at startup or has been cycled).
//   Step 4: If start conditions met:
//           - Turn off green LED, turn on yellow LED.
//           - Set cuttingCycleInProgress flag to true.
//           - Transition to CUTTING state.
//           - Configure cut motor for cutting speed.
//           - Ensure position and wood secure clamps are engaged.
//           - If no wood is detected, turn on blue LED for NO_WOOD mode indication.

void IdleState::execute(StateManager& stateManager) {
    // Handle reload mode logic first
    handleReloadModeLogic(stateManager);
    
    // Check for FirstCut conditions if not in reload mode
    if (!stateManager.getIsReloadMode()) {
        checkFirstCutConditions(stateManager);
        checkStartConditions(stateManager);
    }
}

void IdleState::onEnter(StateManager& stateManager) {
    // Maintain clamp states: secure wood clamp extended, position clamp retracted
    extendWoodSecureClamp();
    retractPositionClamp();
    Serial.println("Idle: Secure wood clamp extended, position clamp retracted");
}

void IdleState::handleReloadModeLogic(StateManager& stateManager) {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool reloadSwitchOn = stateManager.getReloadSwitch()->read() == HIGH;
    bool isReloadMode = stateManager.getIsReloadMode();
    
    if (reloadSwitchOn && !isReloadMode) {
        // Enter reload mode
        stateManager.setIsReloadMode(true);
        retractPositionClamp(); // Disengage position clamp
        retractWoodSecureClamp(); // Disengage wood secure clamp
        turnBlueLedOn();     // Turn on blue LED for reload mode
    } else if (!reloadSwitchOn && isReloadMode) {
        // Exit reload mode
        stateManager.setIsReloadMode(false);
        extendWoodSecureClamp(); // Re-engage wood secure clamp
        retractPositionClamp();   // Keep position clamp retracted (idle state default)
        turnBlueLedOff();       // Turn off blue LED
    }
}

void IdleState::checkFirstCutConditions(StateManager& stateManager) {
    // Check for pushwood forward switch press AND wood sensor HIGH
    extern Bounce pushwoodForwardSwitch;
    extern Bounce fixPositionSwitch;
    extern const int WOOD_SENSOR;
    bool pushwoodPressed = pushwoodForwardSwitch.rose();
    bool fixPositionPressed = fixPositionSwitch.rose();
    bool woodSensorHigh = (digitalRead(WOOD_SENSOR) == HIGH);
    bool woodSensorLow = (digitalRead(WOOD_SENSOR) == LOW);
    
    if (pushwoodPressed && woodSensorHigh) {
        Serial.println("Idle: Pushwood forward switch pressed with wood sensor HIGH - transitioning to FIRSTCUT");
        stateManager.changeState(FIRSTCUT);
    }
    else if (fixPositionPressed && woodSensorHigh) {
        Serial.println("Idle: Fix position button pressed with wood sensor HIGH - transitioning to FIRSTCUT");
        stateManager.changeState(FIRSTCUT);
    }
    else if (fixPositionPressed && woodSensorLow) {
        Serial.println("Idle: Fix position button pressed with wood sensor LOW - transitioning to PUSHWOOD_FORWARD");
        stateManager.changeState(PUSHWOOD_FORWARD);
    }
    else if (pushwoodPressed && woodSensorLow) {
        Serial.println("Idle: Pushwood forward switch pressed with wood sensor LOW - executing pushwood forward action");
        // TODO: Add pushwood forward action here (what should this do?)
        // For now, just log the action
    }
}

void IdleState::checkStartConditions(StateManager& stateManager) {
    turnGreenLedOn();
    
    bool startCycleRose = stateManager.getStartCycleSwitch()->rose();
    bool continuousModeActive = stateManager.getContinuousModeActive();
    bool cuttingCycleInProgress = stateManager.getCuttingCycleInProgress();
    bool woodSuctionError = stateManager.getWoodSuctionError();
    bool startSwitchSafe = stateManager.getStartSwitchSafe();
    bool woodPresent = stateManager.getWoodPresent();
    
    if (((startCycleRose || (continuousModeActive && !cuttingCycleInProgress)) 
        && !woodSuctionError) && startSwitchSafe) {
        turnGreenLedOff();
        turnYellowLedOn();
        turnBlueLedOff();
        
        stateManager.setCuttingCycleInProgress(true);
        stateManager.changeState(CUTTING);
        configureCutMotorForCutting();
        
        extendPositionClamp();
        extendWoodSecureClamp();
        
        if (!woodPresent) {
            turnBlueLedOn();
        }
    }
} 