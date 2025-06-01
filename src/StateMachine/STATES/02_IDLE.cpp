#include "StateMachine/02_IDLE.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/99_GENERAL_FUNCTIONS.h"

//* ************************************************************************
//* ************************** IDLE STATE **********************************
//* ************************************************************************
// Handles the idle state, awaiting user input or automatic cycle start.
// Maintains secure wood clamp extended and feed clamp retracted.
// Checks for pushwood forward switch press to transition to FeedFirstCut state.
// If not in reload mode:
//   Step 1: Turn on green LED to indicate system is idle.
//   Step 2: Check for pushwood forward switch press AND 2x4 sensor high to start FeedFirstCut.
//           - AND Reload mode is not active.
//           - AND Start cycle switch safety is not active.
//           - AND Wood suction error is not present.
//
//   Additional behavior:
//   Step 3: Check for fix position button press and 2x4 sensor state:
//           - If high, transition to FeedFirstCut state.
//           - If low, transition to FeedWoodFwdOne state.
//
//   Loop maintenance:
//           - Ensure position and wood secure clamps are engaged.
//           - If no 2x4 is detected, turn on blue LED for NO_WOOD mode indication.
//   Step 4: Check for start cycle conditions:
//           - Start switch just flipped ON (rising edge).
//           - OR Continuous mode active AND not already in a cutting cycle.
//           - AND Wood suction error is not present.
//           - AND Start switch is safe to use (wasn't ON at startup or has been cycled).
//   Step 5: If start conditions met:
//           - Turn off green LED, turn on yellow LED.
//           - Set cuttingCycleInProgress flag to true.
//           - Transition to CUTTING state.
//           - Configure cut motor for cutting speed.
//           - Ensure position and wood secure clamps are engaged.
//           - If no wood is detected, turn on blue LED for NO_WOOD mode indication.

void IdleState::execute(StateManager& stateManager) {
    // Handle reload mode logic first
    handleReloadModeLogic(stateManager);
    
    // Check for FeedFirstCut conditions if not in reload mode
    if (!stateManager.getIsReloadMode()) {
        checkFirstCutConditions(stateManager);
        checkStartConditions(stateManager);
    }
}

void IdleState::onEnter(StateManager& stateManager) {
    // Maintain clamp states: secure wood clamp extended, feed clamp retracted
    extend2x4SecureClamp();
    retractFeedClamp();
    Serial.println("Idle: Secure wood clamp extended, feed clamp retracted");
}

void IdleState::handleReloadModeLogic(StateManager& stateManager) {
    // Check current state of reload switch (HIGH = ON with pull-down resistor)
    bool reloadSwitchOn = stateManager.getReloadSwitch()->read() == HIGH;
    bool isReloadMode = stateManager.getIsReloadMode();
    
    if (reloadSwitchOn && !isReloadMode) {
        // Enter reload mode
        stateManager.setIsReloadMode(true);
        retractFeedClamp(); // Retract feed clamp
        retract2x4SecureClamp(); // Retract 2x4 secure clamp
        turnBlueLedOn();     // Turn on blue LED for reload mode
    } else if (!reloadSwitchOn && isReloadMode) {
        // Exit reload mode
        stateManager.setIsReloadMode(false);
        extend2x4SecureClamp(); // Re-extend 2x4 secure clamp
        retractFeedClamp();   // Keep feed clamp retracted (idle state default)
        turnBlueLedOff();       // Turn off blue LED
    }
}

void IdleState::checkFirstCutConditions(StateManager& stateManager) {
    // Check for pushwood forward switch press and 2x4 sensor state
    extern Bounce pushwoodForwardSwitch;
    extern const int _2x4_PRESENT_SENSOR;
    bool pushwoodPressed = pushwoodForwardSwitch.rose();
    bool _2x4SensorHigh = (digitalRead(_2x4_PRESENT_SENSOR) == HIGH);
    bool _2x4SensorLow = (digitalRead(_2x4_PRESENT_SENSOR) == LOW);
    
    if (pushwoodPressed && _2x4SensorHigh) {
        Serial.println("Idle: Manual feed switch pressed with 2x4 sensor HIGH - transitioning to FEED_FIRST_CUT");
        stateManager.changeState(FEED_FIRST_CUT);
    }
    else if (pushwoodPressed && _2x4SensorLow) {
        Serial.println("Idle: Manual feed switch pressed with 2x4 sensor LOW - transitioning to FEED_WOOD_FWD_ONE");
        stateManager.changeState(FEED_WOOD_FWD_ONE);
    }
}

void IdleState::checkStartConditions(StateManager& stateManager) {
    turnGreenLedOn();
    
    bool startCycleRose = stateManager.getStartCycleSwitch()->rose();
    bool continuousModeActive = stateManager.getContinuousModeActive();
    bool cuttingCycleInProgress = stateManager.getCuttingCycleInProgress();
    bool woodSuctionError = stateManager.getWoodSuctionError();
    bool startSwitchSafe = stateManager.getStartSwitchSafe();
    bool _2x4Present = stateManager.get2x4Present();
    
    if (((startCycleRose || (continuousModeActive && !cuttingCycleInProgress)) 
        && !woodSuctionError) && startSwitchSafe) {
        turnGreenLedOff();
        turnYellowLedOn();
        turnBlueLedOff();
        
        stateManager.setCuttingCycleInProgress(true);
        stateManager.changeState(CUTTING);
        configureCutMotorForCutting();
        
        extendFeedClamp();
        extend2x4SecureClamp();
        
        if (!_2x4Present) {
            turnBlueLedOn();
        }
    }
} 