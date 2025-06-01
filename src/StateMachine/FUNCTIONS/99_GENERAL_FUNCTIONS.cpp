// IMPORTANT NOTE: This file contains helper functions used by 'Stage 1 Feb25.cpp'.
// It relies on 'Stage 1 Feb25.cpp' for pin definitions and global variable declarations (via extern).
#include "StateMachine/99_GENERAL_FUNCTIONS.h"

//* ************************************************************************
//* *********************** HELPER FUNCTIONS ******************************
//* ************************************************************************
// Helper functions for clamp control, LED control, inter-stage signaling,
// and motor control used by the main Stage 1 control system.

// Note: Pin definitions, global servo variables, motor objects, and related constants
// are declared as 'extern' in functions.h and defined in the main .cpp file.

//* ************************************************************************
//* *********************** SIGNALING FUNCTIONS ****************************
//* ************************************************************************
// Contains functions related to signaling other stages or components.

void sendSignalToTA() {
  // Set the signal pin HIGH to trigger Transfer Arm (active HIGH)
  digitalWrite(TRANSFER_ARM_SIGNAL_PIN, HIGH);
  signalTAStartTime = millis();
  signalTAActive = true;
  Serial.println("TA Signal activated (HIGH).");

  // Only activate servo if it hasn't been activated early
  if (!rotationServoIsActiveAndTiming) {
    rotationServo.write(ROTATION_SERVO_ACTIVE_POSITION);
    rotationServoActiveStartTime = millis();
    rotationServoIsActiveAndTiming = true;
    Serial.print("Rotation servo moved to ");
    Serial.print(ROTATION_SERVO_ACTIVE_POSITION);
    Serial.println(" degrees with TA signal.");
  } else {
    Serial.println("Rotation servo already activated early - skipping normal activation.");
  }
}

//* ************************************************************************
//* ************************* CLAMP FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling various clamps.
// Clamp Logic: LOW = extended, HIGH = retracted
// Rotation Clamp Logic: HIGH = extended, LOW = retracted

void extendFeedClamp() {
    // Feed clamp extends when LOW (inversed logic)
    digitalWrite(FEED_CLAMP, LOW); // Extended
    Serial.println("Feed Clamp Extended");
}

void retractFeedClamp() {
    // Feed clamp retracts when HIGH (inversed logic)
    digitalWrite(FEED_CLAMP, HIGH); // Retracted
    Serial.println("Feed Clamp Retracted");
}

void extendWoodSecureClamp() {
    // Wood secure clamp extends when LOW (inversed logic)
    digitalWrite(WOOD_SECURE_CLAMP, LOW); // Extended
    Serial.println("Wood Secure Clamp Extended");
}

void retractWoodSecureClamp() {
    // Wood secure clamp retracts when HIGH (inversed logic)
    digitalWrite(WOOD_SECURE_CLAMP, HIGH); // Retracted
    Serial.println("Wood Secure Clamp Retracted");
}

void extendRotationClamp() {
    // Rotation clamp extends when HIGH
    digitalWrite(ROTATION_CLAMP, HIGH); // Extended (Reversed Logic)
    rotationClampExtendTime = millis();
    rotationClampIsExtended = true;
    Serial.println("Rotation Clamp Extended");
}

void retractRotationClamp() {
    // Rotation clamp retracts when LOW
    digitalWrite(ROTATION_CLAMP, LOW); // Retracted (Reversed Logic)
    rotationClampIsExtended = false; // Assuming we want to clear the flag when explicitly retracting
    Serial.println("Rotation Clamp Retracted");
}

//* ************************************************************************
//* *************************** LED FUNCTIONS ******************************
//* ************************************************************************
// Contains functions for controlling LEDs.

void turnRedLedOn() {
  static bool lastRedLedState = false;
  digitalWrite(STATUS_LED_RED, HIGH);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastRedLedState) {
    Serial.println("Red LED ON");
    lastRedLedState = true;
  }
}

void turnRedLedOff() {
  static bool lastRedLedState = true;
  digitalWrite(STATUS_LED_RED, LOW);
  if (lastRedLedState) {
    Serial.println("Red LED OFF");
    lastRedLedState = false;
  }
}

void turnYellowLedOn() {
  static bool lastYellowLedState = false;
  digitalWrite(STATUS_LED_YELLOW, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastYellowLedState) {
    Serial.println("Yellow LED ON");
    lastYellowLedState = true;
  }
}

void turnYellowLedOff() {
  static bool lastYellowLedState = true;
  digitalWrite(STATUS_LED_YELLOW, LOW);
  if (lastYellowLedState) {
    Serial.println("Yellow LED OFF");
    lastYellowLedState = false;
  }
}

void turnGreenLedOn() {
  static bool lastGreenLedState = false;
  digitalWrite(STATUS_LED_GREEN, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (!lastGreenLedState) {
    Serial.println("Green LED ON");
    lastGreenLedState = true;
  }
}

void turnGreenLedOff() {
  static bool lastGreenLedState = true;
  digitalWrite(STATUS_LED_GREEN, LOW);
  if (lastGreenLedState) {
    Serial.println("Green LED OFF");
    lastGreenLedState = false;
  }
}

void turnBlueLedOn() {
  static bool lastBlueLedState = false;
  digitalWrite(STATUS_LED_BLUE, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_YELLOW, LOW);
  if (!lastBlueLedState) {
    Serial.println("Blue LED ON");
    lastBlueLedState = true;
  }
}

void turnBlueLedOff() {
  static bool lastBlueLedState = true;
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (lastBlueLedState) {
    Serial.println("Blue LED OFF");
    lastBlueLedState = false;
  }
}

void allLedsOff() {
    turnRedLedOff();
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

void handleHomingLedBlink() {
    static unsigned long blinkTimer = 0;
    if (millis() - blinkTimer > 500) {
        blinkState = !blinkState;
        if (blinkState) turnBlueLedOn(); else turnBlueLedOff();
        blinkTimer = millis();
    }
}

void handleErrorLedBlink() {
    if (millis() - lastErrorBlinkTime > 250) {
        errorBlinkState = !errorBlinkState;
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        lastErrorBlinkTime = millis();
    }
}

void handleSuctionErrorLedBlink(unsigned long& lastBlinkTimeRef, bool& blinkStateRef) {
    if (millis() - lastBlinkTimeRef >= 1500) {
        lastBlinkTimeRef = millis();
        blinkStateRef = !blinkStateRef;
        if(blinkStateRef) turnRedLedOn(); else turnRedLedOff();
    }
    turnYellowLedOff();
    turnGreenLedOff();
    turnBlueLedOff();
}

//* ************************************************************************
//* *********************** MOTOR CONTROL FUNCTIONS ************************
//* ************************************************************************

void configureCutMotorForCutting() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_NORMAL_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureCutMotorForReturn() {
    if (cutMotor) {
        cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_RETURN_SPEED);
        cutMotor->setAcceleration((uint32_t)CUT_MOTOR_NORMAL_ACCELERATION);
    }
}

void configurePositionMotorForNormalOperation() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)FEED_MOTOR_NORMAL_SPEED);
        positionMotor->setAcceleration((uint32_t)FEED_MOTOR_NORMAL_ACCELERATION);
    }
}

void configurePositionMotorForReturn() {
    if (positionMotor) {
        positionMotor->setSpeedInHz((uint32_t)FEED_MOTOR_RETURN_SPEED);
        positionMotor->setAcceleration((uint32_t)FEED_MOTOR_RETURN_ACCELERATION);
    }
}

void moveCutMotorToCut() {
    if (cutMotor) {
        cutMotor->moveTo(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
    }
}

void moveCutMotorToHome() {
    if (cutMotor) {
        cutMotor->moveTo(-0.02 * CUT_MOTOR_STEPS_PER_INCH); // Minimal overshoot
    }
}

void movePositionMotorToTravel() {
    if (positionMotor) {
        positionMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void movePositionMotorToHome() {
    if (positionMotor) {
        positionMotor->moveTo(0);
    }
}

void movePositionMotorToPosition(float targetPositionInches) {
    if (positionMotor) {
        positionMotor->moveTo(targetPositionInches * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->stopMove();
    }
}

void stopPositionMotor() {
    if (positionMotor) {
        positionMotor->stopMove();
    }
}

// Basic blocking homing function for Cut Motor - can be expanded
void homeCutMotorBlocking(Bounce& homingSwitch, unsigned long timeout) {
    if (!cutMotor) return;
    unsigned long startTime = millis();
    cutMotor->setSpeedInHz((uint32_t)CUT_MOTOR_HOMING_SPEED);
    cutMotor->moveTo(-40000);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
        if (millis() - startTime > timeout) {
            Serial.println("Cut motor homing timeout!");
            cutMotor->stopMove();
            return;
        }
    }
    cutMotor->stopMove();
    cutMotor->setCurrentPosition(0);
    Serial.println("Cut motor homed.");
}

// Basic blocking homing function for Position Motor - can be expanded
void homePositionMotorBlocking(Bounce& homingSwitch) {
    if (!positionMotor) return;
    
    // Step 1: Move toward home switch until it triggers
    positionMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
    positionMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
    }
    positionMotor->stopMove();
    positionMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    Serial.println("Position motor hit home switch.");
    
    // Step 2: Move to -1 inch from home switch to establish working zero
    Serial.println("Moving position motor to -1 inch from home switch...");
    positionMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 1.0 * FEED_MOTOR_STEPS_PER_INCH);
    
    // Wait for move to complete
    while (positionMotor->isRunning()) {
        // Wait for move to finish
    }
    
    // Step 3: Set this position (-0.5 inch from switch) as the new zero
    positionMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    Serial.println("Position motor homed: 1 inch from switch set as position 0.");
    
    configurePositionMotorForNormalOperation();
}

void movePositionMotorToInitialAfterHoming() {
    if (positionMotor) {
        configurePositionMotorForNormalOperation();
        movePositionMotorToHome();
        while(positionMotor->isRunning()){
        }
    }
}

// Point 3: Complex conditional logic
// Checks the cut motor homing switch multiple times and recalibrates if detected.
// Returns true if home detected and recalibrated, false otherwise.
bool checkAndRecalibrateCutMotorHome(int attempts) {
    if (!cutMotor) return false;

    bool sensorDetectedHome = false;
    for (int i = 0; i < attempts; i++) {
        cutHomingSwitch.update();
        Serial.print("Cut position switch read attempt "); Serial.print(i + 1); Serial.print(": "); Serial.println(cutHomingSwitch.read());
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            cutMotor->setCurrentPosition(0);
            Serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
            break;
        }
    }
    return sensorDetectedHome;
}

//* ************************************************************************
//* ************************* SWITCH LOGIC FUNCTIONS ***********************
//* ************************************************************************
// Point 2: Switch handling

void handleReloadMode() {
    if (currentState == IDLE) {
        bool reloadSwitchOn = reloadSwitch.read() == HIGH;
        if (reloadSwitchOn && !isReloadMode) {
            isReloadMode = true;
            retractFeedClamp();
            retractWoodSecureClamp();
            turnYellowLedOn();
            Serial.println("Entered reload mode");
        } else if (!reloadSwitchOn && isReloadMode) {
            isReloadMode = false;
            extendFeedClamp();
            extendWoodSecureClamp();
            turnYellowLedOff();
            Serial.println("Exited reload mode, ready for operation");
        }
    }
}

void handleErrorAcknowledgement() {
    // This handles the general error acknowledgement via reloadSwitch
    // It was present in the main loop and also within the CUTTING state's homePositionErrorDetected block.
    if (reloadSwitch.rose() && (currentState == ERROR || currentState == CUTTING)) { // Check if in ERROR or if a cutting error is active
        // For CUTTING state, the homePositionErrorDetected flag logic needs to remain there,
        // but the transition to ERROR_RESET can be centralized if errorAcknowledged is set.
        if (currentState == ERROR) {
            currentState = ERROR_RESET;
            errorAcknowledged = true; // Set flag, main loop will see this for ERROR state
            Serial.println("Error acknowledged by reload switch (from ERROR state). Transitioning to ERROR_RESET.");
        }
        // If in CUTTING, setting errorAcknowledged might be used by the CUTTING state to proceed.
        // The original CUTTING state logic directly transitioned. For now, we set the flag.
        // The calling code in CUTTING will need to check this flag if it relies on it.
        // For direct transition from specific cutting error, that logic is better kept in cutting stage.
        // This function primarily handles the generic ERROR state reset.
    }
}

void handleStartSwitchSafety() {
    // Original logic from setup() and main loop for startSwitchSafe
    // Call this once in setup() after startCycleSwitch.update()
    // And continuously in the main loop before checking shouldStartCycle()
    if (!startSwitchSafe && startCycleSwitch.fell()) {
        startSwitchSafe = true;
        Serial.println("Start switch is now safe to use (cycled OFF).");
    }
    // Initial check (typically for setup)
    // This part might be better directly in setup, but included here for completeness if called from there.
    // If called repeatedly from loop, this `else if` might be redundant if startSwitchSafe is managed correctly.
    /* else if (startCycleSwitch.read() == HIGH && !startSwitchSafe) {
        Serial.println("WARNING: Start switch is ON. Turn it OFF before operation.");
    }*/
}

void handleStartSwitchContinuousMode(){
    bool startSwitchOn = startCycleSwitch.read() == HIGH;
    if (startSwitchOn != continuousModeActive && startSwitchSafe) {
        continuousModeActive = startSwitchOn;
        if (continuousModeActive) {
            Serial.println("Continuous operation mode activated");
        } else {
            Serial.println("Continuous operation mode deactivated");
        }
    }
}

//* ************************************************************************
//* ************************* STATE LOGIC HELPERS **************************
//* ************************************************************************
// Point 3: Complex conditional logic

bool shouldStartCycle() {
    // Condition from IDLE state to start a cycle
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

// Point 4: Rotation Servo Timing
void activateRotationServo() {
    // Activate rotation servo without sending TA signal
    if (!rotationServoIsActiveAndTiming) {
        rotationServo.write(ROTATION_SERVO_ACTIVE_POSITION);
        rotationServoActiveStartTime = millis();
        rotationServoIsActiveAndTiming = true;
        Serial.print("Rotation servo activated to ");
        Serial.print(ROTATION_SERVO_ACTIVE_POSITION);
        Serial.println(" degrees.");
    } else {
        Serial.println("Rotation servo already active - skipping activation.");
    }
}

void handleRotationServoReturn() {
    // Move rotation servo to home position
    rotationServo.write(ROTATION_SERVO_HOME_POSITION);
    Serial.print("Rotation servo returned to home position (");
    Serial.print(ROTATION_SERVO_HOME_POSITION);
    Serial.println(" degrees).");
}

void handleTASignalTiming() { 
  if (signalTAActive && millis() - signalTAStartTime >= TA_SIGNAL_DURATION) {
    digitalWrite(TRANSFER_ARM_SIGNAL_PIN, LOW); // Return to inactive state (LOW)
    signalTAActive = false;
    Serial.println("Signal to Transfer Arm (TA) completed"); 
  }
}

void handleRotationClampRetract() { // Point 4
    if (rotationClampIsExtended && (millis() - rotationClampExtendTime >= ROTATION_CLAMP_EXTEND_DURATION_MS)) {
        retractRotationClamp();
        Serial.println("Rotation Clamp retracted after 1 second.");
    }
}

void movePositionMotorToYes2x4Home() {
    if (positionMotor) {
        positionMotor->moveTo(0);
        Serial.println("Position motor moving to Yes_2x4 home position (0 inches)");
    }
}