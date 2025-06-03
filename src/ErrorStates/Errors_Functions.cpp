// IMPORTANT NOTE: This file contains helper functions used by the error handling system.
// It relies on the main file for pin definitions and global variable declarations (via extern).
#include "ErrorStates/Errors_Functions.h"

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

void extend2x4SecureClamp() {
    // 2x4 secure clamp extends when LOW (inversed logic)
    digitalWrite(_2x4_SECURE_CLAMP, LOW); // Extended
    Serial.println("2x4 Secure Clamp Extended");
}

void retract2x4SecureClamp() {
    // 2x4 secure clamp retracts when HIGH (inversed logic)
    digitalWrite(_2x4_SECURE_CLAMP, HIGH); // Retracted
    Serial.println("2x4 Secure Clamp Retracted");
}

void extendRotationClamp() {
    // Rotation clamp extends when HIGH
    digitalWrite(ROTATION_CLAMP, HIGH); // Extended 
    rotationClampExtendTime = millis();
    rotationClampIsExtended = true;
    Serial.println("Rotation Clamp Extended");
}

void retractRotationClamp() {
    // Rotation clamp retracts when LOW
    digitalWrite(ROTATION_CLAMP, LOW); // Retracted 
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

void configureFeedMotorForNormalOperation() {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_NORMAL_SPEED);
        feedMotor->setAcceleration((uint32_t)FEED_MOTOR_NORMAL_ACCELERATION);
    }
}

void configureFeedMotorForReturn() {
    if (feedMotor) {
        feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_RETURN_SPEED);
        feedMotor->setAcceleration((uint32_t)FEED_MOTOR_RETURN_ACCELERATION);
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

void moveFeedMotorToTravel() {
    if (feedMotor) {
        feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void moveFeedMotorToHome() {
    if (feedMotor) {
        feedMotor->moveTo(0);
    }
}

void moveFeedMotorToPosition(float targetPositionInches) {
    if (feedMotor) {
        feedMotor->moveTo(targetPositionInches * FEED_MOTOR_STEPS_PER_INCH);
    }
}

void stopCutMotor() {
    if (cutMotor) {
        cutMotor->stopMove();
    }
}

void stopFeedMotor() {
    if (feedMotor) {
        feedMotor->stopMove();
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

// Basic blocking homing function for Feed Motor - can be expanded
void homeFeedMotorBlocking(Bounce& homingSwitch) {
    if (!feedMotor) return;
    
    // Step 1: Move toward home switch until it triggers
    feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
    feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH);

    while (homingSwitch.read() != HIGH) {
        homingSwitch.update();
    }
    feedMotor->stopMove();
    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    Serial.println("Feed motor hit home switch.");
    
    // Step 2: Move to -1 inch from home switch to establish working zero
    Serial.println("Moving feed motor to -1 inch from home switch...");
    feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.3 * FEED_MOTOR_STEPS_PER_INCH);
    
    // Wait for move to complete
    while (feedMotor->isRunning()) {
        // Wait for move to finish
    }
    
    // Step 3: Set this position (-0.5 inch from switch) as the new zero
    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
    Serial.println("Feed motor homed: 1 inch from switch set as position 0.");
    
    configureFeedMotorForNormalOperation();
}

void moveFeedMotorToInitialAfterHoming() {
    if (feedMotor) {
        configureFeedMotorForNormalOperation();
        moveFeedMotorToHome();
        while(feedMotor->isRunning()){
        }
    }
}

// Complex conditional logic
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

void handleReloadMode() {
    if (currentState == IDLE) {
        bool reloadSwitchOn = reloadSwitch.read() == HIGH;
        if (reloadSwitchOn && !isReloadMode) {
            isReloadMode = true;
            retractFeedClamp();
            retract2x4SecureClamp();
            turnYellowLedOn();
            Serial.println("Entered reload mode");
        } else if (!reloadSwitchOn && isReloadMode) {
            isReloadMode = false;
            extendFeedClamp();
            extend2x4SecureClamp();
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

bool shouldStartCycle() {
    // Condition from IDLE state to start a cycle
    return ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress))
            && !woodSuctionError && startSwitchSafe);
}

// Rotation Servo Timing
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

void handleRotationClampRetract() {
    if (rotationClampIsExtended && (millis() - rotationClampExtendTime >= ROTATION_CLAMP_EXTEND_DURATION_MS)) {
        retractRotationClamp();
        Serial.println("Rotation Clamp retracted after 1 second.");
    }
}

void moveFeedMotorToPostCutHome() {
    if (feedMotor) {
        feedMotor->moveTo(0);
        Serial.println("Feed motor moving to post-cut home position (0 inches)");
    }
}

//* ************************************************************************
//* ****************** CUT MOTOR HOME ERROR HANDLER **********************
//* ************************************************************************
//! CRITICAL SAFETY SYSTEM: Cut motor home position error detection and recovery
//
//! SAFETY-CRITICAL IMPORTANCE:
//! Cut motor home detection is absolutely essential for machine safety.
//! If the cut motor fails to return to its true home position, and the position motor
//! subsequently pushes a wood board forward, the board could collide with the cut motor
//! blade or mechanism, causing:
//! 1. Catastrophic damage to the cutting blade
//! 2. Destruction of the workpiece  
//! 3. Potential injury to operators
//! 4. Misalignment of the entire cutting system
//! 5. Unpredictable machine behavior in subsequent cycles
//!
//! Therefore, this error detection system implements a recovery approach
//! that attempts to slowly move the cut motor back to home position.

// ========================================================================
//! RECOVERY SYSTEM CONFIGURATION
// ========================================================================

//! TIMEOUT AND SPEED CONSTANTS
const unsigned long CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS = 5000; // 5 second maximum recovery time
const float CUT_MOTOR_HOME_RECOVERY_SPEED = 1000; // Recovery speed (same as homing speed)

// ========================================================================
//! RESULT STRUCTURE CREATION HELPERS
// ========================================================================

//! SUCCESS RESULT - Home position detected successfully
CutMotorHomeErrorResult createSuccessResult() {
    CutMotorHomeErrorResult result;
    result.wasHomeDetected = true;
    result.shouldTransitionToError = false;
    result.shouldAttemptSlowRecovery = false;
    result.shouldContinueWithWarning = false;
    result.errorMessage = "";
    return result;
}

//! ERROR TRANSITION RESULT - Critical error requiring ERROR state
CutMotorHomeErrorResult createErrorTransitionResult(const String& message) {
    CutMotorHomeErrorResult result;
    result.wasHomeDetected = false;
    result.shouldTransitionToError = true;
    result.shouldAttemptSlowRecovery = false;
    result.shouldContinueWithWarning = false;
    result.errorMessage = message;
    return result;
}

//! WARNING RESULT - Issue detected but operation can continue
CutMotorHomeErrorResult createWarningOnlyResult(const String& message) {
    CutMotorHomeErrorResult result;
    result.wasHomeDetected = false;
    result.shouldTransitionToError = false;
    result.shouldAttemptSlowRecovery = false;
    result.shouldContinueWithWarning = true;
    result.errorMessage = message;
    return result;
}

// ========================================================================
//! REAL-TIME HOME SENSOR MONITORING SYSTEM
// ========================================================================

//! REAL-TIME CUT MOTOR HOME DETECTION DURING Yes_2x4 RETURN
//! This function provides continuous monitoring of the cut motor home sensor
//! during Yes_2x4 return sequences, implementing controlled deceleration
//! instead of abrupt stops for reliable sensor engagement.
void performCutMotorRealTimeHomeSensorCheck(FastAccelStepper* cutMotor, Bounce& cutHomingSwitch, bool& cutMotorInYes2x4Return) {
    
    //! STATE MACHINE FOR CONTROLLED HOME DETECTION
    static enum {
        MONITORING,           // Normal monitoring state - watching for sensor activation
        DECELERATING,        // Sensor detected, motor decelerating within safety distance  
        WAITING_FOR_DELAY,   // Motor stopped, waiting for sensor stabilization
        VERIFYING_SENSOR     // Verification delay complete, confirming sensor state
    } realTimeCheckState = MONITORING;
    
    static unsigned long verificationDelayStartTime = 0;
    
    //! SAFETY DISTANCE AND TIMING CONSTANTS
    const float DECELERATION_DISTANCE_INCHES = 0.2; // Maximum 0.2 inch deceleration distance
    const unsigned long SENSOR_VERIFICATION_DELAY_MS = 30; // 30ms sensor stabilization delay
    
    //! ONLY ACTIVE DURING Yes_2x4 RETURN SEQUENCES
    if (cutMotorInYes2x4Return && cutMotor) {
        switch (realTimeCheckState) {
            
            //! MONITORING PHASE - Watch for home sensor activation during movement
            case MONITORING:
                if (cutMotor->isRunning() && cutHomingSwitch.read() == HIGH) {
                    //! HOME SENSOR DETECTED DURING MOVEMENT!
                    Serial.println("REAL-TIME DETECTION: Cut motor hit homing sensor during Yes_2x4 return - beginning controlled deceleration...");
                    
                    //! Calculate safe target position for controlled stop
                    // Move further toward home (more negative) to ensure sensor is firmly pressed
                    long currentPosition = cutMotor->getCurrentPosition();
                    long targetPosition = currentPosition - (DECELERATION_DISTANCE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                    
                    //! Set high deceleration for quick but controlled stop within safety distance
                    cutMotor->setAcceleration(30000); // High deceleration for quick stop within 0.2 inch
                    cutMotor->moveTo(targetPosition);
                    
                    Serial.print("Decelerating from position ");
                    Serial.print(currentPosition);
                    Serial.print(" to target position ");
                    Serial.print(targetPosition);
                    Serial.print(" (");
                    Serial.print(DECELERATION_DISTANCE_INCHES);
                    Serial.println(" inch max distance)");
                    
                    realTimeCheckState = DECELERATING;
                }
                break;
                
            //! DECELERATION PHASE - Monitor motor until it comes to controlled stop
            case DECELERATING:
                // Check if motor has completed controlled deceleration
                if (!cutMotor->isRunning()) {
                    // Motor has stopped after controlled deceleration, start verification delay
                    verificationDelayStartTime = millis();
                    realTimeCheckState = WAITING_FOR_DELAY;
                    Serial.println("Cut motor deceleration complete. Starting 30ms sensor verification delay...");
                }
                //? Continue monitoring sensor during deceleration in case it goes LOW
                if (cutHomingSwitch.read() == LOW) {
                    Serial.println("WARNING: Home sensor went LOW during deceleration - possible contact issue");
                }
                break;
                
            //! DELAY PHASE - Wait for sensor to stabilize before verification
            case WAITING_FOR_DELAY:
                // Wait for the 30ms verification delay to complete
                if (millis() - verificationDelayStartTime >= SENSOR_VERIFICATION_DELAY_MS) {
                    realTimeCheckState = VERIFYING_SENSOR;
                }
                break;
                
            //! VERIFICATION PHASE - Confirm sensor is still active after delay
            case VERIFYING_SENSOR:
                cutHomingSwitch.update(); // Ensure latest sensor reading
                if (cutHomingSwitch.read() == HIGH) {
                    //! SUCCESSFUL HOME DETECTION WITH STABLE CONTACT
                    cutMotor->setCurrentPosition(0); // Recalibrate position to home
                    cutMotorInYes2x4Return = false;  // Clear the Yes_2x4 return flag
                    Serial.println("SUCCESS: Home sensor verified as stable after 30ms delay.");
                    Serial.println("Cut motor position recalibrated to 0, Yes_2x4 return flag cleared.");
                } else {
                    //! FALSE TRIGGER OR INSUFFICIENT CONTACT
                    Serial.println("WARNING: Home sensor not active after 30ms verification delay.");
                    Serial.println("This was likely a false trigger or insufficient contact. Motor will continue movement.");
                    //? Note: cutMotorInYes2x4Return flag remains true so movement can continue
                }
                realTimeCheckState = MONITORING; // Return to monitoring state
                break;
        }
    } else {
        //! RESET STATE MACHINE when not in Yes_2x4 return mode
        if (realTimeCheckState != MONITORING) {
            Serial.println("Real-time check state reset - exiting Yes_2x4 return mode");
            realTimeCheckState = MONITORING;
        }
    }
}

// ========================================================================
//! MAIN HOME ERROR DETECTION AND RECOVERY SYSTEM  
// ========================================================================

//! PRIMARY CUT MOTOR HOME ERROR DETECTION WITH RECOVERY CAPABILITY
//! This is the main function that handles all cut motor home position verification
//! and implements slow recovery when initial detection fails.
CutMotorHomeErrorResult handleCutMotorHomeError(
    Bounce& cutHomingSwitch, 
    FastAccelStepper* cutMotor, 
    const String& contextDescription,
    bool allowSlowRecovery
) {
    bool sensorDetectedHome = false;
    Serial.print("ERROR DETECTION: Checking cut motor home position for context: ");
    Serial.println(contextDescription);
    
    // ====================================================================
    //! PHASE 1: INITIAL HOME VERIFICATION (3-try approach)
    // ====================================================================
    
    for (int attemptNumber = 1; attemptNumber <= 3; attemptNumber++) {
        delay(30);  // Brief delay for sensor stabilization
        cutHomingSwitch.update();
        Serial.print("Initial home verification attempt ");
        Serial.print(attemptNumber);
        Serial.print(" of 3: ");
        Serial.println(cutHomingSwitch.read() == HIGH ? "HOME DETECTED" : "NO HOME");
        
        if (cutHomingSwitch.read() == HIGH) {
            sensorDetectedHome = true;
            if (cutMotor) {
                cutMotor->setCurrentPosition(0); // Recalibrate position to absolute zero
            }
            Serial.print("SUCCESS: Cut motor home position confirmed on initial check for ");
            Serial.println(contextDescription);
            return createSuccessResult();
        }
    }
    
    // ====================================================================
    //! PHASE 2: SLOW RECOVERY ATTEMPT (if enabled for this context)
    // ====================================================================
    
    if (!sensorDetectedHome && allowSlowRecovery) {
        Serial.println("INITIATING SLOW RECOVERY: Moving cut motor slowly back to home at homing speed...");
        
        if (cutMotor) {
            //! CONFIGURE MOTOR FOR RECOVERY MOVEMENT
            cutMotor->setSpeedInHz(CUT_MOTOR_HOME_RECOVERY_SPEED);
            cutMotor->setAcceleration(10000); // Moderate acceleration for controlled movement
            
            //! START RECOVERY MOVEMENT (backward toward home)
            cutMotor->runBackward();
            
            unsigned long recoveryStartTime = millis();
            bool homeFoundDuringRecovery = false;
            
            Serial.print("Slow recovery started at homing speed (");
            Serial.print(CUT_MOTOR_HOME_RECOVERY_SPEED);
            Serial.println(" steps/sec) with 5-second timeout...");
            
            //! MONITOR FOR HOME SENSOR DETECTION DURING RECOVERY
            while ((millis() - recoveryStartTime) < CUT_MOTOR_HOME_RECOVERY_TIMEOUT_MS) {
                cutHomingSwitch.update();
                
                if (cutHomingSwitch.read() == HIGH) {
                    //! HOME SENSOR DETECTED DURING RECOVERY!
                    cutMotor->forceStopAndNewPosition(0);
                    homeFoundDuringRecovery = true;
                    
                    unsigned long recoveryDuration = millis() - recoveryStartTime;
                    Serial.print("SUCCESS: Home sensor detected during slow recovery after ");
                    Serial.print(recoveryDuration);
                    Serial.println(" ms. Cut motor position recalibrated to 0.");
                    
                    String successMessage = "Recovery successful for " + contextDescription + 
                                          " after " + String(recoveryDuration) + " ms";
                    return createSuccessResult();
                }
                
                // Small delay to prevent excessive sensor polling
                delay(10);
            }
            
            //! RECOVERY TIMEOUT - Stop motor and report error
            cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
            
            if (!homeFoundDuringRecovery) {
                String timeoutErrorMessage = String("CRITICAL ERROR: Recovery timeout after 5 seconds. ") +
                                           String("Cut motor failed to find home position during slow recovery for context: ") + 
                                           contextDescription;
                Serial.println(timeoutErrorMessage);
                return createErrorTransitionResult(timeoutErrorMessage);
            }
        } else {
            String motorErrorMessage = "CRITICAL ERROR: Cut motor object is null during recovery for context: " + contextDescription;
            Serial.println(motorErrorMessage);
            return createErrorTransitionResult(motorErrorMessage);
        }
    } 
    // ====================================================================
    //! PHASE 3: HANDLE CASES WHERE SLOW RECOVERY IS DISABLED
    // ====================================================================
    
    else if (!allowSlowRecovery) {
        //? Slow recovery not allowed for this context (currently only used for NO_WOOD sequences)
        String warningMessage = String("WARNING: Cut motor home sensor did not detect home for ") + contextDescription + 
                              String(", but slow recovery disabled. Proceeding with warning.");
        Serial.println(warningMessage);
        return createWarningOnlyResult(warningMessage);
    }
    
    // ====================================================================
    //! FINAL FALLBACK - All detection and recovery attempts failed
    // ====================================================================
    
    String finalErrorMessage = String("FAILED: Cut motor home sensor did not detect home position after 3 attempts. ") +
                             String("Context: ") + contextDescription;
    Serial.println(finalErrorMessage);
    return createErrorTransitionResult(finalErrorMessage);
}

// ========================================================================
//! ERROR STATE TRANSITION HANDLER
// ========================================================================

//! EXECUTE COMPLETE ERROR STATE TRANSITION WITH SAFETY MEASURES
//! This function handles the transition to ERROR state when cut motor home
//! detection fails, implementing all necessary safety measures.
void executeCutMotorErrorStateTransition(
    FastAccelStepper* cutMotor,
    FastAccelStepper* positionMotor,
    SystemState& currentState,
    int& cuttingStep,
    int& cuttingSubStep7,
    int& fixPositionStep,
    int& fixPositionSubStep2,
    unsigned long& errorStartTime,
    bool shouldExtend2x4SecureClamp
) {
    Serial.println("EXECUTING CUT MOTOR ERROR STATE TRANSITION");
    
    //! IMMEDIATE MOTOR SAFETY - Stop all movement immediately
    if (cutMotor) {
        cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
        Serial.println("Cut motor stopped and position locked.");
    }
    if (positionMotor) {
        positionMotor->forceStopAndNewPosition(positionMotor->getCurrentPosition());
        Serial.println("Position motor stopped and position locked.");
    }
    
    //! EXTEND SAFETY CLAMPS - Secure all mechanical systems
    extendFeedClamp();  // Always extend feed clamp for safety
    if (shouldExtend2x4SecureClamp) {
        extend2x4SecureClamp();
        Serial.println("2x4 secure clamp extended for safety.");
    }
    
    //! SET ERROR INDICATION LEDS - Visual status indicators
    turnRedLedOn();      // Red = Error condition
    turnYellowLedOff();  // Yellow off = Operation stopped
    Serial.println("Error LEDs activated (Red ON, Yellow OFF).");
    
    //! TRANSITION TO ERROR STATE
    currentState = ERROR;
    errorStartTime = millis();
    
    //! RESET ALL STATE MACHINE COUNTERS - Clean slate for restart
    cuttingStep = 0;
    cuttingSubStep7 = 0;
    fixPositionStep = 0;
    fixPositionSubStep2 = 0;
    
    Serial.println("System transitioned to ERROR state due to cut motor home detection failure.");
    Serial.println("User must acknowledge error with reload switch to continue.");
}

// ========================================================================
//! RESULT LOGGING AND REPORTING
// ========================================================================

//! LOG CUT MOTOR HOME ERROR RESULTS FOR DEBUGGING AND MONITORING
void logCutMotorHomeErrorResult(const CutMotorHomeErrorResult& result) {
    if (!result.errorMessage.isEmpty()) {
        Serial.print("Cut Motor Home Error Handler Result: ");
        Serial.println(result.errorMessage);
    }
    
    //! VISUAL STATUS INDICATORS FOR SERIAL MONITOR
    if (result.wasHomeDetected) {
        Serial.println("✓ Cut motor home position successfully verified.");
    } else if (result.shouldAttemptSlowRecovery) {
        Serial.println("→ Slow recovery at homing speed will be attempted.");
    } else if (result.shouldTransitionToError) {
        Serial.println("✗ ERROR STATE TRANSITION REQUIRED.");
    } else if (result.shouldContinueWithWarning) {
        Serial.println("⚠ Continuing with warning - monitor system closely.");
    }
} 