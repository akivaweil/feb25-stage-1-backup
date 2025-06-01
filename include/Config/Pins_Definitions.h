#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
extern const int CUT_MOTOR_STEP_PIN;         // Step pulse signal for cutting motor
extern const int CUT_MOTOR_DIR_PIN;          // Direction control for cutting motor
extern const int WOOD_FEED_MOTOR_STEP_PIN;   // Step pulse signal for wood feeding motor (pushes wood forward for angled cuts)
extern const int WOOD_FEED_MOTOR_DIR_PIN;    // Direction control for wood feeding motor

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
extern const int CATCHER_SERVO_PIN;

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
// Homing switches (Active HIGH - input pulldown)
extern const int CUT_MOTOR_HOME_SWITCH;
extern const int WOOD_FEED_MOTOR_HOME_SWITCH;

// Control switches (Active HIGH - input pulldown)
extern const int RELOAD_SWITCH;
extern const int START_CYCLE_SWITCH;
extern const int MANUAL_FEED_SWITCH;         // Manual wood feed control
extern const int FEED_POSITION_LOCK_BUTTON;  // Lock wood feed position

// Sensors (Active LOW - input pullup)
extern const int WOOD_PRESENT_SENSOR;
extern const int WOOD_SUCTION_CONFIRM_SENSOR;  // Confirms wood is secured by suction

//* ************************************************************************
//* ************************ CLAMP PINS ***********************************
//* ************************************************************************
// Pneumatic clamp control pins (HIGH = extend, LOW = retract)
extern const int WOOD_FEED_CLAMP_RELAY;      // Clamps wood during feed positioning
extern const int WOOD_SECURE_CLAMP_RELAY;    // Secures wood during cutting
extern const int CATCHER_CLAMP_RELAY;        // Clamps cut pieces in catcher

//* ************************************************************************
//* ************************ SIGNAL PINS **********************************
//* ************************************************************************
// Communication pins for external systems
extern const int TRANSFER_ARM_SIGNAL_PIN;  // Signal to Transfer Arm system

//* ************************************************************************
//* ************************ LED PINS *************************************
//* ************************************************************************
// Status indication LEDs
extern const int STATUS_LED_RED;      // Error/fault indication
extern const int STATUS_LED_YELLOW;   // Warning/caution indication
extern const int STATUS_LED_GREEN;    // Ready/operation OK indication
extern const int STATUS_LED_BLUE;     // Process active indication

#endif // PIN_DEFINITIONS_H 