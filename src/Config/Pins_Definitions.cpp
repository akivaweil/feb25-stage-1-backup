#include "Config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ PIN DEFINITIONS *****************************
//* ************************************************************************
// Hardware pin assignments for the Automated Table Saw - Stage 1
// ESP32-S3 based system with stepper motors, servo, sensors, and switches

//* ************************************************************************
//* ************************ MOTOR PINS ***********************************
//* ************************************************************************
// Stepper motor control pins
const int CUT_MOTOR_STEP_PIN = 12;         // Step pulse signal for cutting motor
const int CUT_MOTOR_DIR_PIN = 11;          // Direction control for cutting motor
const int FEED_MOTOR_STEP_PIN = 17;   // Step pulse signal for feed motor (pushes wood forward for angled cuts)
const int FEED_MOTOR_DIR_PIN = 18;    // Direction control for feed motor

//* ************************************************************************
//* ************************ SERVO PINS ***********************************
//* ************************************************************************
// Servo control pins
const int CATCHER_SERVO_PIN = 14;

//* ************************************************************************
//* ************************ SWITCH & SENSOR PINS ************************
//* ************************************************************************
// Homing switches (Active HIGH - input pulldown)
const int CUT_MOTOR_HOME_SWITCH = 3;
const int FEED_MOTOR_HOME_SWITCH = 16;

// Control switches (Active HIGH - input pulldown)
const int RELOAD_SWITCH = 6;
const int START_CYCLE_SWITCH = 5;
const int MANUAL_FEED_SWITCH = 7;         // Manual wood feed control
const int FEED_POSITION_LOCK_BUTTON = 41; // Lock wood feed position

// Sensors (Active LOW - input pullup)
const int WOOD_PRESENT_SENSOR = 10;
const int WOOD_SUCTION_CONFIRM_SENSOR = 39;  // Confirms wood is secured by suction

//* ************************************************************************
//* ************************ CLAMP PINS ***********************************
//* ************************************************************************
// Pneumatic clamp control pins (HIGH = extend, LOW = retract)
const int FEED_CLAMP = 36;         // Clamps wood during feed positioning
const int WOOD_SECURE_CLAMP = 48;       // Secures wood during cutting
const int CATCHER_CLAMP = 42;          // Clamps cut pieces in catcher

//* ************************************************************************
//* ************************ SIGNAL PINS **********************************
//* ************************************************************************
// Communication pins for external systems
const int TRANSFER_ARM_SIGNAL_PIN = 8;  // Signal to Transfer Arm system

//* ************************************************************************
//* ************************ LED PINS *************************************
//* ************************************************************************
// Status indication LEDs
const int STATUS_LED_RED = 47;      // Error/fault indication
const int STATUS_LED_YELLOW = 21;   // Warning/caution indication
const int STATUS_LED_GREEN = 37;    // Ready/operation OK indication
const int STATUS_LED_BLUE = 19;     // Process active indication 