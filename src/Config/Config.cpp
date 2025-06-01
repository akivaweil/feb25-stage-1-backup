#include "Config/Config.h"

//* ************************************************************************
//* ************************ SYSTEM CONFIGURATION ************************
//* ************************************************************************
// Configuration constants for the Automated Table Saw - Stage 1
// Motor settings, servo positions, timing, and operational parameters

//* ************************************************************************
//* ************************ SERVO CONFIGURATION **************************
//* ************************************************************************
// Rotation servo position settings
const int ROTATION_SERVO_HOME_POSITION = 24;     // Home position (degrees)
const int ROTATION_SERVO_ACTIVE_POSITION = 90;   // Position when activated (degrees)

//* ************************************************************************
//* ************************ MOTOR CONFIGURATION **************************
//* ************************************************************************
// Motor step calculations and travel distances
const int CUT_MOTOR_STEPS_PER_INCH = 500;  // 4x increase from 38
const int FEED_MOTOR_STEPS_PER_INCH = 1000; // Steps per inch for feed motor
const float CUT_TRAVEL_DISTANCE = 9.0; // inches
const float FEED_TRAVEL_DISTANCE = 3.4; // inches
const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES = 0.1; // Inches for incremental reverse
const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES = 0.4; // Max inches for incremental reverse before error

// Motor homing direction constants
const int CUT_HOMING_DIRECTION = -1;
const int FEED_HOMING_DIRECTION = 1;

//* ************************************************************************
//* ************************ CUT MOTOR SPEED SETTINGS ********************
//* ************************************************************************
// Normal Cutting Operation (Cutting State)
const float CUT_MOTOR_NORMAL_SPEED = 700;      // Speed for the cutting pass (steps/sec)
const float CUT_MOTOR_NORMAL_ACCELERATION = 10000; // Acceleration for the cutting pass (steps/sec^2)

// Return Stroke (Returning State / End of Cutting State)
const float CUT_MOTOR_RETURN_SPEED = 20000;     // Speed for returning after a cut (steps/sec)

// Homing Operation (Homing State)
const float CUT_MOTOR_HOMING_SPEED = 1000;      // Speed for homing the cut motor (steps/sec)

//* ************************************************************************
//* ************************ FEED MOTOR SPEED SETTINGS *******************
//* ************************************************************************
// Normal Feed Operation (Feed State / Parts of Cutting State)
const float FEED_MOTOR_NORMAL_SPEED = 17000;    // Speed for normal feed moves (steps/sec)
const float FEED_MOTOR_NORMAL_ACCELERATION = 20000; // Acceleration for normal feed (steps/sec^2)

// Return to Home/Start (Returning State / End of Cutting State / Homing after initial move)
const float FEED_MOTOR_RETURN_SPEED = 20000;    // Speed for returning to home or start position (steps/sec)
const float FEED_MOTOR_RETURN_ACCELERATION = 20000; // Acceleration for return moves (steps/sec^2)

// Homing Operation (Homing State)
const float FEED_MOTOR_HOMING_SPEED = 2000;     // Speed for homing the feed motor (steps/sec)

//* ************************************************************************
//* ************************ TIMING CONFIGURATION *************************
//* ************************************************************************
// Servo timing configuration
const unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS = 2700;

// Rotation clamp timing
const unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS = 1200; // 1.2 seconds

// Cut motor homing timeout
const unsigned long CUT_HOME_TIMEOUT = 5000; // 5 seconds timeout

// Transfer Arm signal timing
const unsigned long TA_SIGNAL_DURATION = 2000; // Duration for Transfer Arm signal (ms)

//* ************************************************************************
//* ************************ OPERATIONAL CONSTANTS ***********************
//* ************************************************************************
// Rotation clamp early activation offset
const float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES = 1.25; 

// Rotation servo early activation offset
const float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES = .3; 