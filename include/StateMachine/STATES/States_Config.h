#ifndef STATES_CONFIG_H
#define STATES_CONFIG_H

//* ************************************************************************
//* ************************ STATES CONFIGURATION ************************
//* ************************************************************************

// Servo Configuration
extern const int ROTATION_SERVO_HOME_POSITION;
extern const int ROTATION_SERVO_ACTIVE_POSITION;

// Motor Configuration
extern const int CUT_MOTOR_STEPS_PER_INCH;
extern const int FEED_MOTOR_STEPS_PER_INCH;
extern const float CUT_TRAVEL_DISTANCE;
extern const float FEED_TRAVEL_DISTANCE;
extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES;
extern const int CUT_HOMING_DIRECTION;
extern const int FEED_HOMING_DIRECTION;

// Cut Motor Speed Settings
extern const float CUT_MOTOR_NORMAL_SPEED;
extern const float CUT_MOTOR_NORMAL_ACCELERATION;
extern const float CUT_MOTOR_RETURN_SPEED;
extern const float CUT_MOTOR_HOMING_SPEED;

// Feed Motor Speed Settings
extern const float FEED_MOTOR_NORMAL_SPEED;
extern const float FEED_MOTOR_NORMAL_ACCELERATION;
extern const float FEED_MOTOR_RETURN_SPEED;
extern const float FEED_MOTOR_RETURN_ACCELERATION;
extern const float FEED_MOTOR_HOMING_SPEED;

// Timing Configuration
extern const unsigned long ROTATION_SERVO_ACTIVE_HOLD_DURATION_MS;
extern const unsigned long ROTATION_CLAMP_EXTEND_DURATION_MS;
extern const unsigned long CUT_HOME_TIMEOUT;
extern const unsigned long TA_SIGNAL_DURATION;

// Operational Constants
extern const float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES;
extern const float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES;

#endif // STATES_CONFIG_H 