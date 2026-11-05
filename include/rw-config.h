#pragma once
#include "rw/api.h"

// Your own devices (besides the ones RW uses, which are in rw/devices.h), defined in src/rw-config.cpp
// Uncomment a feature define and its device declaration in src/rw-config.cpp
// only after confirming the device's actual smart port.
// #define JOKR_HAS_INTAKE_MOTOR
// Also define JOKR_INTAKE_MOTOR_PORT to its wired smart port.
// #define JOKR_HAS_SIDE_TOGGLE
// Also define JOKR_SIDE_TOGGLE_MOTOR_PORT to its wired smart port.
#ifdef JOKR_HAS_INTAKE_MOTOR
extern pros::Motor intake_motor;
#endif
#ifdef JOKR_HAS_SIDE_TOGGLE
extern pros::Motor side_toggle;
#endif
extern pros::MotorGroup lift;

// Autonomous routines (src/rw-autons.cpp). Select the routine directly in main.cpp.
void exampleAuton();
void exampleAuton2();
void asyncExample();
void pathExample();
void profiledExample();
void boomerangTest();
void runTuningTest();
void runDriveAutotune();

//my autos
void fourPin();
void skills();
void loadDriveAutotuneSettings();

// Called from src/main.cpp (src/rw-user.cpp)
void runPreAutonomous();
void runAutonomous();
void runDriver();
