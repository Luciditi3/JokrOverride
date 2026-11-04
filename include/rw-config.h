#pragma once
#include "rw/api.h"

// Your own devices (besides the ones RW uses, which are in rw/devices.h), defined in src/rw-config.cpp
extern pros::Motor intake_motor;
extern pros::Motor side_toggle;
extern pros::MotorGroup lift;
extern pros::Distance intake_distance;

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
