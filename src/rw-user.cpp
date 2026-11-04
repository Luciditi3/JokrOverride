#include "rw-config.h"
#include "screen.h"

#include <cerrno>
#include "pros/error.h"

// Modify autonomous, driver, or pre-auton code below. src/main.cpp calls these.

void runAutonomous() {
  fourPin();
}

// Controller sticks read -127 to 127; RW's driver functions take -100 to 100
double stick(pros::controller_analog_e_t axis) {
  return controller_1.get_analog(axis) * 100.0 / 127.0;
}

void runDriver() {
  stopChassis(pros::MotorBrake::coast);
  heading_correction = false;
  while (true) {
    driverArcade(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y), stick(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    pros::delay(10);
  }
}

void runPreAutonomous() {
  // Calibrate the inertial sensor and wait for it (about 2 seconds)
  if (inertial_sensor.reset(true) == PROS_ERR) {
    robot_screen::showError("IMU calibration", errno);
  }

  loadDriveAutotuneSettings();

  // X-drive or mecanum: useHolonomicDrive(left_front, right_front, left_back, right_back);
  // H-drive: useHDrive(strafe_motor);   (see the docs: Guide > Drive Types)

  // odom tracking (picks the setup from using_horizontal_tracker / using_vertical_tracker)
  startOdometry();

  // Optional: MCL uses the distance sensors to keep odometry from drifting while driving.
  // Needs field-centered coordinates, so start each auton with setPose() at the real field position.
  //startMcl();

  // The competition autonomous routine is selected directly in main.cpp.
}
  