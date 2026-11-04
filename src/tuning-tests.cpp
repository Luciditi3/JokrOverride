#include "rw-config.h"
#include "claw.h"
#include "lift.h"
#include "screen.h"

#include <cmath>
#include <cstdint>

namespace {
constexpr int tuning_test_count = 10;
// Set to a case from 1-10, then call runTuningTest() from autonomous() in main.cpp.
constexpr int tuning_test_number = 1;
constexpr double test_drive_distance = 12.0;

bool testCanceled() {
  return pros::competition::is_disabled() ||
         !pros::competition::is_autonomous() ||
         controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B);
}

void announceTest(int number, const char* title, const char* instructions) {
  robot_screen::clear();
  robot_screen::printLine(1, "TUNING TEST %02d / %02d", number, tuning_test_count);
  robot_screen::printLine(2, "%s", title);
  robot_screen::printLine(3, "%.42s", instructions);
}

void showDriveResult(double start_x, double start_y, double start_heading,
                     std::uint32_t start_time) {
  robot_screen::printLine(5, "Pose dX %.1f dY %.1f in", x_pos - start_x, y_pos - start_y);
  robot_screen::printLine(6, "Heading %.1f (start %.1f)", getInertialHeading(), start_heading);
  robot_screen::printLine(7, "Motion %lu ms", static_cast<unsigned long>(pros::millis() - start_time));
}

void runImuScaleTest() {
  if (!std::isfinite(imu_scale) || imu_scale <= 0) {
    robot_screen::showError("IMU scale must be positive");
    return;
  }

  robot_screen::printLine(4, "A: start; rotate CW one full turn.");
  robot_screen::printLine(5, "A: finish; B: cancel.");
  robot_screen::printLine(6, "Suggested scale is not auto-applied.");
  while (!testCanceled() &&
         !controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    pros::delay(20);
  }
  if (testCanceled()) return;

  const double start_rotation = inertial_sensor.get_rotation();
  if (!std::isfinite(start_rotation)) {
    robot_screen::showError("IMU rotation reading invalid");
    return;
  }
  while (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
         !testCanceled()) {
    pros::delay(10);
  }

  bool finished = false;
  while (!testCanceled()) {
    const double current_rotation = inertial_sensor.get_rotation();
    if (!std::isfinite(current_rotation)) {
      robot_screen::showError("IMU rotation reading invalid");
      return;
    }
    const double raw_delta = current_rotation - start_rotation;
    robot_screen::printLine(4, "Raw rotation: %.1f deg", raw_delta);
    if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      finished = true;
      const double proposed_scale = 360.0 / std::fabs(raw_delta);
      if (std::fabs(raw_delta) < 90.0 || !std::isfinite(proposed_scale) ||
          proposed_scale < 0.5 || proposed_scale > 1.5) {
        robot_screen::showError("IMU test invalid; rotate exactly 360 deg");
      } else {
        robot_screen::printLine(5, "Suggested imu_scale = %.5f", proposed_scale);
        robot_screen::printLine(6, "Copy to rw-config.cpp; not auto-saved.");
      }
      break;
    }
    pros::delay(50);
  }
  if (finished) {
    while (!testCanceled()) pros::delay(20);
  }
}

bool liftPresetConfigured() {
  return std::isfinite(lift_positions[0]) &&
         std::isfinite(lift_positions[1]) &&
         lift_positions[1] > lift_positions[0] &&
         lift_positions[1] <= 135.0;
}

void runLiftTest() {
  if (!liftPresetConfigured()) {
    robot_screen::printLine(4, "Preset 1 is unset or outside 0-135 deg.");
    robot_screen::printLine(5, "Set lift_positions[1] before testing.");
    return;
  }

  moveTo(1);
  const std::uint32_t start = pros::millis();
  while (!testCanceled() && pros::millis() - start < 3000) {
    const double position = lift.get_position();
    if (!std::isfinite(position)) {
      robot_screen::showError("lift encoder reading invalid");
      return;
    }
    robot_screen::printLine(4, "Target %.1f  actual %.1f deg",
                            lift_positions[1], position);
    if (std::fabs(lift_positions[1] - position) <= 3.0) break;
    pros::delay(20);
  }
  robot_screen::printLine(5, "Final lift %.1f deg", lift.get_position());
  robot_screen::printLine(6, "B: finish; inspect hold and sag.");
  while (!testCanceled()) pros::delay(20);
}
}

void runTuningTest() {
  robot_screen::setTuningActive(true);
  stopChassis(pros::MotorBrake::brake);

  if (tuning_test_number < 1 || tuning_test_number > tuning_test_count) {
    robot_screen::clear();
    robot_screen::showError("invalid tuning test number");
    robot_screen::clearTuning();
    return;
  }
  if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) {
    robot_screen::clear();
    robot_screen::printLine(1, "TUNING TEST NEEDS AUTON");
    robot_screen::clearTuning();
    return;
  }

  switch (tuning_test_number) {
    case 1:
      announceTest(1, "Stationary hardware inspection",
                   "Check ports, motor direction, tires, and clearances.");
      robot_screen::printLine(4, "Wheel scale %.4f in/rev", wheel_distance_in);
      robot_screen::printLine(5, "Track width %.2f in", distance_between_wheels);
      robot_screen::printLine(6, "IMU scale %.5f", imu_scale);
      robot_screen::printLine(7, "Check ports, direction, tires, and clearance.");
      robot_screen::printLine(8, "Fix hardware before tuning gains.");
      break;

    case 2: {
      announceTest(2, "Drive scale and odometry",
                   "Clear 15 in ahead and around robot; measure travel.");
      const double heading = getInertialHeading();
      setPose(0, 0, heading);
      const double start_x = x_pos;
      const double start_y = y_pos;
      const double start_left = getLeftRotationDegree();
      const double start_right = getRightRotationDegree();
      const std::uint32_t start_time = pros::millis();
      driveTo(test_drive_distance, 3000, true, 4);
      const double encoder_distance =
          ((getLeftRotationDegree() - start_left) +
           (getRightRotationDegree() - start_right)) * wheel_distance_in / 720.0;
      robot_screen::printLine(4, "Encoder %.1f in (target %.1f)",
                              encoder_distance, test_drive_distance);
      turnToAngle(heading + 90.0, 2500, true, 4);
      showDriveResult(start_x, start_y, heading, start_time);
      robot_screen::printLine(8, "Compare distance/pose to tape; adjust scale.");
      break;
    }

    case 3:
      announceTest(3, "IMU scale measurement",
                   "Press A, rotate clockwise exactly 360 degrees, press A.");
      runImuScaleTest();
      break;

    case 4:
      announceTest(4, "Distance sensor sanity check",
                   "Robot stationary and square to a measured wall.");
      robot_screen::printDistanceLine(4, "Front", front_sensor.get());
      robot_screen::printDistanceLine(5, "Left", left_sensor.get());
      robot_screen::printDistanceLine(6, "Right", right_sensor.get());
      robot_screen::printDistanceLine(7, "Back", back_sensor.get());
      robot_screen::printLine(8, "Compare used sensors to a tape.");
      break;

    case 5:
      announceTest(5, "PID + feedforward autotuner",
                   "A: tune drive PID; X: measure kS, kV, kA; B: cancel.");
      runDriveAutotune();
      break;

    case 6: {
      announceTest(6, "Profiles and path following",
                   "Clear 30 in ahead; robot should face field north.");
      setPose(0, 0, 0);
      driveProfiled(test_drive_distance, 3000, true, 5);
      robot_screen::printLine(4, "Profile end: X %.1f Y %.1f", x_pos, y_pos);
      setPose(0, 0, 0);
      {
        Path path = pathFromPoints({{0, 0}, {0, test_drive_distance}});
        followPath(path, 4000, false, true, 5, 6);
      }
      robot_screen::printLine(5, "Path end: X %.1f Y %.1f", x_pos, y_pos);
      robot_screen::printLine(6, "Tune one profile/path limit per run.");
      break;
    }

    case 7:
      announceTest(7, "Driver controls",
                   "Practice separately in driver mode; robot stays stopped.");
      robot_screen::printLine(4, "Curve %.1f  deadband %.1f",
                              driver_curve, driver_deadband);
      robot_screen::printLine(5, "Change curve OR deadband, not both.");
      robot_screen::printLine(6, "Check slow, medium, and full stick.");
      break;

    case 8:
      announceTest(8, "Lift position and hold",
                   "Preset 1 must be set safely; keep hands clear.");
      runLiftTest();
      break;

    case 9:
      announceTest(9, "Intake and side-toggle checks",
                   "Clear both mechanisms; keep hands away.");
      intake_motor.move_voltage(4000);
      pros::delay(250);
      intake_motor.brake();
      robot_screen::printLine(4, "Intake pulse complete; check direction.");
      if (!testCanceled()) {
        side_toggle.move_voltage(12000);
        pros::delay(100);
        side_toggle.brake();
      }
      robot_screen::printLine(5, "Toggle pulse complete; check clearance.");
      break;

    case 10:
      announceTest(10, "Match autonomous validation",
                   "Run the selected routine directly from main.cpp.");
      robot_screen::printLine(4, "Verify start pose and field clearance.");
      robot_screen::printLine(5, "Repeat at least 3 times; log failures.");
      robot_screen::printLine(6, "Tune only the failing segment.");
      break;

    default:
      robot_screen::showError("invalid tuning test number");
      break;
  }

  stopChassis(pros::MotorBrake::brake);
  if (tuning_test_number != 3 && tuning_test_number != 5 &&
      tuning_test_number != 8) {
    while (!testCanceled()) pros::delay(20);
  }
  robot_screen::clearTuning();
}
