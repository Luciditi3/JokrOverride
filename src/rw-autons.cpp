#include "rw-config.h"
#include "lift.h"

#include <cmath>
#include <cstdint>

namespace {
using PositionReset = bool (*)();

PositionReset first_position_reset = nullptr;
PositionReset second_position_reset = nullptr;
uint32_t position_reset_duration_ms = 0;

void samplePositionReset(PositionReset reset, double& x_sum, int& x_count, double& y_sum, int& y_count) {
  double previous_x = x_pos;
  double previous_y = y_pos;
  bool accepted = reset();
  double reset_x = x_pos;
  double reset_y = y_pos;
  x_pos = previous_x;
  y_pos = previous_y;
  if (!accepted) return;

  if (std::fabs(reset_x - previous_x) > 1e-6) {
    x_sum += reset_x;
    x_count++;
  }
  if (std::fabs(reset_y - previous_y) > 1e-6) {
    y_sum += reset_y;
    y_count++;
  }
}

void averagePositionResetsTask() {
  double x_sum = 0, y_sum = 0;
  int x_count = 0, y_count = 0;
  uint32_t start = pros::millis();

  while (pros::millis() - start < position_reset_duration_ms) {
    pros::delay(10);
    if (pros::millis() - start >= position_reset_duration_ms) break;
    samplePositionReset(first_position_reset, x_sum, x_count, y_sum, y_count);
    samplePositionReset(second_position_reset, x_sum, x_count, y_sum, y_count);
  }

  if (x_count > 0) x_pos = x_sum / x_count;
  if (y_count > 0) y_pos = y_sum / y_count;
}
}

void averagePositionResets(PositionReset first_reset, PositionReset second_reset, uint32_t duration_ms) {
  if (first_reset == nullptr || second_reset == nullptr || duration_ms == 0) return;
  first_position_reset = first_reset;
  second_position_reset = second_reset;
  position_reset_duration_ms = duration_ms;
  runAsync(averagePositionResetsTask);
}

// IMPORTANT: Remember to add respective function declarations to include/rw-config.h
// Select the routine directly in autonomous() in src/main.cpp.
// Format: returnType functionName() { code }

void exampleAuton() {
  // Use this for tuning linear and turn pid
  driveTo(60, 3000);
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  driveTo(-60, 3000);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

/*
 * asyncExample
 * Shows background motions: start a move, run the intake while it drives, and stop it early with a sensor.
 */
void asyncExample() {
  setPose(0, 0, 0);

  // Start driving in the background, then start the intake 12 inches into the move
  runAsync([]{ moveToPoint(0, 48, 1, 3000); });
  waitUntilTraveled(12);
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(12000);
#endif

  // Stop the move early once the intake sensor sees an object
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.brake();
#endif

  // Turn around and drive back
  runAsync([]{
    turnToAngle(180, 1500);
    moveToPoint(0, 0, 1, 3000);
  });
  waitForMotion();
}

/*
 * pathExample
 * Drives a smooth S-curve with pure pursuit, then backs along it with RAMSETE.
 * Paths use field coordinates, so the auton starts with setPose().
 */
void pathExample() {
  setPose(-36, -60, 0);
  Path out = pathFromPoints({{-36, -60}, {-36, -32}, {-12, -22}, {12, -30}, {34, -24}});
  followPath(out, 5000);
  Path back = pathFromPoints({{34, -24}, {12, -30}, {-12, -22}, {-36, -32}, {-36, -52}});
  followPathRamsete(back, 5000, true);
}

/*
 * profiledExample
 * Straight drives and turns with motion profiles: they speed up and slow down smoothly.
 */
void profiledExample() {
  driveProfiled(36, 3000);
  turnProfiled(90, 2000);
  driveProfiled(24, 3000);
  turnProfiled(0, 2000);
  driveProfiled(-36, 3000);
}

void fourPin() {
  setPose(-62, 0, 90);
#ifdef JOKR_HAS_SIDE_TOGGLE
  side_toggle.move_voltage(12000);
#endif
  pros::delay(500); // Wait for the side toggle to move

  //drive to the 
  driveProfiled(3, 200);
  runAsync([]{boomerang(-49, -17, 1, 165, 0.3, 1000); });
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(1000);
#endif

  // score once at the goal
  moveTo(1);
  waitForMotion();
  score();
  pros::delay(500);

  // back up
  runAsync([]{
  driveProfiled(-5, 400);
  turnToAngle(220, 500);
  });
  reset();
  //go to the inside pin
  driveProfiled(13, 750);
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif
  swing(260, 1, 350);
  swing(227, -1, 300);

  //leave and go to the goal
  driveProfiled(-17,600);
  runAsync([]{
    swing(180, -1, 500);
    driveProfiled(8, 800);
  });
  moveTo(2);
  waitForMotion();

  //score
  score();
  pros::delay(500);
  driveProfiled(-5, 400);
  reset();

  //turn to the stacks
  turnProfiled(125, 400);
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif
  driveProfiled(27, 1000);

  //go to the goal
  turnProfiled(268, 700);
  runAsync([]{
    driveProfiled(10, 700);
  });
  moveTo(3);
  waitForMotion();
  score();
  pros::delay(500);

  //back away and go to next pin
  runAsync([]{
    driveProfiled(-5,500);
    turnProfiled(215,300);
  });
  reset();
  driveProfiled(30,1200);
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif

  //go to score
  turnProfiled(0,800);
  runAsync([]{
    driveProfiled(12,800);
  });
  moveTo(4); 
  waitForMotion();
  score();

  //back into matchloader
  runAsync([]{
    driveProfiled(-5,500, false);
  });
  reset();
}//Everything is tested but the last pin

void skills() {
  fourPin();//I'm too lazy to rerun

  //matchload for pin 5
  runAsync([]{
    driveProfiled(-6,750);
  });
  reset();
  turnProfiled(20,200, false);
  driveProfiled(-11, 1000, false);
  turnProfiled(270, 600, false);
  runAsync([]{
  driveProfiled(10, 1000);
  });
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif

  //go back to the goal
  driveProfiled(-6, 500);
  swing(240, -1, 500, false);
  driveProfiled(-5, 500, false);
  turnProfiled(0, 700);
  runAsync([]{
  driveProfiled(20, 1000);
  });
  moveTo(5);
  waitForMotion();
  score();
  // averagePositionResets(resetPositionLeft, resetPositionBack, 500); // DISABLED for no-reset test
  pros::delay(500);

  //back to loader for pin 6
 runAsync([]{
    driveProfiled(-11,750);
  });
  reset();
  turnProfiled(20,200, false);
  driveProfiled(-12, 1000, false);
  turnProfiled(270, 600, false);
  runAsync([]{
  driveProfiled(10, 1000);
  });
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif

  //go back to the goal
  driveProfiled(-6, 500);
  swing(240, -1, 500, false);
  driveProfiled(-5, 500, false);
  turnProfiled(0, 700);
  runAsync([]{
  driveProfiled(20, 1000);
  });
  moveTo(6);
  waitForMotion();
  score();
  // averagePositionResets(resetPositionLeft, resetPositionBack, 500); // DISABLED for no-reset test
  pros::delay(500);

  //back to loader for start pin
   runAsync([]{
    driveProfiled(-11,750);
  });
  reset();
  turnProfiled(20,200, false);
  driveProfiled(-12, 1000, false);
  turnProfiled(270, 600, false);
  runAsync([]{
  driveProfiled(10, 1000);
  });
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif

  //go to next loader
  driveProfiled(-6, 500);
  turnProfiled(0,700);
  driveProfiled(55, 2500);
  swing(15,1,500);
  runAsync([]{
    driveProfiled(18, 1000);
  });
  moveTo(1);
  waitForMotion();
  score();
  pros::delay(500);

  //go to pin stack and work around 
  driveProfiled(-6,600);
  turnProfiled(330, 500);
  driveProfiled(11, 1000);
#ifdef JOKR_HAS_INTAKE_MOTOR
  intake_motor.move_voltage(7000);
#endif
  swing(270, 1, 1000);
  swing(0,-1, 1000);

  //go to goal
  driveProfiled(23, 1000);
  turnProfiled(135,1,1000);
  runAsync([]{
    driveProfiled(10, 1000);
  });
  moveTo(2);
  waitForMotion();
  score();
  pros::delay(500);

}