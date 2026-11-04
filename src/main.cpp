#include "main.h"
#include "claw.h"
#include "lift.h"
#include "rw-config.h"
#include "screen.h"

#include <atomic>
#include <cerrno>
#include <cstdint>

namespace {
std::atomic<bool> score_requested{false};
std::atomic<bool> scoring_active{false};
std::atomic<bool> scoring_task_available{false};

constexpr pros::controller_analog_e_t drive_forward_axis = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::controller_analog_e_t drive_turn_axis = pros::E_CONTROLLER_ANALOG_RIGHT_X;
constexpr std::int32_t manual_lift_voltage = 9000;

double readStick(pros::controller_analog_e_t axis) {
	return controller_1.get_analog(axis) * 100.0 / 127.0;
}

}

void initialize() {
	pros::delay(100);
	if (side_toggle.set_brake_mode(pros::MotorBrake::coast) == PROS_ERR) {
		robot_screen::showError("side toggle brake", errno);
	}
	initializeLift();
	initializeClaw();
	runPreAutonomous();
	robot_screen::startTelemetry();

	static const auto scoring_task = pros::Task::create([] {
		while (true) {
			if (score_requested.exchange(false)) {
				if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
					scoring_active.store(false);
				} else {
					const bool completed = score(true);
					if (completed && !pros::competition::is_disabled() &&
					    !pros::competition::is_autonomous()) {
						pros::delay(250);
						if (!pros::competition::is_disabled() &&
						    !pros::competition::is_autonomous()) {
							reset();
						}
					}
					scoring_active.store(false);
				}
			}
			pros::delay(10);
		}
	}, "Scoring sequence");
	if (scoring_task == nullptr) {
		robot_screen::showError("scoring task", errno);
	} else {
		scoring_task_available.store(true);
	}
}

void disabled() {
	score_requested.store(false);
	scoring_active.store(false);
	stopChassis(pros::MotorBrake::brake);
	side_toggle.brake();
	stopClaw();
	lift.move_voltage(0);
}

void competition_initialize() {}

void autonomous() {
	runDriveAutotune();
}

void opcontrol() {
	stopChassis(pros::MotorBrake::coast);
	heading_correction = false;

	bool previous_b = false;
	bool previous_l2 = false;
	bool previous_r2 = false;
	bool manual_lift_active = false;
	bool l1_raise_active = false;
	bool side_toggle_active = false;
	bool scoring_was_active = false;
	std::uint32_t side_toggle_stop_time = 0;
	int claw_mode = 0;

	while (true) {
		driverArcade(readStick(drive_forward_axis), readStick(drive_turn_axis));

		const bool l1 = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		const bool x = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		const bool y = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
		const bool b = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B);
		const bool a = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		const bool l2 = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		const bool r1 = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		const bool r2 = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

		if (!scoring_active.load()) {
			if (x || y) {
				setManualLiftVoltage(x ? manual_lift_voltage : -manual_lift_voltage);
				manual_lift_active = true;
				l1_raise_active = false;
			} else {
				if (manual_lift_active) {
					setManualLiftVoltage(0);
					manual_lift_active = false;
				}
				if (l1 && !l1_raise_active) {
					beginLiftRaise();
					l1_raise_active = true;
				} else if (!l1 && l1_raise_active) {
					finishLiftRaise();
					l1_raise_active = false;
				}
			}

			if (b && !previous_b) {
				reset();
			}
		} else {
			manual_lift_active = false;
			l1_raise_active = false;
		}
		if (l2 && !previous_l2) {
			bool expected = false;
			if (scoring_task_available.load() &&
			    scoring_active.compare_exchange_strong(expected, true)) {
				score_requested.store(true);
			}
		}

		if (r2 && !previous_r2) {
			side_toggle.move_voltage(12000);
			side_toggle_active = true;
			side_toggle_stop_time = pros::millis() + 100;
		}
		if (side_toggle_active &&
		    static_cast<std::int32_t>(pros::millis() - side_toggle_stop_time) >= 0) {
			side_toggle.brake();
			side_toggle_active = false;
		}

		const bool currently_scoring = scoring_active.load();
		if (currently_scoring) {
			claw_mode = 0;
		} else {
			if (scoring_was_active) {
				claw_mode = 2;
			}
			const int requested_claw_mode = r1 ? 1 : (a ? -1 : 0);
			if (requested_claw_mode != claw_mode) {
				if (requested_claw_mode == 1) {
					intakeClaw();
				} else if (requested_claw_mode == -1) {
					openClaw();
				} else {
					stopClaw();
				}
				claw_mode = requested_claw_mode;
			}
		}
		scoring_was_active = currently_scoring;

		previous_b = b;
		previous_l2 = l2;
		previous_r2 = r2;
		pros::delay(10);
	}
}