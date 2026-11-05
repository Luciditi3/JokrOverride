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
constexpr std::int32_t manual_lift_up_voltage = 11880;
constexpr std::int32_t manual_lift_down_voltage = 6000;

double readStick(pros::controller_analog_e_t axis) {
	return controller_1.get_analog(axis) * 100.0 / 127.0;
}

}

void initialize() {
	pros::delay(100);
#ifdef JOKR_HAS_SIDE_TOGGLE
	if (side_toggle.set_brake_mode(pros::MotorBrake::coast) == PROS_ERR) {
		robot_screen::showError("side toggle brake", errno);
	}
#endif
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
					score(true);
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
#ifdef JOKR_HAS_SIDE_TOGGLE
	side_toggle.brake();
#endif
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
#ifdef JOKR_HAS_SIDE_TOGGLE
	bool previous_r2 = false;
#endif
	bool manual_lift_active = false;
#ifdef JOKR_HAS_SIDE_TOGGLE
	bool side_toggle_active = false;
#endif
	bool scoring_was_active = false;
#ifdef JOKR_HAS_SIDE_TOGGLE
	std::uint32_t side_toggle_stop_time = 0;
#endif
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
#ifdef JOKR_HAS_SIDE_TOGGLE
		const bool r2 = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
#endif

		if (l1 || x || y) {
			setManualLiftVoltage((l1 || x) ? manual_lift_up_voltage
			                               : -manual_lift_down_voltage);
			manual_lift_active = true;
		} else {
			if (manual_lift_active) {
				setManualLiftVoltage(0);
				manual_lift_active = false;
			}
			if (!scoring_active.load() && b && !previous_b) {
				reset();
			}
		}
		if (l2 && !previous_l2) {
			bool expected = false;
			if (scoring_task_available.load() &&
			    scoring_active.compare_exchange_strong(expected, true)) {
				score_requested.store(true);
			}
		}

#ifdef JOKR_HAS_SIDE_TOGGLE
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
#endif

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
#ifdef JOKR_HAS_SIDE_TOGGLE
		previous_r2 = r2;
#endif
		pros::delay(10);
	}
}