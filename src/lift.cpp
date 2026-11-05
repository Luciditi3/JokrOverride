#include "lift.h"
#include "claw.h"
#include "rw-config.h"
#include "screen.h"
#include "pros/error.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>

std::array<double, 7> lift_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double lift_kp = 25.0;
double lift_kd = 1.5;
double lift_hold_mv = 1500.0;

namespace {
pros::Mutex target_mutex;
double target_degrees;
double voltage_limit = 12000.0;
std::int32_t manual_voltage = 0;
bool raising_to_checkpoint = false;
bool scoring_in_progress = false;
bool gravity_hold_enabled = false;
std::atomic<bool> driver_lift_override{false};
constexpr double intake_nudge_degrees = 10.0;
constexpr double lift_max_degrees = 135.0;
constexpr double raise_rate_degrees_per_second = 45.0;

double clampLiftTarget(double target) {
	return std::max(lift_positions[0], std::min(target, lift_max_degrees));
}

void liftControlTask(void*) {
	while (true) {
		if (pros::competition::is_disabled()) {
			const double position = lift.get_position();
			target_mutex.take();
			if (std::isfinite(position)) {
				target_degrees = clampLiftTarget(position);
			}
			manual_voltage = 0;
			driver_lift_override.store(false);
			raising_to_checkpoint = false;
			scoring_in_progress = false;
			gravity_hold_enabled = false;
			target_mutex.give();
			lift.move_voltage(0);
			pros::delay(20);
			continue;
		}

		target_mutex.take();
		const double current_position = lift.get_position();
		if (raising_to_checkpoint && std::isfinite(current_position) &&
		    target_degrees < lift_max_degrees) {
			target_degrees = std::min(
				lift_max_degrees,
				std::max(target_degrees, current_position) + raise_rate_degrees_per_second * 0.01);
		}
		const double target = target_degrees;
		const double limit = voltage_limit;
		const std::int32_t manual = manual_voltage;
		target_mutex.give();

		if (manual != 0) {
			lift.move_voltage(manual);
			pros::delay(10);
			continue;
		}

		const double position = lift.get_position();
		const double velocity = lift.get_actual_velocity();
		if (!std::isfinite(position) || !std::isfinite(velocity)) {
			lift.move_voltage(0);
			pros::delay(10);
			continue;
		}

		double output = static_cast<double>(manual);
		if (manual == 0) {
			const double error = target - position;
			const double velocity_degrees_per_second = velocity * 6.0;
			const double hold = gravity_hold_enabled && target > lift_positions[0] ? lift_hold_mv : 0.0;
			output = std::max(
				-limit,
				std::min(lift_kp * error - lift_kd * velocity_degrees_per_second + hold, limit));
		}
		if ((output > 0.0 && position >= lift_max_degrees) ||
		    (output < 0.0 && position <= lift_positions[0])) {
			output = 0.0;
		}
		lift.move_voltage(static_cast<std::int32_t>(output));
		pros::delay(10);
	}
}
}

void initializeLift() {
	if (lift.set_brake_mode_all(pros::MotorBrake::hold) == PROS_ERR) {
		robot_screen::showError("lift brake mode", errno);
	}

	if (lift.tare_position_all() == PROS_ERR) {
		robot_screen::showError("lift tare", errno);
	}

	target_mutex.take();
	const double position = lift.get_position();
	if (!std::isfinite(position)) {
		robot_screen::showError("invalid lift encoder");
		target_degrees = 0.0;
		gravity_hold_enabled = false;
	} else {
		target_degrees = clampLiftTarget(position);
		gravity_hold_enabled = position > lift_positions[0];
	}
	manual_voltage = 0;
	driver_lift_override.store(false);
	raising_to_checkpoint = false;
	scoring_in_progress = false;
	target_mutex.give();

	static const auto controller = pros::Task::create(
		[] { liftControlTask(nullptr); }, "Lift PD");
	if (controller == nullptr) {
		robot_screen::showError("lift task", errno);
	}
}

void moveTo(int presetIndex) {
	if (presetIndex < 0 || presetIndex >= static_cast<int>(lift_positions.size())) {
		return;
	}

	target_mutex.take();
	target_degrees = clampLiftTarget(lift_positions[presetIndex]);
	voltage_limit = 12000.0;
	manual_voltage = 0;
	raising_to_checkpoint = false;
	gravity_hold_enabled = target_degrees > lift_positions[0] &&
		target_degrees >= lift.get_position();
	target_mutex.give();
}

void raiseLift() {
	moveTo(static_cast<int>(lift_positions.size()) - 1);
}

void raiseSlightly() {
	target_mutex.take();
	if (scoring_in_progress) {
		target_mutex.give();
		return;
	}
	target_degrees = clampLiftTarget(lift.get_position() + intake_nudge_degrees);
	voltage_limit = 12000.0;
	manual_voltage = 0;
	raising_to_checkpoint = false;
	gravity_hold_enabled = true;
	target_mutex.give();
}

void slowDescent() {
	target_mutex.take();
	target_degrees = lift_positions[0];
	voltage_limit = 1500.0;
	manual_voltage = 0;
	raising_to_checkpoint = false;
	gravity_hold_enabled = false;
	target_mutex.give();
}

void setManualLiftVoltage(std::int32_t millivolts) {
	target_mutex.take();
	if (millivolts == 0) {
		driver_lift_override.store(false);
		const double position = lift.get_position();
		if (std::isfinite(position)) {
			target_degrees = position;
		}
		manual_voltage = 0;
		gravity_hold_enabled = std::isfinite(position) && target_degrees > lift_positions[0];
	} else {
		driver_lift_override.store(true);
		manual_voltage = millivolts > 12000 ? 12000 : (millivolts < -12000 ? -12000 : millivolts);
		raising_to_checkpoint = false;
		gravity_hold_enabled = false;
	}
	voltage_limit = 12000.0;
	target_mutex.give();
}

void beginLiftRaise() {
	target_mutex.take();
	target_degrees = clampLiftTarget(lift.get_position());
	manual_voltage = 0;
	voltage_limit = 12000.0;
	raising_to_checkpoint = true;
	gravity_hold_enabled = true;
	target_mutex.give();
}

void finishLiftRaise() {
	const double position = lift.get_position();
	double next_checkpoint = lift_max_degrees;
	for (const double checkpoint : lift_positions) {
		if (checkpoint > 0.0 && checkpoint > position && checkpoint < next_checkpoint) {
			next_checkpoint = checkpoint;
		}
	}

	target_mutex.take();
	target_degrees = clampLiftTarget(std::max(position, next_checkpoint));
	manual_voltage = 0;
	raising_to_checkpoint = false;
	voltage_limit = 12000.0;
	gravity_hold_enabled = target_degrees > lift_positions[0];
	target_mutex.give();
}

void score(bool driverRequest) {
	const double position = lift.get_position();
	if (!std::isfinite(position)) {
		robot_screen::showError("invalid lift encoder");
		return;
	}

	const double target = std::max(lift_positions[0], position - 5.0);
	target_mutex.take();
	if (driverRequest && driver_lift_override.load()) {
		target_mutex.give();
		return;
	}
	scoring_in_progress = true;
	target_degrees = target;
	manual_voltage = -1500;
	raising_to_checkpoint = false;
	gravity_hold_enabled = false;
	voltage_limit = 1500.0;
	target_mutex.give();
	stopClaw();

	const std::uint32_t start_time = pros::millis();
	bool encoder_fault = false;
	while (pros::millis() - start_time < 750) {
		if (pros::competition::is_disabled() ||
		    (driverRequest && (pros::competition::is_autonomous() ||
		                       driver_lift_override.load()))) {
			break;
		}
		const double current_position = lift.get_position();
		if (!std::isfinite(current_position)) {
			encoder_fault = true;
			break;
		}
		if (current_position <= target) {
			break;
		}
		pros::delay(10);
	}

	const bool canceled = pros::competition::is_disabled() ||
		(driverRequest && (pros::competition::is_autonomous() ||
		                   driver_lift_override.load()));
	if (canceled && driver_lift_override.load()) {
		target_mutex.take();
		scoring_in_progress = false;
		target_mutex.give();
		return;
	}
	const double final_position = lift.get_position();
	encoder_fault = encoder_fault || !std::isfinite(final_position);
	target_mutex.take();
	if (std::isfinite(final_position)) {
		target_degrees = clampLiftTarget(final_position);
	}
	manual_voltage = 0;
	gravity_hold_enabled = std::isfinite(final_position) && target_degrees > lift_positions[0];
	voltage_limit = 12000.0;
	if (canceled || encoder_fault) {
		scoring_in_progress = false;
	}
	target_mutex.give();
	if (encoder_fault) {
		robot_screen::showError("lift encoder during descent");
		return;
	}
	if (canceled) {
		return;
	}

	openClawFor(150);
	target_mutex.take();
	scoring_in_progress = false;
	target_mutex.give();

	if (driverRequest && !pros::competition::is_disabled() &&
	    !pros::competition::is_autonomous()) {
		pros::delay(250);
		if (!pros::competition::is_disabled() &&
		    !pros::competition::is_autonomous()) {
			reset();
		}
	}
}

void reset() {
	moveTo(0);
}
