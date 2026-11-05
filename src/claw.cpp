#include "claw.h"
#include "lift.h"
#include "rw-config.h"
#include "screen.h"
#include "pros/error.h"

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>

namespace {
#ifdef JOKR_HAS_INTAKE_MOTOR
std::atomic<bool> intake_active{false};
#endif
}

void initializeClaw() {
#ifdef JOKR_HAS_INTAKE_MOTOR
	if (intake_motor.set_brake_mode(pros::MotorBrake::brake) == PROS_ERR) {
		robot_screen::showError("claw brake mode", errno);
	}
	stopClaw();

	static const auto intake_load_task = pros::Task::create([] {
		bool lift_triggered = false;
		int high_power_samples = 0;

		while (true) {
			const double intake_power_watts = intake_active.load() ? std::abs(intake_motor.get_power()) : 0.0;

			if (std::isfinite(intake_power_watts) && intake_power_watts >= 5.0) {
				if (high_power_samples < 3) {
					high_power_samples++;
				}
				if (!lift_triggered && high_power_samples >= 3) {
					raiseSlightly();
					lift_triggered = true;
				}
			} else {
				high_power_samples = 0;
				if (!std::isfinite(intake_power_watts) || intake_power_watts < 3.0) {
					lift_triggered = false;
				}
			}

			pros::delay(20);
		}
	}, "Claw load detection");
	if (intake_load_task == nullptr) {
		robot_screen::showError("claw task", errno);
	}
#endif
}

void intakeClaw() {
#ifdef JOKR_HAS_INTAKE_MOTOR
	intake_active.store(true);
	intake_motor.move_voltage(12000);
#endif
}

void openClaw() {
#ifdef JOKR_HAS_INTAKE_MOTOR
	intake_active.store(false);
	intake_motor.move_voltage(-12000);
#endif
}

void stopClaw() {
#ifdef JOKR_HAS_INTAKE_MOTOR
	intake_active.store(false);
	intake_motor.move_voltage(0);
	intake_motor.brake();
#endif
}

void openClawFor(std::uint32_t durationMs) {
#ifdef JOKR_HAS_INTAKE_MOTOR
	openClaw();
	const std::uint32_t startTime = pros::millis();
	while (pros::millis() - startTime < durationMs && !pros::competition::is_disabled()) {
		pros::delay(10);
	}
	stopClaw();
#else
	(void)durationMs;
#endif
}