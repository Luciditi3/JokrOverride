#include "screen.h"

#include "main.h"
#include "lift.h"
#include "rw-config.h"

#include <atomic>
#include <cerrno>
#include <cstdarg>
#include <cstdio>

namespace robot_screen {
namespace {
std::atomic<bool> tuning_display_active{false};
std::atomic<bool> error_status_read_failure_reported{false};
pros::Mutex error_mutex;
char latest_error[96] = "";

void drawTelemetry() {
	pros::screen::print(pros::E_TEXT_MEDIUM, 0,
	                    "POSE X %.1f Y %.1f in  T %.1f deg          ",
	                    x_pos, y_pos, getInertialHeading());
	pros::screen::print(pros::E_TEXT_MEDIUM, 1, "LIFT %.1f deg                    ",
	                    lift.get_position());
	pros::screen::print(pros::E_TEXT_MEDIUM, 2, "CLAW DIST --                    ");
	pros::screen::print(pros::E_TEXT_MEDIUM, 3, "DIST F --  L --  R --  B --     ");
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, "DRIVE L %.0f  R %.0f rpm        ",
	                    left_chassis.get_actual_velocity(),
	                    right_chassis.get_actual_velocity());
	pros::screen::print(pros::E_TEXT_MEDIUM, 5, "TRACK H %.1f  V %.1f deg        ",
	                    horizontal_tracker.get_position() / 100.0,
	                    vertical_tracker.get_position() / 100.0);

	char error[sizeof(latest_error)];
	if (error_mutex.take()) {
		std::snprintf(error, sizeof(error), "%s", latest_error);
		if (!error_mutex.give()) {
			std::printf("telemetry: failed to release the error-status mutex (errno %d)\n", errno);
		} else if (error[0] != '\0') {
			pros::screen::print(pros::E_TEXT_MEDIUM, 9, "ERROR: %.35s", error);
		}
		error_status_read_failure_reported.store(false);
	} else if (!error_status_read_failure_reported.exchange(true)) {
		std::printf("telemetry: failed to read the latest error status (errno %d)\n", errno);
	}
}
}

void startTelemetry() {
	static const auto telemetry_task = pros::Task::create([] {
		while (true) {
			if (!tuningActive()) drawTelemetry();
			pros::delay(200);
		}
	}, "Robot telemetry");

	if (telemetry_task == nullptr) {
		showError("telemetry task", errno);
	}
}

void showError(const char* message, int error_code) {
	char formatted[sizeof(latest_error)];
	if (error_code != 0) {
		std::snprintf(formatted, sizeof(formatted), "%s (errno %d)", message, error_code);
	} else {
		std::snprintf(formatted, sizeof(formatted), "%s", message);
	}
	std::printf("ERROR: %s\n", formatted);

	if (error_mutex.take()) {
		std::snprintf(latest_error, sizeof(latest_error), "%s", formatted);
		if (!error_mutex.give()) {
			std::printf("showError: failed to release the error-status mutex (errno %d)\n", errno);
		}
	} else {
		std::printf("showError: failed to store the latest error (errno %d)\n", errno);
	}

	pros::screen::print(pros::E_TEXT_MEDIUM, 9, "ERROR: %.35s", formatted);
}

void setTuningActive(bool active) {
	tuning_display_active.store(active);
}

bool tuningActive() {
	return tuning_display_active.load();
}

void clearTuning() {
	setTuningActive(false);
}

void clear() {
	pros::screen::erase();
}

void printLine(std::uint8_t line, const char* format, ...) {
	char text[128];
	va_list args;
	va_start(args, format);
	std::vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	pros::screen::print(pros::E_TEXT_MEDIUM, line, "%s", text);
}

void printDistanceLine(std::uint8_t line, const char* label, std::int32_t millimeters) {
	if (millimeters >= 0 && millimeters < 9999) {
		printLine(line, "%s: %.2f in", label, millimeters / 25.4);
	} else {
		printLine(line, "%s: --", label);
	}
}

}
