#include "rw-config.h"
#include "screen.h"
#include "pros/misc.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
constexpr int repetitions = 2;
constexpr std::uint32_t movement_settle_ms = 250;
constexpr double drive_distance = 24.0;
constexpr double turn_angle = 90.0;
// Match RW's driveTo (0.5 in) and turnToAngle (1 degree) arrival thresholds.
constexpr double drive_tolerance = 0.5;
constexpr double turn_tolerance = 1.0;

enum class Parameter {
  distance_kp,
  distance_kd,
  turn_kp,
  turn_kd,
  heading_kp,
  heading_kd,
  drive_velocity_kp,
};

enum class SampleMode { distance, angle, heading, velocity };

struct Measurement {
  double final_error = 0;
  double position_error = 0;
  double overshoot = 0;
  double final_x = 0;
  double final_y = 0;
  double final_heading_error = 0;
  double mean_heading_error = 0;
  std::uint32_t duration_ms = 0;
  bool timed_out = false;
};

struct Capture {
  std::atomic<bool> active{false};
  std::atomic<bool> done{true};
  std::atomic<bool> sensor_fault{false};
  SampleMode mode = SampleMode::distance;
  double start_heading = 0;
  double target = 0;
  double direction = 1;
  double last_heading = 0;
  double last_left_degrees = 0;
  double last_right_degrees = 0;
  double last_horizontal_tracker_degrees = 0;
  double last_vertical_tracker_degrees = 0;
  double x = 0;
  double y = 0;
  double progress = 0;
  double max_progress = 0;
  double heading_error_sum = 0;
  double velocity_error_sum = 0;
  double velocity_error_max = 0;
  std::uint32_t samples = 0;
  std::uint32_t start_time = 0;
  Measurement result;
};

Capture capture;
std::atomic<bool> sampler_shutdown{false};
std::atomic<bool> abort_requested{false};
bool all_limits_met = true;
bool settings_saved = false;
double starting_heading = 0;
double distance_direction = 1;
double turn_direction = 1;
double heading_direction = 1;
double velocity_direction = 1;
constexpr char settings_file[] = "rw_drive_tuning.txt";

struct TuningSettings {
  double distance_kp;
  double distance_kd;
  double turn_kp;
  double turn_kd;
  double heading_kp;
  double heading_kd;
  double drive_velocity_kp;
  double drive_kS;
  double drive_kV;
  double drive_kA;
};

bool writeSettingsFile(const char* data, int length) {
  if (!pros::usd::is_installed()) return false;
  char path[64];
  const int path_length = std::snprintf(path, sizeof(path), "/usd/%s", settings_file);
  if (path_length <= 0 || path_length >= static_cast<int>(sizeof(path))) return false;
  std::FILE* file = std::fopen(path, "wb");
  if (file == nullptr) return false;
  const std::size_t written = std::fwrite(data, 1, static_cast<std::size_t>(length), file);
  const int close_result = std::fclose(file);
  return written == static_cast<std::size_t>(length) && close_result == 0;
}

int readSettingsFile(char* buffer, int max_length) {
  if (!pros::usd::is_installed()) return -1;
  char path[64];
  const int path_length = std::snprintf(path, sizeof(path), "/usd/%s", settings_file);
  if (path_length <= 0 || path_length >= static_cast<int>(sizeof(path))) return -1;
  std::FILE* file = std::fopen(path, "rb");
  if (file == nullptr) return -1;
  const std::size_t length = std::fread(buffer, 1, static_cast<std::size_t>(max_length), file);
  const bool read_failed = std::ferror(file) != 0;
  const int close_result = std::fclose(file);
  if (read_failed || close_result != 0) return -1;
  if (length == static_cast<std::size_t>(max_length)) return -2;
  return static_cast<int>(length);
}

double wrapDegrees(double degrees) {
  return std::remainder(degrees, 360.0);
}

double arcToChord(double wheel_delta, double heading_delta, double offset) {
  if (std::fabs(heading_delta) < 1e-6) return wheel_delta;
  return 2.0 * std::sin(heading_delta / 2.0) *
         (wheel_delta / heading_delta + offset);
}

void failCaptureForSensor() {
  if (!capture.sensor_fault.exchange(true)) {
    capture.active.store(false);
    abort_requested.store(true);
    cancelMotion();
    robot_screen::showError("autotune encoder or IMU reading invalid");
  }
}

void sampleOnce() {
  if (capture.sensor_fault.load()) return;
  const double heading = getInertialHeading();
  const double left_degrees = left_chassis.get_position();
  const double right_degrees = right_chassis.get_position();
  const double horizontal_tracker_degrees =
      using_horizontal_tracker ? horizontal_tracker.get_position() / 100.0 : 0;
  const double vertical_tracker_degrees =
      using_vertical_tracker ? vertical_tracker.get_position() / 100.0 : 0;
  if (!std::isfinite(heading) || !std::isfinite(left_degrees) ||
      !std::isfinite(right_degrees) ||
      !std::isfinite(horizontal_tracker_degrees) ||
      !std::isfinite(vertical_tracker_degrees)) {
    failCaptureForSensor();
    return;
  }
  const double heading_delta =
      wrapDegrees(heading - capture.last_heading) * M_PI / 180.0;
  const double left_delta =
      (left_degrees - capture.last_left_degrees) * wheel_distance_in / 360.0;
  const double right_delta =
      (right_degrees - capture.last_right_degrees) * wheel_distance_in / 360.0;
  const double horizontal_delta =
      (horizontal_tracker_degrees - capture.last_horizontal_tracker_degrees) *
      horizontal_tracker_diameter * M_PI / 360.0;
  const double vertical_delta =
      (vertical_tracker_degrees - capture.last_vertical_tracker_degrees) *
      vertical_tracker_diameter * M_PI / 360.0;
  const double local_x = using_horizontal_tracker
      ? arcToChord(horizontal_delta, heading_delta, horizontal_tracker_dist_from_center)
      : 0;
  const double local_y = using_vertical_tracker
      ? arcToChord(vertical_delta, heading_delta, vertical_tracker_dist_from_center)
      : (arcToChord(left_delta, heading_delta, -distance_between_wheels / 2.0) +
         arcToChord(right_delta, heading_delta, distance_between_wheels / 2.0)) / 2.0;
  const double midpoint_heading =
      (capture.last_heading + heading_delta * 180.0 / M_PI / 2.0) * M_PI / 180.0;
  capture.x += local_x * std::cos(midpoint_heading) +
               local_y * std::sin(midpoint_heading);
  capture.y += -local_x * std::sin(midpoint_heading) +
               local_y * std::cos(midpoint_heading);

  if (capture.mode == SampleMode::distance) {
    const double radians = capture.start_heading * M_PI / 180.0;
    capture.progress = capture.direction *
        (capture.x * std::sin(radians) + capture.y * std::cos(radians));
    capture.max_progress = std::max(capture.max_progress, capture.progress);
  } else if (capture.mode == SampleMode::angle) {
    capture.progress += capture.direction * wrapDegrees(heading - capture.last_heading);
    capture.max_progress = std::max(capture.max_progress, capture.progress);
  } else if (capture.mode == SampleMode::heading) {
    capture.heading_error_sum += std::fabs(wrapDegrees(capture.target - heading));
    ++capture.samples;
  } else if (capture.mode == SampleMode::velocity) {
    if (pros::millis() - capture.start_time >= 300) {
      const double left_rpm = left_chassis.get_actual_velocity();
      const double right_rpm = right_chassis.get_actual_velocity();
      if (!std::isfinite(left_rpm) || !std::isfinite(right_rpm)) {
        failCaptureForSensor();
        return;
      }
      const double left_speed = left_rpm * wheel_distance_in / 60.0;
      const double right_speed = right_rpm * wheel_distance_in / 60.0;
      const double speed_error =
          (std::fabs(capture.target - left_speed) + std::fabs(capture.target - right_speed)) / 2.0;
      capture.velocity_error_sum += speed_error;
      capture.velocity_error_max = std::max(capture.velocity_error_max, speed_error);
      ++capture.samples;
    }
  }

  capture.last_heading = heading;
  capture.last_left_degrees = left_degrees;
  capture.last_right_degrees = right_degrees;
  capture.last_horizontal_tracker_degrees = horizontal_tracker_degrees;
  capture.last_vertical_tracker_degrees = vertical_tracker_degrees;
}

void samplerTask() {
  bool was_active = false;
  while (!sampler_shutdown.load()) {
    if ((pros::competition::is_disabled() ||
         controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) &&
        !abort_requested.exchange(true)) {
      cancelMotion();
    }

    if (capture.active.load()) {
      was_active = true;
      sampleOnce();
    } else if (was_active) {
      capture.done.store(true);
      was_active = false;
    }
    pros::delay(10);
  }
}

void beginCapture(SampleMode mode, double target, double direction = 1) {
  capture.done.store(false);
  capture.sensor_fault.store(false);
  capture.mode = mode;
  capture.start_heading = getInertialHeading();
  capture.last_heading = capture.start_heading;
  capture.last_left_degrees = left_chassis.get_position();
  capture.last_right_degrees = right_chassis.get_position();
  capture.last_horizontal_tracker_degrees =
      using_horizontal_tracker ? horizontal_tracker.get_position() / 100.0 : 0;
  capture.last_vertical_tracker_degrees =
      using_vertical_tracker ? vertical_tracker.get_position() / 100.0 : 0;
  capture.x = 0;
  capture.y = 0;
  capture.target = target;
  capture.direction = direction;
  capture.progress = 0;
  capture.max_progress = 0;
  capture.heading_error_sum = 0;
  capture.velocity_error_sum = 0;
  capture.velocity_error_max = 0;
  capture.samples = 0;
  capture.result = Measurement{};
  capture.start_time = pros::millis();
  capture.active.store(true);
}

Measurement endCapture(std::uint32_t timeout_ms) {
  capture.active.store(false);
  const std::uint32_t wait_start = pros::millis();
  while (!capture.done.load()) {
    if (pros::millis() - wait_start >= 500) {
      abort_requested.store(true);
      cancelMotion();
      stopChassis(pros::MotorBrake::brake);
      robot_screen::showError("autotune monitor stopped responding");
      Measurement failed;
      failed.timed_out = true;
      return failed;
    }
    pros::delay(2);
  }

  if (capture.sensor_fault.load()) {
    Measurement failed;
    failed.timed_out = true;
    return failed;
  }

  Measurement result = capture.result;
  result.duration_ms = pros::millis() - capture.start_time;
  result.final_x = capture.x;
  result.final_y = capture.y;
  const double target_heading = capture.mode == SampleMode::heading
      ? capture.target
      : capture.start_heading;
  result.final_heading_error = std::fabs(wrapDegrees(target_heading - getInertialHeading()));
  result.mean_heading_error = capture.samples > 0
      ? capture.heading_error_sum / capture.samples
      : 0;

  if (capture.mode == SampleMode::distance) {
    const double radians = capture.start_heading * M_PI / 180.0;
    const double progress = capture.direction *
        (result.final_x * std::sin(radians) + result.final_y * std::cos(radians));
    result.final_error = std::fabs(std::fabs(capture.target) - progress);
    result.position_error = result.final_error;
    result.overshoot =
        std::max(0.0, capture.max_progress - std::fabs(capture.target));
  } else if (capture.mode == SampleMode::angle) {
    result.final_error = std::fabs(std::fabs(capture.target) - capture.progress);
    result.overshoot =
        std::max(0.0, capture.max_progress - std::fabs(capture.target));
  } else if (capture.mode == SampleMode::heading) {
    result.final_error = result.final_heading_error;
    const double radians = capture.start_heading * M_PI / 180.0;
    const double progress = capture.direction *
        (result.final_x * std::sin(radians) + result.final_y * std::cos(radians));
    result.position_error = std::fabs(drive_distance - progress);
  } else if (capture.mode == SampleMode::velocity) {
    result.final_error = capture.samples > 0
        ? capture.velocity_error_sum / capture.samples
        : 1e9;
    result.overshoot = capture.velocity_error_max;
  }

  result.timed_out = result.duration_ms >= timeout_ms;
  return result;
}

void prepareTrial() {
  if (abort_requested.load()) return;
  pros::delay(movement_settle_ms);
  correct_angle = getInertialHeading();
}

Measurement runDistanceTrial() {
  prepareTrial();
  if (abort_requested.load()) return Measurement{};

  constexpr std::uint32_t timeout = 5000;
  const double direction = distance_direction;
  distance_direction = -distance_direction;
  beginCapture(SampleMode::distance, drive_distance, direction);
  driveTo(direction * drive_distance, timeout, true, 6);
  return endCapture(timeout);
}

Measurement runTurnTrial() {
  prepareTrial();
  if (abort_requested.load()) return Measurement{};

  constexpr std::uint32_t timeout = 3500;
  const double direction = turn_direction;
  turn_direction = -turn_direction;
  const double heading = getInertialHeading();
  beginCapture(SampleMode::angle, turn_angle, direction);
  turnToAngle(heading + direction * turn_angle, timeout, true, 4);
  return endCapture(timeout);
}

Measurement runHeadingTrial() {
  prepareTrial();
  if (abort_requested.load()) return Measurement{};

  const double direction = heading_direction;
  heading_direction = -heading_direction;
  const double heading = getInertialHeading();
  correct_angle = heading + direction * 5.0;
  heading_correction = true;

  beginCapture(SampleMode::heading, correct_angle, direction);
  driveTo(direction * drive_distance, 5000, true, 5);
  const Measurement result = endCapture(5000);
  correct_angle = getInertialHeading();
  return result;
}

Measurement runVelocityTrial() {
  prepareTrial();
  if (abort_requested.load()) return Measurement{};

  constexpr double target_speed = 12.0;
  constexpr std::uint32_t duration = 1200;
  const double direction = velocity_direction;
  velocity_direction = -velocity_direction;
  beginCapture(SampleMode::velocity, direction * target_speed);
  const std::uint32_t start = pros::millis();
  while (pros::millis() - start < duration && !abort_requested.load()) {
    driveWheelSpeeds(direction * target_speed, direction * target_speed, 0, 0);
    pros::delay(10);
  }
  stopChassis(pros::MotorBrake::brake);
  const Measurement result = endCapture(duration + 100);
  correct_angle = getInertialHeading();
  return result;
}

enum class Benchmark { distance, angle, heading, velocity };

Benchmark benchmarkFor(Parameter parameter) {
  switch (parameter) {
    case Parameter::distance_kp:
    case Parameter::distance_kd:
      return Benchmark::distance;
    case Parameter::turn_kp:
    case Parameter::turn_kd:
      return Benchmark::angle;
    case Parameter::heading_kp:
    case Parameter::heading_kd:
      return Benchmark::heading;
    case Parameter::drive_velocity_kp:
      return Benchmark::velocity;
  }
  return Benchmark::distance;
}

double readParameter(Parameter parameter) {
  switch (parameter) {
    case Parameter::distance_kp: return distance_kp;
    case Parameter::distance_kd: return distance_kd;
    case Parameter::turn_kp: return turn_kp;
    case Parameter::turn_kd: return turn_kd;
    case Parameter::heading_kp: return heading_correction_kp;
    case Parameter::heading_kd: return heading_correction_kd;
    case Parameter::drive_velocity_kp: return drive_velocity_kP;
  }
  return 0;
}

void writeParameter(Parameter parameter, double value) {
  switch (parameter) {
    case Parameter::distance_kp: distance_kp = value; break;
    case Parameter::distance_kd: distance_kd = value; break;
    case Parameter::turn_kp: turn_kp = value; break;
    case Parameter::turn_kd: turn_kd = value; break;
    case Parameter::heading_kp: heading_correction_kp = value; break;
    case Parameter::heading_kd: heading_correction_kd = value; break;
    case Parameter::drive_velocity_kp: drive_velocity_kP = value; break;
  }
}

void parameterBounds(Parameter parameter, double& low, double& high) {
  switch (parameter) {
    case Parameter::distance_kp: low = 0.1; high = 5; break;
    case Parameter::distance_kd: low = 0; high = 30; break;
    case Parameter::turn_kp: low = 0.05; high = 3; break;
    case Parameter::turn_kd: low = 0; high = 12; break;
    case Parameter::heading_kp: low = 0.05; high = 3; break;
    case Parameter::heading_kd: low = 0; high = 15; break;
    case Parameter::drive_velocity_kp: low = 0; high = 0.2; break;
  }
}

TuningSettings currentSettings() {
  return {distance_kp, distance_kd, turn_kp, turn_kd, heading_correction_kp,
          heading_correction_kd, drive_velocity_kP, drive_kS, drive_kV, drive_kA};
}

bool validSettings(const TuningSettings& settings) {
  const double values[] = {
      settings.distance_kp, settings.distance_kd, settings.turn_kp, settings.turn_kd,
      settings.heading_kp, settings.heading_kd, settings.drive_velocity_kp,
      settings.drive_kS, settings.drive_kV, settings.drive_kA
  };
  for (const double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return settings.distance_kp >= 0.1 && settings.distance_kp <= 5 &&
         settings.distance_kd >= 0 && settings.distance_kd <= 30 &&
         settings.turn_kp >= 0.05 && settings.turn_kp <= 3 &&
         settings.turn_kd >= 0 && settings.turn_kd <= 12 &&
         settings.heading_kp >= 0.05 && settings.heading_kp <= 3 &&
         settings.heading_kd >= 0 && settings.heading_kd <= 15 &&
         settings.drive_velocity_kp >= 0 && settings.drive_velocity_kp <= 0.2 &&
         settings.drive_kS >= 0 && settings.drive_kS <= 3 &&
         (settings.drive_kV == 0 || (settings.drive_kV >= 0.01 && settings.drive_kV <= 1)) &&
         (settings.drive_kA == 0 || (settings.drive_kA >= 0.001 && settings.drive_kA <= 1));
}

bool saveSettings() {
  const TuningSettings settings = currentSettings();
  if (!validSettings(settings)) {
    robot_screen::showError("autotune settings out of range");
    return false;
  }

  char data[256];
  const int length = std::snprintf(
      data, sizeof(data), "1 %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
      settings.distance_kp, settings.distance_kd, settings.turn_kp, settings.turn_kd,
      settings.heading_kp, settings.heading_kd, settings.drive_velocity_kp,
      settings.drive_kS, settings.drive_kV, settings.drive_kA);
  if (length <= 0 || length >= static_cast<int>(sizeof(data))) return false;
  settings_saved = writeSettingsFile(data, length);
  if (!settings_saved && pros::usd::is_installed()) {
    robot_screen::showError("failed to write drive tuning settings");
  }
  return settings_saved;
}

void applySettings(const TuningSettings& settings) {
  distance_kp = settings.distance_kp;
  distance_kd = settings.distance_kd;
  turn_kp = settings.turn_kp;
  turn_kd = settings.turn_kd;
  heading_correction_kp = settings.heading_kp;
  heading_correction_kd = settings.heading_kd;
  drive_velocity_kP = settings.drive_velocity_kp;
  drive_kS = settings.drive_kS;
  drive_kV = settings.drive_kV;
  drive_kA = settings.drive_kA;
}

bool calibrationCancelled() {
  return pros::competition::is_disabled() ||
         !pros::competition::is_autonomous() ||
         controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B);
}

bool runFeedforwardCalibration() {
  constexpr double voltage_steps[] = {3, -3, 6, -6, 9, -9};
  constexpr std::uint32_t step_duration_ms = 700;
  constexpr double maximum_excursion_in = 36;
  double steady_speeds[6] = {};
  double acceleration_samples[2] = {};
  double acceleration_speeds[2] = {};
  int acceleration_count = 0;
  const double initial_left = left_chassis.get_position();
  const double initial_right = right_chassis.get_position();

  if (!std::isfinite(initial_left) || !std::isfinite(initial_right) ||
      !std::isfinite(wheel_distance_in) || wheel_distance_in <= 0) {
    robot_screen::showError("feedforward encoder or wheel setup invalid");
    return false;
  }

  robot_screen::clear();
  robot_screen::printLine(1, "FEEDFORWARD CALIBRATION");
  robot_screen::printLine(2, "Keep 3 ft clear forward/back.");
  robot_screen::printLine(3, "B cancels; robot will not return.");
  robot_screen::printLine(4, "Measuring kS, kV, kA...");

  for (int step = 0; step < 6; ++step) {
    if (calibrationCancelled()) {
      stopChassis(pros::MotorBrake::brake);
      robot_screen::printLine(1, "FEEDFORWARD CANCELED");
      robot_screen::printLine(2, "Reposition robot before driving.");
      return false;
    }

    double steady_speed_sum = 0;
    int steady_speed_samples = 0;
    double speed_at_100_ms = 0;
    double speed_at_250_ms = 0;
    bool have_speed_at_100_ms = false;
    bool have_speed_at_250_ms = false;
    const double direction = voltage_steps[step] > 0 ? 1.0 : -1.0;
    const std::uint32_t start = pros::millis();

    while (pros::millis() - start < step_duration_ms) {
      if (calibrationCancelled()) {
        stopChassis(pros::MotorBrake::brake);
        robot_screen::printLine(1, "FEEDFORWARD CANCELED");
        robot_screen::printLine(2, "Reposition robot before driving.");
        return false;
      }

      driveChassis(voltage_steps[step], voltage_steps[step]);
      const double left_position = left_chassis.get_position();
      const double right_position = right_chassis.get_position();
      const double left_rpm = left_chassis.get_actual_velocity();
      const double right_rpm = right_chassis.get_actual_velocity();
      if (!std::isfinite(left_position) || !std::isfinite(right_position) ||
          !std::isfinite(left_rpm) || !std::isfinite(right_rpm)) {
        stopChassis(pros::MotorBrake::brake);
        robot_screen::showError("feedforward motor reading invalid");
        return false;
      }

      const double excursion =
          ((left_position - initial_left) + (right_position - initial_right)) *
          wheel_distance_in / 720.0;
      if (std::fabs(excursion) > maximum_excursion_in) {
        stopChassis(pros::MotorBrake::brake);
        robot_screen::showError("feedforward exceeded 36 in travel limit");
        return false;
      }

      const double elapsed = static_cast<double>(pros::millis() - start);
      const double directional_speed =
          direction * (left_rpm + right_rpm) * wheel_distance_in / 120.0;
      if (!std::isfinite(directional_speed)) {
        stopChassis(pros::MotorBrake::brake);
        robot_screen::showError("feedforward speed reading invalid");
        return false;
      }
      if (elapsed >= 100 && !have_speed_at_100_ms) {
        speed_at_100_ms = directional_speed;
        have_speed_at_100_ms = true;
      }
      if (elapsed >= 250 && !have_speed_at_250_ms) {
        speed_at_250_ms = directional_speed;
        have_speed_at_250_ms = true;
      }
      if (elapsed >= 400) {
        steady_speed_sum += directional_speed;
        ++steady_speed_samples;
      }
      pros::delay(10);
    }

    stopChassis(pros::MotorBrake::brake);
    steady_speeds[step] = steady_speed_samples > 0
        ? steady_speed_sum / steady_speed_samples
        : 0;
    if (std::fabs(voltage_steps[step]) == 9 &&
        have_speed_at_100_ms && have_speed_at_250_ms) {
      const double acceleration =
          (speed_at_250_ms - speed_at_100_ms) / 0.15;
      if (acceleration > 0.5) {
        acceleration_samples[acceleration_count] = acceleration;
        acceleration_speeds[acceleration_count] =
            (speed_at_100_ms + speed_at_250_ms) / 2;
        ++acceleration_count;
      }
    }
    robot_screen::printLine(5, "Voltage step %d / 6 complete", step + 1);
    for (int wait = 0; wait < 30 && !calibrationCancelled(); ++wait) {
      pros::delay(10);
    }
  }

  double sum_speed = 0;
  double sum_voltage = 0;
  double sum_speed_squared = 0;
  double sum_speed_voltage = 0;
  for (int i = 0; i < 6; ++i) {
    if (!(steady_speeds[i] > 0.5) || !std::isfinite(steady_speeds[i])) {
      robot_screen::showError("feedforward test speed too low");
      return false;
    }
    const double speed = steady_speeds[i];
    const double voltage = std::fabs(voltage_steps[i]);
    sum_speed += speed;
    sum_voltage += voltage;
    sum_speed_squared += speed * speed;
    sum_speed_voltage += speed * voltage;
  }

  const double denominator = 6 * sum_speed_squared - sum_speed * sum_speed;
  if (!(denominator > 1e-6) || acceleration_count == 0) {
    robot_screen::showError("feedforward fit has insufficient data");
    return false;
  }
  const double kV = (6 * sum_speed_voltage - sum_speed * sum_voltage) / denominator;
  const double kS = (sum_voltage - kV * sum_speed) / 6;
  double squared_residual = 0;
  for (int i = 0; i < 6; ++i) {
    const double residual =
        std::fabs(voltage_steps[i]) - (kS + kV * steady_speeds[i]);
    squared_residual += residual * residual;
  }
  const double fit_error = std::sqrt(squared_residual / 6);
  double kA = 0;
  for (int i = 0; i < acceleration_count; ++i) {
    kA += (9 - kS - kV * acceleration_speeds[i]) / acceleration_samples[i];
  }
  kA /= acceleration_count;

  if (!std::isfinite(kS) || !std::isfinite(kV) || !std::isfinite(kA) ||
      kS < 0 || kS > 3 || kV < 0.01 || kV > 1 ||
      kA < 0.001 || kA > 1 || fit_error > 1.5) {
    robot_screen::showError("feedforward fit failed; settings unchanged");
    return false;
  }

  drive_kS = kS;
  drive_kV = kV;
  drive_kA = kA;
  settings_saved = saveSettings();
  std::printf("Drive feedforward: kS=%.3f V, kV=%.4f V/(in/s), kA=%.4f V/(in/s^2)\n",
              drive_kS, drive_kV, drive_kA);
  robot_screen::clear();
  robot_screen::printLine(1, "FEEDFORWARD RESULTS");
  robot_screen::printLine(2, "kS = %.3f V", drive_kS);
  robot_screen::printLine(3, "kV = %.4f V/(in/s)", drive_kV);
  robot_screen::printLine(4, "kA = %.4f V/(in/s^2)", drive_kA);
  robot_screen::printLine(5, "Fit error %.2f V", fit_error);
  robot_screen::printLine(6, settings_saved ? "Saved to SD card." : "No SD: copy to rw-config.cpp");
  robot_screen::printLine(7, "Reposition; B exits.");
  while (!pros::competition::is_disabled() &&
         pros::competition::is_autonomous() &&
         !controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    pros::delay(20);
  }
  return true;
}

double candidateFor(Parameter parameter, int index, double center, int pass) {
  double low, high;
  parameterBounds(parameter, low, high);
  if (index == 0) return center;

  const double span = high - low;
  const double scale = pass == 0 ? 0.5 : 0.25;
  const double zero_center_offset = pass == 0 ? 0.25 : 0.125;
  const double offset = std::fabs(center) > 1e-9
      ? std::max(std::fabs(center) * scale, span * 0.02)
      : span * zero_center_offset;
  const double candidate = center + (index == 1 ? -offset : offset);
  return std::max(low, std::min(high, candidate));
}

double trialMetric(Benchmark benchmark, const Measurement& result) {
  switch (benchmark) {
    case Benchmark::distance:
      return result.final_error + 0.1 * result.overshoot;
    case Benchmark::angle:
      return result.final_error + 0.02 * result.overshoot;
    case Benchmark::heading:
      return result.final_heading_error + 0.2 * result.mean_heading_error;
    case Benchmark::velocity:
      return result.final_error + 0.1 * result.overshoot;
  }
  return 1e9;
}

bool withinLimits(Benchmark benchmark, const Measurement& result) {
  if (result.timed_out) return false;
  switch (benchmark) {
    case Benchmark::distance:
      return result.position_error <= drive_tolerance;
    case Benchmark::velocity:
      return result.final_error <= 2.0 && result.overshoot <= 4.0;
    case Benchmark::angle:
      return result.final_error <= turn_tolerance;
    case Benchmark::heading:
      return result.final_heading_error <= turn_tolerance &&
             result.position_error <= drive_tolerance;
  }
  return false;
}

Measurement runBenchmark(Benchmark benchmark) {
  switch (benchmark) {
    case Benchmark::distance: return runDistanceTrial();
    case Benchmark::angle: return runTurnTrial();
    case Benchmark::heading: return runHeadingTrial();
    case Benchmark::velocity: return runVelocityTrial();
  }
  return Measurement{};
}

const char* parameterName(Parameter parameter) {
  switch (parameter) {
    case Parameter::distance_kp: return "Distance Kp";
    case Parameter::distance_kd: return "Distance Kd";
    case Parameter::turn_kp: return "Turn Kp";
    case Parameter::turn_kd: return "Turn Kd";
    case Parameter::heading_kp: return "Heading Kp";
    case Parameter::heading_kd: return "Heading Kd";
    case Parameter::drive_velocity_kp: return "Velocity Kp";
  }
  return "Unknown";
}

double tuneParameter(Parameter parameter, int step, int total_steps, int pass) {
  const Benchmark benchmark = benchmarkFor(parameter);
  const double center = readParameter(parameter);
  double best_value = center;
  double best_score = 1e12;
  const double tolerance = benchmark == Benchmark::distance
      ? drive_tolerance
      : benchmark == Benchmark::velocity ? 2.0 : turn_tolerance;

  for (int candidate_index = 0; candidate_index < 3 && !abort_requested.load(); ++candidate_index) {
    const double value = candidateFor(parameter, candidate_index, center, pass);
    if (candidate_index > 0 &&
        std::fabs(value - candidateFor(parameter, candidate_index - 1, center, pass)) < 1e-9) {
      continue;
    }
    writeParameter(parameter, value);

    double worst_metric = 0;
    double min_metric = 1e9;
    double max_metric = 0;
    double sum_time = 0;
    bool timed_out = false;
    bool passed = true;

    for (int run = 0; run < repetitions && !abort_requested.load(); ++run) {
      robot_screen::printLine(1, "DRIVE AUTOTUNE %d / %d", step, total_steps);
      robot_screen::printLine(2, "%s: %.3f", parameterName(parameter), value);
      robot_screen::printLine(3, "Benchmark repeat %d / %d", run + 1, repetitions);

      const Measurement result = runBenchmark(benchmark);
      if (abort_requested.load()) break;
      const double metric = trialMetric(benchmark, result);
      worst_metric = std::max(worst_metric, metric);
      min_metric = std::min(min_metric, metric);
      max_metric = std::max(max_metric, metric);
      sum_time += result.duration_ms;
      timed_out = timed_out || result.timed_out;
      passed = passed && withinLimits(benchmark, result);
    }
    if (abort_requested.load()) break;

    const double variation = max_metric - min_metric;
    const bool candidate_passed = passed && variation <= tolerance;
    double score;
    if (candidate_passed) {
      score = 1000.0 * worst_metric / tolerance +
              200.0 * variation / tolerance +
              0.001 * sum_time / repetitions;
    } else {
      score = 100000.0 + (timed_out ? 100000.0 : 0.0) +
              1000.0 * worst_metric / tolerance +
              200.0 * variation / tolerance;
    }
    if (score < best_score) {
      best_score = score;
      best_value = value;
    }
#ifdef RW_SIM_PROS
    std::printf(
        "AUTOTUNE pass %d candidate %d: %s=%.4f, %s, worst=%.3f, repeatability=%.3f, "
        "mean-time=%.0fms, score=%.2f\n",
        pass + 1, candidate_index + 1, parameterName(parameter), value,
        candidate_passed ? "PASS" : "FAIL", worst_metric, variation,
        sum_time / repetitions, score);
#endif
    robot_screen::printLine(4, "Worst error metric: %.2f", worst_metric);
    robot_screen::printLine(5, "Best candidate: %.3f", best_value);
    pros::delay(100);
  }

  writeParameter(parameter, abort_requested.load() ? center : best_value);
#ifdef RW_SIM_PROS
  std::printf("AUTOTUNE selected %s=%.4f after pass %d%s\n",
              parameterName(parameter), abort_requested.load() ? center : best_value,
              pass + 1, abort_requested.load() ? " (aborted)" : "");
#endif
  return abort_requested.load() ? center : best_value;
}

void showResults() {
  while (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) pros::delay(10);
  robot_screen::clear();
  int page = 0;

  while (!pros::competition::is_disabled()) {
    robot_screen::clear();
    if (page == 0) {
      robot_screen::printLine(1, all_limits_met ? "PID TUNE PASSED" : "PID RESULTS - REVIEW");
      robot_screen::printLine(2, "Drive Kp %.3f  Kd %.3f", distance_kp, distance_kd);
      robot_screen::printLine(3, "Turn  Kp %.3f  Kd %.3f", turn_kp, turn_kd);
      robot_screen::printLine(4, "Head  Kp %.3f  Kd %.3f",
                              heading_correction_kp, heading_correction_kd);
      robot_screen::printLine(5, "I gains left unchanged.");
      robot_screen::printLine(6, settings_saved ? "Settings saved to SD." : "Not saved to SD.");
      robot_screen::printLine(7, settings_saved ? "SD values load at startup." : "Copy values to rw-config.cpp");
      robot_screen::printLine(8, "A: feedforward values   B: exit");
    } else {
      const double free_speed = drive_motor_rpm * wheel_distance_in / 60.0;
      const double estimated_kV = free_speed > 0 ? 12.0 / (free_speed * 0.92) : 0;
      const double shown_kV = drive_kV > 0 ? drive_kV : estimated_kV;
      const double shown_kA = drive_kA > 0 ? drive_kA : shown_kV * 0.13;
      robot_screen::printLine(1, "FEEDFORWARD + MOTION");
      robot_screen::printLine(2, "kS %.3f V", drive_kS);
      robot_screen::printLine(3, "kV %.4f%s V/(in/s)", shown_kV,
                              drive_kV > 0 ? "" : " auto");
      robot_screen::printLine(4, "kA %.4f%s V/(in/s^2)", shown_kA,
                              drive_kA > 0 ? "" : " auto");
      robot_screen::printLine(5, "Velocity Kp %.4f", drive_velocity_kP);
      robot_screen::printLine(6, "Profile %.1f in/s, %.1f in/s^2",
                              profile_max_speed, profile_max_accel);
      robot_screen::printLine(7, settings_saved ? "Settings saved to SD." : "Copy values to rw-config.cpp");
      robot_screen::printLine(8, "A: PID values   B: exit");
    }

    if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      page = 1 - page;
      while (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(10);
    }
    if (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) break;
    pros::delay(20);
  }
}
}

void loadDriveAutotuneSettings() {
  char data[256] = {};
  const int length = readSettingsFile(data, sizeof(data) - 1);
  if (length == -2) {
    robot_screen::showError("autotune SD settings too large");
    return;
  }
  if (length <= 0) return;
  data[length] = '\0';

  int version = 0;
  int parsed_chars = 0;
  TuningSettings settings{};
  const int fields = std::sscanf(
      data, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf%n", &version,
      &settings.distance_kp, &settings.distance_kd, &settings.turn_kp,
      &settings.turn_kd, &settings.heading_kp, &settings.heading_kd,
      &settings.drive_velocity_kp, &settings.drive_kS, &settings.drive_kV,
      &settings.drive_kA, &parsed_chars);
  while (data[parsed_chars] == ' ' || data[parsed_chars] == '\t' ||
         data[parsed_chars] == '\r' || data[parsed_chars] == '\n') {
    ++parsed_chars;
  }
  if (fields != 11 || data[parsed_chars] != '\0' ||
      version != 1 || !validSettings(settings)) {
    robot_screen::showError("autotune SD settings invalid; using source values");
    return;
  }

  applySettings(settings);
  settings_saved = true;
  std::printf("Loaded saved drive tuning settings from SD.\n");
}

void runDriveAutotune() {
  const bool previous_heading_correction = heading_correction;
  robot_screen::setTuningActive(true);
  robot_screen::clear();
#ifndef RW_SIM_PROS
  robot_screen::printLine(1, "DRIVE AUTOTUNE");
  robot_screen::printLine(2, "Clear 24 in in every direction.");
  robot_screen::printLine(3, "A: tune PID gains.");
  robot_screen::printLine(4, "X: measure kS, kV, kA.");
  robot_screen::printLine(5, "B: cancel.");
  while (!pros::competition::is_disabled() && pros::competition::is_autonomous()) {
    if (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      robot_screen::clearTuning();
      return;
    }
    if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      runFeedforwardCalibration();
      robot_screen::clearTuning();
      return;
    }
    if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) break;
    pros::delay(20);
  }
  if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) {
    robot_screen::clearTuning();
    return;
  }
#else
  distance_kp = 1.0;
  distance_ki = 0.0;
  distance_kd = 0.0;
  turn_kp = 1.0;
  turn_ki = 0.0;
  turn_kd = 0.0;
  heading_correction_ki = 0.0;
  heading_correction_kp = 1.5;
  heading_correction_kd = 8.0;
  drive_velocity_kP = 0.0;
  std::printf(
      "PID tuner simulation baseline: drive/turn Kp=1, Ki=0, Kd=0; "
      "heading correction Kp=1.5, Kd=8 (held strong for linear tuning); velocity Kp=0\n");
#endif

  if (!std::isfinite(wheel_distance_in) || wheel_distance_in <= 0 ||
      !std::isfinite(distance_between_wheels) || distance_between_wheels <= 0 ||
      !std::isfinite(drive_motor_rpm) || drive_motor_rpm <= 0 ||
      !validSettings(currentSettings())) {
    robot_screen::showError("autotune setup values invalid");
    robot_screen::clearTuning();
    return;
  }

  abort_requested.store(false);
  all_limits_met = true;
  settings_saved = false;
  distance_direction = 1;
  turn_direction = 1;
  heading_direction = 1;
  velocity_direction = 1;
  sampler_shutdown.store(false);
  const pros::task_t sampler_handle =
      pros::Task::create([] { samplerTask(); }, "Drive autotune monitor");
  if (sampler_handle == nullptr) {
    robot_screen::showError("autotune monitor task", errno);
    robot_screen::clearTuning();
    return;
  }
  pros::Task sampler(sampler_handle);

  starting_heading = getInertialHeading();
  setPose(0, 0, starting_heading);
  correct_angle = starting_heading;
  heading_correction = true;

  const Parameter parameters[] = {
      Parameter::distance_kp, Parameter::distance_kd,
      Parameter::drive_velocity_kp,
      Parameter::turn_kp, Parameter::turn_kd,
      Parameter::heading_kp, Parameter::heading_kd
  };
  constexpr int parameter_count = sizeof(parameters) / sizeof(parameters[0]);
  constexpr int optimization_passes = 2;
  constexpr int total_steps = parameter_count * optimization_passes;
  for (int pass = 0; pass < optimization_passes && !abort_requested.load(); ++pass) {
    for (int i = 0; i < parameter_count && !abort_requested.load(); ++i) {
      tuneParameter(parameters[i], pass * parameter_count + i + 1, total_steps, pass);
    }
  }

  if (!abort_requested.load()) {
    all_limits_met = true;
    const Benchmark validation_benchmarks[] = {
        Benchmark::distance, Benchmark::angle, Benchmark::heading, Benchmark::velocity
    };
    constexpr int validation_count =
        sizeof(validation_benchmarks) / sizeof(validation_benchmarks[0]);
    for (const Benchmark benchmark : validation_benchmarks) {
      bool passed = true;
      double min_metric = 1e9;
      double max_metric = 0;
      const double tolerance = benchmark == Benchmark::distance
          ? drive_tolerance
          : benchmark == Benchmark::velocity ? 2.0 : turn_tolerance;
      for (int run = 0; run < repetitions && !abort_requested.load(); ++run) {
        robot_screen::printLine(1, "FINAL VALIDATION");
        robot_screen::printLine(2, "Benchmark %d / %d", static_cast<int>(benchmark) + 1,
                                validation_count);
        robot_screen::printLine(3, "Repeat %d / %d", run + 1, repetitions);
        const Measurement result = runBenchmark(benchmark);
        passed = passed && withinLimits(benchmark, result);
        const double metric = trialMetric(benchmark, result);
        min_metric = std::min(min_metric, metric);
        max_metric = std::max(max_metric, metric);
      }
      passed = passed && max_metric - min_metric <= tolerance;
      all_limits_met = all_limits_met && passed && !abort_requested.load();
    }
    if (all_limits_met) {
      settings_saved = saveSettings();
    }
  }

  sampler_shutdown.store(true);
  sampler.remove();
  stopChassis(pros::MotorBrake::brake);
  if (abort_requested.load()) {
    all_limits_met = false;
    robot_screen::clear();
    robot_screen::printLine(1, "AUTOTUNE STOPPED");
    robot_screen::printLine(2, "B pressed / robot disabled");
    robot_screen::printLine(3, "Completed settings kept.");
    pros::delay(1500);
  }
  heading_correction = previous_heading_correction;
#ifdef RW_SIM_PROS
  std::printf("AUTOTUNE %s\n", all_limits_met ? "PASSED" : "REVIEW REQUIRED");
  std::printf("Final PID: distance Kp=%.4f Ki=%.4f Kd=%.4f; turn Kp=%.4f Ki=%.4f Kd=%.4f\n",
              distance_kp, distance_ki, distance_kd, turn_kp, turn_ki, turn_kd);
  std::printf("Final PID: heading Kp=%.4f Ki=%.4f Kd=%.4f; velocity Kp=%.4f\n",
              heading_correction_kp, heading_correction_ki, heading_correction_kd,
              drive_velocity_kP);
#else
  showResults();
#endif
  robot_screen::clearTuning();
}
