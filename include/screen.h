#ifndef ROBOT_SCREEN_H
#define ROBOT_SCREEN_H

#include <cstdint>

namespace robot_screen {

void startTelemetry();
void showError(const char* message, int error_code = 0);
void setTuningActive(bool active);
bool tuningActive();
void clearTuning();
void clear();
void printLine(std::uint8_t line, const char* format, ...);
void printDistanceLine(std::uint8_t line, const char* label, std::int32_t millimeters);

}

#endif
