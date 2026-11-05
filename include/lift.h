#ifndef LIFT_H
#define LIFT_H

#include <array>
#include <cstdint>

extern std::array<double, 7> lift_positions;
extern double lift_kp;
extern double lift_kd;
extern double lift_hold_mv;

void initializeLift();
void moveTo(int presetIndex);
void raiseLift();
void raiseSlightly();
void slowDescent();
void setManualLiftVoltage(std::int32_t millivolts);
void beginLiftRaise();
void finishLiftRaise();
void reset();
void score(bool driverRequest = false);


#endif