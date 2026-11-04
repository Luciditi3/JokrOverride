#ifndef CLAW_H
#define CLAW_H

#include <cstdint>

void initializeClaw();
void intakeClaw();
void openClaw();
void stopClaw();
void openClawFor(std::uint32_t durationMs);

#endif