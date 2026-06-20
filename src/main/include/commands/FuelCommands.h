#pragma once

#include <frc2/command/CommandPtr.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr ReverseIntakeSequence();

frc2::CommandPtr StationaryShootAt();
bool IsReadyToShoot();

frc2::CommandPtr ToggleBrakeCoast();
frc2::CommandPtr EjectFuel();
}  // namespace cmd