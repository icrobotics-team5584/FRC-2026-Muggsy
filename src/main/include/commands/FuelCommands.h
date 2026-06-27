#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr ReverseIntakeSequence();

frc2::CommandPtr StationaryShootAt(frc::Translation2d target);
bool IsReadyToShoot();

frc2::CommandPtr ToggleBrakeCoast();
frc2::CommandPtr EjectFuel();
}  // namespace cmd