#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr ReverseIntakeSequence();

frc2::CommandPtr StationaryShootAt(frc::Translation2d target);
bool IsReadyToShoot();

frc2::CommandPtr ToggleBrakeCoast();
frc2::CommandPtr EjectFuel();

frc::Translation2d GetShotTarget();
units::degree_t CalcAngleToShotTarget();

static constexpr units::millisecond_t LATENCY_OFFSET = 100_ms;
}  // namespace cmd