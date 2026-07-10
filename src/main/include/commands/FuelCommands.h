#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <units/angle.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr ReverseIntakeSequence();

frc2::CommandPtr StationaryShoot();
frc2::CommandPtr ResetAfterShoot();
frc2::CommandPtr StationaryShootWithoutAim();
bool IsReadyToShoot();
frc2::CommandPtr ShootWhenReady();

frc2::CommandPtr ToggleBrakeCoast();
frc2::CommandPtr EjectFuel();

frc::Translation2d GetShotTarget();
frc::Rotation2d CalcAngleToShotTarget();

frc2::CommandPtr TuneShooterAndHoodTables();
frc::Pose2d CalcFutureDrumPose();

units::degree_t CalcShootOnTheMoveAngle();
units::meter_t CalcShootOnTheMoveDistance();
frc2::CommandPtr ShootOnTheMove(frc2::CommandXboxController& controller);

static constexpr units::millisecond_t LATENCY_OFFSET = 100_ms;
}  // namespace cmd