#pragma once

#include <frc2/command/Commands.h>
#include <functional>
#include <units/length.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
  frc2::CommandPtr IntakePass(bool flip);
  frc2::CommandPtr DepotAuton();
  
  units::meter_t CalcDist(frc::Pose2d robotPos, frc::Translation2d targetPos);
  frc::Translation2d GetHubPos();
}