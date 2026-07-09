#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/Commands.h>

#include <functional>
#include <units/length.h>

namespace cmd {
frc2::CommandPtr IntakePass(bool flip);
frc2::CommandPtr DepotAuton(bool flip);
frc2::CommandPtr AutonCommand();
frc::Translation2d GetHubPos();

frc2::CommandPtr SetAutonStartPos(const std::function<frc::Pose2d()>& pose);
}  // namespace cmd