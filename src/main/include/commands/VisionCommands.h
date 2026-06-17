// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>

struct ProcessedPose {
  std::string camName;
  frc::Pose2d pose;
  units::second_t timestamp;
  units::meter_t distance;
};

namespace cmd {
frc2::CommandPtr AddVisionMeasurement();
}
