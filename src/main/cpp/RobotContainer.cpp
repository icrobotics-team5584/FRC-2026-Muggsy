// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIndexer.h"

#include "utilities/Logger.h"

#include <frc2/command/Commands.h>
#include <commands/AutonCommands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().OnTrue(frc2::cmd::RunOnce([]{
    SubDrivebase::GetInstance().SetPose(frc::Pose2d{3.478_m, 7.450_m, 0.0_deg});
  }));

  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ZeroRotation([]{return 0_deg;}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return cmd::IntakeAuton();
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
  return frc2::cmd::Run([this, force] {
    _driverController.SetRumble(frc::XboxController::RumbleType::kBothRumble, force);
    logger::Log("DriverStation/Rumble", true);
  })
    .WithTimeout(duration)
    .FinallyDo([this] {
      _driverController.SetRumble(frc::XboxController::RumbleType::kBothRumble, 0);
      logger::Log("DriverStation/Rumble", false);
    });
}

