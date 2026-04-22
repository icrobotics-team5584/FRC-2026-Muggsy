// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include "utilities/Logger.h"
#include <frc2/command/Commands.h>
#include <commands/AutonCommands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return cmd::IntakeAuton();
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
  return frc2::cmd::Run([this, force, duration] {
    _driverController.SetRumble(frc::XboxController::RumbleType::kBothRumble, force);
    Logger::Log("DriverStation/Rumble", true);
  })
    .WithTimeout(duration)
    .FinallyDo([this] {
      _driverController.SetRumble(frc::XboxController::RumbleType::kBothRumble, 0);
      Logger::Log("DriverStation/Rumble", false);
    });
}
