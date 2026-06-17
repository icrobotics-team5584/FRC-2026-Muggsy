// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubVision.h"

#include "commands/VisionCommands.h"

#include "utilities/Logger.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
  _driverController.Start().OnTrue(SubDeploy::GetInstance().Zero());
  _driverController.POVUp().WhileTrue(SubDeploy::GetInstance().ManualExtendUp());
  _driverController.POVDown().WhileTrue(SubDeploy::GetInstance().ManualExtendDown());
  _driverController.RightTrigger().WhileTrue(SubIntake::GetInstance().RunIntake());
  _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().RunReverseIntake());

  _driverController.RightBumper().WhileTrue(SubFeeder::GetInstance().Feed());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
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
