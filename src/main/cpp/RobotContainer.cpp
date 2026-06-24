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
#include "subsystems/SubShooter.h"
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
  _driverController.Start().WhileTrue(SubDeploy::GetInstance().Zero());
  // _driverController.POVUp().WhileTrue(SubDeploy::GetInstance().ManualExtendUp());
  // _driverController.POVDown().WhileTrue(SubDeploy::GetInstance().ManualExtendDown());
  _driverController.RightTrigger().WhileTrue(SubIntake::GetInstance().RunIntake());
  _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().RunReverseIntake());
  _driverController.Y().WhileTrue(
    frc2::cmd::Parallel(SubFeeder::GetInstance().Feed(), SubIndexer::GetInstance().SpinIndexer()));
  _driverController.A().WhileTrue(frc2::cmd::Parallel(
    SubFeeder::GetInstance().FeedBackwards(), SubIndexer::GetInstance().ReverseIndexer()));
  _driverController.POVUp().WhileTrue(
    SubShooter::GetInstance().SetSpeedTarget([] { return 1000_rpm; }));
  _driverController.POVDown().WhileTrue(SubShooter::GetInstance().Stop());
  _driverController.POVLeft().WhileTrue(
    SubShooter::GetInstance().SetSpeedTarget([] { return 3000_rpm; }));
  _driverController.POVRight().WhileTrue(
    SubDrivebase::GetInstance().ZeroRotation([] { return 0_deg; }));
  _driverController.Back().OnTrue(SubHood::GetInstance().RunZeroingSequence());
  _driverController.X().OnTrue(SubHood::GetInstance().HoodToEjectAngle());
  _driverController.B().OnTrue(SubHood::GetInstance().HoodToStowAngle());
  _driverController.RightBumper().OnTrue(SubHood::GetInstance().HoodToPassingAngle());
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
