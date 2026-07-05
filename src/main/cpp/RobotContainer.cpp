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

#include <commands/AutonCommands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
  _driverController.Start().WhileTrue(SubDeploy::GetInstance().Zero());
  _driverController.Back().WhileTrue(SubHood::GetInstance().RunZeroingSequence());

  _driverController.Y().OnTrue(
    frc2::cmd::Parallel(frc2::cmd::RunOnce([] { SubDrivebase::GetInstance().ResetGyroHeading(); }),
      frc2::cmd::RunOnce([] { SubDrivebase::GetInstance().SyncSensors(); })));
  _driverController.POVUp().WhileTrue(SubDeploy::GetInstance().ManualExtendUp());
  _driverController.POVDown().WhileTrue(SubDeploy::GetInstance().ManualExtendDown());
  _driverController.X().WhileTrue(SubDeploy::GetInstance().ExtendToStow());
  _driverController.B().WhileTrue(SubDeploy::GetInstance().ExtendToDeploy());
  _driverController.RightTrigger().WhileTrue(SubIntake::GetInstance().RunIntake());
  _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().RunReverseIntake());
  _driverController.RightBumper().WhileTrue(
    frc2::cmd::Parallel(SubFeeder::GetInstance().Feed(), SubIndexer::GetInstance().SpinIndexer()));
  _driverController.LeftBumper().WhileTrue(frc2::cmd::Parallel(
    SubFeeder::GetInstance().FeedBackwards(), SubIndexer::GetInstance().ReverseIndexer()));
  _driverController.POVLeft().WhileTrue(
    SubShooter::GetInstance().SetSpeedTarget([] { return 3000_rpm; }));
  _driverController.POVRight().WhileTrue(
    SubShooter::GetInstance().SetSpeedTarget([] { return 0_rpm; }));
  _driverController.A().ToggleOnTrue(frc2::cmd::StartEnd(
    [] {
      frc2::CommandScheduler::GetInstance().Schedule(SubHood::GetInstance().HoodToPassingAngle());
    },
    [] {
      frc2::CommandScheduler::GetInstance().Schedule(SubHood::GetInstance().HoodToStowAngle());
    }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return cmd::AutonCommand();
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
