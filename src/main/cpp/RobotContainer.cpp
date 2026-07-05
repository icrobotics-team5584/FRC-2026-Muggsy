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

#include "commands/FuelCommands.h"
#include "commands/VisionCommands.h"

#include "utilities/FieldConstants.h"
#include "utilities/Logger.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
  _driverController.LeftTrigger().WhileTrue(cmd::IntakeSequence());
  _driverController.RightTrigger().WhileTrue(cmd::StationaryShootAt(fieldpos::HUB_POSITION.ToTranslation2d()));


  _driverController.LeftBumper().WhileTrue(SubDeploy::GetInstance().ExtendToStow());


  _driverController.Y().OnTrue(
    frc2::cmd::RunOnce([] { SubDrivebase::GetInstance().ResetGyroHeading(); })),
  _driverController.B().OnTrue(
    frc2::cmd::RunOnce([] { SubDrivebase::GetInstance().SyncSensors(); }));
  _driverController.A().WhileTrue(cmd::EjectFuel());


  _driverController.POVUp().WhileTrue(SubHood::GetInstance().RunZeroingSequence());
  _driverController.POVRight().WhileTrue(SubDeploy::GetInstance().Zero());


  _driverController.RightTrigger().OnFalse(frc2::cmd::Parallel(
    SubHood::GetInstance().HoodToStowAngle(),
    SubShooter::GetInstance().Stop(),
    SubDeploy::GetInstance().ExtendToDeploy()
  ));
  
  _driverController.RightBumper().WhileTrue(cmd::TuneShooterAndHoodTables());
  _driverController.Back().OnTrue(SubHood::GetInstance().MoveHoodDown1Degree());
  _driverController.Start().OnTrue(SubHood::GetInstance().MoveHoodUp1Degree());
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
