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

#include <commands/AutonCommands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController));

  _autoManager.AddDefaultAuton(
    "Left Trench", autonHelper::MakeCommandPtrAuto(cmd::TwoPassAuto(false)));
  _autoManager.AddDefaultAuton(
    "Right Trench", autonHelper::MakeCommandPtrAuto(cmd::TwoPassAuto(true)));

  frc::SmartDashboard::PutData("CHOSEN AUTON", &_autoManager.GetAutonChooser());
}

void RobotContainer::ConfigureBindings() {
  _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().RunIntake());
  _driverController.RightTrigger().WhileTrue(cmd::StationaryShoot());
  _driverController.RightTrigger().OnFalse(cmd::ResetAfterShoot());

  _driverController.LeftBumper().ToggleOnTrue(SubDeploy::GetInstance().ToggleStow());
  _driverController.RightBumper().WhileTrue(cmd::StationaryShootWithoutAim());
  _driverController.RightBumper().OnFalse(cmd::ResetAfterShoot());

  _driverController.Y().OnTrue(
    frc2::cmd::RunOnce([] { SubDrivebase::GetInstance().ResetGyroHeading(); }));
  _driverController.B().OnTrue(
    frc2::cmd::RunOnce([] { SubDrivebase::GetInstance().SyncSensors(); }));
  _driverController.A().WhileTrue(cmd::EjectFuel());
  _driverController.A().OnFalse(cmd::ResetAfterShoot());

  _driverController.POVLeft().WhileTrue(SubHood::GetInstance().RunZeroingSequence());
  _driverController.POVRight().WhileTrue(SubDeploy::GetInstance().AutonZero());

  _operatorController.POVUp().OnTrue(SubHood::GetInstance().AddManualAngleOffset(1_deg));
  _operatorController.POVDown().OnTrue(SubHood::GetInstance().AddManualAngleOffset(-1_deg));

  _operatorController.Y().OnTrue(SubShooter::GetInstance().AddManualSpeedOffset(1_tps));
  _operatorController.A().OnTrue(SubShooter::GetInstance().AddManualSpeedOffset(-1_tps));

  _operatorController.Back().OnTrue(frc2::cmd::RunOnce([] { SubHood::GetInstance().SetManualAngleOffset(0_deg); }));
  _operatorController.Start().OnTrue(frc2::cmd::RunOnce([] { SubShooter::GetInstance().SetManualSpeedOffset(0_tps);  }));

  SubDrivebase::GetInstance().CheckCoastButton().WhileTrue(cmd::ToggleBrakeCoast());
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  autonHelper::AutonPtr chosen = _autoManager.GetChosenAuton();
  return chosen;
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
