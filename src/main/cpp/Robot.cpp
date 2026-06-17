// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "utilities/Logger.h"
#include "utilities/ShiftHandler.h"

#include <frc/DataLogManager.h>
#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <wpinet/WebServer.h>

Robot::Robot() {
  // USB logging
  frc::DataLogManager::Start();
  frc::SmartDashboard::PutData(&frc2::CommandScheduler::GetInstance());
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  ShiftHandler::GetInstance().Periodic();
  CurrentController::GetInstance().Periodic();

  logger::Log("Robot/RioBrownOut", frc::RobotController::IsBrownedOut());
  logger::Log("Robot/RioInputVoltage", frc::RobotController::GetInputVoltage() * 1_V);
  logger::Log("Robot/RioInputCurrent", frc::RobotController::GetInputCurrent() * 1_A);
  logger::Log("Robot/BatteryVoltage", frc::RobotController::GetBatteryVoltage());
  logger::Log("Robot/PDHInputVoltage", _pdh.GetVoltage() * 1_V);
  logger::Log("Robot/PDHTotalCurrent", _pdh.GetTotalCurrent() * 1_A);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  _autonomousCommand = _container.GetAutonomousCommand();

  if (_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(_autonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (_autonomousCommand) {
    _autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
