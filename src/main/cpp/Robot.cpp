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

  Logger::Log("Robot/RioBrownOut", frc::RobotController::IsBrownedOut());
  Logger::Log("Robot/RioInputVoltage", frc::RobotController::GetInputVoltage() * 1_V);
  Logger::Log("Robot/RioInputCurrent", frc::RobotController::GetInputCurrent() * 1_A);
  Logger::Log("Robot/BatteryVoltage", frc::RobotController::GetBatteryVoltage());
  Logger::Log("Robot/PDHInputVoltage", _mPdh.GetVoltage() * 1_V);
  Logger::Log("Robot/PDHTotalCurrent", _mPdh.GetTotalCurrent() * 1_A);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  _mAutonomousCommand = _mContainer.GetAutonomousCommand();

  if (_mAutonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(_mAutonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (_mAutonomousCommand) {
    _mAutonomousCommand->Cancel();
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
