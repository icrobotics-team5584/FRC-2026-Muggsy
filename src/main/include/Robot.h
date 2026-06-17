// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <optional>

#include "Constants.h"
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
   std::shared_ptr<frc2::CommandPtr> _autonomousCommand;

  RobotContainer _container;

  frc::PowerDistribution _pdh{canid::PDH, frc::PowerDistribution::ModuleType::kRev};
};
