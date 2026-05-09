// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"

#include "utilities/Logger.h"

#include <frc/smartdashboard/SmartDashboard.h>

SubIndexer::SubIndexer() {
  frc::SmartDashboard::PutData("Indexer/Indexer Motor", &_indexerMotor);
  frc::SmartDashboard::PutData("Indexer/Indexer Follow Motor", &_indexerFollowMotor);

  _indexerMotorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
  _indexerMotor.OverwriteConfig(_indexerMotorConfig);
  Logger::Log("Indexer/Indexer Motor", &_indexerMotor);

  _indexerFollowMotorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
  _indexerFollowMotorConfig.Follow(_indexerMotor, true);
  _indexerFollowMotor.OverwriteConfig(_indexerFollowMotorConfig);
  Logger::Log("Indexer/Indexer Follow Motor", &_indexerFollowMotor);
}

// This method will be called once per scheduler run
void SubIndexer::Periodic() {}

void SubIndexer::SimulationPeriodic() {
  _indexerSim.SetInputVoltage(_indexerMotor.CalcSimVoltage());
  _indexerSim.Update(20_ms);
  _indexerMotor.IterateSim(_indexerSim.GetAngularVelocity());
}

frc2::CommandPtr SubIndexer::SpinIndexer() {
  return StartEnd([this] { _indexerMotor.Set(0.8); }, [this] { _indexerMotor.Set(0); });
}
frc2::CommandPtr SubIndexer::ReverseIndexer() {
  return StartEnd([this] { _indexerMotor.Set(-0.8); }, [this] { _indexerMotor.Set(0); });
}
frc2::CommandPtr SubIndexer::StopIndexer() {
  return RunOnce([this] { _indexerMotor.Set(0); });
}
