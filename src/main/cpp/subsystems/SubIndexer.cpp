// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "utilities/Logger.h"

SubIndexer::SubIndexer() {
  frc::SmartDashboard::PutData("indexer motor", &_indexerMotor);
  frc::SmartDashboard::PutData("indexer follow motor", &_indexerFollowMotor);

  _indexerMotorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
  _indexerMotor.OverwriteConfig(_indexerMotorConfig);
  Logger::Log("Indexer/Indexer Motor", &_indexerMotor);
  

  _indexerFollowMotorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
  _indexerFollowMotorConfig.Inverted(true);
  _indexerFollowMotorConfig.Follow(_indexerMotor, true);
  _indexerFollowMotor.OverwriteConfig(_indexerFollowMotorConfig);
  Logger::Log("Indexer/Indexer Follow Motor", &_indexerFollowMotor);
}

// This method will be called once per scheduler run
void SubIndexer::Periodic() {
  units::celsius_t indexerTemperature = _indexerMotor.GetTemperature();

  units::ampere_t indexerCurrent = _indexerMotor.GetStatorCurrent();
}


void SubIndexer::SimulationPeriodic() {
  _sim.SetInputVoltage(_indexerMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _indexerMotor.IterateSim(_sim.GetAngularVelocity());
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
