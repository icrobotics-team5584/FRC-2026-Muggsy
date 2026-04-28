// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexer.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

SubIndexer::SubIndexer() {
  frc::SmartDashboard::PutData("indexer motor", &_indexerMotor);
  frc::SmartDashboard::PutData("indexer follow motor", &_indexerFollowMotor);

  _indexerMotorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
  _indexerMotor.OverwriteConfig(_indexerMotorConfig);

  _indexerFollowMotorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
 // _indexerFollowMotorConfig.Inverted(true);
  _indexerFollowMotorConfig.Follow(canid::INDEXER_MOTOR, true);
  _indexerFollowMotor.OverwriteConfig(_indexerFollowMotorConfig);
}

// This method will be called once per scheduler run
void SubIndexer::Periodic() {}

void SubIndexer::SimulationPeriodic() {}

frc2::CommandPtr SubIndexer::SpinIndexer() {
  return StartEnd([this] { _indexerMotor.Set(0.8); }, [this] { _indexerMotor.Set(0); });
}
frc2::CommandPtr SubIndexer::ReverseIndexer() {
  return StartEnd([this] { _indexerMotor.Set(-0.8); }, [this] { _indexerMotor.Set(0); });
}
frc2::CommandPtr SubIndexer::StopIndexer() {
  return RunOnce([this] { _indexerMotor.Set(0); });
}
