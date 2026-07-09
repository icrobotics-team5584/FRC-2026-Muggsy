// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDeploy.h"

#include "utilities/Logger.h"
#include "utilities/RobotVisualisation.h"

#include <frc/RobotBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

SubDeploy::SubDeploy() {
  _motorConfig.SmartCurrentLimit(60);
  _motorConfig.encoder.PositionConversionFactor(1 / GEARING);
  _motorConfig.encoder.VelocityConversionFactor(1 / GEARING);
  _motorConfig.closedLoop.Pid(0.25, 0.0, 0.0);
  _motorConfig.Inverted(true);
  _motorConfig.SetIdleMode(rev::spark::SparkBaseConfig::kCoast);

  _motor.OverwriteConfig(_motorConfig);

  logger::Log("Deploy/Motor1", &_motor);
}

void SubDeploy::Periodic() {
  units::second_t loopStart = frc::GetTime();
  if (!_hasZeroed && !_zeroing) {
    _motor.StopMotor();
  }

  logger::Log("Deploy/Has Zeroed", _hasZeroed);
  logger::Log("Deploy/Zeroing", _zeroing);
  logger::Log("Deploy/On Target", IsAtTarget());
  logger::Log("Deploy/Extension", GetLength());
  logger::Log("Deploy/Loop Time", (frc::GetTime() - loopStart));
}

void SubDeploy::SimulationPeriodic() {
  _rackSim.SetInputVoltage(_motor.CalcSimVoltage());

  _rackSim.Update(20_ms);
  RobotVisualisation::GetInstance()._deployLigament->SetLength(_rackSim.GetPosition().value());

  _motor.IterateSim(ConvertVelocityToAngularVelocity(_rackSim.GetVelocity()));
}

/* Command Functions*/
frc2::CommandPtr SubDeploy::Zero() {
  return RunOnce([this] {
    _zeroing = true;
    _hasZeroed = false;
    _motor.SetVoltage(1_V);
  })
    .AndThen(frc2::cmd::WaitUntil([this] {
      return units::math::abs(_motor.GetStatorCurrent()) > ZERO_CURRENT_LIMIT ||
             frc::RobotBase::IsSimulation();
    }))
    .AndThen([this] {
      _motor.StopMotor();
      _motor.SetPosition(ConvertLengthToPosition(MAX_LENGTH));
      _hasZeroed = true;
    })
    .FinallyDo([this] {
      _motor.StopMotor();
      _zeroing = false;
    });
}

void SubDeploy::SetLength(units::meter_t length) {
  if (_hasZeroed) {
    units::meter_t clampedLength = std::clamp(length, 0_m, MAX_LENGTH);
    _motor.SetPositionTarget(ConvertLengthToPosition(clampedLength));
  }
}

frc2::CommandPtr SubDeploy::ExtendTo(units::meter_t length) {
  return RunOnce([this, length] { SetLength(length); });
}

frc2::CommandPtr SubDeploy::ExtendToLerp(double t) {
  t = std::clamp(t, 0.0, 1.0);
  units::meter_t length = MAX_LENGTH * t;
  return ExtendTo(length);
}

frc2::CommandPtr SubDeploy::ManualExtendDown() {
  return RunOnce([this] {
    if (_hasZeroed && GetLength() > 0_m) {
      _motor.SetVoltage(-1_V);
    }
  })
    .AndThen(frc2::cmd::WaitUntil([this] { return GetLength() <= 0_m; }))
    .FinallyDo([this] { _motor.SetVoltage(0_V); });
}

frc2::CommandPtr SubDeploy::ManualExtendUp() {
  return RunOnce([this] {
    if (_hasZeroed && GetLength() < MAX_LENGTH) {
      _motor.SetVoltage(1_V);
    }
  })
    .AndThen(frc2::cmd::WaitUntil([this] { return GetLength() >= MAX_LENGTH; }))
    .FinallyDo([this] { _motor.SetVoltage(0_V); });
}

frc2::CommandPtr SubDeploy::ExtendToStow() {
  return ExtendTo(STOW_LENGTH);
}

frc2::CommandPtr SubDeploy::ExtendToDeploy() {
  return ExtendTo(DEPLOY_LENGTH);
}

frc2::CommandPtr SubDeploy::ToggleStow() {
  return StartEnd([this] { return SetLength(SubDeploy::STOW_LENGTH); },
    [this] { SetLength(SubDeploy::DEPLOY_LENGTH); });
}

/* Instant Functions*/

bool SubDeploy::IsAtTarget() {
  return _motor.OnPosTarget(TOLERANCE);
}

units::meter_t SubDeploy::GetLength() {
  return ConvertPositionToLength(_motor.GetPosition());
}

units::meter_t SubDeploy::ConvertPositionToLength(units::turn_t pos) {
  return (pos.value() * PINION_CIRCUM);
}

units::turn_t SubDeploy::ConvertLengthToPosition(units::meter_t length) {
  return 1_tr * (length / PINION_CIRCUM).value();
}

units::turns_per_second_t SubDeploy::ConvertVelocityToAngularVelocity(
  units::meters_per_second_t velo) {
  return 1_tps * (velo / PINION_CIRCUM).value();
}

void SubDeploy::DeployAfterShoot() {
  SetLength(DEPLOY_LENGTH);
}
