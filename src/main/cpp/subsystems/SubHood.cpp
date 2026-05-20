// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubHood.h"

#include "utilities/Logger.h"

#include "frc/RobotBase.h"

SubHood::SubHood() {
  //_hoodPitchTable.insert(x_m, y_deg);
  _config.encoder.PositionConversionFactor(1 / GEAR_RATIO);
  _config.encoder.VelocityConversionFactor(1 / GEAR_RATIO);
  _config.closedLoop.Pid(16, 0, 8);
  _config.closedLoop.feedForward.kS(0.6);
  _config.SmartCurrentLimit(30);
  _config.Inverted(false);
  _config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  _hoodMotor.OverwriteConfig(_config);

  Logger::Log("Hood/Motor", &_hoodMotor);
}

// This method will be called once per scheduler run
void SubHood::Periodic() {
  auto loopStart = frc::GetTime();
  units::celsius_t hoodTemperature = _hoodMotor.GetTemperature();
  units::ampere_t hoodCurrent = _hoodMotor.GetStatorCurrent();

  AlertController::UpdateTemperatureAlert(_hoodAlertConfig, hoodTemperature);
  AlertController::UpdateCurrentAlert(_hoodAlertConfig, hoodCurrent);

  if (_hasZeroed == false && _zeroing == false) {
    _hoodMotor.StopMotor();
  }

  Logger::Log("Hood/Has zeroed", _hasZeroed);
  Logger::Log("Hood/Zeroing", _zeroing);
  Logger::Log("Hood/IsAtTarget", IsAtTarget());
    
  Logger::Log("Hood/Loop Time", (frc::GetTime() - loopStart));
}

void SubHood::SimulationPeriodic() {
  _hoodSim.SetInputVoltage(_hoodMotor.CalcSimVoltage());
  _hoodSim.Update(20_ms);
  _hoodMotor.IterateSim(_hoodSim.GetVelocity(), _hoodSim.GetAngle());
}

frc2::CommandPtr SubHood::SetPositionTarget(std::function<units::degree_t()> angle) {
  return Run([this, angle] {
    units::degree_t target = std::clamp(angle(), LOWER_LIMIT, UPPER_LIMIT);

    if (_hasZeroed) {
      _hoodMotor.SetPositionTarget(target);
    }
  });
}

frc2::CommandPtr SubHood::ZeroHood() {
  return RunOnce([this] {
    _hasZeroed = false;
    _zeroing = true;
  })
    .AndThen(ManualHoodDown())
    .Until([this] { return (HoodCurrentCheck());})
    .AndThen([this] { _hoodMotor.SetPosition(LOWER_LIMIT); })
    .FinallyDo([this] {
      _hoodMotor.StopMotor();
      _hoodMotor.SetPositionTarget(LOWER_LIMIT);
      _zeroing = false;
    });
}

bool SubHood::HoodCurrentCheck() {
  _hasZeroed = false;
  if (units::math::abs(GetHoodMotorCurrent()) > zeroingCurrentLimit || frc::RobotBase::IsSimulation()) {
    _hasZeroed = true;
    return true;
  }

  return false;
}

units::ampere_t SubHood::GetHoodMotorCurrent() {
  return _hoodMotor.GetStatorCurrent();
}

units::degree_t SubHood::GetManualAngleOffset() {
  return _manualAngleOffset;
}

void SubHood::SetManualAngleOffset(units::degree_t offset) {
  _manualAngleOffset = offset;
  Logger::Log("Hood/Manual Angle Offset", _manualAngleOffset);
}

frc2::CommandPtr SubHood::StowHood() {
  return RunOnce([this] { _hoodMotor.SetPositionTarget(STOW_ANGLE); });
}

frc2::CommandPtr SubHood::ManualHoodDown() {
  return StartEnd([this] { _hoodMotor.SetVoltage(-1_V); },
    [this] {
      auto targRot = _hoodMotor.GetPosition();
      _hoodMotor.SetPositionTarget(targRot);
    });
}

frc2::CommandPtr SubHood::SetPositionFromDistanceTarget(
  std::function<units::meter_t()> distanceToTarget) {
  return SetPositionTarget([this, distanceToTarget] {
    return _hoodPitchTable[distanceToTarget()] + _manualAngleOffset;
  });
}

frc2::CommandPtr SubHood::AddManualAngleOffset(units::degree_t offset) {
  // Using frc2 cmd so we dont require subsystem
  return frc2::cmd::RunOnce([this, offset] {
    SetManualAngleOffset(_manualAngleOffset + offset);
  });
}

bool SubHood::IsAtTarget() {
  return units::math::abs(_hoodMotor.GetPosError()) < TOLERANCE;
}

frc2::CommandPtr SubHood::MoveHoodUp1Degree() {
  return SetPositionTarget([this] { return _hoodMotor.GetPositionTarget() + 1_deg; }).WithTimeout(1_ms);
}

frc2::CommandPtr SubHood::MoveHoodDown1Degree() {
  return SetPositionTarget([this] { return _hoodMotor.GetPositionTarget() - 1_deg; }).WithTimeout(1_ms);
}

frc2::CommandPtr SubHood::HoodToEjectAngle() {
  return SubHood::GetInstance().SetPositionTarget([] { return LOWER_LIMIT + 5_deg; });
}

void SubHood::SetBrakeMode(bool brakeMode){
  rev::spark::SparkBaseConfig _brakeModeConfig;
  if (brakeMode) {
    _brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  } else {
    _brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  }
  _hoodMotor.AdjustConfigNoPersist(_brakeModeConfig);
}