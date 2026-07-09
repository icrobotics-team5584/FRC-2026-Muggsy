// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubHood.h"

#include "utilities/Logger.h"

#include "frc/RobotBase.h"

SubHood::SubHood() {
  rev::spark::SparkBaseConfig config;
  config.encoder.PositionConversionFactor(1 / GEAR_RATIO);
  config.encoder.VelocityConversionFactor(1 / GEAR_RATIO);
  config.closedLoop.Pid(60, 0, 0);
  config.closedLoop.feedForward.kS(0);
  config.SmartCurrentLimit(60);
  config.Inverted(true);
  config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  _hoodMotor.OverwriteConfig(config);

  logger::Log("Hood/Motor", &_hoodMotor);

  _hoodPitchTable.insert(1.356_m, 13_deg);
  _hoodPitchTable.insert(1.98_m, 17_deg);
  _hoodPitchTable.insert(2.35_m, 19_deg);
  _hoodPitchTable.insert(3.02_m, 23_deg);
  _hoodPitchTable.insert(3.41_m, 25.5_deg);
  _hoodPitchTable.insert(3.73_m, 26_deg);
  _hoodPitchTable.insert(4.05_m, 27.5_deg);
  _hoodPitchTable.insert(4.19_m, 29_deg);
}

// This method will be called once per scheduler run
void SubHood::Periodic() {
  auto loopStart = frc::GetTime();
  units::celsius_t hoodTemperature = _hoodMotor.GetTemperature();
  units::ampere_t hoodCurrent = _hoodMotor.GetStatorCurrent();

  alertController::UpdateTemperatureAlert(_hoodAlertConfig, hoodTemperature);
  alertController::UpdateCurrentAlert(_hoodAlertConfig, hoodCurrent);

  if (!_hasZeroed && !_zeroing) {
    _hoodMotor.StopMotor();
  }

  logger::Log("Hood/Has zeroed", _hasZeroed);
  logger::Log("Hood/Zeroing", _zeroing);
  logger::Log("Hood/IsAtTarget", IsAtTarget());

  logger::Log("Hood/Loop Time", (frc::GetTime() - loopStart));
}

void SubHood::SimulationPeriodic() {
  _hoodSim.SetInputVoltage(_hoodMotor.CalcSimVoltage());
  _hoodSim.Update(20_ms);
  _hoodMotor.IterateSim(_hoodSim.GetVelocity(), _hoodSim.GetAngle());
}

frc2::CommandPtr SubHood::SetPositionTarget(const std::function<units::degree_t()>& angle) {
  return Run([this, angle] {
    units::degree_t target = std::clamp(angle(), LOWER_LIMIT, UPPER_LIMIT);

    if (_hasZeroed) {
      _hoodMotor.SetPositionTarget(target);
    }
  });
}

frc2::CommandPtr SubHood::RunZeroingSequence() {
  return frc2::cmd::RunOnce([this] {
    _hasZeroed = false;
    _zeroing = true;
    // ToggleSoftLimit(false); // disable soft limit
    _hoodMotor.SetVoltage(-1_V);  // start moving hood down slowly
  })
    .AndThen(frc2::cmd::WaitUntil([this] {
      return (units::math::abs(GetHoodMotorCurrent()) > ZEROING_CURRENT_LIMIT ||
              frc::RobotBase::IsSimulation());  // stop moving down when current limit reached (i.e.
                                                // hood hits the lower limit)
    }))
    .AndThen([this] {
      _hoodMotor.StopMotor();  // stop motors
      _hoodMotor.SetPosition(LOWER_LIMIT);
      _hasZeroed = true;
    })  // set hood motor to zero at lower limit
    .FinallyDo([this] {
      _hoodMotor.StopMotor();  // ensure motor is stopped whether zeroed or not
      _hoodMotor.SetPositionTarget(LOWER_LIMIT);
      _zeroing = false;
      // ToggleSoftLimit(true); // re-enable soft limit
    });
};

units::ampere_t SubHood::GetHoodMotorCurrent() {
  return _hoodMotor.GetStatorCurrent();
}

units::degree_t SubHood::GetManualAngleOffset() {
  return _manualAngleOffset;
}

void SubHood::SetManualAngleOffset(units::degree_t offset) {
  _manualAngleOffset = offset;
  logger::Log("Hood/Manual Angle Offset", _manualAngleOffset);
}

frc2::CommandPtr SubHood::HoodToStowAngle() {
  return SetPositionTarget([] { return STOW_ANGLE; });
}

frc2::CommandPtr SubHood::HoodToEjectAngle() {
  return SetPositionTarget([] { return LOWER_LIMIT + 5_deg; });
}

frc2::CommandPtr SubHood::SetPositionFromDistanceToTarget(
  const std::function<units::meter_t()>& distanceToTarget) {
  return SetPositionTarget(
    [this, distanceToTarget] { return _hoodPitchTable[distanceToTarget()] + _manualAngleOffset; });
}

frc2::CommandPtr SubHood::AddManualAngleOffset(units::degree_t offset) {
  // Using frc2 cmd so we dont require subsystem
  return frc2::cmd::RunOnce([this, offset] { SetManualAngleOffset(_manualAngleOffset + offset); });
}

bool SubHood::IsAtTarget() {
  return units::math::abs(_hoodMotor.GetPosError()) < TOLERANCE;
}

frc2::CommandPtr SubHood::MoveHoodUp1Degree() {
  return SetPositionTarget([this] {
    return _hoodMotor.GetPositionTarget() + 1_deg;
  }).WithTimeout(1_ms);  // stops the command from running repeatedly
}

frc2::CommandPtr SubHood::MoveHoodDown1Degree() {
  return SetPositionTarget([this] {
    return _hoodMotor.GetPositionTarget() - 1_deg;
  }).WithTimeout(1_ms);  // stops the command from running repeatedly
}

void SubHood::SetBrakeMode(bool brakeMode) {
  rev::spark::SparkBaseConfig brakeModeConfig;
  if (brakeMode) {
    brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  } else {
    brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  }
  _hoodMotor.AdjustConfigNoPersist(brakeModeConfig);
}
