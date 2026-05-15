// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hood/SubHood.h"

#include "utilities/Logger.h"

#include "frc/RobotBase.h"

SubHood::SubHood() {
  //_hoodPitchTable.insert(x_m, y_deg);

  _hoodMotor->ConfigMotor();
}

// This method will be called once per scheduler run
void SubHood::Periodic() {
  auto loopStart = frc::GetTime();
  units::celsius_t hoodTemperature = _hoodMotor->GetTemperature();
  units::ampere_t hoodCurrent = _hoodMotor->GetCurrent();

  AlertController::UpdateTemperatureAlert(_hoodAlertConfig, hoodTemperature);
  AlertController::UpdateCurrentAlert(_hoodAlertConfig, hoodCurrent);

  if (_hasZeroed == false && _zeroing == false) {
    _hoodMotor->StopMotor();
  }

  Logger::Log("Hood/Has zeroed", _hasZeroed);
  Logger::Log("Hood/Zeroing", _zeroing);
  Logger::Log("Hood/IsAtTarget", HoodIsAtTarget());
  
  _hoodMotor->Log("Hood/Motor");
  
  Logger::Log("Hood/Loop Time", (frc::GetTime() - loopStart));
}

void SubHood::SimulationPeriodic() {
  _hoodSim.SetInputVoltage(_hoodMotor->CalcSimVoltage());
  _hoodSim.Update(20_ms);
  _hoodMotor->IterateSim(_hoodSim.GetVelocity(), _hoodSim.GetAngle());
}

frc2::CommandPtr SubHood::SetHoodPositionTarget(std::function<units::degree_t()> angle) {
    return Run([this, angle] {
    units::degree_t target = angle();

    if (target > UPPER_LIMIT) {
      target = UPPER_LIMIT;
    }

    if (target < LOWER_LIMIT) {
      target = LOWER_LIMIT;
    }

    if (_hasZeroed) {
      _hoodMotor->SetPositionTarget(target);
    }
    
  });
}

frc2::CommandPtr SubHood::ZeroHood() {
  return RunOnce([this] { _zeroing = true; })
    .AndThen(ManualHoodDown())
    .Until([this] { return (HoodCurrentCheck());})
    .AndThen([this] { _hoodMotor->SetPosition(LOWER_LIMIT); })
    .FinallyDo([this] {
      _hoodMotor->StopMotor();
      _hoodMotor->SetPositionTarget(LOWER_LIMIT);
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
  return _hoodMotor->GetCurrent();
}

units::degree_t SubHood::GetManualAngleOffset() {
  return _manualAngleOffset;
}

void SubHood::SetManualAngleOffset(units::degree_t offset) {
  _manualSpeedOffset = offset;
  Logger::Log("Shooter/Manual Speed Offset", _manualSpeedOffset);
}

frc2::CommandPtr SubHood::StowHood() {
  return RunOnce([this] { _hoodMotor->SetPositionTarget(STOW_ANGLE); });
}

frc2::CommandPtr SubHood::ManualHoodDown() {
  return StartEnd([this] { _hoodMotor->SetVoltage(-1_V); },
    [this] {
      auto targRot = _hoodMotor->GetPosition();
      _hoodMotor->SetPositionTarget(targRot);
    });
}

frc2::CommandPtr SubHood::SetHoodPositionTargetFromDist(
  std::function<units::meter_t()> distanceToTarget) {
  return SetHoodPositionTarget([this, distanceToTarget] {
    return _hoodPitchTable[distanceToTarget()] + Logger::Tune("Hood/Angle Manual Offset", DEFAULT_HOOD_OFFSET);
  });
}

frc2::CommandPtr SubHood::AddManualAngleOffset(units::degree_t offset) {
  // Using frc2 cmd so we dont require subsystem
  return frc2::cmd::RunOnce([offset] {
    units::degree_t oldOffset = Logger::Tune("Hood/Angle Manual Offset", DEFAULT_HOOD_OFFSET);
    units::degree_t newOffset = oldOffset + offset;
    Logger::Log("Hood/Angle Manual Offset", newOffset);
  });
}

bool SubHood::HoodIsAtTarget() {
  return units::math::abs(_hoodMotor->GetPositionError()) < TOLERANCE;
}

frc2::CommandPtr SubHood::MoveHoodUp1Degree() {
  return SetHoodPositionTarget([this] { return _hoodMotor->GetPositionTarget() + 1_deg; }).WithTimeout(1_ms);
}

frc2::CommandPtr SubHood::MoveHoodDown1Degree() {
  return SetHoodPositionTarget([this] { return _hoodMotor->GetPositionTarget() - 1_deg; }).WithTimeout(1_ms);
}

frc2::CommandPtr SubHood::HoodToEjectAngle() {
  return SubHood::GetInstance().SetHoodPositionTarget([] { return LOWER_LIMIT + 5_deg; });
}

void SubHood::SetBrakeMode(bool brakeMode){
  if (brakeMode == true){
    _hoodMotor->SetBrakeMode(true);
  }
  else {
    _hoodMotor->SetBrakeMode(false);
  }
}
