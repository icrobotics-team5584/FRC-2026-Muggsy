// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

#include "utilities/Logger.h"

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

SubShooter::SubShooter() {
  _shooterMotorConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  _shooterMotorConfig.Voltage.PeakReverseVoltage = 0_V;

  _shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  _shooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20.0_A;
  _shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0_A;
  _shooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5_s;
  _shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  _shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 60.0_A;

  // PIDs
  _shooterMotorConfig.Slot0.kP = 0.0;
  _shooterMotorConfig.Slot0.kI = 0;
  _shooterMotorConfig.Slot0.kD = 0;
  _shooterMotorConfig.Slot0.kV = 0.235;

  _shooterMotorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
  _shooterMotorConfig.MotorOutput.Inverted = true;

  _shooterMotor2.SetControl(ctre::phoenix6::controls::Follower(
    canid::SHOOTER_MOTOR_1, ctre::phoenix6::signals::MotorAlignmentValue::Aligned));
  _shooterMotor3.SetControl(ctre::phoenix6::controls::Follower(
    canid::SHOOTER_MOTOR_1, ctre::phoenix6::signals::MotorAlignmentValue::Opposed));
  _shooterMotor4.SetControl(ctre::phoenix6::controls::Follower(
    canid::SHOOTER_MOTOR_1, ctre::phoenix6::signals::MotorAlignmentValue::Opposed));

  _shooterMotor1.GetConfigurator().Apply(_shooterMotorConfig);
  _shooterMotor2.GetConfigurator().Apply(_shooterMotorConfig);
  _shooterMotor3.GetConfigurator().Apply(_shooterMotorConfig);
  _shooterMotor4.GetConfigurator().Apply(_shooterMotorConfig);

  // Shooter tables - to be tuned
  //_timeOfFlightTable.insert(x_m, y_s);

  // Testing Values
  _flyWheelSpeedTableScoring.insert(1_m, 5_tps);
  //_flyWheelSpeedTableScoring.insert(5_m, 10_tps);

  //_flyWheelSpeedTablePassing.insert(x_m, y_tps);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
  auto loopStart = frc::GetTime();

  logger::LogFalcon("Shooter/Motor1", _shooterMotor1);
  logger::LogFalcon("Shooter/Motor2", _shooterMotor2);
  logger::LogFalcon("Shooter/Motor3", _shooterMotor3);
  logger::LogFalcon("Shooter/Motor4", _shooterMotor4);
  logger::Log("Shooter/IsReadyToShoot", IsReadyToShoot());

  alertController::UpdateTemperatureAlert(
    _shooter1AlertConfig, _shooterMotor1.GetDeviceTemp().GetValue());
  alertController::UpdateCurrentAlert(
    _shooter1AlertConfig, _shooterMotor1.GetStatorCurrent().GetValue());

  alertController::UpdateTemperatureAlert(
    _shooter2AlertConfig, _shooterMotor2.GetDeviceTemp().GetValue());
  alertController::UpdateCurrentAlert(
    _shooter2AlertConfig, _shooterMotor2.GetStatorCurrent().GetValue());

  alertController::UpdateTemperatureAlert(
    _shooter3AlertConfig, _shooterMotor3.GetDeviceTemp().GetValue());
  alertController::UpdateCurrentAlert(
    _shooter3AlertConfig, _shooterMotor3.GetStatorCurrent().GetValue());

  alertController::UpdateTemperatureAlert(
    _shooter4AlertConfig, _shooterMotor4.GetDeviceTemp().GetValue());
  alertController::UpdateCurrentAlert(
    _shooter4AlertConfig, _shooterMotor4.GetStatorCurrent().GetValue());

  logger::Log("Shooter/Loop Time", (frc::GetTime() - loopStart));
}

void SubShooter::SimulationPeriodic() {
  auto& simState = _shooterMotor1.GetSimState();
  simState.SetSupplyVoltage(12.0_V);

  _flywheelSim.SetInputVoltage(simState.GetMotorVoltage());
  _flywheelSim.Update(20_ms);

  simState.SetRotorVelocity(_flywheelSim.GetAngularVelocity() * GEAR_RATIO);
  simState.SetRotorAcceleration(_flywheelSim.GetAngularAcceleration() * GEAR_RATIO);
  simState.AddRotorPosition(_flywheelSim.GetAngularVelocity() * GEAR_RATIO * 20_ms);
}

frc2::CommandPtr SubShooter::SetSpeedTarget(
  const std::function<units::turns_per_second_t()>& speed) {
  return Run(
    [this, speed] { _shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(speed())); });
}

frc2::CommandPtr SubShooter::Stop() {
  return RunOnce(
    [this] { _shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(0_tps)); });
}

frc2::CommandPtr SubShooter::SpinSlowly() {
  return Run([this] { _shooterMotor1.SetControl(_flywheelTargetVelocity.WithVelocity(10_tps)); });
}

frc2::CommandPtr SubShooter::AddManualSpeedOffset(units::turns_per_second_t offset) {
  // Using frc2 cmd so we dont require subsystem
  return frc2::cmd::RunOnce([this, offset] { SetManualSpeedOffset(_manualSpeedOffset + offset); });
}

bool SubShooter::IsReadyToShoot() {
  bool motor1Ready = units::math::abs(_shooterMotor1.GetVelocity().GetValue() -
                                      _flywheelTargetVelocity.Velocity) < 20_tps;
  bool motor2Ready = units::math::abs(_shooterMotor2.GetVelocity().GetValue() -
                                      _flywheelTargetVelocity.Velocity) < 20_tps;
  bool motor3Ready = units::math::abs(_shooterMotor3.GetVelocity().GetValue() -
                                      _flywheelTargetVelocity.Velocity) < 20_tps;
  bool motor4Ready = units::math::abs(_shooterMotor4.GetVelocity().GetValue() -
                                      _flywheelTargetVelocity.Velocity) < 20_tps;
  return motor1Ready && motor2Ready && motor3Ready && motor4Ready;
}

frc2::CommandPtr SubShooter::SetSpeedFromDistanceTarget(
  const std::function<units::meter_t()>& distance, const std::function<bool()>& isPassing) {
  return SetSpeedTarget([this, distance, isPassing] {
    logger::Log("Shooter/Distance Target", distance());
    return isPassing() ? _flyWheelSpeedTablePassing[distance()]
                       : _flyWheelSpeedTableScoring[distance()] + _manualSpeedOffset;
  });
}

units::turns_per_second_t SubShooter::GetManualSpeedOffset() {
  return _manualSpeedOffset;
}

void SubShooter::SetManualSpeedOffset(units::turns_per_second_t offset) {
  _manualSpeedOffset = offset;
  logger::Log("Shooter/Manual Speed Offset", _manualSpeedOffset);
}

units::second_t SubShooter::GetTimeOfFlightFromDistance(units::meter_t distance) {
  return _timeOfFlightTable[distance];
}
