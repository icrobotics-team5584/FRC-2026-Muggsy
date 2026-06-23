// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"

#include "utilities/Logger.h"

SubFeeder::SubFeeder() {
  logger::Log("Feeder/FeederMotor", &_feederMotor);

  static constexpr double p = 0.005;
  static constexpr double i = 0.0;
  static constexpr double d = 0.0;
  static constexpr double f = 0.0024;

  _feederMotorConfig.SmartCurrentLimit(60);
  _feederMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  _feederMotorConfig.closedLoop.Pid(p, i, d);
  _feederMotorConfig.closedLoop.feedForward.kV(f);
  _feederMotorConfig.encoder.PositionConversionFactor(1 / GEARING);
  _feederMotorConfig.encoder.VelocityConversionFactor(1 / GEARING);
  _feederMotor.OverwriteConfig(_feederMotorConfig);
}

frc2::CommandPtr SubFeeder::Feed() {
  return StartEnd(
    [this] { _feederMotor.SetVelocityTarget(3000_rpm); }, [this] { _feederMotor.Set(0); });
}

frc2::CommandPtr SubFeeder::FeedBackwards() {
  return StartEnd(
    [this] { _feederMotor.SetVelocityTarget(-3000_rpm); }, [this] { _feederMotor.Set(0); });
}

frc2::CommandPtr SubFeeder::FeederOn() {
  return RunOnce([this] { _feederMotor.Set(1); });
}

frc2::CommandPtr SubFeeder::FeederOff() {
  return RunOnce([this] { _feederMotor.Set(0); });
}

void SubFeeder::Periodic() {
  auto loopStart = frc::GetTime();

  logger::Log("Feeder/Loop Time", (frc::GetTime() - loopStart));
}

void SubFeeder::SimulationPeriodic() {
  _sim.SetInputVoltage(_feederMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _feederMotor.IterateSim(_sim.GetAngularVelocity());
}