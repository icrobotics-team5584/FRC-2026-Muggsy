// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"

#include <frc/Alert.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <wpi/interpolating_map.h>

#include "Constants.h"

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();
  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }

  void SimulationPeriodic() override;

  frc2::CommandPtr SetSpeedTarget(const std::function<units::turns_per_second_t()>& speed);
  frc2::CommandPtr StopShooter();
  frc2::CommandPtr SetSpeedFromDistanceToTarget(const std::function<units::meter_t()>& distance);
  frc2::CommandPtr SpinSlowly();

  frc2::CommandPtr AddManualSpeedOffset(units::turns_per_second_t offset);
  units::turns_per_second_t GetManualSpeedOffset();
  void SetManualSpeedOffset(units::turns_per_second_t offset);

  bool IsReadyToShoot();

  units::second_t GetTimeOfFlightFromDistance(units::meter_t distance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX _shooterMotor1{canid::SHOOTER_MOTOR_1};
  ctre::phoenix6::hardware::TalonFX _shooterMotor2{canid::SHOOTER_MOTOR_2};
  ctre::phoenix6::hardware::TalonFX _shooterMotor3{canid::SHOOTER_MOTOR_3};
  ctre::phoenix6::hardware::TalonFX _shooterMotor4{canid::SHOOTER_MOTOR_4};

  static constexpr units::turns_per_second_t DEFAULT_SPEED_OFFSET = 1.5_tps;
  //_manualSpeedOffset should only be modified using the SetManualSpeedOffset function
  units::turns_per_second_t _manualSpeedOffset{DEFAULT_SPEED_OFFSET};

  ctre::phoenix6::configs::TalonFXConfiguration _shooterMotorConfig;
  ctre::phoenix6::controls::VelocityVoltage _flywheelTargetVelocity{0_tps};

  frc::Alert _shooter1HighTempAlert{
    "Shooter Motor 1 high temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter1CurrentAlert{"Shooter Motor 1 overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter1StickyTempAlert{
    "Shooter Motor 1 max temperature was reached!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter1StickyCurrentAlert{
    "Shooter Motor 1 max current was reached!", frc::Alert::AlertType::kWarning};
  alertController::MotorAlertConfig _shooter1AlertConfig{_shooter1HighTempAlert,
    _shooter1CurrentAlert, _shooter1StickyTempAlert, _shooter1StickyCurrentAlert, 60_degC, 40_A};

  frc::Alert _shooter2HighTempAlert{
    "Shooter Motor 2 high temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter2CurrentAlert{"Shooter Motor 2 overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter2StickyTempAlert{
    "Shooter Motor 2 max temperature was reached!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter2StickyCurrentAlert{
    "Shooter Motor 2 max current was reached!", frc::Alert::AlertType::kWarning};
  alertController::MotorAlertConfig _shooter2AlertConfig{_shooter2HighTempAlert,
    _shooter2CurrentAlert, _shooter2StickyTempAlert, _shooter2StickyCurrentAlert, 60_degC, 40_A};

  frc::Alert _shooter3HighTempAlert{
    "Shooter Motor 3 high temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter3CurrentAlert{"Shooter Motor 3 overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter3StickyTempAlert{
    "Shooter Motor 3 max temperature was reached!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter3StickyCurrentAlert{
    "Shooter Motor 3 max current was reached!", frc::Alert::AlertType::kWarning};
  alertController::MotorAlertConfig _shooter3AlertConfig{_shooter3HighTempAlert,
    _shooter3CurrentAlert, _shooter3StickyTempAlert, _shooter3StickyCurrentAlert, 60_degC, 40_A};

  frc::Alert _shooter4HighTempAlert{
    "Shooter Motor 4 high temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter4CurrentAlert{"Shooter Motor 4 overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter4StickyTempAlert{
    "Shooter Motor 4 max temperature was reached!", frc::Alert::AlertType::kWarning};
  frc::Alert _shooter4StickyCurrentAlert{
    "Shooter Motor 4 max current was reached!", frc::Alert::AlertType::kWarning};
  alertController::MotorAlertConfig _shooter4AlertConfig{_shooter4HighTempAlert,
    _shooter4CurrentAlert, _shooter4StickyTempAlert, _shooter4StickyCurrentAlert, 60_degC, 40_A};

  wpi::interpolating_map<units::meter_t, units::turns_per_second_t> _flywheelSpeedTable;
  wpi::interpolating_map<units::meter_t, units::second_t> _timeOfFlightTable;

  // Sim
  static constexpr units::kilogram_square_meter_t MOI = 0.05_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::KrakenX60FOC(4);
  static constexpr double GEAR_RATIO = 2;

  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _flywheelSim{_flywheelSystem, MOTOR_MODEL};
};
