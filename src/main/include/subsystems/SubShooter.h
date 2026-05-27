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
  frc2::CommandPtr Stop();
  frc2::CommandPtr SetSpeedFromDistanceTarget(
    const std::function<units::meter_t()>& distance, const std::function<bool()>& isPassing);
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

  static constexpr units::turns_per_second_t DEFAULT_SPEED_OFFSET = 0_tps;
  //_manualSpeedOffset should only be modified using the SetManualSpeedOffset function
  units::turns_per_second_t _manualSpeedOffset{DEFAULT_SPEED_OFFSET};

  ctre::phoenix6::configs::TalonFXConfiguration _shooterMotorConfig;
  ctre::phoenix6::controls::VelocityVoltage _flywheelTargetVelocity{0_tps};

  std::shared_ptr<alertController::AlertConfig> _shooterMotor1AlertConfig =
    alertController::AlertConfig::Create("Shooter Motor 1", &_shooterMotor1, -1_degC, -1_A);
  std::shared_ptr<alertController::AlertConfig> _shooterMotor2AlertConfig =
    alertController::AlertConfig::Create("Shooter Motor 2", &_shooterMotor2, 60_degC, 40_A);
  std::shared_ptr<alertController::AlertConfig> _shooterMotor3AlertConfig =
    alertController::AlertConfig::Create("Shooter Motor 3", &_shooterMotor3, 60_degC, 40_A);
  std::shared_ptr<alertController::AlertConfig> _shooterMotor4AlertConfig =
    alertController::AlertConfig::Create("Shooter Motor 4", &_shooterMotor4, 60_degC, 40_A);

  wpi::interpolating_map<units::meter_t, units::turns_per_second_t> _flyWheelSpeedTableScoring;
  wpi::interpolating_map<units::meter_t, units::turns_per_second_t> _flyWheelSpeedTablePassing;
  wpi::interpolating_map<units::meter_t, units::second_t> _timeOfFlightTable;

  // Sim
  static constexpr units::kilogram_square_meter_t MOI = 0.05_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::KrakenX60FOC(4);
  static constexpr double GEAR_RATIO = 10;

  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _flywheelSim{_flywheelSystem, MOTOR_MODEL};
};
