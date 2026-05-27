// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/config/SparkFlexConfig.h>

#include "Constants.h"
#include "utilities/ICSparkFlex.h"

class SubDeploy : public frc2::SubsystemBase {
 public:
  SubDeploy();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr Zero();
  frc2::CommandPtr ExtendTo(units::meter_t height);
  frc2::CommandPtr ExtendToLerp(double t);
  frc2::CommandPtr MannualExtendDown();
  frc2::CommandPtr MannualExtendUp();
  frc2::CommandPtr Stow();

  units::meter_t GetHeight();
  bool IsAtTarget();

 private:
  units::meter_t ConvertPositionToHeight(units::turn_t pos);
  units::turn_t ConvertHeightToPosition(units::meter_t height);

  ICSparkFlex _motor{canid::DEPLOY_MOTOR};
  rev::spark::SparkFlexConfig _motorConfig;

  static constexpr units::meter_t _UPPER_LIMIT = 205.8_mm;
  static constexpr units::meter_t _LOWER_LIMIT = 0_m;
  static constexpr units::meter_t _STOW_HEIGHT = 0_m;
  static constexpr units::meter_t _PINION_RAD = 0.05_m;
  static constexpr units::meter_t _PINION_CIRCUM = _PINION_RAD * 2 * 3.14;
  static constexpr units::meter_t _TOLERANCE = 5_mm;
  ICSparkFlex _motor{canid::DEPLOY_MOTOR};

  rev::spark::SparkFlexConfig _motorConfig;

  static constexpr units::ampere_t CURRENT_LIMIT = 60_A;
  static constexpr double GEARING = 1.0;

  /* Simulation */
  static constexpr units::kilogram_t MASS = 5_kg;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _linearArmSystem = 
    frc::LinearSystemId::ElevatorSystem(MOTOR_MODEL, MASS, 10_mm, GEARING);
};
