// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <math.h>

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
  frc2::CommandPtr ExtendTo(units::meter_t length);
  frc2::CommandPtr ExtendToLerp(double t);
  frc2::CommandPtr MannualExtendDown();
  frc2::CommandPtr MannualExtendUp();
  frc2::CommandPtr Stow();

  units::meter_t GetLength();
  bool IsAtTarget();

 private:
  units::meter_t ConvertPositionToLength(units::turn_t pos);
  units::turn_t ConvertLengthToPosition(units::meter_t length);

  ICSparkFlex _motor{canid::DEPLOY_MOTOR};
  rev::spark::SparkFlexConfig _motorConfig;
  bool _hasZeroed = false;
  bool _zeroing = false;

  static constexpr units::ampere_t ZERO_CURRENT_LIMIT = 40_A;
  static constexpr units::meter_t MAX_LENGTH = 300_mm;
  static constexpr units::meter_t STOW_LENGTH = 0_m;
  static constexpr units::meter_t PINION_RAD = 0.05_m;
  static constexpr units::meter_t PINION_CIRCUM = PINION_RAD * 2 * M_PI;
  static constexpr double GEARING = 1.0;

  /* Simulation */
  static constexpr units::kilogram_t MASS = 5_kg;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _linearArmSystem = 
    frc::LinearSystemId::ElevatorSystem(MOTOR_MODEL, MASS, PINION_RAD, GEARING);
};
