// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include <numbers>
#include <rev/config/SparkFlexConfig.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class SubDeploy : public frc2::SubsystemBase {
 public:
  SubDeploy();
  static SubDeploy& GetInstance() {
    static SubDeploy inst;
    return inst;
  }

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr Zero(units::second_t forceActuateTime);
  frc2::CommandPtr ExtendTo(units::meter_t length);
  frc2::CommandPtr ExtendToLerp(double t);
  frc2::CommandPtr ManualExtendDown();
  frc2::CommandPtr ManualExtendUp();
  frc2::CommandPtr ExtendToStow();
  frc2::CommandPtr ExtendToDeploy();
  frc2::CommandPtr ToggleStow();

  units::meter_t GetLength();
  bool IsAtTarget();
  void SetBrakeMode(bool brakeMode);

 private:
  void SetLength(units::meter_t length);

  static units::meter_t ConvertPositionToLength(units::turn_t pos);
  static units::turn_t ConvertLengthToPosition(units::meter_t length);
  static units::turns_per_second_t ConvertVelocityToAngularVelocity(
    units::meters_per_second_t velo);

  ICSparkFlex _motor{canid::DEPLOY_MOTOR};
  rev::spark::SparkFlexConfig _motorConfig;
  bool _hasZeroed = false;
  bool _zeroing = false;

  static constexpr units::ampere_t ZERO_CURRENT_LIMIT = 45_A;
  static constexpr units::meter_t MAX_LENGTH = 0.308_m;
  static constexpr units::meter_t STOW_LENGTH = 0.01_m;
  static constexpr units::meter_t DEPLOY_LENGTH = MAX_LENGTH - 0.005_m;
  static constexpr units::meter_t PINION_RAD = 1_in / 2.0;
  static constexpr units::meter_t PINION_CIRCUM = PINION_RAD * 2 * std::numbers::pi;
  static constexpr double GEARING = 5.0;

  /* Since ConvertLengthToPosition is defined in a class, it cannot be defined
   * as a static constexpr function and called in a constexpr expression within
   * the class. Constexpr functions require to be defined before being called
   * and the compiler will not recognise the function defintion as complete
   * until the entire class is also complete. */
  units::turn_t TOLERANCE = ConvertLengthToPosition(0.05_m);

  /* Simulation */
  static constexpr units::kilogram_t MASS = 5_kg;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<2, 1, 2> _rackSystem =
    frc::LinearSystemId::ElevatorSystem(MOTOR_MODEL, MASS, PINION_RAD, GEARING);
  frc::sim::ElevatorSim _rackSim{_rackSystem, MOTOR_MODEL, 0_m, MAX_LENGTH, false, STOW_LENGTH};
};
