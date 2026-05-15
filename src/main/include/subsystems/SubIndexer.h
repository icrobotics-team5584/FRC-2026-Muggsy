// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkFlexConfigAccessor.h>

#include "Constants.h"

class SubIndexer : public frc2::SubsystemBase {
 public:
  static SubIndexer& GetInstance() {
    static SubIndexer inst;
    return inst;
  }
  SubIndexer();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr SpinIndexer();
  frc2::CommandPtr ReverseIndexer();
  frc2::CommandPtr StopIndexer();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Config
  ICSparkFlex _indexerMotor{canid::INDEXER_MOTOR};
  ICSparkFlex _indexerFollowMotor{canid::INDEXER_FOLLOW_MOTOR};

  rev::spark::SparkFlexConfig _indexerMotorConfig;
  rev::spark::SparkFlexConfig _indexerFollowMotorConfig;

  static constexpr units::ampere_t CURRENT_LIMIT = 60_A;

  // Simulation Config
  static constexpr double GEARING = 1.0;
  static constexpr units::kilogram_square_meter_t MOI = 1_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _indexerSim{_flywheelSystem, MOTOR_MODEL};
};
