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
#include <units/moment_of_inertia.h>

#include "Constants.h"

class SubFeeder : public frc2::SubsystemBase {
 public:
  static SubFeeder& GetInstance() {
    static SubFeeder inst;
    return inst;
  }
  SubFeeder();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SimulationPeriodic() override;

  frc2::CommandPtr Feed();
  frc2::CommandPtr FeedBackwards();
  frc2::CommandPtr FeederOn();
  frc2::CommandPtr FeederOff();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ICSparkFlex _feederMotor{canid::FEEDER_MOTOR};
  rev::spark::SparkFlexConfig _feederMotorConfig;


  // Simulation components
  static constexpr double GEARING = 22.0 / 18.0;
  static constexpr units::kilogram_square_meter_t MOI = 0.000001_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex(1);

  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};
};
