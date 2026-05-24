// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/ICSparkFlex.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <wpi/interpolating_map.h>

#include "Constants.h"

class SubHood : public frc2::SubsystemBase {
 public:
  SubHood();
  static SubHood& GetInstance() {
    static SubHood inst;
    return inst;
  }

  void SimulationPeriodic() override;

  bool IsAtTarget();

  units::ampere_t GetHoodMotorCurrent();

  frc2::CommandPtr RunZeroingSequence();
  frc2::CommandPtr SetPositionTarget(const std::function<units::degree_t()>& angle);
  frc2::CommandPtr SetPositionFromDistanceTarget(const std::function<units::meter_t()>& distanceToTarget);
  frc2::CommandPtr MoveHoodUp1Degree();
  frc2::CommandPtr MoveHoodDown1Degree();
  void SetBrakeMode(bool brakeMode);

  frc2::CommandPtr HoodToEjectAngle();
  frc2::CommandPtr HoodToStowAngle();
  frc2::CommandPtr HoodToPassingAngle();

  units::degree_t GetManualAngleOffset();
  void SetManualAngleOffset(units::degree_t offset);
  frc2::CommandPtr AddManualAngleOffset(units::degree_t offset);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  static constexpr units::degree_t TOLERANCE = 1.0_deg;

  static constexpr units::degree_t STOW_ANGLE = 5_deg;
  static constexpr units::degree_t PASSING_ANGLE = 20_deg;
  static constexpr units::degree_t UPPER_LIMIT = 30_deg;
  static constexpr units::degree_t LOWER_LIMIT = 5_deg;

  static constexpr units::ampere_t ZEROING_CURRENT_LIMIT = 20_A;

  static constexpr units::degree_t DEFAULT_ANGLE_OFFSET = 0_deg;
  //_manualAngleOffset should only be modified using the SetManualAngleOffset function
  units::degree_t _manualAngleOffset{DEFAULT_ANGLE_OFFSET};

  bool _zeroing = false;
  bool _hasZeroed = false;

  ICSparkFlex _hoodMotor{canid::HOOD_MOTOR};

  frc::Alert _hoodHighTempAlert{"Hood Motor high temperature!", frc::Alert::AlertType::kWarning};
  frc::Alert _hoodCurrentAlert{"Hood Motor overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert _hoodStickyTempAlert{
    "Hood Motor max temperature was reached!", frc::Alert::AlertType::kWarning};
  frc::Alert _hoodStickyCurrentAlert{
    "Hood Motor max current was reached!", frc::Alert::AlertType::kWarning};
  alertController::MotorAlertConfig _hoodAlertConfig{_hoodHighTempAlert, _hoodCurrentAlert,
    _hoodStickyTempAlert, _hoodStickyCurrentAlert, 60_degC, 30_A};

  wpi::interpolating_map<units::meter_t, units::degree_t> _hoodPitchTable;

  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex(1);
  static constexpr units::kilogram_square_meter_t MOI = 0.0001_kg_sq_m;
  static constexpr double GEAR_RATIO = 10.0;

  // Sim
  static constexpr bool SIMULATE_GRAVITY = false;
  static constexpr units::degree_t STARTING_ANGLE = 5_deg;
  static constexpr units::centimeter_t ARM_LENGTH = 20_cm;

  frc::LinearSystem<2, 1, 2> _hoodSystem =
    frc::LinearSystemId::SingleJointedArmSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::SingleJointedArmSim _hoodSim{_hoodSystem, MOTOR_MODEL, GEAR_RATIO, ARM_LENGTH,
    LOWER_LIMIT, UPPER_LIMIT, SIMULATE_GRAVITY, STARTING_ANGLE};
};
