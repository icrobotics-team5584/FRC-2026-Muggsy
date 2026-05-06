#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/current.h>

#include "Constants.h"
#include "utilities/RobotVisualisation.h"
class SubIntake : public frc2::SubsystemBase {
 public:
  static SubIntake& GetInstance() {
    static SubIntake inst;
    return inst;
  }
  SubIntake();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr RunIntake();

  bool IsIntakeEnabled();

 private:
  bool _intakeOn = false;
  ctre::phoenix6::hardware::TalonFX _motor1{canid::INTAKE_MOTOR_1};
  ctre::phoenix6::hardware::TalonFX _motor2{canid::INTAKE_MOTOR_2};
  static constexpr ctre::phoenix6::signals::NeutralModeValue _IDLE_MODE =
    ctre::phoenix6::signals::NeutralModeValue::Coast;
  static constexpr ctre::phoenix6::signals::InvertedValue _INVERTED =
    ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

  static constexpr units::ampere_t _SUPPLY_CURRENT_LIMT = 40_A;
  static constexpr units::ampere_t _SUPPLY_CURRENT_LOWER_LIMIT = 35_A;
  static constexpr units::ampere_t _STATOR_CURRENT_LIMIT = 60_A;
  static constexpr units::second_t _SUPPLY_CURRENT_LOWER_TIME = 0.1_s;
  static constexpr double _P = 1.0;
  static constexpr double _I = 0.0;
  static constexpr double _D = 0.0;
  static constexpr double _GEAR_RATIO = 0.5; /* 2:1 */

  /* The TalonFX's sim SetRotor(Acceleration|Velocity)() set the motor's
   * without velocity without taking in gear ratio. Ergo, the simulation
   * will have a gearing of 1. */
  static constexpr units::kilogram_square_meter_t _MOI = 0.0000001_kg_sq_m;
  static constexpr frc::DCMotor _MOTOR_MODEL = frc::DCMotor::Falcon500();
  frc::LinearSystem<1, 1, 1> _rollerSystem
    = frc::LinearSystemId::FlywheelSystem(_MOTOR_MODEL, _MOI, 1.0);
  frc::sim::FlywheelSim _sim{_rollerSystem, _MOTOR_MODEL};
};
