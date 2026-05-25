#pragma once

#include "utilities/RobotVisualisation.h"

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/current.h>

#include "Constants.h"
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
  frc2::CommandPtr RunReverseIntake();

  bool IsIntaking();

 private:
  bool _intakeOn = false;
  ctre::phoenix6::hardware::TalonFX _motor1{canid::INTAKE_MOTOR_1};
  ctre::phoenix6::hardware::TalonFX _motor2{canid::INTAKE_MOTOR_2};

  static constexpr double GEAR_RATIO = 2.0;

  static constexpr units::kilogram_square_meter_t MOI = 0.0000001_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::Falcon500();
  frc::LinearSystem<1, 1, 1> _rollerSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEAR_RATIO);
  frc::sim::FlywheelSim _sim{_rollerSystem, MOTOR_MODEL};
};
