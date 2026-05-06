#include "subsystems/SubIntake.h"

#include "utilities/Logger.h"

SubIntake::SubIntake() {
  ctre::phoenix6::configs::TalonFXConfiguration config;

  config.Feedback.SensorToMechanismRatio = _GEAR_RATIO;
  config.Slot0.kP = _P;
  config.Slot0.kI = _I;
  config.Slot0.kD = _D;
  config.MotorOutput.Inverted = _INVERTED;
  config.MotorOutput.NeutralMode = _IDLE_MODE;
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = _SUPPLY_CURRENT_LIMT;
  config.CurrentLimits.SupplyCurrentLowerLimit = _SUPPLY_CURRENT_LOWER_LIMIT;
  config.CurrentLimits.SupplyCurrentLowerTime = _SUPPLY_CURRENT_LOWER_TIME;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = _STATOR_CURRENT_LIMIT;

  _motor1.GetConfigurator().Apply(config);
  _motor2.SetControl(ctre::phoenix6::controls::Follower(
    canid::INTAKE_MOTOR_2, ctre::phoenix6::signals::MotorAlignmentValue::Opposed));
}

void SubIntake::Periodic() {
  Logger::LogFalcon("Intake/Motor1", _motor1);
  Logger::LogFalcon("Intake/Motor2", _motor2);

  RobotVisualisation::GetInstance()._indexerMechWheel.SetAngle(_motor1.GetPosition().GetValue());

  Logger::Log("Intake/Is Intaking",
    !(_motor1.GetMotorVoltage().GetValue() == 0_V && _motor2.GetMotorVoltage().GetValue() == 0_V));
}

void SubIntake::SimulationPeriodic() {
  _sim.SetInputVoltage(_motor1.GetSimState().GetMotorVoltage());
  _sim.Update(20_ms);
  _motor1.GetSimState().SetRotorVelocity(_sim.GetAngularVelocity()*_GEAR_RATIO);
  _motor1.GetSimState().SetRotorAcceleration(_sim.GetAngularAcceleration()*_GEAR_RATIO);
  _motor1.GetSimState().AddRotorPosition(_sim.GetAngularVelocity()*20_ms*_GEAR_RATIO);
}

frc2::CommandPtr SubIntake::RunIntake() {
  return frc2::cmd::StartEnd(
    [this] {
      _intakeOn = true;

      _motor1.SetControl(ctre::phoenix6::controls::VoltageOut{5_V});
      _motor2.SetControl(ctre::phoenix6::controls::VoltageOut{5_V});
    },
    [this] {
      _intakeOn = false;

      _motor1.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
      _motor2.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
    });
}

bool SubIntake::IsIntakeEnabled() {
  return _intakeOn;
}
