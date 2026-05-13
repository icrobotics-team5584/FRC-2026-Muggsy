#include "subsystems/SubIntake.h"

#include "utilities/Logger.h"

SubIntake::SubIntake() {
  ctre::phoenix6::configs::TalonFXConfiguration config;

  config.Feedback.SensorToMechanismRatio = 2.0;
  config.Slot0.kP = 1.0;
  config.Slot0.kI = 0.0;
  config.Slot0.kD = 0.0;
  config.MotorOutput.Inverted =  ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = 40_A;
  config.CurrentLimits.SupplyCurrentLowerLimit = 35_A;
  config.CurrentLimits.SupplyCurrentLowerTime = 0.1_A;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = 60_A;

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
  _motor1.GetSimState().SetRotorVelocity(_sim.GetAngularVelocity() * _GEAR_RATIO);
  _motor1.GetSimState().SetRotorAcceleration(_sim.GetAngularAcceleration() * _GEAR_RATIO);
  _motor1.GetSimState().AddRotorPosition(_sim.GetAngularVelocity() * 20_ms * _GEAR_RATIO);
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
