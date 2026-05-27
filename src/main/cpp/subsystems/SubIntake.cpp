#include "subsystems/SubIntake.h"

#include "utilities/Logger.h"

SubIntake::SubIntake() {
  ctre::phoenix6::configs::TalonFXConfiguration config;

  config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
  config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = 40_A;
  config.CurrentLimits.SupplyCurrentLowerLimit = 35_A;
  config.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = 60_A;

  _motor1.GetConfigurator().Apply(config);
  _motor2.GetConfigurator().Apply(config);
  _motor2.SetControl(ctre::phoenix6::controls::Follower(
    canid::INTAKE_MOTOR_1, ctre::phoenix6::signals::MotorAlignmentValue::Opposed));
}

void SubIntake::Periodic() {
  units::second_t loopStart = frc::GetTime();

  logger::LogFalcon("Intake/Motor1", _motor1);
  logger::LogFalcon("Intake/Motor2", _motor2);

  RobotVisualisation::GetInstance()._intakeMechWheel.SetAngle(_motor1.GetPosition().GetValue());

  logger::Log("Intake/Is Intaking", _intakeOn);

  logger::Log("Intake/Loop Time", (frc::GetTime() - loopStart));
}

void SubIntake::SimulationPeriodic() {
  _sim.SetInputVoltage(_motor1.GetSimState().GetMotorVoltage());
  _sim.Update(20_ms);
  _motor1.GetSimState().SetRotorVelocity(_sim.GetAngularVelocity() * GEAR_RATIO);
  _motor1.GetSimState().SetRotorAcceleration(_sim.GetAngularAcceleration() * GEAR_RATIO);
  _motor1.GetSimState().AddRotorPosition(_sim.GetAngularVelocity() * 20_ms * GEAR_RATIO);
}

frc2::CommandPtr SubIntake::RunIntake(double motorOut) {
  return StartEnd(
    [this, motorOut] {
      _intakeOn = true;
      _motor1.SetControl(ctre::phoenix6::controls::DutyCycleOut{motorOut});
    },
    [this] {
      _intakeOn = false;
      _motor1.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    });
}

frc2::CommandPtr SubIntake::RunReverseIntake(double motorOut) {
  return StartEnd(
    [this, motorOut] {
      _motor1.SetControl(ctre::phoenix6::controls::DutyCycleOut{-motorOut});
    },
    [this] {
      _motor1.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    });
}

bool SubIntake::IsIntaking() {
  return _intakeOn;
}
