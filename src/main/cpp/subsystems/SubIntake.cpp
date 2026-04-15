#include "subsystems/SubIntake.h"

#include "utilities/Logger.h"

SubIntake::SubIntake() {
	rev::spark::SparkFlexConfig motor1Config;
	rev::spark::SparkFlexConfig motor2Config;
	_motorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
	_motorConfig.encoder.PositionConversionFactor(1 / _GEAR_RATIO);
	_motorConfig.encoder.VelocityConversionFactor(1 / _GEAR_RATIO);
	_motorConfig.closedLoop.Pid(_P, _I, _D);
	_motorConfig.Inverted(_INVERTED);
	_motorConfig.SetIdleMode(_IDLE_MODE);


	_motor1.OverwriteConfig(motor1Config.Apply(_motorConfig));
	_motor2.OverwriteConfig(motor2Config.Apply(_motorConfig).Follow(
    	canid::INTAKE_MOTOR_1).Inverted(!_INVERTED));
}

void SubIntake::Periodic() {
	Logger::Log("Intake/Motor1", &_motor1);
	Logger::Log("Intake/Motor2", &_motor2);
}

frc2::CommandPtr SubIntake::IntakeEnable() {
	return frc2::cmd::RunOnce([this] {
		_motor1.SetVoltage(5_V);
		_motor2.SetVoltage(5_V);
	});
}

frc2::CommandPtr SubIntake::IntakeDisable() {
	return frc2::cmd::RunOnce([this] {
		_motor1.SetVoltage(0_V);
		_motor2.SetVoltage(0_V);
	});
}
