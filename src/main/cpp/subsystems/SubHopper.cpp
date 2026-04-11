#include <frc2/command/Commands.h>
#include <units/math.h>

#include "subsystems/SubHopper.h"
#include "utilities/ICSparkFlex.h"
#include "utilities/Logger.h"
#include "Constants.h"


SubHopper::SubExtend() {
	_motor1Config.SmartCurrentLimit(_CURRENT_LIMIT.value());
	_motor1Config.encoder.PositionConversionFactor(1 / _GEAR_RATIO);
	_motor1Config.encoder.VelocityConversionFactor(1 / _GEAR_RATIO);
	_motor1Config.closedLoop.Pid(_P, _I, _D);
	_motor1Config.Inverted(_INVERTED);

	_motor2Config.Follow(canid::CLIMBER_MOTOR_1).Inverted(!_INVERTED);

	_motor1.OverwriteConfig(_motor1Config);
	_motor2.OverwriteConfig(_motor2Config);

	Logger::Log("Hopper/Motor1", &_motor1);
	Logger::Log("Hopper/Motor2", &_motor2);
}

void SubHopper::Periodic() {
	if(_hasZeroed == false && _zeroing == false) {
		_motor1.StopMotor();
		_motor2.StopMotor();
	}

	Logger::Log("Hopper/Has Zeroed", _hasZeroed);
	Logger::Log("Hopper/Zeroing", _zeroing);
	Logger::Log("Hopper/On Target", 
		_motor1.OnPosTarget(ConvertHeightToPosition(_TOLERANCE)) && 
		_motor2.OnPosTarget(ConvertHeightToPosition(_TOLERANCE)));
}

frc2::CommandPtr SubHopper::Zero() {
    return frc2::cmd::RunOnce([this] {
		_zeroing = true;
		_hasZeroed = false;
		_motor1.SetVoltage(-1_V);
		_motor2.SetVoltage(-2_V);
    })
    .AndThen(frc2::cmd::WaitUntil([this] {
		return std::abs(_motor1.GetStatorCurrent() > _ZERO_CURRENT_LIMIT) &&
			std::abs(_motor2.GetStatorCurrent() > _ZERO_CURRENT_LIMIT);
    }))
    .AndThen([this] {
		_motor1.SetPosition(0_deg);
		_motor2.SetPosition(0_deg);
		_motor1.StopMotor();
		_motor2.StopMotor();
		_hasZeroed = true;
    })
   	.FinallyDo([this] {
		_zeroing = false;
   	});
}

frc2::CommandPtr SubHopper::ExtendTo(units::meter_t height) {
	return frc2::cmd::RunOnce([this, height] {
    	if(_hasZeroed) {
    		_motor1.SetPositionTarget(ConvertHeightToPosition(height));
    		_motor2.SetPositionTarget(ConvertHeightToPosition(height));
    	}
	});
}

frc2::CommandPtr SubHopper::MannualExtendDown() {
	return frc2::cmd::RunOnce([this] {
    	if(_hasZeroed) {
    		_motor1.SetVoltage(-1_V);
    		_motor2.SetVoltage(-1_V);
    	}
	}).WithTimeout(1_ms);
}

frc2::CommandPtr SubHopper::MannualExtendUp() {
	return frc2::cmd::RunOnce([this] {
    	if(_hasZeroed) {
    		_motor1.SetVoltage(1_V);
    		_motor2.SetVoltage(1_V);
    	}
	}).WithTimeout(1_ms);
}
            
frc2::CommandPtr SubHopper::Stow() {
	return ExtendTo(_STOW_HEIGHT);
}

units::meter_t SubHopper::GetHeight() {
	return ConvertPositionToHeight(_motor1.GetPosition());
}

units::meter_t SubHopper::ConvertPositionToHeight(units::turn_t pos) {
    /* For anyone confused about the <2> syntax, see:
     * https://github.com/nholthaus/units#exponentials-and-square-roots */
    return (pos.value() * _PINION_CIRCUM);
}

units::turn_t SubHopper::ConvertHeightToPosition(units::meter_t height) {
    /* For anyone confused about the <2> syntax, see:
     * https://github.com/nholthaus/units#exponentials-and-square-roots */
	return 1_tr * (height / _PINION_CIRCUM).value();
}

