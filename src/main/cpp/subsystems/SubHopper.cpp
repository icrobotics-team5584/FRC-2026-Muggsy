#include <frc2/command/Commands.h>
#include <units/math.h>

#include "subsystems/SubHopper.h"
#include "utilities/ICSparkFlex.h"
#include "utilities/Logger.h"
#include "Constants.h"


SubHopper::SubExtend() {
	_motorConfig.SmartCurrentLimit(_CURRENT_LIMIT.value());
	_motorConfig.encoder.PositionConversionFactor(1 / _GEAR_RATIO);
	_motorConfig.encoder.VelocityConversionFactor(1 / _GEAR_RATIO);
	_motorConfig.closedLoop.Pid(_P, _I, _D);
	_motorConfig.Inverted(_INVERTED);

	_motor.OverwriteConfig(_motorConfig);

	Logger::Log("Hopper/Motor1", &_motor);
}

void SubHopper::Periodic() {
	if(_hasZeroed == false && _zeroing == false) {
		_motor.StopMotor();
	}

	Logger::Log("Hopper/Has Zeroed", _hasZeroed);
	Logger::Log("Hopper/Zeroing", _zeroing);
	Logger::Log("Hopper/On Target", 
		_motor.OnPosTarget(ConvertHeightToPosition(_TOLERANCE))); 
}

frc2::CommandPtr SubHopper::Zero() {
    return frc2::cmd::RunOnce([this] {
		_zeroing = true;
		_hasZeroed = false;
		_motor.SetVoltage(-1_V);
    })
    .AndThen(frc2::cmd::WaitUntil([this] {
		return std::abs(_motor.GetStatorCurrent() > _ZERO_CURRENT_LIMIT));
    }))
    .AndThen([this] {
		_motor.SetPosition(0_deg);
		_motor.StopMotor();
		_hasZeroed = true;
    })
   	.FinallyDo([this] {
		_zeroing = false;
   	});
}

frc2::CommandPtr SubHopper::ExtendTo(units::meter_t height) {
	return frc2::cmd::RunOnce([this, height] {
    	if(_hasZeroed) {
    		_motor.SetPositionTarget(ConvertHeightToPosition(height));
    	}
	});
}

frc2::CommandPtr SubHopper::MannualExtendDown() {
	return frc2::cmd::RunOnce([this] {
    	if(_hasZeroed) {
    		_motor.SetVoltage(-1_V);
    	}
	}).WithTimeout(1_ms);
}

frc2::CommandPtr SubHopper::MannualExtendUp() {
	return frc2::cmd::RunOnce([this] {
    	if(_hasZeroed) {
    		_motor.SetVoltage(1_V);
    	}
	}).WithTimeout(1_ms);
}
            
frc2::CommandPtr SubHopper::Stow() {
	return ExtendTo(_STOW_HEIGHT);
}

units::meter_t SubHopper::GetHeight() {
	return ConvertPositionToHeight(_motor.GetPosition());
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

