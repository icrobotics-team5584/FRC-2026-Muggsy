#include <ctre/phoenix6/TalonFX.hpp>

#include "utilities/GeneralMotor/KrakenIO.h"

namespace GeneralMotor {
    KrakenIO::KrakenIO(int canID) : _motor(canID) {}

    void KrakenIO::setIdleMode(GeneralMotor::IdleMode idleMode) {
        ctre::phoenix6::configs::TalonFXConfiguration ctreConfig;
        ctreConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

        if(idleMode == IdleMode::idleCoast) {
            ctreConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        }

        _motor.GetConfigurator().Apply(ctreConfig);
    }

    void KrakenIO::SetPos(units::turn_t pos) {
        _motor.SetPosition(pos);
    }

    void KrakenIO::SetPosTarg(units::turn_t targ) {
        _desiredAngle = targ;
        _motor.SetControl(ctre::phoenix6::controls::PositionVoltage(_desiredAngle));
    }

    void KrakenIO::SetSpeed(double speed) {
        double clampedSpeed = std::clamp(speed, -1.0, 1.0);
        _motor.SetControl(ctre::phoenix6::controls::DutyCycleOut{clampedSpeed});
    }

    void KrakenIO::SetConfig(GeneralMotor::Config config) {
        ctre::phoenix6::configs::TalonFXConfiguration ctreConfig;

        if(config.gearRatio.has_value()) {
            ctreConfig.Feedback.SensorToMechanismRatio = config.gearRatio.value();
        }
        if(config.P.has_value()) {
            ctreConfig.Slot0.kP = config.P.value();
        }
        if(config.I.has_value()) {
            ctreConfig.Slot0.kI = config.I.value();
        }
        if(config.D.has_value()) {
            ctreConfig.Slot0.kD = config.D.value();
        }
        if(config.inverted.has_value()) {
            if(!config.inverted.value()) { /* CounterClockwise_Positive is default */
                ctreConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
            } else { 
                ctreConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
            }
        }
        if(config.idleMode.has_value()) {
            ctreConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake; /*Default*/

            if(config.idleMode.value() == IdleMode::idleCoast) {
                ctreConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
            }
        }
        if(config.currentLimit.has_value()) {
            ctreConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            ctreConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            ctreConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimit.value();
            ctreConfig.CurrentLimits.SupplyCurrentLowerLimit = config.currentLimit.value();
            ctreConfig.CurrentLimits.SupplyCurrentLowerTime = 0_s;

            ctreConfig.CurrentLimits.StatorCurrentLimit = config.currentLimit.value();
        }
        
        _motor.GetConfigurator().Apply(ctreConfig);
        if(config.vendor.has_value()) {
            _motor.GetConfigurator().Apply(config.vendor.value().ctre);
        }
    }

    units::turn_t KrakenIO::GetPos() {
        return _motor.GetPosition().GetValue();
    }

    units::turn_t KrakenIO::GetPosTarg() {
        return _desiredAngle;
    }

    units::turn_t KrakenIO::GetPosErr() {
        return _desiredAngle - _motor.GetPosition().GetValue();
    }

    double KrakenIO::GetSpeed() {
        return _motor.GetDutyCycle().GetValue();
    }

    units::volt_t KrakenIO::GetVolts(){
        return _motor.GetMotorVoltage().GetValue();
    }

    units::ampere_t KrakenIO::GetAmps() {
        return _motor.GetStatorCurrent().GetValue();
    }

    units::celsius_t KrakenIO::GetTemp() {
        return _motor.GetDeviceTemp().GetValue();
    }
}