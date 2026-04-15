#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
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

		frc2::CommandPtr IntakeEnable();
		frc2::CommandPtr IntakeDisable();

		bool isIntakeEnabled();
	private:
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
};
