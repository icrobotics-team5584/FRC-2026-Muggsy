#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkFlexConfigAccessor.h>

#include "utilities/ICSparkFlex.h"
#include "Constants.h"

class SubClimb : public frc2::SubsystemBase {
	public:
    	static SubClimb& GetInstance() {
			static SubClimb inst;
			return inst;
    	}	
    	SubClimb();

    	void Periodic() override;
    	/*void SimulationPeriodic() override; no impl yet*/
    	
		frc2::CommandPtr Zero();
		frc2::CommandPtr ClimbTo(units::meter_t height);
        frc2::CommandPtr MannualClimbDown();
        frc2::CommandPtr MannualClimbUp();
		frc2::CommandPtr ClimbToL1();
		frc2::CommandPtr Stow();

		units::meter_t GetHeight();
	private:
		units::meter_t ConvertPositionToHeight(units::turn_t pos);
		units::turn_t ConvertHeightToPosition(units::meter_t height);
    	
		ICSparkFlex _motor1{canid::CLIMBER_MOTOR_1};
		ICSparkFlex _motor2{canid::CLIMBER_MOTOR_2};
		rev::spark::SparkFlexConfig _motor1Config;
		rev::spark::SparkFlexConfig _motor2Config;
		rev::spark::SparkBaseConfig::IdleMode _IDLE_MODE =
			rev::spark::SparkBaseConfig::IdleMode::kBrake;

		static constexpr units::ampere_t _ZERO_CURRENT_LIMIT = 40_A;
		static constexpr units::ampere_t _CURRENT_LIMIT = 60_A;
		static constexpr bool _INVERTED = false; /* _motor2 inverts this */
		static constexpr double _P = 1.0;
		static constexpr double _I = 0.0;
		static constexpr double _D = 0.0;
		static constexpr double _GEAR_RATIO = 45;
    	
		static constexpr units::meter_t _UPPER_LIMIT = 205.8_mm;
		static constexpr units::meter_t _LOWER_LIMIT = 0_m;
		static constexpr units::meter_t _L1_HEIGHT = 100_mm;
		static constexpr units::meter_t _STOW_HEIGHT = 0_m;
		static constexpr units::meter_t _PINION_RAD = 0.05_m;
		static constexpr units::meter_t _PINION_CIRCUM = _PINION_RAD*2*3.14;
		static constexpr units::meter_t _TOLERANCE = 5_mm;
		
    	bool _zeroing = false;
    	bool _hasZeroed = false;
};
