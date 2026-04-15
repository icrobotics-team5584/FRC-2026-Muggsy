#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <rev/config/SparkFlexConfigAccessor.h>
#include <rev/config/SparkFlexConfig.h>

#include "utilities/ICSparkFlex.h"
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
		ICSparkFlex _motor1{canid::INTAKE_MOTOR_1};
		ICSparkFlex _motor2{canid::INTAKE_MOTOR_2};
		rev::spark::SparkFlexConfig _motorConfig;
		rev::spark::SparkBaseConfig::IdleMode _IDLE_MODE =
			rev::spark::SparkBaseConfig::IdleMode::kBrake;
    	
		static constexpr units::ampere_t _CURRENT_LIMIT = 60_A;
		static constexpr bool _INVERTED = false; /* _motor2 inverts this */
		static constexpr double _P = 1.0;
		static constexpr double _I = 0.0;
		static constexpr double _D = 0.0;
		static constexpr double _GEAR_RATIO = 3;
};
