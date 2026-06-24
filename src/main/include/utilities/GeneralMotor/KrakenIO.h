#include <ctre/phoenix6/TalonFX.hpp>

#include "utilities/GeneralMotor/IO.h"

namespace GeneralMotor{
    class KrakenIO : public IO {
        public:
            KrakenIO(int canID);
            void setIdleMode(GeneralMotor::IdleMode idleMode);
            void SetPos(units::turn_t pos);
            void SetPosTarg(units::turn_t targ);
            void SetVelocityTarg(units::tps vel);
            void SetSpeed(double speed);
            void SetConfig(Config config);
            void SetVoltage(units::volt_t volts);
            GeneralMotor::IdleMode GetIdleMode();
            units::tps GetVelocity();
            units::turn_t GetPos();
            units::turn_t GetPosTarg();
            units::turns_per_second_t GetVelocityTarg();
            units::turn_t GetPosErr();
            double GetSpeed();
            units::volt_t GetVolts();
            units::ampere_t GetAmps();
            units::celsius_t GetTemp();

        private:
            units::turn_t _desiredAngle = 0_deg;
            ctre::phoenix6::hardware::TalonFX _motor;
    };
}
