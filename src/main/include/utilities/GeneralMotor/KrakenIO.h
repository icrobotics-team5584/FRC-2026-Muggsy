#include <ctre/phoenix6/TalonFX.hpp>

#include "utilities/GeneralMotor/IO.h"

namespace GeneralMotor{
    class KrakenIO : public IO {
        public:
            KrakenIO(int canID);
            void SetPos(units::turn_t pos);    
            void SetPosTarg(units::turn_t targ);    
            void SetSpeed(double speed);    
            void SetConfig(Config config);    
            units::turn_t GetPos();    
            units::turn_t GetPosTarg();    
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
