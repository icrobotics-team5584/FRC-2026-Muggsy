#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/angular_velocity.h>

#include "utilities/GeneralMotor/Config.h"

namespace GeneralMotor {
    typedef enum Type {
        TalonFX,
        Neo,
        Neo500,
        NeoVortex,
        VexBag
    } Type;

    class IO {
        public:
            virtual void setIdleMode(GeneralMotor::IdleMode idleMode) = 0;
            virtual void SetPos(units::turn_t pos) = 0;
            virtual void SetPosTarg(units::turn_t targ) = 0;
            virtual void SetVelocityTarg(units::tps vel) = 0;
            virtual void SetSpeed(double speed) = 0;
            virtual void SetConfig(Config config) = 0;
            virtual void SetVoltage(units::volt_t volts) = 0;
            virtual GeneralMotor::IdleMode GetIdleMode() = 0;
            virtual units::tps GetVelocity() = 0;
            virtual units::turn_t GetPos() = 0;
            virtual units::turn_t GetPosTarg() = 0;
            virtual units::turns_per_second_t GetVelocityTarg() = 0;
            virtual units::turn_t GetPosErr() = 0;
            virtual double GetSpeed() = 0;
            virtual units::volt_t GetVolts() = 0;
            virtual units::ampere_t GetAmps() = 0;
            virtual units::celsius_t GetTemp() = 0;
    };
}