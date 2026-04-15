#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>
#include <units/temperature.h>
#include <units/time.h>
#include <optional>

namespace GeneralMotor {
    typedef enum IdleMode {
        idleBreak,
        idleCoast
    } IdleMode;

    typedef union VendorSpecificConfig {
        ctre::phoenix6::configs::TalonFXConfiguration ctre;
    } VendorSpecificConfig;

    typedef struct Config {
        std::optional<IdleMode> idleMode;
        std::optional<double> gearRatio;
        std::optional<double> P;
        std::optional<double> I;
        std::optional<double> D;
        std::optional<units::ampere_t> currentLimit;
        std::optional<bool> inverted;
        std::optional<VendorSpecificConfig> vendor;
    } Config;
}