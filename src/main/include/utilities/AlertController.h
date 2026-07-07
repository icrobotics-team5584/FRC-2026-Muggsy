#pragma once
#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <memory>
#include <units/current.h>
#include <units/temperature.h>
#include <utilities/ICSpark.h>
namespace alertController {
using MotorVariant = std::variant<ICSpark*, ctre::phoenix6::hardware::TalonFX*>;

struct AlertConfig;

void RegisterAlertConfig(const std::weak_ptr<AlertConfig>& config);

void MotorCheck(
 MotorVariant motor, AlertConfig& config);

units::celsius_t GetMotorTemperature(MotorVariant motor);
units::ampere_t GetMotorCurrent(MotorVariant motor);

frc2::CommandPtr RemoveAlerts();

struct AlertConfig : public std::enable_shared_from_this<AlertConfig> {
  std::string motorString;
  units::celsius_t maxDegrees;

  units::ampere_t maxCurrent;

  // Temperature Alerts//
  frc::Alert responsiveHighTemperatureAlert = frc::Alert("", frc::Alert::AlertType::kWarning);
  frc::Alert reachedTemperatureAlert = frc::Alert("", frc::Alert::AlertType::kWarning);

  // Current Alert//
  frc::Alert responsiveHighCurrentAlert = frc::Alert("", frc::Alert::AlertType::kWarning);
  frc::Alert reachedHighCurrentAlert = frc::Alert("", frc::Alert::AlertType::kWarning);

  // Timers for Current
  frc::Timer highCurrentTimer = frc::Timer();

  // Counts for how many times the motor has reached the thresholds
  int temperatureReachedCount = 0;
  int highCurrentReachedCount = 0;

  // Booleans to only allow recorded temperatures to record once per alert
  bool shouldRecordTemp = true;
  bool shouldRecordHighCurrent = true;

  /* [Implement In a Subsystem Header file somewhere under private]
  
  ALERT CONFIG: A initialized struct that takes the motor's name as a string and the maximum threshold for
  the motors temperature along with its current, this will then be used to run the alert calculation logic.
  
  */
  
  AlertConfig(const std::string& motorName, units::celsius_t maxTemp, units::ampere_t maxCurr) {
    responsiveHighTemperatureAlert.SetText(motorName + ": " + " HIGH TEMP");

    responsiveHighCurrentAlert.SetText(motorName + ": " + "HIGH CURRENT");
    reachedHighCurrentAlert.SetText(
      motorName + "Reached Max Current Threshold: " + std::to_string(highCurrentReachedCount) +
      (highCurrentReachedCount > 1 ? " Times" : " Time"));

    motorString = motorName;
    maxDegrees = maxTemp;
    maxCurrent = maxCurr;
  }
  static std::shared_ptr<AlertConfig> Create(
    const std::string& motorName, units::celsius_t maxTemp, units::ampere_t maxCurr) {
    auto cfg = std::make_shared<AlertConfig>(motorName, maxTemp, maxCurr);
    RegisterAlertConfig(cfg->weak_from_this());
    return cfg;
  };
};

};  // namespace alertController
