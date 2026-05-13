#pragma once
#include <frc/Alert.h>
#include <frc/Timer.h>

#include <units/current.h>
#include <units/temperature.h>

namespace alertController {
struct MotorAlertConfig {
  frc::Alert _temperatureAlert;
  frc::Alert _currentAlert;
  frc::Alert _recordedTemperatureAlert;
  frc::Alert _recordedCurrentAlert;
  units::celsius_t _maxDegrees;
  units::ampere_t _maxCurrent;
  frc::Timer _highCurrentTimer = frc::Timer();
};

// updates the alert for the temperature. automatically updates to true or false
void UpdateTemperatureAlert(MotorAlertConfig& config, units::celsius_t motorTemperature);

// updates the alert for the current. automatically updates to true if has been over 3s or false.
void UpdateCurrentAlert(MotorAlertConfig& config, units::ampere_t motorCurrent);
};  // namespace alertController
