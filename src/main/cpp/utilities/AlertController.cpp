#include "utilities/AlertController.h"

namespace alertController {

// updates the alert for the temperature. automatically updates to true or false
void UpdateTemperatureAlert(MotorAlertConfig& config, units::celsius_t motorTemperature) {
  if (motorTemperature > config._maxDegrees) {
    config._temperatureAlert.Set(true);
    config._recordedTemperatureAlert.Set(true);
  } else {
    config._temperatureAlert.Set(false);
  }
}
// updates the alert for the current. automatically updates to true if has been over 3s or false.
void UpdateCurrentAlert(MotorAlertConfig& config, units::ampere_t motorCurrent) {
  if (motorCurrent > config._maxCurrent) {
    config._highCurrentTimer.Start();
    if (config._highCurrentTimer.Get() > 3_s) {
      config._currentAlert.Set(true);
      config._recordedCurrentAlert.Set(true);
    }

  } else {
    config._currentAlert.Set(false);
    config._highCurrentTimer.Reset();
  }
}

}  // namespace AlertController