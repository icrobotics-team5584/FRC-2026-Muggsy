#include "utilities/AlertController.h"

#include <unordered_map>

namespace AlertController {
std::vector<std::weak_ptr<AlertConfig>> configList;
// Registers the Alert Config to be used in the ForceRemoveAllAlerts Command
void RegisterAlertConfig(std::weak_ptr<AlertConfig> config) {
  configList.emplace_back(config);
}

// Check Every Alert for the Motor and Set them accordingly
void MotorCheck(ICSpark& Motor, AlertConfig& config) {
  // Check Temperature
  if (Motor.GetTemperature() >= config.maxDegrees) {
    config.responsiveHighTemperatureAlert.Set(true);
    config.tempuratureReachedCount++;
    config.reachedTemperatureAlert.Set(true);
  } else {
    config.responsiveHighTemperatureAlert.Set(false);
  }

  // Check High Current
  if (Motor.GetStatorCurrent() >= config.maxCurrent) {
    config.responsiveHighCurrentAlert.Set(true);
    config.highCurrentTimer.Start();
    if (config.highCurrentTimer.HasElapsed(1_s)) {
      config.highCurrentTimer.Stop();
      config.highCurrentTimer.Reset();
      config.highCurrentReachedCount++;
      config.reachedHighCurrentAlert.Set(true);
    }
  } else {
    config.responsiveHighCurrentAlert.Set(false);
    config.highCurrentTimer.Stop();
    config.highCurrentTimer.Reset();
  }
  // Check Low Current
  if (Motor.GetStatorCurrent() <= config.minCurrent) {
    config.responsiveLowCurrentAlert.Set(true);
    if (config.highCurrentTimer.HasElapsed(1_s)) {
      config.highCurrentTimer.Stop();
      config.highCurrentTimer.Reset();
      config.lowCurrentReachedCount++;
      config.reachedLowCurrentAlert.Set(true);
    }
  } else {
    config.responsiveLowCurrentAlert.Set(false);
  }
}
// Command to Force Remove All Alerts, used for testing and to reset the alert counts
frc2::CommandPtr ForceRemoveAllAlerts() {
  return frc2::cmd::RunOnce([] {
    for (std::weak_ptr<AlertConfig>& configWeak : configList) {
      if (auto config = configWeak.lock()) {  // lock once
      }
    }
  })
    .WithName("Force Remove All Alerts")
    .IgnoringDisable(true);
}

}  // namespace AlertController