#include "utilities/AlertController.h"

#include <unordered_map>
namespace alertController {

std::vector<std::weak_ptr<AlertConfig>> configList;
// Registers the Alert Config to be used in the ForceRemoveAllAlerts Command
void RegisterAlertConfig(const std::weak_ptr<AlertConfig>& config) {
  configList.emplace_back(config);
}

// Check Every Alert for the Motor and Set them accordingly
void MotorCheck(MotorVariant motor, AlertConfig& config) {
  //  High Temperature Check
  units::celsius_t motorTemp = GetMotorTemperature(motor);
  units::ampere_t motorCurrent = GetMotorCurrent(motor);

  if (motorTemp >= config.maxDegrees) {
    if (config.shouldRecordTemp) {
      config.temperatureReachedCount++;
      //Updates the text to the high temperature reach count, if count is higher than 1, then concat " Times" to count else concat " Time" when the value is 1 
      config.reachedTemperatureAlert.SetText(
        config.motorString +
        " Reached Max Temperature Threshold: " + std::to_string(config.temperatureReachedCount) +
        (config.temperatureReachedCount > 1 ? " Times" : " Time"));

      config.reachedTemperatureAlert.Set(true);
      config.responsiveHighTemperatureAlert.Set(true);
      config.shouldRecordTemp = false;
    }

  } else {
    config.responsiveHighTemperatureAlert.Set(false);
    config.shouldRecordTemp = true;
  }

  // High Current Check
  if (motorCurrent >= config.maxCurrent) {
    config.highCurrentTimer.Start();

    if (config.highCurrentTimer.HasElapsed(1_s)) {
      config.highCurrentTimer.Stop();
      config.highCurrentTimer.Reset();

      if (config.shouldRecordHighCurrent) {
        config.highCurrentReachedCount++;

        //Updates the text to the high current reach count, if count is higher than 1, then concat " Times" to count else  concat " Time" when the value is 1
        config.reachedHighCurrentAlert.SetText(
          config.motorString +
          " Reached Max Current Threshold: " + std::to_string(config.highCurrentReachedCount) +
          (config.highCurrentReachedCount > 1 ? " Times" : " Time")); 
        config.reachedHighCurrentAlert.Set(true);
        config.responsiveHighCurrentAlert.Set(true);
        config.shouldRecordHighCurrent = false;
      }
    }
  } else {
    config.responsiveHighCurrentAlert.Set(false);
    config.highCurrentTimer.Stop();
    config.highCurrentTimer.Reset();
    config.shouldRecordHighCurrent = true;
  }
}

units::celsius_t GetMotorTemperature(MotorVariant motor) {
  if(std::holds_alternative<ICSpark*>(motor)){
    return std::get<ICSpark*>(motor)->GetTemperature();
  } else if (std::holds_alternative<ctre::phoenix6::hardware::TalonFX*>(motor)){
    return std::get<ctre::phoenix6::hardware::TalonFX*>(motor)->GetDeviceTemp().GetValue();
  }
  return 0_degC; // Default return 
}

units::ampere_t GetMotorCurrent(MotorVariant motor) {
   if(std::holds_alternative<ICSpark*>(motor)){
    return std::get<ICSpark*>(motor)->GetStatorCurrent();
  } else if (std::holds_alternative<ctre::phoenix6::hardware::TalonFX*>(motor)){
    return std::get<ctre::phoenix6::hardware::TalonFX*>(motor)->GetStatorCurrent().GetValue();
  }
  return 0_A; // Default return 
}
// Command to Force Remove All Alerts
frc2::CommandPtr RemoveAlerts() {
  return frc2::cmd::RunOnce([] {
    for (auto& weak : configList) {
      if (auto config = weak.lock()) {
        config->reachedTemperatureAlert.Set(false);
        config->responsiveHighTemperatureAlert.Set(false);
        config->responsiveHighCurrentAlert.Set(false);
        config->reachedHighCurrentAlert.Set(false);
        config->temperatureReachedCount = 0;
        config->highCurrentReachedCount = 0;
        config->shouldRecordTemp = true;
        config->shouldRecordHighCurrent = true;
        config->highCurrentTimer.Reset();
      }
    }
  })
    .WithName("Remove Alerts")
    .IgnoringDisable(true);
}

}  // namespace alertController
