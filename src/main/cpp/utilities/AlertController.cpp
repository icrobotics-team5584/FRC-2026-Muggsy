#include "utilities/AlertController.h"

#include <unordered_map>
namespace alertController {


std::vector<std::weak_ptr<AlertConfig>> configList;
// Registers the Alert Config to be used in the ForceRemoveAllAlerts Command
void RegisterAlertConfig(std::weak_ptr<AlertConfig> config) {
  configList.emplace_back(config);
}

// Check Every Alert for the Motor and Set them accordingly
void MotorCheck(MotorVariant motor, AlertConfig& config) {
  //  High Temperature Check
  units::celsius_t motorTemp = GetMotorTemperature(motor);
  units::ampere_t motorCurrent = GetMotorCurrent(motor);

  if (motorTemp >= config.maxDegrees) {
    if (config.shouldRecordTemp) {
      config.tempuratureReachedCount++;
      config.reachedTemperatureAlert.SetText(
        config.motorString +
        " Reached Max Temperature Threshold: " + std::to_string(config.tempuratureReachedCount) +
        (config.tempuratureReachedCount > 1 ? " Times" : " Time"));

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
  return std::visit(
    [](auto* m) -> units::celsius_t {
      using T = std::decay_t<decltype(*m)>;

      if constexpr (std::is_same_v<T, ICSpark>) {
        return m->GetTemperature();
      } else {
        return m->GetDeviceTemp().GetValue();
      }
    },
    motor);
}

units::ampere_t GetMotorCurrent(MotorVariant motor) {
  return std::visit(
    [](auto* m) -> units::ampere_t {
      using T = std::decay_t<decltype(*m)>;

      if constexpr (std::is_same_v<T, ICSpark>) {
        return m->GetStatorCurrent();
      } else if constexpr (std::is_same_v<T, ctre::phoenix6::hardware::TalonFX>) {
        return m->GetStatorCurrent().GetValue();
      }
    },
    motor);
}
// Command to Force Remove All Alerts
frc2::CommandPtr ForceRemoveAllAlerts() {
  return frc2::cmd::RunOnce([] {
    for (auto& weak : configList) {
      if (auto config = weak.lock()) {
        config->reachedTemperatureAlert.Set(false);
        config->responsiveHighTemperatureAlert.Set(false);
        config->responsiveHighCurrentAlert.Set(false);
        config->reachedHighCurrentAlert.Set(false);
      }
    }
  })
    .WithName("Force Remove All Alerts")
    .IgnoringDisable(true);
}

}  // namespace AlertController
