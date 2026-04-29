#pragma once
#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <units/current.h>
#include <units/temperature.h>

namespace AlertController {

struct AlertConfig {
  std::string motorString;
  units::celsius_t maxDegrees;

  units::ampere_t maxCurrent;
  units::ampere_t minCurrent;

  // Tempurature Alerts//
  frc::Alert responsiveHighTemperatureAlert = frc::Alert(motorString + ": " + "!HIGH TEMP!", frc::Alert::AlertType::kWarning);
  frc::Alert reachedTemperatureAlert = frc::Alert(motorString + "Reached Max Temperature Threshold: " + std::to_string(tempuratureReachedCount) + (tempuratureReachedCount > 1 ? " Times" : " Time") , frc::Alert::AlertType::kWarning);

  // Current Alerts//
  frc::Alert responsiveHighCurrentAlert = frc::Alert(motorString + ": " + "!HIGH CURRENT!", frc::Alert::AlertType::kWarning);
  frc::Alert reachedHighCurrentAlert = frc::Alert(motorString + "Reached Max Current Threshold: " + std::to_string(highCurrentReachedCount) + (highCurrentReachedCount > 1 ? " Times" : " Time") , frc::Alert::AlertType::kWarning);
  
  frc::Alert responsiveLowCurrentAlert = frc::Alert(motorString + ": " + "!LOW CURRENT!", frc::Alert::AlertType::kWarning);
  frc::Alert reachedLowCurrentAlert = frc::Alert(motorString + "Reached Min Current Threshold: " + std::to_string(lowCurrentReachedCount) + (lowCurrentReachedCount > 1 ? " Times" : " Time") , frc::Alert::AlertType::kWarning);

  // Timers for Current
  frc::Timer highCurrentTimer = frc::Timer();

 // Counts for how many times the motor has reached the thresholds
  int tempuratureReachedCount = 0;
  int highCurrentReachedCount = 0;
  int lowCurrentReachedCount = 0;

  AlertConfig(){
    responsiveHighTemperatureAlert.SetText(motorString + ": " + "!HIGH TEMP!");
    reachedTemperatureAlert.SetText(motorString + "Reached Max Temperature Threshold: " + std::to_string(tempuratureReachedCount) + " Times" );

   responsiveHighCurrentAlert.SetText(motorString + ": " + "!HIGH CURRENT!");
   reachedHighCurrentAlert.SetText(motorString + "Reached Max Current Threshold: " + std::to_string(highCurrentReachedCount) + " Times" );
   responsiveLowCurrentAlert.SetText(motorString + ": " + "!LOW CURRENT!");
   reachedLowCurrentAlert.SetText(motorString + "Reached Min Current Threshold: " + std::to_string(lowCurrentReachedCount) + " Times" );  
  }
};


void RegisterAlertConfig(std::weak_ptr<AlertConfig> config);

void MotorCheck(auto Motor, AlertConfig& config);

frc2::CommandPtr ForceRemoveAllAlerts();

};  // namespace AlertController
