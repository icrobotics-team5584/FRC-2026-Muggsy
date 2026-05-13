#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

namespace autonHelper {
using AutonPtr = std::shared_ptr<frc2::CommandPtr>;
using AutonChooser = frc::SendableChooser<AutonPtr>;
AutonPtr MakePathPlannerAuto(pathplanner::PathPlannerAuto auton);
AutonPtr MakeCommandPtrAuto(frc2::CommandPtr auton);

class AutonManager {
 public:
  void AddAuton(const std::string& autoname, AutonPtr autoptr);
  void AddDefaultAuton(const std::string& autoname, AutonPtr autoptr);
  AutonChooser& GetAutonChooser();
  AutonPtr GetChosenAuton();

 private:
  AutonChooser _autoChooser;
};
}  // namespace AutonHelper