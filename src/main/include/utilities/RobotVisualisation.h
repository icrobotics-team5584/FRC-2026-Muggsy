#pragma once

#include "utilities/Logger.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

class RobotVisualisation {
 public:
  RobotVisualisation() { logger::Log("RobotVisualisation", &_display); }
  static RobotVisualisation& GetInstance() {
    static RobotVisualisation intstance;
    return intstance;
  }

  frc::Mechanism2d _display{0.75, 0.75};

  /* Sub(Intake)Deploy Visualisation */
  frc::MechanismRoot2d* _deployRoot = _display.GetRoot("deployRoot", 0.375, 0.075);
  frc::MechanismLigament2d* _deployLigament =
    _deployRoot->Append<frc::MechanismLigament2d>("deployLigament", 0.0, 90_deg);
  
  /* Intake Visualisation */
  frc::MechanismRoot2d* _intakeMechRoot = _display.GetRoot("intakeRoot", 0.250, 0.125);
  MechanismCircle2d _intakeMechWheel{_intakeMechRoot, "intakeCircle", 0.10, 0_deg};
};
