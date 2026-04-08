#pragma once

#include "utilities/Logger.h"
#include "utilities/MechanismCircle2d.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>

class RobotVisualisation {
 public:
  RobotVisualisation() { Logger::Log("RobotVisualisation", &_display); }
  static RobotVisualisation& GetInstance() {
    static RobotVisualisation intstance;
    return intstance;
  }

  frc::Mechanism2d _display{0.75, 0.75};
};