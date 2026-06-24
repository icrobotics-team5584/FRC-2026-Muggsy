#include <utilities/CurrentController.h>
#include <utilities/Logger.h>

std::optional<int> CurrentController::RegisterSubsystem(
  const CurrentControllerSubsystem& conf, const std::optional<YellowCritLevel>& yellowConf) {
  SubsystemData data = {};
  data.subsystem = conf;

  if (yellowConf) {
    const YellowCritLevel& yconf = yellowConf.value();
    if (yconf.yellowMaxCurrentThreshold < conf.greenMaxCurrentThreshold) {
      logger::Log("Current Management System/" + data.subsystem.name + 
        "/Misconfigured Yellow-Green current thresholds", true);
      return std::nullopt;
    }
    data.yellowCritLevelEnabled = true;
    data.yellowSubsystem = yconf;
  }

  unsigned int id = 0;
  while (_subsystemList.contains(id)) {
    id++;
  }
  _subsystemList[id] = data;

  return id;
}

void CurrentController::UnregisterSubsystem(unsigned int id) {
  if (_subsystemList.contains(id)) {
    _subsystemList.erase(id);
  }
}

void CurrentController::Periodic() {
  for (const auto& [id, data] : _subsystemList) {
    /* cld stands for current level difference
     * cld > 0: go up a current level cld times.
     * cld = 0: no change.
     * cld < 0: go down a current level cld times.
     */
    std::optional<CritLevel> ccl = GetCritLevel(id, data.subsystem.getSubsystemCurrent());
    if (!ccl) {
      continue;
    }

    int lcl = static_cast<int>(data.critLevel);
    int cclResult = static_cast<int>(ccl.value());

    int cld = std::clamp(cclResult - lcl, -2, 2);

    if (!cld) {
      continue;
    }

    switch (cld) {
      case 2:
        ReallowSubsystemFunctionality(id);
        ReallowSubsystemFunctionality(id);
        break;
      case 1:
        ReallowSubsystemFunctionality(id);
        break;
      case -1:
        LimitSubsystemFunctionality(id);
        break;
      case -2:
        LimitSubsystemFunctionality(id);
        LimitSubsystemFunctionality(id);
        break;
    }

    ccl = GetCritLevel(id, data.subsystem.getSubsystemCurrent());
    if (!ccl) {
      continue;
    }

    logger::Log("Current Management System/" + data.subsystem.name + "/CritLevel",
      CritLevelToString(ccl.value()));
  }
}

void CurrentController::LimitSubsystemFunctionality(unsigned int id) {
  if (!_subsystemList.contains(id)) {
    return;
  }

  SubsystemData& data = _subsystemList[id];
  if (data.yellowCritLevelEnabled) {
    if (data.critLevel == CritLevel::CRIT_RED) {
      data.subsystem.exitRedCritLevel();
      data.yellowSubsystem.enterYellowCritLevel();
      data.critLevel = CritLevel::CRIT_YELLOW;
      return;
    }

    if (data.critLevel == CritLevel::CRIT_YELLOW) {
      data.yellowSubsystem.exitYellowCritLevel();
      data.subsystem.enterGreenCritLevel();
      data.critLevel = CritLevel::CRIT_GREEN;
      return;
    }
  } else {
    if (data.critLevel == CritLevel::CRIT_RED) {
      data.subsystem.exitRedCritLevel();
      data.subsystem.enterGreenCritLevel();
      data.critLevel = CritLevel::CRIT_GREEN;
      return;
    }
  }
}
void CurrentController::ReallowSubsystemFunctionality(unsigned int id) {
  if (!_subsystemList.contains(id)) {
    return;
  }

  SubsystemData& data = _subsystemList[id];
  if (data.yellowCritLevelEnabled) {
    if (data.critLevel == CritLevel::CRIT_GREEN) {
      data.subsystem.exitGreenCritLevel();
      data.yellowSubsystem.enterYellowCritLevel();
      data.critLevel = CritLevel::CRIT_YELLOW;
      return;
    }

    if (data.critLevel == CritLevel::CRIT_YELLOW) {
      data.yellowSubsystem.exitYellowCritLevel();
      data.subsystem.enterRedCritLevel();
      data.critLevel = CritLevel::CRIT_RED;
      return;
    }
  } else {
    if (data.critLevel == CritLevel::CRIT_GREEN) {
      data.subsystem.exitGreenCritLevel();
      data.subsystem.enterRedCritLevel();
      data.critLevel = CritLevel::CRIT_RED;
      return;
    }
  }
}

std::optional<CritLevel> CurrentController::GetCritLevel(unsigned int id, units::ampere_t current) {
  if (!_subsystemList.contains(id)) {
    return std::nullopt;
  }

  SubsystemData data = _subsystemList[id];
  if (current < data.subsystem.greenMaxCurrentThreshold) {
    return std::make_optional<CritLevel>(CritLevel::CRIT_GREEN);
  }

  if (current < data.yellowSubsystem.yellowMaxCurrentThreshold && data.yellowCritLevelEnabled) {
    return std::make_optional<CritLevel>(CritLevel::CRIT_YELLOW);
  }

  return std::make_optional<CritLevel>(CritLevel::CRIT_RED);
}

std::string CurrentController::CritLevelToString(enum CritLevel level) {
  switch (level) {
    case CritLevel::CRIT_RED:
      return "Red";
    case CritLevel::CRIT_YELLOW:
      return "Yellow";
    case CritLevel::CRIT_GREEN:
      return "Green";
  }

  return "Unknown";
}