#include <utilities/CurrentController.h>
#include <utilities/Logger.h>

int CurrentController::RegisterSubsystem(
  const CurrentControllerSubsystem& conf, std::optional<YellowCurrentLevel> yellowConf) {
  SubsystemData data = {nullptr};
  data.name = conf.name;
  data.greenMaxCurrentThreshold = conf.greenMaxCurrentThreshold;
  data.getSubsystemCurrent = conf.getSubsystemCurrent;
  data.enterGreenCurrentLevel = conf.enterGreenCurrentLevel;
  data.exitGreenCurrentLevel = conf.exitGreenCurrentLevel;
  data.enterRedCurrentLevel = conf.enterRedCurrentLevel;
  data.exitRedCurrentLevel = conf.exitRedCurrentLevel;

  if (yellowConf) {
    const YellowCurrentLevel& yconf = yellowConf.value();
    if (yconf.currentThreshold < conf.greenMaxCurrentThreshold) {
      return -1;
    }
    data.yellowCurrentLevelEnabled = true;
    data.yellowMaxCurrentThreshold = yconf.currentThreshold;
    data.enterYellowCurrentLevel = yconf.enterCurrentLevel;
    data.exitYellowCurrentLevel = yconf.exitCurrentLevel;
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
    CurrentLevel cl = GetCurrentLevel(id, data.getSubsystemCurrent());
    int lcl = static_cast<int>(data.currentLevel);
    int ccl = static_cast<int>(cl);
    int cld = std::clamp(ccl - lcl, -2, 2);

    if (!cld) {
      continue;
    }

    switch (cld) {
      case 2:
        IncreaseCurrentLevel(id);
        IncreaseCurrentLevel(id);
        break;
      case 1:
        IncreaseCurrentLevel(id);
        break;
      case -1:
        DecreaseCurrentLevel(id);
        break;
      case -2:
        DecreaseCurrentLevel(id);
        DecreaseCurrentLevel(id);
        break;
    }

    logger::Log(
      "Current Management System/" + data.name + "/CurrentLevel", CurrentLevelToString(cl));
  }
}

void CurrentController::DecreaseCurrentLevel(unsigned int id) {
  if (!_subsystemList.contains(id)) {
    return;
  }

  SubsystemData data = _subsystemList[id];
  if (data.yellowCurrentLevelEnabled) {
    if (data.currentLevel == CurrentLevel::CURRENT_RED) {
      data.exitRedCurrentLevel();
      data.enterYellowCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::CURRENT_YELLOW;
    }

    if (data.currentLevel == CurrentLevel::CURRENT_YELLOW) {
      data.exitYellowCurrentLevel();
      data.enterGreenCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::CURRENT_GREEN;
    }
  } else {
    if (data.currentLevel == CurrentLevel::CURRENT_RED) {
      data.exitRedCurrentLevel();
      data.enterGreenCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::CURRENT_GREEN;
    }
  }
}
void CurrentController::IncreaseCurrentLevel(unsigned int id) {
  if (!_subsystemList.contains(id)) {
    return;
  }

  SubsystemData data = _subsystemList[id];
  if (data.yellowCurrentLevelEnabled) {
    if (data.currentLevel == CurrentLevel::CURRENT_GREEN) {
      data.exitGreenCurrentLevel();
      data.enterYellowCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::CURRENT_YELLOW;
    }

    if (data.currentLevel == CurrentLevel::CURRENT_YELLOW) {
      data.exitYellowCurrentLevel();
      data.enterRedCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::CURRENT_RED;
    }
  } else {
    if (data.currentLevel == CurrentLevel::CURRENT_GREEN) {
      data.exitGreenCurrentLevel();
      data.enterRedCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::CURRENT_RED;
    }
  }
}

CurrentLevel CurrentController::GetCurrentLevel(unsigned int id, units::ampere_t current) {
  if (!_subsystemList.contains(id)) {
    return CurrentLevel::CURRENT_GREEN;
  }

  SubsystemData data = _subsystemList[id];
  if (current < data.greenMaxCurrentThreshold) {
    return CurrentLevel::CURRENT_GREEN;
  }

  if (current < data.yellowMaxCurrentThreshold && data.yellowCurrentLevelEnabled) {
    return CurrentLevel::CURRENT_YELLOW;
  }

  return CurrentLevel::CURRENT_RED;
}

std::string CurrentController::CurrentLevelToString(enum CurrentLevel level) {
  switch (level) {
    case CurrentLevel::CURRENT_RED:
      return "Red";
    case CurrentLevel::CURRENT_YELLOW:
      return "Yellow";
    case CurrentLevel::CURRENT_GREEN:
      return "Green";
  }

  return "Unknown";
}