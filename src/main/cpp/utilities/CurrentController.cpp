#include <utilities/CurrentController.h>

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
    data.enterYellowCurrentLevel = conf.enterRedCurrentLevel;
    data.exitYellowCurrentLevel = conf.exitRedCurrentLevel;
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
    int lcl = static_cast<int>(data.currentLevel);
    int ccl = static_cast<int>(GetCurrentLevel(id, data.getSubsystemCurrent()));
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
  }
}

void CurrentController::DecreaseCurrentLevel(unsigned int id) {
  if (!_subsystemList.contains(id)) {
    return;
  }

  SubsystemData data = _subsystemList[id];
  if (data.yellowCurrentLevelEnabled) {
    if (data.currentLevel == CurrentLevel::RED) {
      data.exitRedCurrentLevel();
      data.enterYellowCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::YELLOW;
    }

    if (data.currentLevel == CurrentLevel::YELLOW) {
      data.exitYellowCurrentLevel();
      data.enterGreenCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::GREEN;
    }
  } else {
    if (data.currentLevel == CurrentLevel::RED) {
      data.exitRedCurrentLevel();
      data.enterGreenCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::GREEN;
    }
  }
}
void CurrentController::IncreaseCurrentLevel(unsigned int id) {
  if (!_subsystemList.contains(id)) {
    return;
  }

  SubsystemData data = _subsystemList[id];
  if (data.yellowCurrentLevelEnabled) {
    if (data.currentLevel == CurrentLevel::GREEN) {
      data.exitGreenCurrentLevel();
      data.enterYellowCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::YELLOW;
    }

    if (data.currentLevel == CurrentLevel::YELLOW) {
      data.exitYellowCurrentLevel();
      data.enterRedCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::RED;
    }
  } else {
    if (data.currentLevel == CurrentLevel::GREEN) {
      data.exitGreenCurrentLevel();
      data.enterRedCurrentLevel();
      _subsystemList[id].currentLevel = CurrentLevel::RED;
    }
  }
}

CurrentLevel CurrentController::GetCurrentLevel(unsigned int id, units::ampere_t current) {
  if (!_subsystemList.contains(id)) {
    return CurrentLevel::GREEN;
  }

  SubsystemData data = _subsystemList[id];
  if (current < data.greenMaxCurrentThreshold) {
    return CurrentLevel::GREEN;
  }

  if (current < data.yellowMaxCurrentThreshold && data.yellowCurrentLevelEnabled) {
    return CurrentLevel::YELLOW;
  }

  return CurrentLevel::RED;
}