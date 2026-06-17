#include <utilities/CurrentController.h>


int CurrentController::RegisterSubsystem(CurrentControllerSubsystem conf, std::optional<YellowCurrentLevel> yellowConf) {
    SubsystemData data = {0};
    data.name = conf.name;
    data.greenMaxCurrentThreshold = conf.greenMaxCurrentThreshold;
	data.getSubsystemCurrent = conf.getSubsystemCurrent;
	data.enterGreenCurrentLevel = conf.enterGreenCurrentLevel;
	data.exitGreenCurrentLevel = conf.exitGreenCurrentLevel;
	data.enterRedCurrentLevel = conf.enterRedCurrentLevel;
	data.exitRedCurrentLevel = conf.exitRedCurrentLevel;

    if(yellowConf) {
        YellowCurrentLevel yconf = yellowConf.value();
        if(yconf.currentThreshold < conf.greenMaxCurrentThreshold) {
            return -1;
        }
        data.yellowCurrentLevelEnabled = true;
        data.yellowMaxCurrentThreshold = yconf.currentThreshold;
        data.enterYellowCurrentLevel = conf.enterRedCurrentLevel;
	    data.exitYellowCurrentLevel = conf.exitRedCurrentLevel;
    }

    unsigned int id = 0;
    while(_subsystemList.contains(id)) {
        id++;
    }
    _subsystemList[id] = data;

    return id;
}

void CurrentController::UnregisterSubsystem(unsigned int id) {
    if(_subsystemList.contains(id)) {
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

        if(!cld)
            continue;

        switch(cld) {
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
    if(!_subsystemList.contains(id)) {
        return;
    }

	SubsystemData data = _subsystemList[id];
    if(data.yellowCurrentLevelEnabled) {
        if(data.currentLevel == CurrentLevel::Red) {
            data.exitRedCurrentLevel();
            data.enterYellowCurrentLevel();
            _subsystemList[id].currentLevel = CurrentLevel::Yellow;
        }

        if(data.currentLevel == CurrentLevel::Yellow) {
            data.exitYellowCurrentLevel();
            data.enterGreenCurrentLevel();
            _subsystemList[id].currentLevel = CurrentLevel::Green;
        }
    } else {
        if(data.currentLevel == CurrentLevel::Red) {
            data.exitRedCurrentLevel();
            data.enterGreenCurrentLevel();
            _subsystemList[id].currentLevel = CurrentLevel::Green;
        }
    }
}
void CurrentController::IncreaseCurrentLevel(unsigned int id) {
    if(!_subsystemList.contains(id)) {
        return;
    }

	SubsystemData data = _subsystemList[id];
    if(data.yellowCurrentLevelEnabled) {
        if(data.currentLevel == CurrentLevel::Green) {
            data.exitGreenCurrentLevel();
            data.enterYellowCurrentLevel();
            _subsystemList[id].currentLevel = CurrentLevel::Yellow;
        }

        if(data.currentLevel == CurrentLevel::Yellow) {
            data.exitYellowCurrentLevel();
            data.enterRedCurrentLevel();
            _subsystemList[id].currentLevel = CurrentLevel::Red;
        }
    } else {
        if(data.currentLevel == CurrentLevel::Green) {
            data.exitGreenCurrentLevel();
            data.enterRedCurrentLevel();
            _subsystemList[id].currentLevel = CurrentLevel::Red;
        }
    }
} 


CurrentLevel CurrentController::GetCurrentLevel(unsigned int id, units::ampere_t current) {
    if(!_subsystemList.contains(id)) {
        return CurrentLevel::Green;
    }

    SubsystemData data = _subsystemList[id];
    if(current < data.greenMaxCurrentThreshold) {
        return CurrentLevel::Green;
    }

    if(current < data.yellowMaxCurrentThreshold && data.yellowCurrentLevelEnabled) {
        return CurrentLevel::Yellow;
    }

    return CurrentLevel::Red;
}