#include <utilities/CurrentController.h>


int CurrentController::RegisterSubsystem(CurrentControllerSubsystem conf, std::optional<YellowCurrentLevel> yellowConf) {
    if(conf.greenCurrentThreshold > conf.redCurrentThreshold) {
        return -1;
    }

    SubsystemData data = {0};
    data.name = conf.name;
    data.greenCurrentThreshold = conf.greenCurrentThreshold;
    data.redCurrentThreshold = conf.redCurrentThreshold;
	data.getSubsystemCurrent = conf.getSubsystemCurrent;
	data.enterGreenCurrentLevel = conf.enterGreenCurrentLevel;
	data.exitGreenCurrentLevel = conf.exitGreenCurrentLevel;
	data.enterRedCurrentLevel = conf.enterRedCurrentLevel;
	data.exitRedCurrentLevel = conf.exitRedCurrentLevel;

    if(yellowConf) {
        YellowCurrentLevel yconf = yellowConf.value();
        if(yconf.currentThreshold > conf.redCurrentThreshold || yconf.currentThreshold < conf.greenCurrentThreshold) {
            return -1;
        }
        data.yellowCurrentLevelEnabled = true;
        data.yellowcurrentThreshold = yconf.currentThreshold;
        data.enterYellowCurrentLevel = conf.enterRedCurrentLevel;
	    data.exitYellowCurrentLevel = conf.exitRedCurrentLevel;
    }

    int id = 0;
    while(_subsystemList.contains(id)) {
        id++;
    }
    _subsystemList[id] = data;

    return id;
}