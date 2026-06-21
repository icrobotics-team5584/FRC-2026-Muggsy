#include <functional>
#include <map>
#include <optional>
#include <units/current.h>

enum CritLevel {
  CRIT_RED = 2, /* Highest Current Level */
  CRIT_YELLOW = 1,
  CRIT_GREEN = 0, /* Lowest Current Level */
};

struct CurrentControllerSubsystem {
  std::string name = "Default Subsystem";
  units::ampere_t greenMaxCurrentThreshold = 1_A;
  std::function<units::ampere_t()> getSubsystemCurrent = [] { return 1_A; };
  std::function<void()> enterGreenCritLevel = [] {};
  std::function<void()> exitGreenCritLevel = [] {};
  std::function<void()> enterRedCritLevel = [] {};
  std::function<void()> exitRedCritLevel = [] {};
};

struct YellowCritLevel {
  units::ampere_t yellowMaxCurrentThreshold = 2_A;
  std::function<void()> enterYellowCritLevel = [] {};
  std::function<void()> exitYellowCritLevel = [] {};
};

class CurrentController {
 public:
  /* Turn into singleton */
  CurrentController() = default;
  static CurrentController& GetInstance() {
    static CurrentController inst;
    return inst;
  }
  CurrentController(CurrentController const&) = delete;
  void operator=(CurrentController const&) = delete;

  std::optional<int> RegisterSubsystem(const CurrentControllerSubsystem& conf,
    std::optional<YellowCritLevel> yellowConf = std::nullopt);
  void UnregisterSubsystem(unsigned int id);
  void Periodic();
  void LimitSubsystemFunctionality(unsigned int id);
  void ReallowSubsystemFunctionality(unsigned int id);
  std::optional<CritLevel> GetCritLevel(unsigned int id, units::ampere_t current);
  std::string CritLevelToString(enum CritLevel level);

 private:
  struct SubsystemData {
    CritLevel critLevel = CritLevel::CRIT_GREEN;

    CurrentControllerSubsystem subsystem = {};

    bool yellowCritLevelEnabled = false;
    YellowCritLevel yellowSubsystem = {};
  };

  std::map<unsigned int, SubsystemData> _subsystemList{};
};
