#include <functional>
#include <map>
#include <optional>
#include <units/current.h>

enum CurrentLevel {
  CURRENT_RED = 2, /* Highest Current Level */
  CURRENT_YELLOW = 1,
  CURRENT_GREEN = 0, /* Lowest Current Level */
};

struct CurrentControllerSubsystem {
  std::string_view name;
  units::ampere_t greenMaxCurrentThreshold;
  std::function<units::ampere_t()> getSubsystemCurrent;
  std::function<void()> enterGreenCurrentLevel;
  std::function<void()> exitGreenCurrentLevel;
  std::function<void()> enterRedCurrentLevel;
  std::function<void()> exitRedCurrentLevel;
};

struct YellowCurrentLevel {
  units::ampere_t currentThreshold;
  std::function<void()> enterCurrentLevel;
  std::function<void()> exitCurrentLevel;
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

  int RegisterSubsystem(
    const CurrentControllerSubsystem& conf, std::optional<YellowCurrentLevel> yellowConf);
  void UnregisterSubsystem(unsigned int id);
  void Periodic();
  void DecreaseCurrentLevel(unsigned int id);
  void IncreaseCurrentLevel(unsigned int id);
  CurrentLevel GetCurrentLevel(unsigned int id, units::ampere_t current);
  std::string CurrentLevelToString(enum CurrentLevel level);

 private:
  struct SubsystemData {
    std::string name;
    CurrentLevel currentLevel;

    units::ampere_t greenMaxCurrentThreshold;
    std::function<units::ampere_t()> getSubsystemCurrent;
    std::function<void()> enterGreenCurrentLevel;
    std::function<void()> exitGreenCurrentLevel;
    std::function<void()> enterRedCurrentLevel;
    std::function<void()> exitRedCurrentLevel;

    bool yellowCurrentLevelEnabled;
    units::ampere_t yellowMaxCurrentThreshold;
    std::function<void()> enterYellowCurrentLevel;
    std::function<void()> exitYellowCurrentLevel;
  };

  std::map<unsigned int, SubsystemData> _subsystemList{};
};
