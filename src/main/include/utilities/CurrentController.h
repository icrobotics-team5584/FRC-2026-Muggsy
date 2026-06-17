#include <functional>
#include <optional>
#include <map>
#include <units/current.h>

enum CurrentLevel {
    Red = 2,
    Yellow = 1,
    Green = 0,
};

struct CurrentControllerSubsystem {
    std::string_view name;
    units::ampere_t greenCurrentThreshold;
    units::ampere_t redCurrentThreshold;
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
        static CurrentController& getInstance() {
            static CurrentController inst;
            return inst;
        }
        CurrentController(CurrentController const&) = delete;
        void operator=(CurrentController const&) = delete;

        int RegisterSubsystem(CurrentControllerSubsystem conf, std::optional<YellowCurrentLevel> yellowConf);
        void UnregisterSubsystem(unsigned int id);
        void Periodic();
        enum CurrentLevel GetCurrentLevel(unsigned int id, units::ampere_t current);
        std::string ConvertCurrentLevelToString(enum CurrentLevel level);

    private:
        struct SubsystemData {
            std::string_view name;
            CurrentLevel lastCurrentLevel;

            units::ampere_t greenCurrentThreshold;
            units::ampere_t redCurrentThreshold;
            std::function<units::ampere_t()> getSubsystemCurrent;
            std::function<void()> enterGreenCurrentLevel;
            std::function<void()> exitGreenCurrentLevel;
            std::function<void()> enterRedCurrentLevel;
            std::function<void()> exitRedCurrentLevel;
            
            bool yellowCurrentLevelEnabled;
            units::ampere_t yellowcurrentThreshold;
            std::function<void()> enterYellowCurrentLevel;
            std::function<void()> exitYellowCurrentLevel;
        };
        
        std::map<unsigned int, SubsystemData> _subsystemList{};
};
