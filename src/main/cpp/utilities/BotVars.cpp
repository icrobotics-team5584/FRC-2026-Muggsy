#include "utilities/BotVars.h"

#include <iostream>

botVars::Robot botVars::DetermineRobot() {
  std::string filePath = "/sys/class/net/eth0/address";
  std::cout << "Running DetermineRobot()\n";
  frc::SmartDashboard::PutString("botVars/comp bot MAC address", COMP_BOT_MAC_ADDRESS);
  frc::SmartDashboard::PutString("botVars/prac bot MAC address", PRACTICE_BOT_MAC_ADDRESS);
  if (std::filesystem::exists(filePath)) {
    std::ifstream file(filePath);
    std::string macAddress;
    file >> macAddress;
    frc::SmartDashboard::PutString("botVars/MAC address", macAddress);

    Robot robotType = Robot::COMP;
    frc::SmartDashboard::PutString("botVars/active robot",
      "ERROR! MAC address is not equal to COMP or PRACTISE. Defaulting to COMP Bot.");
    if (macAddress == COMP_BOT_MAC_ADDRESS) {
      frc::SmartDashboard::PutString("botVars/active robot", "COMP");
    } else if (macAddress == PRACTICE_BOT_MAC_ADDRESS) {
      frc::SmartDashboard::PutString("botVars/active robot", "PRACTICE");
      robotType = Robot::PRACTICE;
    }
    return robotType;
  }
  frc::SmartDashboard::PutString(
    "botVars/active robot", "ERROR! Could not match MAC address. Defaulting to COMP Bot.");
  return Robot::COMP;
}

botVars::Robot botVars::GetRobot() {
  static Robot activeRobot = DetermineRobot();
  return activeRobot;
}
