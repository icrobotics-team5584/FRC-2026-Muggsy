#pragma once

namespace canid {
constexpr int DRIVEBASE_FRONT_RIGHT_DRIVE = 1;
constexpr int DRIVEBASE_FRONT_RIGHT_TURN = 2;
constexpr int DRIVEBASE_FRONT_RIGHT_ENCODER = 3;

constexpr int DRIVEBASE_FRONT_LEFT_DRIVE = 4;
constexpr int DRIVEBASE_FRONT_LEFT_TURN = 5;
constexpr int DRIVEBASE_FRONT_LEFT_ENCODER = 6;

constexpr int DRIVEBASE_BACK_RIGHT_DRIVE = 7;
constexpr int DRIVEBASE_BACK_RIGHT_TURN = 8;
constexpr int DRIVEBASE_BACK_RIGHT_ENCODER = 9;

constexpr int DRIVEBASE_BACK_LEFT_DRIVE = 10;
constexpr int DRIVEBASE_BACK_LEFT_TURN = 11;
constexpr int DRIVEBASE_BACK_LEFT_ENCODER = 12;

constexpr int PIGEON_2 = 13;

constexpr int SHOOTER_MOTOR_1 = 14;
constexpr int SHOOTER_MOTOR_2 = 16;
constexpr int SHOOTER_MOTOR_3 = 15;
constexpr int SHOOTER_MOTOR_4 = 17;

constexpr int INDEXER_MOTOR = 18;
constexpr int INDEXER_FOLLOW_MOTOR = 19;

constexpr int INTAKE_MOTOR_1 = 20;
constexpr int INTAKE_MOTOR_2 = 21; /* Follower */

constexpr int DEPLOY_MOTOR = 58;

constexpr int HOOD_MOTOR = 23;

constexpr int FEEDER_MOTOR = 24;

constexpr int PDH = 60;
}  // namespace canid

namespace dio {
constexpr int BRAKE_COAST_BUTTON = 0;
}

namespace pwm {
constexpr int LED = 0;
}
