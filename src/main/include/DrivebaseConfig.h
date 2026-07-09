#pragma once

#include "utilities/BotVars.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace drivebaseConfig {
// Drive controls
constexpr units::meters_per_second_t MAX_VELOCITY = 4_mps;
constexpr units::turns_per_second_t MAX_TELEOP_ANGULAR_VELOCITY = 290_deg_per_s;

constexpr units::meters_per_second_t MAX_P2P_VELOCITY = 4_mps;
constexpr units::meters_per_second_squared_t MAX_P2P_ACCEL = 15_mps_sq;
constexpr units::turns_per_second_t MAX_P2P_ANGULAR_VELOCITY = 300_deg_per_s;
constexpr units::turns_per_second_squared_t MAX_P2P_ANGULAR_ACCEL = 6_tr_per_s_sq;

constexpr double MAX_JOYSTICK_ACCEL = 5;
constexpr double MAX_ANGULAR_JOYSTICK_ACCEL = 3;
constexpr double JOYSTICK_DEADBAND = 0.08;
constexpr double TRANSLATION_EXPONENT = 2;  // Set to 1 for linear scaling
constexpr double ROTATION_EXPONENT = 1;     // Set to 1 for linear scaling

// Swerve config
constexpr frc::Translation2d FL_POSITION{+0.281_m, +0.281_m};
constexpr frc::Translation2d FR_POSITION{+0.281_m, -0.281_m};
constexpr frc::Translation2d BL_POSITION{-0.281_m, +0.281_m};
constexpr frc::Translation2d BR_POSITION{-0.281_m, -0.281_m};

const units::turn_t FRONT_RIGHT_MAG_OFFSET = -0.51806640625_tr;
const units::turn_t FRONT_LEFT_MAG_OFFSET = -0.077392578125_tr;
const units::turn_t BACK_RIGHT_MAG_OFFSET = -0.67333984375_tr;
const units::turn_t BACK_LEFT_MAG_OFFSET = -0.94482421875_tr;

// PID constants for translation and rotation controllers
/* We use std::array<T, int> instead of frc::PIDController to avoid the
 * static initialization order fiasco. i.e. frc::PIDController gets
 * incorrectly deconstructed and causes a segfaualt when exiting the sim.*/
constexpr std::array<double, 3> P2P_TRANSLATION_PID{5, 0, 0};
constexpr std::array<double, 3> P2P_ROTATION_PID{26, 0, 0};

static constexpr units::meter_t CENTRE_TO_BUMPER_EDGE = 0.435_m;
}  // namespace drivebaseConfig