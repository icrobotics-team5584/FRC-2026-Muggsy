// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubFeeder::SubFeeder() = default;

void SubFeeder::Periodic() {
    frc::SmartDashboard::PutNumber("Feeder Motor Output", _feederMotor.Get());
}

frc2::CommandPtr SubFeeder::Feed() {
    return StartEnd(
        [this] {
            _feederMotor.Set(1);
        },
        [this] {
            _feederMotor.Set(0);
        }
    );
}
