#include "utilities/NeoIO.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkMax.h>
#include <string>

NeoIO::NeoIO(int turnCanID, int driveCanID, int encoderCanID, units::turn_t cancoderMagOffset)
  : canTurnMotor(turnCanID), canDriveMotor(driveCanID), _canEncoder(encoderCanID) {
  frc::SmartDashboard::PutData(
    "swerve/DriveMotor" + std::to_string(driveCanID), (wpi::Sendable*)&canDriveMotor);
  frc::SmartDashboard::PutData(
    "swerve/TurnMotor" + std::to_string(turnCanID), (wpi::Sendable*)&canTurnMotor);
}

void NeoIO::ConfigTurnMotor() {
  rev::spark::SparkBaseConfig canTurnConfig;

  canTurnConfig.SmartCurrentLimit(40);
  canTurnConfig.encoder.PositionConversionFactor(1.0 / TURNING_GEAR_RATIO)
    .VelocityConversionFactor(TURNING_GEAR_RATIO);
  canTurnConfig.closedLoop.Pid(TURN_P, TURN_I, TURN_D);
  canTurnConfig.closedLoop.PositionWrappingEnabled(true)
    .PositionWrappingMinInput(0)
    .PositionWrappingMaxInput(1);
  canTurnConfig.Inverted(true).SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

  canTurnMotor.OverwriteConfig(canTurnConfig);
}

void NeoIO::SetDesiredAngle(units::degree_t angle) {
  canTurnMotor.SetPositionTarget(angle);
  desiredAngle = angle;
}

void NeoIO::SetAngle(units::turn_t angle) {
  /*
    @param angle: angle of encoder
  */
  canTurnMotor.SetCANTimeout(500);
  int maxAttempts = 15;
  int currentAttempts = 0;
  units::turn_t tolerance = 0.01_tr;
  while (units::math::abs(canTurnMotor.GetPosition() - angle) > tolerance &&
         currentAttempts < maxAttempts) {
    canTurnMotor.SetPosition(angle);
    currentAttempts++;
  }

  currentAttempts = 0;
  canTurnMotor.SetCANTimeout(10);
}

void NeoIO::SendSensorsToDash() {
  // no-op, spark logging handled by ICSpark
}

void NeoIO::SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF) {
  units::turns_per_second_t turnsPerSec = (velocity.value() / WHEEL_CIRCUMFERENCE.value()) * 1_tps;
  canDriveMotor.SetVelocityTarget(turnsPerSec);
  desiredSpeed = velocity;
}

void NeoIO::DriveStraightVolts(units::volt_t volts) {
  SetDesiredAngle(0_deg);
  canDriveMotor.SetVoltage(volts);
}

void NeoIO::StopMotors() {
  canDriveMotor.Set(0);
  canTurnMotor.Set(0);
}

void NeoIO::UpdateSim(units::second_t deltaTime) {}

void NeoIO::SetNeutralMode(bool brakeModeToggle) {
  rev::spark::SparkBaseConfig neutralModeConfig;

  if (brakeModeToggle) {
    neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    canDriveMotor.AdjustConfigNoPersist(neutralModeConfig);
    canTurnMotor.AdjustConfigNoPersist(neutralModeConfig);
  } else if (!brakeModeToggle) {
    neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    canDriveMotor.AdjustConfigNoPersist(neutralModeConfig);
    canTurnMotor.AdjustConfigNoPersist(neutralModeConfig);
  }
}

void NeoIO::ConfigDriveMotor() {
  rev::spark::SparkBaseConfig canDriveConfig;

  canDriveConfig.SmartCurrentLimit(40);
  canDriveConfig.closedLoop.Pid(DRIVE_P, DRIVE_I, DRIVE_D);
  canDriveConfig.closedLoop.feedForward.kV(DRIVE_FF);
  canDriveConfig.encoder.PositionConversionFactor(1.0 / DRIVE_GEAR_RATIO)
    .VelocityConversionFactor(1.0 / DRIVE_GEAR_RATIO);
  canDriveConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

  canDriveMotor.OverwriteConfig(canDriveConfig);
}

frc::SwerveModulePosition NeoIO::GetPosition() {
  units::meter_t distance = canDriveMotor.GetPosition().value() * WHEEL_CIRCUMFERENCE;
  return {distance, GetAngle()};
}

frc::Rotation2d NeoIO::GetAngle() {
  units::radian_t turnAngle = canTurnMotor.GetPosition();
  return turnAngle;
}

frc::Rotation2d NeoIO::GetDesiredAngle() {
  return desiredAngle;
}

units::meters_per_second_t NeoIO::GetSpeed() {
  return (canDriveMotor.GetVelocity().convert<units::turns_per_second>().value() *
           WHEEL_CIRCUMFERENCE.value()) *
         1_mps;
}

units::meters_per_second_t NeoIO::GetDesiredSpeed() {
  return desiredSpeed;
}

units::volt_t NeoIO::GetDriveVoltage() {
  return canDriveMotor.GetAppliedOutput() * canDriveMotor.GetBusVoltage() * 1_V;
}

frc::SwerveModuleState NeoIO::GetState() {
  return {GetSpeed(), GetAngle()};
}

frc::SwerveModuleState NeoIO::GetDesiredState() {
  return {GetDesiredSpeed(), GetDesiredAngle()};
}

units::radian_t NeoIO::GetDrivenRotations() {
  return canDriveMotor.GetPosition();
}
