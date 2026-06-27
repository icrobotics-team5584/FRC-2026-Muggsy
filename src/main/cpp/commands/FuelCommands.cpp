#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include "utilities/ShotPlanner.h"

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance().ExtendToLerp(1.0).AndThen(SubIntake::GetInstance().RunIntake());
}

frc2::CommandPtr ReverseIntakeSequence() {
  return SubDeploy::GetInstance().ExtendToLerp(1.0).AndThen(
    SubIntake::GetInstance().RunReverseIntake());
}

frc2::CommandPtr StationaryShootAt(frc::Translation2d target) {
  logger::FieldDisplay::GetInstance().DisplayPose(
    "Shooter/StationaryShootAt/Target", frc::Pose2d(target, frc::Rotation2d(0_deg)));

  auto distanceToTarget = [target] {
    units::meter_t dis = PoseHandler::GetInstance().GetPose().Translation().Distance(target);
    logger::Log("Shooter/StationaryShootAt/Distance To Target", dis);

    return dis;
  };

  auto aimmingSpeeds = [] {
    units::degree_t angleToTarget = SubDrivebase::GetInstance().CalcAngleToShotTarget();
    logger::Log("Shooter/StationaryShootAt/Angle to Target", angleToTarget);

    units::angular_velocity::turns_per_second_t rotationSpeeds = SubDrivebase::GetInstance().CalcRotateSpeed(SubDrivebase::GetInstance().GetGyroAngle().Degrees() - angleToTarget);

    return frc::ChassisSpeeds{0_mps, 0_mps, rotationSpeeds};
  };

  auto isPassing = [] {
    return ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;
  };

  return frc2::cmd::Parallel(
    SubDrivebase::GetInstance().Drive(aimmingSpeeds, true),
    SubShooter::GetInstance().SetSpeedFromDistanceTarget(distanceToTarget, isPassing),
    SubHood::GetInstance().SetPositionFromDistanceToTarget(distanceToTarget))
    .Until([] { 
      bool ready = IsReadyToShoot();
      logger::Log("Shooter/StationaryShootAt/Ready to shoot", ready);
      return ready; })
    .AndThen(frc2::cmd::Parallel(
      SubFeeder::GetInstance().Feed(), SubIndexer::GetInstance().SpinIndexer()));
}

bool IsReadyToShoot() {
  return SubHood::GetInstance().IsAtTarget() && SubShooter::GetInstance().IsReadyToShoot() &&
         SubDrivebase::GetInstance().CalcAngleToShotTarget() < 5_deg &&
         ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).shouldShoot;
}

frc2::CommandPtr ShootWhenReady() {
  return frc2::cmd::WaitUntil([] { return IsReadyToShoot(); })
    .AndThen(frc2::cmd::Parallel(SubFeeder::GetInstance().Feed(),
      SubIndexer::GetInstance().SpinIndexer(), SubIntake::GetInstance().RunIntake())
        .Until([] { return !IsReadyToShoot(); }))
    .Repeatedly();
};

frc2::CommandPtr ToggleBrakeCoast() {
  return frc2::cmd::StartEnd(
    [] {
      SubDrivebase::GetInstance().SetBrakeMode(false);
      SubHood::GetInstance().SetBrakeMode(false);
    },
    [] {
      SubDrivebase::GetInstance().SetBrakeMode(true);
      SubHood::GetInstance().SetBrakeMode(true);
    })
    .IgnoringDisable(true)
    .Until([] { return frc::DriverStation::IsEnabled(); });
}

frc2::CommandPtr EjectFuel() {
  return frc2::cmd::Parallel(SubShooter::GetInstance().SpinSlowly(),
    SubIntake::GetInstance().RunReverseIntake(), SubIndexer::GetInstance().SpinIndexer(),
    SubFeeder::GetInstance().Feed(), SubHood::GetInstance().HoodToEjectAngle());
}
}  // namespace cmd