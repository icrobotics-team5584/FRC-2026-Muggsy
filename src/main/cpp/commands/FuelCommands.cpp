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
  units::degree_t angleToTarget = SubDrivebase::GetInstance().CalcAngleToShotTarget();

  frc::Pose2d curPose = PoseHandler::GetInstance().GetPose();
  frc::Pose2d aimmingPose = frc::Pose2d(curPose.Translation(), angleToTarget);

  units::length::meter_t distanceToTarget = curPose.Translation().Distance(target);
  logger::Log("Shooter/distToTargetInner", distanceToTarget);

  bool isPassing = ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;

  return frc2::cmd::StartEnd(
    [aimmingPose, distanceToTarget, isPassing] {
      frc2::cmd::Parallel(
        SubDrivebase::GetInstance().DriveToPose([aimmingPose] { return aimmingPose; }),
        SubShooter::GetInstance().SetSpeedFromDistanceTarget(
          [distanceToTarget] { return distanceToTarget; }, [isPassing] { return isPassing; }),
        SubHood::GetInstance().SetPositionFromDistanceToTarget(
          [distanceToTarget] { return distanceToTarget; }))
        .Until([] { return IsReadyToShoot(); })
        .AndThen(frc2::cmd::Parallel(
          SubFeeder::GetInstance().Feed(), SubIndexer::GetInstance().SpinIndexer()));
    },
    [] {
      return frc2::cmd::Parallel(
        SubHood::GetInstance().HoodToStowAngle(), SubShooter::GetInstance().Stop());
    });
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