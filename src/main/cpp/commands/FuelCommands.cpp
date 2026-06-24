#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

#include "utilities/PoseHandler.h"
#include "utilities/ShotPlanner.h"
#include "utilities/Logger.h"

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance().ExtendToLerp(1.0).AndThen(SubIntake::GetInstance().RunIntake());
}

frc2::CommandPtr ReverseIntakeSequence() {
  return SubDeploy::GetInstance().ExtendToLerp(1.0).AndThen(
    SubIntake::GetInstance().RunReverseIntake());
}

frc2::CommandPtr StationaryShootAt(frc::Translation2d target, frc2::CommandXboxController& controller) {
  auto distanceToTarget = [target] {
    auto curPose = PoseHandler::GetInstance().GetPose();
    auto shooterPose = curPose.TransformBy(SubDrivebase::ROBOT_CENTRE_TO_SHOOTER);

    logger::FieldDisplay::GetInstance().DisplayPose("Shooter/shooterPose", shooterPose);
    logger::Log("Shooter/distToTargetInner", target.Distance(shooterPose.Translation()));

    return target.Distance(shooterPose.Translation());  
  };

  auto angleToTarget = [] {return SubDrivebase::GetInstance().CalcAngleToShotTarget();};

  return frc2::cmd::Parallel(SubDrivebase::GetInstance().JoystickDriveWithAngle(controller, angleToTarget, 0.05),
    SubShooter::GetInstance().SetSpeedFromDistanceTarget(distanceToTarget, []{return ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;}),
    SubHood::GetInstance().SetPositionFromDistanceToTarget(distanceToTarget)
  ).Until([] {
    return IsReadyToShoot();
  }).AndThen(frc2::cmd::Parallel(
    SubFeeder::GetInstance().Feed(),
    SubIndexer::GetInstance().SpinIndexer()
  ));
}

bool IsReadyToShoot() {
  return SubHood::GetInstance().IsAtTarget() &&
         SubShooter::GetInstance().IsReadyToShoot() &&
         SubDrivebase::GetInstance().CalcAngleToShotTarget() < 5_deg &&
         ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).shouldShoot;
}

frc2::CommandPtr ShootWhenReady() {
  return frc2::cmd::WaitUntil([] { return IsReadyToShoot(); })
    .AndThen(frc2::cmd::Parallel(
      SubFeeder::GetInstance().Feed(),
      SubIndexer::GetInstance().SpinIndexer(),
      SubIntake::GetInstance().RunIntake()
    ).Until([] {
      return !IsReadyToShoot();
    })).Repeatedly();
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