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
#include "utilities/Logger.h"

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
    auto currentPose = PoseHandler::GetInstance().GetPose();
    units::meter_t dis = target.Distance(currentPose.Translation());
    logger::Log("Shooter/StationaryShootAt/Distance To Target", dis);

    return dis;
  };

  auto aimingSpeeds = [] {
    units::degree_t desiredAngle = CalcAngleToShotTarget();
    units::degree_t currentAngle = SubDrivebase::GetInstance().GetGyroAngle().Degrees();
    logger::Log("Shooter/StationaryShootAt/Angle to Target", desiredAngle);
    units::angular_velocity::turns_per_second_t rotationSpeeds = SubDrivebase::GetInstance().CalcRotateSpeed(currentAngle, desiredAngle);

    return frc::ChassisSpeeds{0_mps, 0_mps, rotationSpeeds};
  };

  auto isPassing = [] {
    return ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;
  };

  return frc2::cmd::Parallel(SubDrivebase::GetInstance().Drive(aimingSpeeds, true),
    SubShooter::GetInstance().SetSpeedFromDistanceTarget(distanceToTarget, isPassing),
    SubHood::GetInstance().SetPositionFromDistanceToTarget(distanceToTarget)).Until([] { 
      bool ready = IsReadyToShoot();
      logger::Log("Shooter/StationaryShootAt/Ready to shoot", ready);
      return ready; })
    .AndThen(frc2::cmd::Parallel(
      SubFeeder::GetInstance().Feed(), SubIndexer::GetInstance().SpinIndexer()));
}

bool IsReadyToShoot() {
  return SubHood::GetInstance().IsAtTarget() && SubShooter::GetInstance().IsReadyToShoot() &&
         units::math::abs(CalcAngleToShotTarget()) < 5_deg &&
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
}

frc2::CommandPtr CalcFutureDrumPose() {
  units::millisecond_t offset = logger::Tune("SOTM/LatencyOffset", LATENCY_OFFSET);
  
  // Calculate distance to target from robot
  auto target = GetShotTarget();
  auto robot = PoseHandler::GetInstance().GetPose();
  logger::FieldDisplay::GetInstance().DisplayPose("SOTM/Robot pose", robot);
  units::meter_t distance = target.Distance(robot.Translation());

  // Calculate field relative robot velocity
  frc::ChassisSpeeds robotVel = SubDrivebase::GetInstance().GetChassisSpeeds();
  units::meters_per_second_t robotVelX = robotVel.vx;
  units::meters_per_second_t robotVelY = robotVel.vy;
  units::degrees_per_second_t robotVelRot = SubDrivebase::GetInstance().GetDesiredAngularVelocity();

  // Account for latency
  frc::ChassisSpeeds robotRelativeVel = SubDrivebase::GetInstance().GetChassisSpeeds(false);
  units::meters_per_second_t robotRelativeVelX = robotRelativeVel.vx;
  units::meters_per_second_t robotRelativeVelY = robotRelativeVel.vy;
  frc::Transform2d latencyTransform = frc::Transform2d(robotRelativeVelX * offset, robotRelativeVelY * offset, robotVelRot * offset);
  robot = robot.TransformBy(latencyTransform);

  logger::Log("SOTM/velX", robotVelX);
  logger::Log("SOTM/velY", robotVelY);
  logger::Log("SOTM/velRot", robotVelRot);
  logger::Log("SOTM/robotRelativeVelX", robotRelativeVelX);
  logger::Log("SOTM/robotRelativeVelY", robotRelativeVelY);

  // Estimate time of flight
  units::second_t TOF;
  frc::Pose2d futurePose;
  
  return frc2::cmd::Idle();
}

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

frc::Translation2d GetShotTarget() {
  frc::Pose2d currentPose = PoseHandler::GetInstance().GetPose();
  frc::Translation2d target = ShotPlanner::CalculateShotTarget(currentPose).targetPosition.ToTranslation2d();
  logger::FieldDisplay::GetInstance().DisplayPose("Shot target", frc::Pose2d{target, {0_deg}});
  return target;
}

units::degree_t CalcAngleToShotTarget() {
  frc::Pose2d currentPose = PoseHandler::GetInstance().GetPose();
  frc::Translation2d target = GetShotTarget();

  units::degree_t desired = units::radian_t(std::atan2((target.Y() - currentPose.Y()).value(), (target.X() - currentPose.X()).value()));

  return desired + 180_deg;
}

}  // namespace cmd