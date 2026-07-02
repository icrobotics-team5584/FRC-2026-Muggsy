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
    "StationaryShootAt/Target", frc::Pose2d(target, frc::Rotation2d(0_deg)));

  auto distanceToTarget = [target] {
    auto currentPose = PoseHandler::GetInstance().GetPose();
    auto shooterPose = currentPose.TransformBy(SubDrivebase::ROBOT_CENTRE_TO_SHOOTER);
    units::meter_t dis = target.Distance(shooterPose.Translation());
    logger::FieldDisplay::GetInstance().DisplayPose("StationaryShootAt/ShooterPose", shooterPose);

    logger::Log("StationaryShootAt/Distance To Target", dis);

    return dis;
  };

  auto isPassing = [] {
    return ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;
  };

  return frc2::cmd::Parallel(SubDrivebase::GetInstance().RotateTo([] { return CalcAngleToShotTarget().Radians(); }),
    SubShooter::GetInstance().SetSpeedFromDistanceTarget(distanceToTarget, isPassing),
    SubHood::GetInstance().SetPositionFromDistanceToTarget(distanceToTarget), ShootWhenReady());
}

frc2::CommandPtr TuneShooterAndHoodTables() {
  auto aimingSpeeds = [] {
    auto targetAngle = CalcAngleToShotTarget();
    auto currentAngle = SubDrivebase::GetInstance().GetGyroAngle();
    units::angular_velocity::turns_per_second_t rotationSpeeds = SubDrivebase::GetInstance().CalcRotateSpeed(currentAngle.Degrees(), targetAngle.Degrees());

    return frc::ChassisSpeeds{0_mps, 0_mps, rotationSpeeds};
  };

  return SubDrivebase::GetInstance().Drive(aimingSpeeds, true).Until([] {
    return units::math::abs(SubDrivebase::GetInstance().GetRotationError()) < 5_deg;
  }).AndThen(frc2::cmd::Parallel(
    SubFeeder::GetInstance().Feed(),
    SubIndexer::GetInstance().SpinIndexer(),
    SubIntake::GetInstance().RunIntake(),
    SubDeploy::GetInstance().ExtendToStow()));
}

bool IsReadyToShoot() {
  return SubHood::GetInstance().IsAtTarget() && SubShooter::GetInstance().IsReadyToShoot() &&
         units::math::abs(SubDrivebase::GetInstance().GetRotationError()) < 5_deg &&
         ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).shouldShoot;
}

frc2::CommandPtr ShootWhenReady() {
  return frc2::cmd::WaitUntil([] { return IsReadyToShoot(); })
    .AndThen(
      frc2::cmd::Parallel(SubFeeder::GetInstance().Feed(), SubIndexer::GetInstance().SpinIndexer(),
        SubIntake::GetInstance().RunIntake(), SubDeploy::GetInstance().ExtendToStow())
        .Until([] { return !IsReadyToShoot(); }))
    .Repeatedly();
}

frc::Pose2d CalcFutureDrumPose() {
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
  frc::Transform2d latencyTransform =
    frc::Transform2d(robotRelativeVelX * offset, robotRelativeVelY * offset, robotVelRot * offset);
  robot = robot.TransformBy(latencyTransform);

  logger::Log("SOTM/velX", robotVelX);
  logger::Log("SOTM/velY", robotVelY);
  logger::Log("SOTM/velRot", robotVelRot);
  logger::Log("SOTM/robotRelativeVelX", robotRelativeVelX);
  logger::Log("SOTM/robotRelativeVelY", robotRelativeVelY);

  // Estimate time of flight
  units::second_t TOF;
  frc::Pose2d futurePose;

  for (int i = 0; i < 15; i++) {
    // Get future pose
    TOF = SubShooter::GetInstance().GetTimeOfFlightFromDistance(distance);
    logger::Log("SOTM/TimeOfFlight", TOF);

    // calculate offset due to velocity
    units::meter_t offsetX = robotVelX * TOF;
    units::meter_t offsetY = robotVelY * TOF;
    units::degree_t robotRotation = robot.Rotation().Degrees();
    units::meter_t robotX = robot.X();
    units::meter_t robotY = robot.Y();

    // calculate future pose by adding offsets to current robot position
    futurePose = frc::Pose2d(robotX + offsetX, robotY + offsetY, robot.Rotation().Degrees());

    // Re-update distance to target for next calculation
    distance = target.Distance(futurePose.Translation());
  }

  frc::Pose2d futureDrumPose = futurePose.TransformBy(SubDrivebase::ROBOT_CENTRE_TO_SHOOTER);
  return futureDrumPose;
}

units::meter_t CalcShootOnTheMoveDistance() {
  // Find distance from future drum pose to target
  units::meter_t futureDistance = GetShotTarget().Distance(CalcFutureDrumPose().Translation());
  logger::Log("SOTM/futureDistance", futureDistance);

  return futureDistance;
}

units::degree_t CalcShootOnTheMoveAngle() {
  // Find angle from future drum pose to target
  auto futurePoseToTarget = GetShotTarget() - CalcFutureDrumPose().Translation();
  units::degree_t angleFromFutureToTarget = futurePoseToTarget.Angle().Degrees();

  logger::Log("SOTM/futureAngle", angleFromFutureToTarget);
  return angleFromFutureToTarget;
}

frc2::CommandPtr ShootOnTheMove() {
  return frc2::cmd::Parallel(
    SubShooter::GetInstance().SetSpeedFromDistanceTarget(
      [] { return CalcShootOnTheMoveDistance(); },
      [] {
        return ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;
      }),
    frc2::cmd::Either(
      SubHood::GetInstance().HoodToPassingAngle(),
      SubHood::GetInstance().SetPositionFromDistanceToTarget([] { return CalcShootOnTheMoveDistance(); }),
      [] {
        return ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).isPassing;
      }),
    SubDrivebase::GetInstance().RotateTo([] { return CalcShootOnTheMoveAngle(); }),
    ShootWhenReady());
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
  frc::Translation2d target =
    ShotPlanner::CalculateShotTarget(currentPose).targetPosition.ToTranslation2d();
  logger::FieldDisplay::GetInstance().DisplayPose("Shot target", frc::Pose2d{target, {0_deg}});
  return target;
}

frc::Rotation2d CalcAngleToShotTarget() {
  frc::Pose2d currentPose = PoseHandler::GetInstance().GetPose();
  frc::Translation2d target = GetShotTarget();

  frc::Translation2d robotToTarget = target - currentPose.Translation();
  return robotToTarget.Angle().RotateBy({180_deg});
}

}  // namespace cmd