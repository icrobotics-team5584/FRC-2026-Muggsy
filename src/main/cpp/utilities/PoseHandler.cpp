#include "utilities/PoseHandler.h"

#include "utilities/ICGeometry.h"
#include "utilities/Logger.h"

PoseHandler::PoseHandler() = default;

frc::Pose2d PoseHandler::GetPose() {
  auto pose = _poseEstimator.GetEstimatedPosition();
  return {std::clamp(pose.X(), drivebaseConfig::CENTRE_TO_BUMPER_EDGE,
            ICGeometry::FIELD_LENGTH - drivebaseConfig::CENTRE_TO_BUMPER_EDGE),
    std::clamp(pose.Y(), drivebaseConfig::CENTRE_TO_BUMPER_EDGE,
      ICGeometry::FIELD_WIDTH - drivebaseConfig::CENTRE_TO_BUMPER_EDGE),
    pose.Rotation()};
}

frc::Pose2d PoseHandler::GetSimPose() {
  return _simPoseEstimator.GetEstimatedPosition();
}

void PoseHandler::SetPose(frc::Pose2d pose, wpi::array<frc::SwerveModulePosition, 4U> states) {
  _poseEstimator.ResetPosition(pose.Rotation().Degrees(), states, pose);
  _simPoseEstimator.ResetPosition(pose.Rotation().Degrees(), states, pose);
}

void PoseHandler::AddOdometryMeasurement(
  frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states) {
  _poseEstimator.Update(angle, states);
  logger::FieldDisplay::GetInstance().SetRobotPose(GetPose());
  logger::FieldDisplay::GetInstance().DisplayPose(
    "Unclamped Robot Pose", _poseEstimator.GetEstimatedPosition());
}

void PoseHandler::AddSimOdometryMeasurement(frc::Rotation2d angle,
  wpi::array<frc::SwerveModulePosition, 4U> states, bool resetHeading, frc::Rotation2d heading) {
  _simPoseEstimator.Update(angle, states);
  if (resetHeading) {
    _simPoseEstimator.ResetRotation(heading);
  }
  logger::FieldDisplay::GetInstance().DisplayPose(
    "Sim pose", _simPoseEstimator.GetEstimatedPosition());
}

void PoseHandler::AddVisionMeasurement(
  frc::Pose2d pose, units::second_t timeStamp, wpi::array<double, 3> dev) {
  _poseEstimator.AddVisionMeasurement(pose, timeStamp, dev);
  logger::FieldDisplay::GetInstance().SetRobotPose(GetPose());
}