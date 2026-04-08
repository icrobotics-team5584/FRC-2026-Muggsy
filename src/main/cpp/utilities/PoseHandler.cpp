#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"
#include "utilities/ICgeometry.h"

PoseHandler::PoseHandler() {
}

frc::Pose2d PoseHandler::GetPose() {
    auto pose = _poseEstimator.GetEstimatedPosition();
    return {
        std::clamp(pose.X(), DrivebaseConfig::CENTRE_TO_BUMPER_EDGE, ICgeometry::FIELD_LENGTH - DrivebaseConfig::CENTRE_TO_BUMPER_EDGE),
        std::clamp(pose.Y(), DrivebaseConfig::CENTRE_TO_BUMPER_EDGE, ICgeometry::FIELD_WIDTH - DrivebaseConfig::CENTRE_TO_BUMPER_EDGE),
        pose.Rotation()
    };
}

frc::Pose2d PoseHandler::GetSimPose() {
    return _simPoseEstimator.GetEstimatedPosition();
}

void PoseHandler::SetPose(frc::Pose2d pose, wpi::array<frc::SwerveModulePosition, 4U> states) {
    _poseEstimator.ResetPosition(pose.Rotation().Degrees(), states, pose);
    _simPoseEstimator.ResetPosition(pose.Rotation().Degrees(), states, pose);
}

void PoseHandler::AddOdometryMeasurement(frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states) {
    _poseEstimator.Update(angle, states);
    Logger::FieldDisplay::GetInstance().SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void PoseHandler::AddSimOdometryMeasurement(frc::Rotation2d angle, wpi::array<frc::SwerveModulePosition, 4U> states, bool resetHeading, frc::Rotation2d heading) {
    _simPoseEstimator.Update(angle, states);
    if (resetHeading) { _simPoseEstimator.ResetRotation(heading); }
    Logger::FieldDisplay::GetInstance().DisplayPose("Sim pose", _simPoseEstimator.GetEstimatedPosition());
}

void PoseHandler::AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev) {
    _poseEstimator.AddVisionMeasurement(pose, timeStamp, dev);
    Logger::FieldDisplay::GetInstance().SetRobotPose(_poseEstimator.GetEstimatedPosition());
}