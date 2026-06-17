// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"

#include "subsystems/SubDrivebase.h"

#include "utilities/ICGeometry.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>

#include <photon/estimation/CameraTargetRelation.h>

#include "DrivebaseConfig.h"

SubVision::SubVision() {
  // Set dev table for distance based deviance
  _devTable.insert(0_m, 0);
  _devTable.insert(0.71_m, 0.002);
  _devTable.insert(1_m, 0.006);
  _devTable.insert(1.5_m, 0.02);
  _devTable.insert(2_m, 0.068);
  _devTable.insert(3_m, 0.060);
  _devTable.insert(4_m, 0.330);
  _devTable.insert(5_m, 0.850);

  // Sim set up
  _visionSim.AddAprilTags(_tagMap);

  for (ICCamera* cam : _camList) {
    _visionSim.AddCamera(cam->GetCamSim(), cam->GetBotToCam());
  }

  // Display tags on field
  for (const photon::VisionTargetSim& target : _visionSim.GetVisionTargets()) {
    logger::FieldDisplay::GetInstance().DisplayPose(
      fmt::format("tag{}", target.GetFiducialId()), target.GetPose().ToPose2d());
  }
}

void SubVision::Periodic() {
  units::second_t loopStart = frc::GetTime();
  UpdateVision();

  logger::Log("Vision/Loop Time", (frc::GetTime() - loopStart));
}

void SubVision::UpdateVision() {
  for (ICCamera* cam : _camList) {
    cam->Update();
  }
}

void SubVision::SimulationPeriodic() {
  _visionSim.Update(PoseHandler::GetInstance().GetSimPose());
}

std::map<std::string, std::optional<photon::EstimatedRobotPose>> SubVision::GetPose() {
  std::map<std::string, std::optional<photon::EstimatedRobotPose>> poses = {};
  for (ICCamera* cam : _camList) {
    poses.insert({cam->GetCamName(), cam->GetLatestEstPose()});
  }
  return poses;
}

double SubVision::GetDev(units::meter_t distance) {
  return _devTable[distance];
}

std::optional<units::meter_t> SubVision::GetAvgDistanceFromCamera(
  const photon::EstimatedRobotPose& est) {
  units::meter_t distance = 0_m;
  if (est.targetsUsed.empty()) {
    return std::nullopt;
  }
  for (const photon::PhotonTrackedTarget& target : est.targetsUsed) {
    distance += target.GetBestCameraToTarget().Translation().Norm();
  }
  distance /= est.targetsUsed.size();

  return distance;
}

bool SubVision::IsEstimateUsable(const photon::EstimatedRobotPose& est, std::optional<units::meter_t> precomputedDistance) {
  /* If precomputed distance exists, set avg dist as precomputed distance, else call a function */
  std::optional<units::meter_t> avgDist = precomputedDistance ? precomputedDistance : GetAvgDistanceFromCamera(est);
  if (!avgDist) {
    return false;
  }

  bool targetsUsable = (avgDist.value() < 5_m) || (est.targetsUsed.size() > 1);
  frc::Pose3d pose = est.estimatedPose;
  bool estimateOnField =
    (pose.X() > drivebaseConfig::CENTRE_TO_BUMPER_EDGE &&
      pose.X() < icGeometry::FIELD_LENGTH - drivebaseConfig::CENTRE_TO_BUMPER_EDGE &&
      pose.Y() > drivebaseConfig::CENTRE_TO_BUMPER_EDGE &&
      pose.Y() < icGeometry::FIELD_WIDTH - drivebaseConfig::CENTRE_TO_BUMPER_EDGE);
  return (targetsUsable && estimateOnField);
}

std::optional<frc::Pose2d> SubVision::GetAprilTagPose(int id) {
  std::optional<frc::Pose3d> pose = _tagMap.GetTagPose(id);
  if (pose.has_value()) {
    return pose.value().ToPose2d();
  }
  return std::nullopt;
}

std::optional<int> SubVision::GetClosestTag(frc::Pose2d currentPose) {
  std::optional<int> closestTagID = std::nullopt;
  units::meter_t closestDistance = 999_m;
  std::vector<frc::AprilTag> tagList = _tagMap.GetTags();

  for (const frc::AprilTag tag : tagList) {
    int id = tag.ID;
    std::optional<frc::Pose2d> aprilTagPoseResult = GetAprilTagPose(id);
    if (!aprilTagPoseResult) {
      continue;
    }
    frc::Pose2d aprilTagPose = aprilTagPoseResult.value();

    units::meter_t distance = currentPose.Translation().Distance(aprilTagPose.Translation());
    if (distance < closestDistance) {
      closestDistance = distance;
      closestTagID = id;
    }
  }

  return closestTagID;
}