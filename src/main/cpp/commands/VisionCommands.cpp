// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommands.h"

#include "subsystems/SubVision.h"

#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc/RobotBase.h>

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr AddVisionMeasurement() {
  Logger::Log("Vision/Timestamp limit", 0.2_s);
  return Run(
    [] {
      if (!Logger::Tune("Vision/Add pose measurement", frc::RobotBase::IsReal())) {
        return;
      }

      auto poses = SubVision::GetInstance().GetPose();

      std::vector<ProcessedPose> processedResults = {};

      for (auto [name, pose] : poses) {
        Logger::FieldDisplay::GetInstance().DisplayPose("Vision/" + name + "/Est pose", {});
        Logger::Log("Vision/" + name + "/Est pose usable", false);
        Logger::Log("Vision/" + name + "/Timestamp difference", 0_s);
        Logger::Log("Vision/" + name + "/Valid timestamp", false);

        // If the pose has value
        Logger::Log("Vision/" + name + "/Has value", pose.has_value());
        if (!pose.has_value()) {
          continue;
        }

        // If the pose is usable, or the timestamp is recent
        bool poseUsable = SubVision::GetInstance().IsEstimateUsable(pose.value());
        bool timestampValid = frc::Timer::GetFPGATimestamp() - pose.value().timestamp < 0.2_s;
        Logger::Log("Vision/" + name + "/Est pose usable", poseUsable);
        Logger::Log("Vision/" + name + "/Timestamp difference",
          frc::Timer::GetFPGATimestamp() - pose.value().timestamp);
        Logger::Log("Vision/" + name + "/Valid timestamp", timestampValid);
        if (!poseUsable || !timestampValid) {
          continue;
        }

        frc::Pose2d botPose;
        if (name == SubVision::GetInstance().SHOOTER_CAM_NAME) {
          botPose = pose.value().estimatedPose.ToPose2d();
        }

        Logger::FieldDisplay::GetInstance().DisplayPose("Vision/" + name + "/Est pose", botPose);

        // Distance between tag and camera
        units::length::meter_t distance =
          SubVision::GetInstance().GetAvgDistanceFromCamera(pose.value());

        processedResults.push_back({name, botPose, pose.value().timestamp, distance});
      }

      // Compare results, prioritize static camera with cloest distance
      if (processedResults.empty()) {
        return;
      }

      ProcessedPose bestResult = processedResults.front();

      for (auto result : processedResults) {
        // Compare distance
        if (result.distance < bestResult.distance) {
          bestResult = result;
        }
      }

      // Add result to PoseHandler
      double dev = SubVision::GetInstance().GetDev(bestResult.distance);
      PoseHandler::GetInstance().AddVisionMeasurement(
        bestResult.pose, bestResult.timestamp, {dev, dev, 0.9});
    },
    {&SubVision::GetInstance()})
    .IgnoringDisable(true);
}

}  // namespace cmd