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
  logger::Log("Vision/Timestamp limit", 0.2_s);
  return Run(
    [] {
      if (!logger::Tune("Vision/Add pose measurement", frc::RobotBase::IsReal())) {
        return;
      }

      auto poses = SubVision::GetInstance().GetPose();

      std::vector<ProcessedPose> processedResults = {};

      for (auto [name, pose] : poses) {
        logger::FieldDisplay::GetInstance().DisplayPose("Vision/" + name + "/Est pose", {});
        logger::Log("Vision/" + name + "/Est pose usable", false);
        logger::Log("Vision/" + name + "/Timestamp difference", 0_s);
        logger::Log("Vision/" + name + "/Valid timestamp", false);

        // If the pose has value
        logger::Log("Vision/" + name + "/Has value", pose.has_value());
        if (!pose.has_value()) {
          continue;
        }

        // If the pose is usable, or the timestamp is recent
        bool poseUsable = SubVision::GetInstance().IsEstimateUsable(pose.value());
        bool timestampValid = frc::Timer::GetFPGATimestamp() - pose.value().timestamp < 0.2_s;
        logger::Log("Vision/" + name + "/Est pose usable", poseUsable);
        logger::Log("Vision/" + name + "/Timestamp difference",
          frc::Timer::GetFPGATimestamp() - pose.value().timestamp);
        logger::Log("Vision/" + name + "/Valid timestamp", timestampValid);
        if (!poseUsable || !timestampValid) {
          continue;
        }

        frc::Pose2d botPose;
        botPose = pose.value().estimatedPose.ToPose2d();

        logger::FieldDisplay::GetInstance().DisplayPose("Vision/" + name + "/Est pose", botPose);

        // Distance between tag and camera
        units::length::meter_t distance =
          SubVision::GetInstance().GetAvgDistanceFromCamera(pose.value());

        processedResults.push_back({name, botPose, pose.value().timestamp, distance});
      }

      // Compare results, prioritize static camera with closest distance
      if (processedResults.empty()) {
        return;
      }

      ProcessedPose bestResult = processedResults.front();

      for (const auto& result : processedResults) {
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