// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICCamera.h"

#include <frc/Filesystem.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/VisionSystemSim.h>
#include <wpi/interpolating_map.h>

class SubVision : public frc2::SubsystemBase {
 public:
  SubVision();
  static SubVision& GetInstance() {
    static SubVision inst;
    return inst;
  }

  void Periodic() override;
  void SimulationPeriodic() override;
  void UpdateVision();

  std::map<std::string, std::optional<photon::EstimatedRobotPose>> GetPose();
  std::optional<units::meter_t> GetAvgDistanceFromCamera(const photon::EstimatedRobotPose& est);
  std::optional<int> GetClosestTag(frc::Pose2d currentPose);
  std::optional<frc::Pose2d> GetAprilTagPose(int id);

  double GetDev(units::meter_t distance);
  bool IsEstimateUsable(const photon::EstimatedRobotPose& est);

 private:
  // Create field layout
  std::string _tagMapFilePath = frc::filesystem::GetDeployDirectory() + "/2026-rebuilt.json";
  frc::AprilTagFieldLayout _tagMap{_tagMapFilePath};

  frc::Transform3d _shooterBotToCam{
    {-295.779_mm, 252.927_mm, 320.249_mm}, {0_deg, -30_deg, 90_deg}};

  static constexpr std::string_view SHOOTER_CAM_NAME = "shooter";
  ICCamera _shooterCam{SHOOTER_CAM_NAME, _shooterBotToCam, _tagMap};

  std::vector<ICCamera*> _camList{
    &_shooterCam,
  };

  photon::VisionSystemSim _visionSim{"VisionSim"};

  // Deviation table for further distances from tag
  wpi::interpolating_map<units::meter_t, double> _devTable;
};

// Link to photon vision
// http://10.55.84.11:5800
