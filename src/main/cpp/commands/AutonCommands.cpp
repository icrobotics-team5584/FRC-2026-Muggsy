#include "commands/AutonCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

#include "commands/FuelCommands.h"

#include "utilities/FieldConstants.h"
#include "utilities/ICGeometry.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc/DriverStation.h>

#include <math.h>
namespace cmd {
frc2::CommandPtr TwoPassAuto(bool flip) {
  return frc2::cmd::Sequence(
    SubDrivebase::GetInstance().ZeroRotation([] { return 0_deg; }),

    frc2::cmd::Parallel(SubDeploy::GetInstance().Zero(1_s),
      SubHood::GetInstance().RunZeroingSequence().AndThen(SubHood::GetInstance().HoodToStowAngle().WithTimeout(1_ms)),
      frc2::cmd::Sequence(SetAutonStartPos(icGeometry::MaybeFlip(
                            frc::Pose2d{4.4_m, 7.435_m, 0_deg}, flip)),  // start position
        SubDrivebase::GetInstance().DriveToPose(
          icGeometry::MaybeFlip(frc::Pose2d{6.458_m, 7.313_m, 0.0_deg}, flip), 2, 80_cm,
          15_deg),  // first pose
        SubDrivebase::GetInstance().DriveToPose(
          icGeometry::MaybeFlip(frc::Pose2d{7.629_m, 7.189_m, -95.0_deg}, flip), 2, 60_cm,
          15_deg))),

    SubIntake::GetInstance().RunIntake().WithDeadline(frc2::cmd::Sequence(
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{7.799_m, 6.554_m, -95.0_deg}, flip), 2, 80_cm, 15_deg),
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{7.799_m, 4.054_m, -95.0_deg}, flip), 0.5, 80_cm, 15_deg),
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{5.6_m, 5.7_m, -135.0_deg}, flip), 1.7, 40_cm, 15_deg))),

    SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3.5_mps, 0_mps, 0_tps},
      icGeometry::MaybeTranslationFlip(frc::Translation2d{3.5_m, 5.8_m}, flip)),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.603_m, 5.571_m, -220.0_deg}, flip), 0.7, 10_cm, 15_deg),

    // Shoot
    cmd::StationaryShoot().WithTimeout(3_s), cmd::ResetAfterShoot(),

    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.603_m, 7.428_m, 0.0_deg}, flip), 2, 80_cm, 15_deg),

    SubIntake::GetInstance().RunIntake().WithDeadline(frc2::cmd::Sequence(
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{5.551_m, 7.435_m, 0.0_deg}, flip), 2, 80_cm, 15_deg),
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{7.456_m, 7.071_m, -90.0_deg}, flip), 2, 80_cm, 15_deg),
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{7.957_m, 5.693_m, -95.0_deg}, flip), 1.7, 80_cm, 15_deg),
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{8.046_m, 4.478_m, -95.0_deg}, flip), 0.8, 50_cm, 15_deg),
      SubDrivebase::GetInstance()
        .DriveToPose(icGeometry::MaybeFlip(frc::Pose2d{5.764_m, 4.019_m, -185.0_deg}, flip), 0.5,
          30_cm, 15_deg)
        .WithTimeout(4_s),
      SubDrivebase::GetInstance().DriveToPose(
        icGeometry::MaybeFlip(frc::Pose2d{5.764_m, 5.7_m, -135.0_deg}, flip), 1.7, 40_cm, 15_deg))),

    SubDrivebase::GetInstance().DriveOverBump(frc::ChassisSpeeds{-3.5_mps, 0_mps, 0_tps},
      icGeometry::MaybeTranslationFlip(frc::Translation2d{3.5_m, 5.8_m}, flip)),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.838_m, 5.899_m, -225.0_deg}, flip), 2, 10_cm, 15_deg),

    // Shoot
    cmd::StationaryShoot())
    .FinallyDo([] { frc2::CommandScheduler::GetInstance().Schedule(cmd::ResetAfterShoot()); });
}

frc2::CommandPtr AutonCommand() {
  return cmd::TwoPassAuto(true);
}

frc2::CommandPtr SetAutonStartPos(const std::function<frc::Pose2d()>& pose) {
  return frc2::cmd::RunOnce(
    [pose] { SubDrivebase::GetInstance().SetPose(icGeometry::GetFieldRelativePose(pose())); });
}
}  // namespace cmd