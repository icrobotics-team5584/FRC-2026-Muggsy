#include "commands/AutonCommands.h"

#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

#include "utilities/FieldConstants.h"
#include "utilities/ICGeometry.h"
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"

#include <frc/DriverStation.h>

#include <math.h>
namespace cmd {

frc2::CommandPtr IntakePass(bool flip) {
  return frc2::cmd::Sequence(
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{9_m, 1_m, 100_deg}, flip), 1.0, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.27_m, 3.2_m, 100_deg}, flip), 1.0, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.5_m, 2.6_m, 200_deg}, flip), 1.0, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{3.5_m, 2.25_m, 200_deg}, flip), 0.5, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.5_m, 3.5_m, 200_deg}, flip), 1.0, 80_cm, 15_deg),
    SubIndexer::GetInstance().SpinIndexer().WithTimeout(1.5_s),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.4_m, 0.5_m, 0_deg}, flip), 1.0, 80_cm, 15_deg))
    .AlongWith(SubShooter::GetInstance().SetSpeedFromDistanceTarget(
      [] { return PoseHandler::GetInstance().GetPose().Translation().Distance(GetHubPos()); },
      [] { return false; }))
    .AlongWith(SubIntake::GetInstance().RunIntake());
}

frc2::CommandPtr DepotAuton(bool flip) {
  return frc2::cmd::Sequence(
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{4.954_m, 7.623_m, 270.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.713_m, 7.638_m, 270.0_deg}, flip), 0.2, 150_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.370_m, 5.248_m, 270.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.087_m, 5.289_m, 180.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.208_m, 5.923_m, 140.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubIndexer::GetInstance().SpinIndexer().WithTimeout(1.5_s),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{1.088_m, 6.013_m, -180.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{0.287_m, 5.949_m, -180.0_deg}, flip), 0.2, 80_cm, 15_deg),
    frc2::cmd::Wait(1.5_s), SubIndexer::GetInstance().StopIndexer(),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{frc::Pose2d{3.078_m, 7.508_m, 0.0_deg}}, flip), 0.1, 80_cm,
      15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{4.954_m, 7.623_m, 270.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{4.954_m, 7.623_m, 270.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.713_m, 7.638_m, 270.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.370_m, 5.248_m, 270.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.087_m, 5.289_m, 180.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.283_m, 4.552_m, 140.0_deg}, flip), 0.2, 80_cm, 15_deg))
    .AlongWith(SubShooter::GetInstance().SetSpeedFromDistanceTarget(
      [] { return PoseHandler::GetInstance().GetPose().Translation().Distance(GetHubPos()); },
      [] { return false; }))
    .AlongWith(SubIntake::GetInstance().RunIntake());
  ;
}
frc2::CommandPtr TwoPassAuto(bool flip) {
  return frc2::cmd::Sequence(
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{4.954_m, 7.623_m, 310.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{6.629_m, 7.086_m, -60.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.113_m, 5.704_m, -80.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.253_m, 5.138_m, -120.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.392_m, 5.661_m, -215.0_deg}, flip), 0.2, 5_cm, 15_deg), // 5
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{3.196_m, 7.131_m, -335.0_deg}, flip), 0.2, 15_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{3.860_m, 7.567_m, -10.0_deg}, flip), 0.2, 15_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{4.954_m, 7.623_m, 310.0_deg}, flip), 0.2, 15_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{6.629_m, 7.086_m, -60.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.948_m, 5.869_m, -75.0_deg}, flip), 0.2, 80_cm, 15_deg), // 10
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.270_m, 4.935_m, -90.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.265_m, 4.190_m, -90.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.233_m, 4.016_m, -480.0_deg}, flip), 0.2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.858_m, 4.016_m, -550.0_deg}, flip), 0.2, 50_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.788_m, 5.477_m, -200.0_deg}, flip), 0.2, 50_cm, 15_deg), // 15
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.392_m, 5.661_m, -215.0_deg}, flip), 0.2, 5_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{3.196_m, 7.131_m, -335.0_deg}, flip), 0.2, 15_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{4.954_m, 7.623_m, 310.0_deg}, flip), 0.2, 15_cm, 15_deg)

  )
    .AlongWith(SubShooter::GetInstance().SetSpeedFromDistanceTarget(
      [] { return PoseHandler::GetInstance().GetPose().Translation().Distance(GetHubPos()); },
      [] { return false; }))
    .AlongWith(SubIntake::GetInstance().RunIntake());
  ;
}

frc2::CommandPtr AutonCommand() {
  // return cmd::IntakePass(false).AndThen(cmd::IntakePass(false));
  return cmd::TwoPassAuto(false);
}

frc::Translation2d GetHubPos() {
  frc::Translation3d target = fieldpos::HUB_POSITION;
  if (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) ==
      frc::DriverStation::kRed) {
    target = icGeometry::XTranslationFlip(target);
  }
  logger::FieldDisplay::GetInstance().DisplayPose(
    "AimAtHub/target", frc::Pose2d{target.ToTranslation2d(), 0_deg});
  return target.ToTranslation2d();
}
}  // namespace cmd