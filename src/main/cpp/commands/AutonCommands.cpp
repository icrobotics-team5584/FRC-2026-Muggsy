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
    // Intake should always be on
    SetAutonStartPos(icGeometry::MaybeFlip(frc::Pose2d{4.4_m, 7.435_m, 0_deg}, flip)),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{6.458_m, 7.313_m, 0.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.629_m, 7.189_m, -95.0_deg}, flip), 2, 60_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.799_m, 6.554_m, -95.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.799_m, 4.054_m, -95.0_deg}, flip), 0.5, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.416_m, 5.834_m, -180.0_deg}, flip), 1.7, 80_cm, 15_deg), // 5
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.856_m, 5.475_m, -180.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.603_m, 5.571_m, -220.0_deg}, flip), 0.7, 10_cm, 15_deg),
    
    // Shoot
    frc2::cmd::Wait(1.5_s),

    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.603_m, 7.428_m, 0.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.551_m, 7.435_m, 0.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.456_m, 7.071_m, -90.0_deg}, flip), 2, 80_cm, 15_deg), // 10
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{7.957_m, 5.693_m, -95.0_deg}, flip), 1.7, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{8.046_m, 4.478_m, -95.0_deg}, flip), 1.2, 50_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.764_m, 4.019_m, -185.0_deg}, flip), 0.7, 30_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.738_m, 6.060_m, -180.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.955_m, 5.782_m, -180.0_deg}, flip), 2, 80_cm, 15_deg), // 15
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.838_m, 5.899_m, -225.0_deg}, flip), 0.7, 10_cm, 15_deg),

    // Shoot
    frc2::cmd::Wait(1.5_s),

    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{2.603_m, 7.428_m, 0.0_deg}, flip), 2, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      icGeometry::MaybeFlip(frc::Pose2d{5.051_m, 7.435_m, 0.0_deg}, flip), 2, 80_cm, 15_deg)
    

    
  )
    .AlongWith(SubShooter::GetInstance().SetSpeedFromDistanceTarget(
      [] { return PoseHandler::GetInstance().GetPose().Translation().Distance(GetHubPos()); },
      [] { return false; }))
    .AlongWith(SubIntake::GetInstance().RunIntake());
  ;
}

frc2::CommandPtr AutonCommand() {
  // return cmd::IntakePass(false).AndThen(cmd::IntakePass(false));
  return cmd::TwoPassAuto(true);
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

frc2::CommandPtr SetAutonStartPos(std::function<frc::Pose2d()> pose) {
  return frc2::cmd::RunOnce([pose] {
    SubDrivebase::GetInstance().SetPose(icGeometry::GetFieldRelativePose(pose()));
  });
} 

}  // namespace cmd