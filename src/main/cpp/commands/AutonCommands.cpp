#include "commands/AutonCommands.h"

#include "subsystems/SubDrivebase.h"

#include "utilities/ICGeometry.h"
#include "utilities/PoseHandler.h"

#include <frc/geometry/Pose2d.h>

namespace cmd {
frc2::CommandPtr IntakePass(bool flip) {
  return frc2::cmd::Sequence(
    SubDrivebase::GetInstance().DriveToPose(
      ICGeometry::MaybeFlip(frc::Pose2d{9_m, 1_m, 100_deg}, flip), 1.0, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      ICGeometry::MaybeFlip(frc::Pose2d{8.27_m, 3.2_m, 100_deg}, flip), 1.0, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      ICGeometry::MaybeFlip(frc::Pose2d{5.5_m, 2.6_m, 200_deg}, flip), 1.0, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      ICGeometry::MaybeFlip(frc::Pose2d{3.5_m, 2.25_m, 200_deg}, flip), 0.5, 80_cm, 15_deg),
    SubDrivebase::GetInstance().DriveToPose(
      ICGeometry::MaybeFlip(frc::Pose2d{2.5_m, 3.5_m, 200_deg}, flip), 1.0, 80_cm, 15_deg),
    frc2::cmd::Wait(0.5_s),
    SubDrivebase::GetInstance().DriveToPose(
      ICGeometry::MaybeFlip(frc::Pose2d{2.4_m, 0.5_m, 0_deg}, flip), 1.0, 80_cm, 15_deg));
}

frc2::CommandPtr Auton(){

}

frc2::CommandPtr IntakeAuton() {
  return cmd::IntakePass(false).AndThen(cmd::IntakePass(false));
}
}  // namespace cmd