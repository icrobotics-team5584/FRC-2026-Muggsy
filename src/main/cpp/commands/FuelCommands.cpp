#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubHood.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubDrivebase.h"

#include "utilities/ShotPlanner.h"
#include "utilities/PoseHandler.h"


namespace cmd {
    frc2::CommandPtr IntakeSequence() {
        return SubDeploy::GetInstance().ExtendToLerp(1.0)
            .AndThen(SubIntake::GetInstance().RunIntake());
    }

    frc2::CommandPtr ReverseIntakeSequence() {
        return SubDeploy::GetInstance().ExtendToLerp(1.0)
            .AndThen(SubIntake::GetInstance().RunReverseIntake());
    }

    frc2::CommandPtr StationaryShootAt(frc::Translation2d target) {
    }

    bool IsReadyToShoot() {
      return SubHood::GetInstance().IsAtTarget() && SubShooter::GetInstance().IsReadyToShoot() &&
             ShotPlanner::CalculateShotTarget(PoseHandler::GetInstance().GetPose()).shouldShoot;
    }
}