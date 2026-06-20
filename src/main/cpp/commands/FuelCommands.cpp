#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubIntake.h"


namespace cmd {
    frc2::CommandPtr IntakeSequence() {
        return SubDeploy::GetInstance().ExtendToLerp(1.0)
            .AndThen(SubIntake::GetInstance().RunIntake());
    }

    frc2::CommandPtr ReverseIntakeSequence() {
        return SubDeploy::GetInstance().ExtendToLerp(1.0)
            .AndThen(SubIntake::GetInstance().RunReverseIntake());
    }

    bool IsReadyToShoot() {
    }
}