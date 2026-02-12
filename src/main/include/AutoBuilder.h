#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"

#include <choreo/Choreo.h>

#include <functional>

namespace AutoBuilder{

    inline auto isRed = []() -> bool {
        return (frc::DriverStation::GetAlliance() ==
          frc::DriverStation::Alliance::kRed);
    };

    inline auto LBDep_Dep =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LBDep_Dep");

    inline frc2::CommandPtr DepotAuto(Drivetrain &swerve){
        return frc2::cmd::Sequence(
            swerve.FollowPathCommand(LBDep_Dep.value())
        );
    }

};