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

    inline auto LBDep_Dep_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LBDep_Dep_Hub_Lad");

    inline auto LTrench_Fuel_LB_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LTrench_Fuel_LB_Hub_Lad");

    inline frc2::CommandPtr DepotAuto(Drivetrain &swerve){
        return frc2::cmd::Sequence(
            swerve.FollowPathCommand(LBDep_Dep_Hub_Lad.value())
        );
    }

    inline frc2::CommandPtr TrenchAuto(Drivetrain &swerve){
        return frc2::cmd::Sequence(
            swerve.FollowPathCommand(LTrench_Fuel_LB_Hub_Lad.value().GetSplit(0).value()),
            frc2::cmd::Wait(2_s),
            swerve.FollowPathCommand(LTrench_Fuel_LB_Hub_Lad.value().GetSplit(1).value())
        );
    }
    

};