#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"

#include <choreo/Choreo.h>
#include <choreo/trajectory/Trajectory.h>

#include <functional>

#include <fmt/ranges.h>

class RobotContainer;

namespace AutoBuilder{

    inline auto isRed = []() -> bool {
        return (frc::DriverStation::GetAlliance() ==
          frc::DriverStation::Alliance::kRed);
    };

    using Trajectory_t = choreo::Trajectory<choreo::SwerveSample>;

    inline auto LBDep_Dep_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LBDep_Dep_Hub_Lad");

    inline auto LTrench_Fuel_LB_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LTrench_Fuel_LB_Hub_Lad");

    inline auto RTrench_Fuel_RB_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("RTrench_Fuel_RB_Hub_Lad"); 
    
    inline auto RHub_Hub_LHub_LBDep =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("RHub_Hub_LHub_LBDep");


    frc2::CommandPtr DepotAuto(RobotContainer &robot);
    
    frc2::CommandPtr LTrenchAuto(RobotContainer &robot);

    frc2::CommandPtr RTrenchAuto(RobotContainer &robot);

    frc2::CommandPtr TrenchToDepotAuto(RobotContainer &robot);

    frc2::CommandPtr BuildAuto(RobotContainer &robot, Trajectory_t trajectory);

};