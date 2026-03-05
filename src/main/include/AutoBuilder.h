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

//Depot Auto
    inline auto LBDep_Dep_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LBDep_Dep_Hub_Lad");


frc2::CommandPtr DepotAuto(RobotContainer &robot);
frc2::CommandPtr BuildAuto(RobotContainer &robot, Trajectory_t trajectory);
frc2::CommandPtr BuildSingleAuto(RobotContainer &robot, Trajectory_t trajectory);
frc2::CommandPtr BuildRepeatedAuto(RobotContainer &robot, Trajectory_t trajectory);

};