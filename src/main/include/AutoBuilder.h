#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"

#include <choreo/Choreo.h>

#include <functional>

#include <fmt/ranges.h>

namespace AutoBuilder{

    inline auto isRed = []() -> bool {
        return (frc::DriverStation::GetAlliance() ==
          frc::DriverStation::Alliance::kRed);
    };

    inline auto LBDep_Dep_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LBDep_Dep_Hub_Lad");

    inline auto LTrench_Fuel_LB_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LTrench_Fuel_LB_Hub_Lad");

    inline auto RTrench_Fuel_RB_Hub_Lad =
        choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("RTrench_Fuel_RB_Hub_Lad"); 

/*    namespace util{
        inline frc2::CommandPtr AutoClimb(Climb &climb){
            return frc2::cmd::Sequence(
                
            );
        }

        inline frc2::CommandPtr AutoIntake(std::optional<choreo::Trajectory<choreo::SwerveSample>> traj,
                                            Intake &intake){

        }

        inline frc2::CommandPtr AutoShoot(Shooter &shooter){
            return frc2::cmd::Sequence(

            );
        }
    }  */


    inline frc2::CommandPtr DepotAuto(Drivetrain &swerve){
        return frc2::cmd::Sequence(
            swerve.FollowPathCommand(LBDep_Dep_Hub_Lad.value()).WithTimeout(2_s),
            frc2::cmd::Print("Driving\n")
        );
    }
    
    inline frc2::CommandPtr LTrenchAuto(Drivetrain &swerve){
        return frc2::cmd::Sequence(
            swerve.FollowPathCommand(LTrench_Fuel_LB_Hub_Lad.value().GetSplit(0).value()),
            frc2::cmd::Wait(2_s),
            swerve.FollowPathCommand(LTrench_Fuel_LB_Hub_Lad.value().GetSplit(1).value())
        );
    }

    inline frc2::CommandPtr RTrenchAuto(Drivetrain &swerve){
        return frc2::cmd::Sequence(
            swerve.FollowPathCommand(RTrench_Fuel_RB_Hub_Lad.value())
        );
    } 

};