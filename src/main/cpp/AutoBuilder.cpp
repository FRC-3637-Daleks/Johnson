#include "AutoBuilder.h"
#include "RobotContainer.h"

namespace AutoBuilder{


   namespace util{
        frc2::CommandPtr AutoClimb(RobotContainer &robot){
            return frc2::cmd::Sequence(
                frc2::cmd::Print("***********Climbing***********")
            );
        }

        frc2::CommandPtr AutoIntake(RobotContainer &robot){
            return frc2::cmd::Sequence(
                frc2::cmd::Print("***********Intaking***********")
            );
        }

        frc2::CommandPtr AutoShoot(RobotContainer &robot){
            return frc2::cmd::Sequence(
                frc2::cmd::Print("***********Shooting***********")
            );
        }
    }  


    frc2::CommandPtr DepotAuto(RobotContainer &robot){
        auto trajectory = LBDep_Dep_Hub_Lad.value();
        return BuildAuto(robot, trajectory);
    }
    
    frc2::CommandPtr LTrenchAuto(RobotContainer &robot){
        auto trajectory = LTrench_Fuel_LB_Hub_Lad.value();
        return BuildAuto(robot, trajectory);
    }

    frc2::CommandPtr RTrenchAuto(RobotContainer &robot){
        auto trajectory = RTrench_Fuel_RB_Hub_Lad.value();
        return BuildAuto(robot, trajectory);
    } 

    frc2::CommandPtr TrenchToDepotAuto(RobotContainer &robot){
        auto& swerve = robot.m_swerve;
        return frc2::cmd::Sequence(
            util::AutoIntake(robot),
            swerve.FollowPathCommand(RTrench_Fuel_RB_Hub_Lad.value().GetSplit(0).value()),
            util::AutoShoot(robot),
            swerve.FollowPathCommand(RHub_Hub_LHub_LBDep.value()),
            util::AutoIntake(robot),
            swerve.FollowPathCommand(LBDep_Dep_Hub_Lad.value()),
            util::AutoClimb(robot)
        );
    }

    frc2::CommandPtr BuildAuto(RobotContainer &robot, Trajectory_t trajectory){
        auto& swerve = robot.m_swerve;
        return frc2::cmd::Sequence(
            util::AutoIntake(robot),
            swerve.FollowPathCommand(trajectory.GetSplit(0).value()),
            util::AutoShoot(robot),
            swerve.FollowPathCommand(trajectory.GetSplit(1).value()),
            util::AutoClimb(robot)
        );
    }

};