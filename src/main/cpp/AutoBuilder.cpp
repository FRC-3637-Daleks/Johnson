#include "AutoBuilder.h"
#include "RobotContainer.h"

namespace AutoBuilder{


   namespace util{
        frc2::CommandPtr AutoClimb(RobotContainer &robot){
            return frc2::cmd::Print("***********Climbing***********").WithTimeout(2_s);
        }

        frc2::CommandPtr AutoIntake(RobotContainer &robot){
            return
                frc2::cmd::StartEnd([]{
                    fmt::println("STARTING INTAKE");
                },[]{
                    fmt::println("ENDING INTAKE");
                });
            
        }

        frc2::CommandPtr AutoShoot(RobotContainer &robot){
             return  frc2::cmd::Print("***********Shooting***********").WithTimeout(3_s);
            
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

    frc2::CommandPtr LTrenchCycleAuto(RobotContainer &robot){
        auto trajectory = LTrench_Fuel_LTrench.value();
        auto trajectory2 = LTrench_Fuel_LTrench_2.value();
      
        return BuildAuto(robot, trajectory2).AndThen(BuildAuto(robot,trajectory))
                                            .AndThen(BuildAuto(robot,trajectory)); 
    }

    frc2::CommandPtr RTrenchCycleAuto(RobotContainer &robot){
        auto trajectory = RTrench_Fuel_RTrench.value();
        auto trajectory2 = RTrench_Fuel_RTrench_2.value();

        return BuildAuto(robot, trajectory2);
    } 

    frc2::CommandPtr BuildAuto(RobotContainer &robot, Trajectory_t trajectory){
        auto& swerve = robot.m_swerve;
          return frc2::cmd::Sequence(
            util::AutoIntake(robot).RaceWith(swerve.FollowPathCommand(trajectory.GetSplit(0).value())),
            util::AutoShoot(robot),
            swerve.FollowPathCommand(trajectory.GetSplit(1).value()) 
        );
    }

};