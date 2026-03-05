#include "AutoBuilder.h"
#include "RobotContainer.h"

namespace AutoBuilder{

   namespace util{
        frc2::CommandPtr AutoClimb(RobotContainer &robot){
            return frc2::cmd::Print("***********Climbing***********").AlongWith(robot.m_climb.Deploy())
                                                                     .AndThen(robot.m_climb.LiftBot());
        }

        frc2::CommandPtr AutoIntake(RobotContainer &robot){
            return frc2::cmd::Print("STARTING INTAKE").AlongWith(robot.m_intake.IntakeFuel())
                                                      .FinallyDo([]{fmt::println("ENDING INTAKE");});

                
        }

        frc2::CommandPtr AutoShoot(RobotContainer &robot){
             return  frc2::cmd::Print("***********Shooting***********")
                                .AlongWith(robot.m_shooter.AimFromHUB());
        }

        // If robot thinks its nowhere NEAR start pose, reset it to start pose
        frc2::CommandPtr ResetStart(Drivetrain& swerve, const Trajectory_t &t) {
            const auto expected_pose = t.GetInitialPose();
            return frc2::cmd::RunOnce([&swerve, expected_pose] {
                if (expected_pose) {
                    const auto current_pose = swerve.GetPose();
                    const auto error = current_pose - expected_pose.value();
                    if (error.Translation().Norm() > 2_m
                        || units::math::abs(error.Rotation().Degrees()) > 30_deg) {
                        swerve.ResetPose(expected_pose.value());
                    }
                }
            });
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
            util::ResetStart(swerve, trajectory),
            util::AutoIntake(robot).RaceWith(swerve.FollowPathCommand(trajectory.GetSplit(0).value())),
            util::AutoShoot(robot),
            swerve.FollowPathCommand(trajectory.GetSplit(1).value()) 
        );
    }

};