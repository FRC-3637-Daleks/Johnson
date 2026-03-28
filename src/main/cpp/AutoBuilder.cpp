#include "AutoBuilder.h"
#include "RobotContainer.h"


namespace AutoBuilder{

   namespace util{
        frc2::CommandPtr AutoIntake(RobotContainer &robot){
            return frc2::cmd::Print("STARTING INTAKE").AlongWith(robot.m_intake.IntakeFuel())
                                                      .FinallyDo([]{fmt::println("ENDING INTAKE");});
        }

        frc2::CommandPtr AutoShoot(RobotContainer &robot){
            return  frc2::cmd::Print("***********Shooting***********")
                .AlongWith(
                    robot.AutoAim()
                ).WithDeadline(frc2::cmd::Wait(0.5_s)
                    .AndThen(
                        frc2::cmd::WaitUntil(
                            [&robot] {return m_container.isReadyToFire();}
                        ).AlongWith(robot.m_feederBottom.setRPMEnd(-25_tps).WithTimeout(0.5_s))
                    ).WithTimeout(2_s)
                    .AndThen(
                        robot.m_feederBottom.setRPMEnd(20_tps)
                        .RaceWith(robot.m_intake.ScoreFuel(1_s).Repeatedly().WithTimeout(3_s))
                    )
                ).AndThen(robot.m_shooter.RetractHood())
            ;
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
        return BuildDepotAuto(robot, trajectory);
    }
    
    frc2::CommandPtr BuildAuto(RobotContainer &robot, Trajectory_t trajectory) {
        auto& swerve = robot.m_swerve;
        if (trajectory.splits.size() == 2) {
            return frc2::cmd::Sequence(
                BuildAuto(robot, trajectory.GetSplit(0).value()),
                BuildAuto(robot, trajectory.GetSplit(1).value())
            );
        } else {  // 1 or fewer splits
            return frc2::cmd::Sequence(
                robot.m_intake.Extend().WithTimeout(0.8_s),
                util::AutoIntake(robot).RaceWith(swerve.FollowPathCommand(trajectory, PathFollower::EndConditionType::NEAR_DEST)),
                util::AutoShoot(robot)
            );
        }
    }

    frc2::CommandPtr BuildSingleAuto(RobotContainer &robot, Trajectory_t trajectory) {
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(robot.m_intake.SeedArm())
            .AndThen(BuildAuto(robot, trajectory));
    }

    frc2::CommandPtr BuildRepeatedAuto(RobotContainer &robot, Trajectory_t trajectory) {
        // path must self-cycle to be willing to repeat
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(robot.m_intake.SeedArm())
            .AndThen(BuildAuto(robot, trajectory).Repeatedly());
    }

    frc2::CommandPtr BuildDepotAuto(RobotContainer &robot, Trajectory_t trajectory){
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(BuildAuto(robot, trajectory.GetSplit(0).value()))
            .AndThen(robot.m_climb.Deploy())
            .AndThen(robot.m_swerve.FollowPathCommand(trajectory.GetSplit(1).value()))
            .AndThen(robot.m_climb.LiftBot());
    }
};