#include "AutoBuilder.h"
#include "RobotContainer.h"

namespace AutoConstants {
    constexpr auto kHubBlue = frc::Translation2d{4.65_m, 4.06_m};
    constexpr auto kHubRed = frc::Translation2d{11.95_m, 4.06_m};
}

namespace AutoBuilder{

   namespace util{
        frc2::CommandPtr AutoIntake(RobotContainer &robot){
            return frc2::cmd::Print("STARTING INTAKE").AlongWith(robot.m_intake.IntakeFuel())
                                                      .FinallyDo([]{fmt::println("ENDING INTAKE");});
        }

        frc2::CommandPtr AutoAim(RobotContainer &robot) {
            return robot.m_swerve.ZTargetCommand(
                [] {return 0_mps;},
                [] {return 0_mps;},
                [&robot] {
                    if (robot.IsRed()) {
                        return frc::Pose2d{AutoConstants::kHubRed, 0_deg};
                    } else {
                        return frc::Pose2d{AutoConstants::kHubBlue, 0_deg};
                    }
                }
            );
        }

        frc2::CommandPtr AutoShoot(RobotContainer &robot){
            return  frc2::cmd::Print("***********Shooting***********")
                .AlongWith(
                    robot.m_shooter.AimFromTrench()
                    .AlongWith(robot.TopFeederShooting())
                    .AlongWith(AutoAim(robot))
                ).WithDeadline(frc2::cmd::WaitUntil(
                        [&robot] {return robot.m_shooter.readyToFire();}
                    ).WithTimeout(2_s)
                    .AndThen(
                        robot.m_feederBottom.setRPMEnd(20_tps)
                        .RaceWith(robot.m_intake.ScoreFuel(1_s).Repeatedly().WithTimeout(2_s))
                    )
                ).AndThen(robot.m_shooter.RetractHood());
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
        return BuildSingleAuto(robot, trajectory);
    }
    
    frc2::CommandPtr BuildAuto(RobotContainer &robot, Trajectory_t trajectory) {
        auto& swerve = robot.m_swerve;
        if (trajectory.splits.size() == 3) {
            return frc2::cmd::Sequence(
                BuildAuto(robot, trajectory.GetSplit(0).value()),
                BuildAuto(robot, trajectory.GetSplit(1).value()),
                swerve.FollowPathCommand(trajectory.GetSplit(2).value())
            );
        } else if (trajectory.splits.size() == 2) {
            return frc2::cmd::Sequence(
                BuildAuto(robot, trajectory.GetSplit(0).value()),
                swerve.FollowPathCommand(trajectory.GetSplit(1).value())
            );
        } else {  // 1 or fewer splits
            return frc2::cmd::Sequence(
                robot.m_intake.Extend(),
                util::AutoIntake(robot).RaceWith(swerve.FollowPathCommand(trajectory)),
                util::AutoShoot(robot)
            );
        }
    }

    frc2::CommandPtr BuildSingleAuto(RobotContainer &robot, Trajectory_t trajectory) {
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(BuildAuto(robot, trajectory));
    }

    frc2::CommandPtr BuildRepeatedAuto(RobotContainer &robot, Trajectory_t trajectory) {
        // path must self-cycle to be willing to repeat
        const auto init_loc = trajectory.GetInitialPose().value().Translation();
        const auto final_loc = trajectory.GetFinalPose().value().Translation();
        if((init_loc - final_loc).Norm() < 0.1_m) {
            return util::ResetStart(robot.m_swerve, trajectory)
                .AndThen(BuildAuto(robot, trajectory).Repeatedly());
        } else {
            return util::ResetStart(robot.m_swerve, trajectory)
                .AndThen(BuildAuto(robot, trajectory));
        }
    }

};