#include "AutoBuilder.h"
#include "RobotContainer.h"


namespace AutoBuilder{

   namespace util{
        frc2::CommandPtr AutoIntake(RobotContainer &robot){
            constexpr auto kRedNeutral = 11.0_m;
            constexpr auto kBlueNeutral = 5.05_m;
            return frc2::cmd::Print("STARTING INTAKE")
                .AlongWith(
                    frc2::cmd::WaitUntil([&robot] {
                        const auto x = robot.m_swerve.GetPose().X();
                        return x > kBlueNeutral && x < kRedNeutral;
                    })
                    .AndThen(robot.m_intake.BlindExtend())
                    .AndThen(robot.m_intake.IntakeFuel())
                )
                .FinallyDo([]{fmt::println("ENDING INTAKE");})
            ;
        }

        frc2::CommandPtr AutoShoot(RobotContainer &robot){
            
            return  frc2::cmd::Print("***********Shooting***********")
                .AlongWith(
                    robot.AutoAim()
                ).WithDeadline(
                    frc2::cmd::WaitUntil(
                        [&robot] {return robot.isReadyToFire();}
                    ).WithTimeout(2_s)
                    .AlongWith(robot.m_feederBottom.setRPMEnd(-25_tps).WithTimeout(0.5_s))
                    .AndThen(
                        robot.m_feederBottom.setRPMEnd(40_tps)
                        .RaceWith(robot.m_intake.ScoreFuel(0.75_s).Repeatedly().WithTimeout(2.5_s)
                            .AndThen(robot.m_intake.Retract().WithTimeout(0.5_s)))  // end retracted
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

                    swerve.ResetControlHeading(swerve.GetPose().Rotation());
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
            const auto final_pose = trajectory.GetFinalPose().value_or(frc::Pose2d{});
            auto positionFunc = [final_pose] {
                return final_pose.Translation();
            };

            return frc2::cmd::Sequence(
                swerve.FollowPathCommand(std::move(trajectory), PathFollower::EndConditionType::NEAR_DEST)
                    .DeadlineFor(
                        util::AutoIntake(robot)
                        .AlongWith(robot.m_shooter.AutoAdjustFlyWheel(positionFunc, isRed))),
                util::AutoShoot(robot)
            );
        }
    }

    frc2::CommandPtr BuildSingleAuto(RobotContainer &robot, Trajectory_t trajectory) {
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(robot.m_intake.SeedArm())
            .AndThen(BuildAuto(robot, std::move(trajectory)))
            .AndThen(util::AutoShoot(robot).Repeatedly());  // just empty hopper
    }

    frc2::CommandPtr BuildRepeatedAuto(RobotContainer &robot, Trajectory_t trajectory) {
        // path must self-cycle to be willing to repeat
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(robot.m_intake.SeedArm())
            .AndThen(BuildAuto(robot, std::move(trajectory)).Repeatedly());
    }

    frc2::CommandPtr BuildDepotAuto(RobotContainer &robot, Trajectory_t trajectory){
        return util::ResetStart(robot.m_swerve, trajectory)
            .AndThen(BuildAuto(robot, trajectory.GetSplit(0).value()))
#ifndef NOCLIMB            
            .AndThen(robot.m_climb.Deploy())
            .AndThen(robot.m_swerve.FollowPathCommand(trajectory.GetSplit(1).value()))
            .AndThen(robot.m_climb.LiftBot())
#endif
        ;
    }
};