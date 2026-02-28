// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/math.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <iostream>

#include <choreo/Choreo.h>

namespace AutoConstants {

constexpr auto kMaxSpeed = 4.5_mps;
constexpr auto kMaxAcceleration = 6_mps_sq;
constexpr auto kPathMaxAcceleration = 4_mps_sq;
// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = PracticeModuleConstants::kPhysicalMaxSpeed / 3; //
// left out as these are repeat values constexpr auto kMaxAcceleration =
// 10_fps_sq;
constexpr auto kMaxAngularSpeed = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s_sq;

constexpr frc::Pose2d desiredPose{0_m, 0_m, 0_deg};
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kStrafeDeadband = 0.08;
constexpr double kRotDeadband = .16;
constexpr double kClimbDeadband = 0.08;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr auto kMaxTeleopSpeed = 15.7_fps;
constexpr auto kMaxTeleopTurnSpeed = 2.5 * std::numbers::pi * 1_rad_per_s;

constexpr double kSlowModeFactor = 0.3;
} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 57_ft + 6.875_in;
constexpr auto field_width = 26_ft + 5_in;
constexpr auto mid_line = field_length / 2;

} // namespace FieldConstants

RobotContainer::RobotContainer()
  : m_swerveController(OperatorConstants::kSwerveControllerPort) {

  fmt::println("made it to robot container");
  // Initialize all of your commands and subsystems here
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Log Match Info
  std::string matchType =
    frc::DriverStation::GetMatchType() ==
    frc::DriverStation::MatchType::kNone
    ? ""
    : (frc::DriverStation::GetMatchType() ==
      frc::DriverStation::MatchType::kElimination
      ? "Elimination"
      : (frc::DriverStation::GetMatchType() ==
        frc::DriverStation::MatchType::kQualification
        ? "Qualification"
        : "Practice"));

  std::string alliance =
    (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed
      ? "Red"
      : "Blue");

  frc::DataLogManager::Log(
    fmt::format("Playing {} Match {} at {} as {} alliance\n", matchType,
      frc::DriverStation::GetMatchNumber(),
      frc::DriverStation::GetEventName(), alliance));

  // Configure the button bindings
  ConfigureBindings();

  // Configure Dashboard
  ConfigureDashboard();

  // Configure Auton.
  ConfigureAuto();

  // Configure routines which one periodically and indefinitely
  ConfigureContinuous();

  frc::DataLogManager::Log(fmt::format("Finished initializing robot."));
}

void RobotContainer::ConfigureBindings() {
    m_swerve.SetDefaultCommand(m_swerve.CustomSwerveCommand(
      [this] { return m_oi.fwd(); }, [this] { return m_oi.strafe(); },
      [this] { return m_oi.rot(); }));

  // auto slow = m_swerve.CustomSwerveCommand(
  //     [this] { return m_oi.fwd() * OperatorConstants::kSlowModeFactor; }, 
  //     [this] { return m_oi.strafe() * OperatorConstants::kSlowModeFactor; },
  //     [this] { return m_oi.rot() * OperatorConstants::kSlowModeFactor; });

  m_oi.ZeroHeadingTrigger.OnTrue(m_swerve.RunOnce([this] {
    if (IsRed())
      m_swerve.ResetControlHeading(0.5_tr);
    else
      m_swerve.ResetControlHeading();
  }));

  // m_swerveController.Button(3).OnTrue(m_shooter.SetHoodPosition(25));
  // m_swerveController.Button(4).OnTrue(m_shooter.SetHoodPosition(70));

  // m_swerveController.Button(5).OnTrue(m_feederTop.setRPM(100));
  // m_swerveController.Button(6).OnTrue(m_feederTop.setRPM(0));

  // m_swerveController.Button(7).OnTrue(m_intake.Extend());
  // m_swerveController.Button(8).OnTrue(m_intake.Retract());

  // m_swerveController.Button(9).OnTrue(m_intake.IntakeFuel());
  // m_swerveController.Button(10).OnTrue(m_intake.OutakeFuel());

  // m_swerveController.POVDown().WhileTrue(
  //   m_swerve.DriveToPoseIndefinitelyCommand(AutoConstants::desiredPose));
  // try {
  //   if (auto traj = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Square"))
  //     m_swerveController.Button(11).WhileTrue(
  //       m_swerve.FollowPathCommand(traj.value()));
  //   else
  //     m_swerveController.Button(11).WhileTrue(frc2::cmd::None());
  // } catch(...) {
  // }

  //Default Command
  //m_intake.SetDefaultCommand(m_intake.IntakeFuel());
  //m_shooter.SetDefaultCommand(m_shooter.SetHoodPositionMin());

  //Driver Controller
  m_oi.RetractHoldArm.WhileTrue(m_intake.Retract());
  //AutoAim
  m_oi.HUBAim.OnTrue(m_shooter.SetFlywheelSpeedAndHoodPosParallel(67_rad_per_s, 67)
                      .AlongWith(m_feederTop.setRPM(10_tps))); //TODO: Change placeholders
  m_oi.TowerAim.OnTrue(m_shooter.SetFlywheelSpeedAndHoodPosParallel(67_rad_per_s, 67)
                      .AlongWith(m_feederTop.setRPM(10_tps))); //TODO: Change placeholders
  m_oi.BottomFeeder.OnTrue(m_feederBottom.ManuallySetMotor(m_oi.getPilotRT));
  m_oi.OutTake.WhileTrue(m_feederBottom.setRPM(-10_tps).AlongWith(m_intake.OutakeFuel()));
  m_oi.LiftArm.WhileTrue(m_intake.Lift());
  // m_oi.ClimbUp.OnTrue(m_climb.Deploy());
  // m_oi.ClimbLift.OnTrue(m_climb.LiftBot());
  // m_oi.ClimbDown.OnTrue(m_climb.Retract());


  //Co-Pilot Commands
  m_oi.ArmDownAndIntake.OnTrue(m_intake.IntakeFuel());
  m_oi.ArmDown.OnTrue(m_intake.Extend());
  m_oi.ArmRetract.OnTrue(m_intake.Retract());
  m_oi.ArmLifted.OnTrue(m_intake.Lift());
  m_oi.ArmIntakeManual.OnTrue(m_intake.ManuallyControlArm(m_oi.getLeftY));
  // m_oi.ClimbUpManual.OnTrue(m_climb.Deploy());
  // m_oi.ClimbDownManual.WhileTrue(m_climb.LiftBot());
  m_oi.RStickFeederAndIntake.WhileTrue(m_feederBottom.ManuallySetMotor(m_oi.getRightY).AlongWith(
    frc2::cmd::Either(m_intake.BlindExtend(),
                      m_intake.BlindExtend(),
                      [this] {return m_oi.m_swerveController.GetRightX();}
  )));
  m_oi.TopFeederInManual.OnTrue(m_feederTop.setRPM(10_tps));
  m_oi.TopFeederOutManual.OnTrue(m_feederTop.setRPM(-10_tps));
  m_oi.HoodRaise.OnTrue(m_shooter.SetHoodPositionRelative(10));
  m_oi.HoodLower.OnTrue(m_shooter.SetHoodPositionRelative(-10));
  // m_oi.HoodRaise.OnTrue(m_shooter.SetHoodPosition(+1));
  // m_oi.HoodLower.OnTrue(m_shooter.SetHoodPosition(-1));
  //RT + class state variable for right trigger shooter stuff
  m_oi.PitReset.OnTrue(m_shooter.SetFlywheelSpeedAndHoodPosParallel(0_rad_per_s, 0));


}

void RobotContainer::ConfigureDashboard() {
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
}

void RobotContainer::ConfigureAuto() {}

void RobotContainer::ConfigureContinuous() {
  // These commands are for transmitting data across subsystems

  // FMS info to ROS
  frc2::CommandScheduler::GetInstance().Schedule(
    frc2::cmd::Run([this] { m_ros.CheckFMS(); }).IgnoringDisable(true));

  // Odom to ROS
  frc2::CommandScheduler::GetInstance().Schedule(
    frc2::cmd::Run([this] {
      m_ros.PubOdom(m_swerve.GetOdomPose(), m_swerve.GetChassisSpeed(),
        m_swerve.GetOdomTimestamp());
      }).IgnoringDisable(true));

  // trigger active when robot is stopped for half a second
  // though this shouldnt be necessary with well timestamped camera frames
  // we dont want any teleportation during high speed motion
  auto robot_still = frc2::Trigger{[this] {return m_swerve.IsStopped();}}.Debounce(0.5_s);

  // ROS to swerve
  robot_still.WhileTrue(
    frc2::cmd::StartRun(
      // toss old values
      [this] {m_ros.GetNewMapToOdom();},
      [this] {
        // this allows other sources to override this if it dies on an erroneous state
        if (auto map_to_odom = m_ros.GetNewMapToOdom())
          m_swerve.SetMapToOdom(map_to_odom.value());
      }
    ).IgnoringDisable(true));

  if constexpr (frc::RobotBase::IsSimulation()) {
    frc2::CommandScheduler::GetInstance().Schedule(
      frc2::cmd::Run([this] {
        m_ros.PubSim(m_swerve.GetSimulatedGroundTruth());
        }).IgnoringDisable(true));
  }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}

bool RobotContainer::IsRed() {
  m_isRed = (frc::DriverStation::GetAlliance() ==
    frc::DriverStation::Alliance::kRed);

  return m_isRed;
}