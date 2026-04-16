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
#include <frc/RobotController.h>
#include <iostream>

#include <choreo/Choreo.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>


#include <frc/Filesystem.h>
#include <filesystem>

#include <frc/Alert.h>

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

// aim tolerance measured in displacement of trajectory from ideal
constexpr auto aimingTolerance = 12_in;
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;
constexpr int kBottomFeederFollowerID = 26;

constexpr double kStrafeDeadband = 0.08;
constexpr double kRotDeadband = .16;
constexpr double kClimbDeadband = 0.08;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr auto kMaxTeleopSpeed = 15.7_fps;
constexpr auto kMaxTeleopTurnSpeed = 2.5 * std::numbers::pi * 1_rad_per_s;

constexpr double kSlowModeFactor = 0.3;

constexpr double BottomFeederScaler = 70;
constexpr double TopFeederScaler = 40;
} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 57_ft + 6.875_in;
constexpr auto field_width = 26_ft + 5_in;
constexpr auto mid_line = field_length / 2;

} // namespace FieldConstants

RobotContainer::RobotContainer()
  : m_followerFeederBottom(OperatorConstants::kBottomFeederFollowerID, rev::spark::SparkFlex::MotorType::kBrushless),
    m_visualization{4, 3}
{
  //Config follower/reverse
  rev::spark::SparkFlexConfig bottomFeederFollowerConfig;
  bottomFeederFollowerConfig.Follow(m_feederBottom.getMotorIDforFollower(), true/*inverted*/);
  m_followerFeederBottom.Configure(bottomFeederFollowerConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);

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

  frc::RobotController::SetBrownoutVoltage(5_V);
}

void RobotContainer::ConfigureBindings() {

  m_swerve.SetDefaultCommand(m_swerve.CustomSwerveCommand(
    [this] { return m_oi.fwd(); }, 
    [this] { return m_oi.strafe(); },
    [this] { return m_oi.rot(); },
    [this] { return m_intake.IsArmOut(); }));

  m_oi.ZeroHeadingTrigger.OnTrue(m_swerve.RunOnce([this] {
    if (IsRed())
      m_swerve.ResetControlHeading(0.5_tr);
    else
      m_swerve.ResetControlHeading();
  }));

  //Default Command
  //m_intake.SetDefaultCommand(m_intake.IntakeFuel());
  m_shooter.SetDefaultCommand(m_shooter.SpinUp());

  constexpr auto kHubBlue = ShooterConstants::kHubBlue;
  constexpr auto kHubRed = ShooterConstants::kHubRed;

  //Driver Controller
  m_oi.RetractHoldArm.OnTrue(m_intake.Retract());
  m_oi.AutoAim.WhileTrue(AutoAim());
  m_oi.HUBAim.WhileTrue(m_shooter.AimFromHUB()
                      .AlongWith(TopFeederShooting()));
  m_oi.TowerAim.WhileTrue(m_shooter.AimFromTower()
                      .AlongWith(TopFeederShooting()));
  
  (m_oi.AutoAim || m_oi.HUBAim || m_oi.TowerAim).OnTrue(
    m_feederBottom.setRPMEnd(-25_tps).WithTimeout(0.5_s)
  );
 
  // (m_oi.HUBAim || m_oi.TowerAim).WhileTrue(m_swerve.CustomSwerveCommand(
  //   [this] { return m_oi.fwd(); },
  //   [this] { return m_oi.strafe(); },
  //   [this] { return IsRed()? 180_deg:0_deg; }
  // ).WithTimeout(2_s));
  
  //(!m_oi.TowerAim && !m_oi.HUBAim && !m_oi.AutoAim).Debounce(1_s).OnTrue(m_shooter.RetractHood());

  frc2::Trigger aimingAtHub{[this] {return isAimingAtHub();}};
  frc2::Trigger shooterReady([this] {return isShooterSpunUp();});
  
  //Manual Position Shoot
  frc2::Trigger shootForReal = 
  (m_oi.ShootFuelPlease && ((shooterReady && (m_oi.TowerAim || m_oi.HUBAim))  //Manual Shoot
                          || (m_oi.ShootFuelNOW)            //Overide
                          || (aimingAtHub && shooterReady)  //Auto is ready to fire
        ));
  
  ((m_oi.HUBAim || m_oi.TowerAim || m_oi.AutoAim) && shooterReady).WhileTrue(
    m_oi.RumbleController(0.75)
  );
        
  shootForReal.WhileTrue(m_feederBottom.ManuallySetMotor(m_oi.getBottomFeederSpeed, OperatorConstants::BottomFeederScaler));
  
  //Auto Jiggle when ready to shoot unless outtaking
  (shootForReal && !m_oi.OutTake).WhileTrue(
    frc2::cmd::Wait(0.25_s)
    .AndThen(m_intake.ScoreFuel(0.5_s)).Repeatedly()
  );

  // Cancel this with "BottomFeeder" trigger simultaneously
  m_oi.OutTake.WhileTrue(
    m_feederBottom.setRPMEnd(-40_tps)
  );
  m_oi.OutTake.WhileTrue(m_intake.OutakeFuel());
  m_oi.OutTake.WhileTrue(m_shooter.CycleHopper()
                        .AlongWith(TopFeederShooting()));

  // m_oi.LiftArm.WhileTrue(m_intake.Lift());
  m_oi.LiftArm.OnTrue(m_intake.IntakeFuel());
  m_oi.LiftArm.MultiPress(2, 0.5_s).OnTrue(m_intake.StopRollerUntilThisCMDInterupted()); //make it stop
#ifndef NOCLIMB
  m_oi.ClimbUp.OnTrue(m_climb.Deploy());
  m_oi.ClimbLift.OnTrue(m_climb.LiftBot());
  m_oi.ClimbDown.OnTrue(m_climb.Retract());
#endif

  //Co-Pilot Commands
  m_oi.ArmDownAndIntake.OnTrue(m_intake.IntakeFuel());
  m_oi.ArmDown.OnTrue(m_intake.Extend());
  m_oi.ArmRetract.OnTrue(m_intake.Retract());
  m_oi.ArmLifted.OnTrue(m_intake.Lift());
  m_oi.ArmIntakeManual.OnTrue(m_intake.ManuallyControlArm(m_oi.getIntakeArmSpeedCOP));
  m_oi.ArmUnzero.OnTrue(m_intake.Unzero());
#ifndef NOCLIMB
  m_oi.ClimbUpManual.WhileTrue(m_climb.BlindUp());
  m_oi.ClimbDownManual.WhileTrue(m_climb.BlindDown());
#endif
  m_oi.MakeRStickBottomFeederAndIntake.WhileTrue(m_feederBottom.ManuallySetMotor(m_oi.getFeederSpeedCOP, OperatorConstants::BottomFeederScaler)
      .AlongWith(m_intake.ManuallyCotrolIntake(m_oi.getFeederSpeedCOP, 1)));
  m_oi.MakeRStickBottomAndTopFeeder.WhileTrue(m_feederBottom.ManuallySetMotor(m_oi.getFeederSpeedCOP, OperatorConstants::BottomFeederScaler)
      .AlongWith(m_feederTop.ManuallySetMotor(m_oi.getFeederSpeedCOP, OperatorConstants::TopFeederScaler)));
  m_oi.MakeRStickBottomAndTopFeederOpposite.WhileTrue(m_feederBottom.ManuallySetMotor(m_oi.getFeederSpeedCOP, OperatorConstants::BottomFeederScaler)
      .AlongWith(m_feederTop.ManuallySetMotor([this] {return -m_oi.getFeederSpeedCOP();} ,OperatorConstants::TopFeederScaler)));
  m_oi.MakeRStickIntakeOnly.WhileTrue(m_intake.ManuallyCotrolIntake(m_oi.getFeederSpeedCOP, 1));

  m_oi.HoodRaise.WhileTrue(m_shooter.SetHoodPositionMin());
  m_oi.HoodLower.WhileTrue(m_shooter.SetHoodPosition(50));
  //RT + class state variable for right trigger shooter stuff
  m_oi.PitReset.OnTrue(
    m_shooter.SpinDown().AndThen(frc2::cmd::Run([] {}))
      .AlongWith(m_shooter.RetractHood())
#ifndef NOCLIMB
      .AlongWith(m_climb.Retract())
#endif
      .AlongWith(m_intake.Retract())
  );

}

void RobotContainer::ConfigureDashboard() {
  ConfigureVisualization();
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);
  frc::SmartDashboard::PutData("Auton", &m_chooser);
  frc::SmartDashboard::PutData("State Visualization", &m_visualization);
  frc::SmartDashboard::PutData("PDH", &m_pdh);
}

void RobotContainer::ConfigureVisualization() {
  m_intake.InitVisualization(m_visualization.GetRoot("intake_pivot", 2.5, 0.5));
  m_shooter.InitVisualization(m_visualization.GetRoot("launcher_pivot", 2, 1.6));
  m_feederBottom.InitVisualization(m_visualization.GetRoot("bottom_feeder", 1.85, 0.9)
    ->Append<frc::MechanismLigament2d>("bottom_feeder_axle", 0, 0_deg, 0, frc::Color::kBlack));
  m_feederTop.InitVisualization(m_visualization.GetRoot("top_feeder", 2, 1.1)
    ->Append<frc::MechanismLigament2d>("top_feeder_axle", 0, 0_deg, 0, frc::Color::kBlack));
#ifndef NOCLIMB
  m_climb.InitVisualization(m_visualization.GetRoot("climb_root", 0.2, 0.33));
#endif
}

void RobotContainer::ConfigureAuto() {
  // sentinel trajectory indicates we run a simple auto not dependent on sensors
  m_chooser.SetDefaultOption(
   "Default Auto: Depot Auto", AutoBuilder::Trajectory_t{});

  std::string folder = frc::filesystem::GetDeployDirectory()+"/choreo";
  fmt::println("Loading autos from the deploy directoyr: {}", folder);

  for (const auto& entry : std::filesystem::directory_iterator(folder)) {
    if(entry.path().extension() != ".traj"){
      continue;
    }
    
    const auto entry_string = entry.path().stem().string();
    
    auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(entry_string);
    if(trajectory.has_value()){
      //Flipped trajectory across horizontal midline
      const auto normal_trajectory = std::move(trajectory.value());
      const auto flipped_trajectory = normal_trajectory.Flipped<2024>().Flipped<2026>();

      m_chooser.AddOption(
        normal_trajectory.name + "_Left", std::move(normal_trajectory)
      );
      m_chooser.AddOption(
        flipped_trajectory.name + "_Right", std::move(flipped_trajectory)
      );
    }
    else{
      fmt::println("FAILED TO LOAD TRAJECTORY: {}", entry_string);
      frc::Alert alert{"FAILED TO LOAD TRAJECTORY", frc::Alert::AlertType::kError};
      alert.Set(true);
    }
  }

  // when user selects an auto, lets preview it so they know what theyre picking
  m_chooser.OnChange([this] (const AutoBuilder::Trajectory_t& t) {
    ReloadAuto();
  });
}

void RobotContainer::ReloadAuto() {
  const auto selected_trajectory = m_chooser.GetSelected();
  const auto t = IsRed()? std::move(selected_trajectory).Flipped():std::move(selected_trajectory);

  if (t.GetPoses().size() == 0) {
    // default auto selected
    m_swerve.GetField().GetObject("preview")->SetPoses({});
    m_swerve.GetField().GetObject("auton start")->SetPose(frc::Pose2d{-1000_m, -1000_m, 0_rad});
    m_loaded_auto = AutoBuilder::DepotAuto(*this);
  } else {
    m_swerve.GetField().GetObject("preview")->SetPoses(t.GetPoses());
    m_swerve.GetField().GetObject("auton start")->SetPose(t.GetInitialPose().value());
    m_loaded_auto = AutoBuilder::BuildSingleAuto(*this, std::move(t));
  }

}

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
  auto robot_still = frc2::Trigger{[this] {
    //return true;
    return m_swerve.IsStopped();
  }}.Debounce(0.35_s);

  // ROS to swerve
  robot_still.WhileTrue(FusePose());

  if constexpr (frc::RobotBase::IsSimulation()) {
    frc2::CommandScheduler::GetInstance().Schedule(
      frc2::cmd::Run([this] {
        m_ros.PubSim(m_swerve.GetSimulatedGroundTruth());
        }).IgnoringDisable(true));
  }
}

frc2::CommandPtr RobotContainer::FusePose() {
  return frc2::cmd::StartRun(
      // toss old values
      [this] {m_ros.GetNewMapToOdom();},
      [this] {
        // this allows other sources to override this if it dies on an erroneous state
        if (auto map_to_odom = m_ros.GetNewMapToOdom())
          m_swerve.SetMapToOdom(map_to_odom.value());
      }
    ).IgnoringDisable(true);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  m_swerve.GetField().GetObject("preview")->SetPoses({});
  return m_loaded_auto.get();
}

frc2::CommandPtr RobotContainer::TopFeederShooting() {
  return m_feederTop.setRPMEnd(40_tps);
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}

void RobotContainer::CheckAlliance() {
  bool wasRed = m_isRed;
  m_isRed = (frc::DriverStation::GetAlliance() ==
    frc::DriverStation::Alliance::kRed);

  if (wasRed != m_isRed) ReloadAuto();
}

bool RobotContainer::isAimingAtHub() {
  const auto hubPoint = IsRed() ? ShooterConstants::kHubRed : ShooterConstants::kHubBlue;

  const auto expectedAngle = (hubPoint - m_swerve.GetPose().Translation()).Angle();
  const auto currentDistance = (hubPoint - m_swerve.GetPose().Translation()).Norm();
  const auto currentAngle = m_swerve.GetPose().Rotation();
  const auto relativeAngle = expectedAngle.RelativeTo(currentAngle);
  const auto shotError = currentDistance * relativeAngle.Sin();
  const bool aimingAtHub = units::math::abs(shotError) < AutoConstants::aimingTolerance;

  return aimingAtHub;
}

bool RobotContainer::isShooterSpunUp() {
  return m_shooter.readyToFire();
}

frc2::CommandPtr RobotContainer::AutoAim(){
  
  auto positionFunc = [this] {
    return m_swerve.GetPose().Translation();
  };

  auto isRed = [this]{
    return IsRed();
  };

  return m_shooter.AutoAdjust(positionFunc, isRed)
                  .AlongWith(TopFeederShooting())
                  .AlongWith(FusePose())
                  .AlongWith(m_swerve.ZTargetCommand(
                    [this] {return m_oi.fwd();},
                    [this] {return m_oi.strafe();},
                    [this] {
                        frc::Transform2d adjustment{0_m, m_oi.aim_adjust(), 0_deg};
                        if (IsRed()) {
                          return frc::Pose2d{ShooterConstants::kHubRed, 0_deg} + adjustment;
                        } else {
                          return frc::Pose2d{ShooterConstants::kHubBlue, 0_deg} + adjustment.Inverse();
                        }
                      }
                  ));
}