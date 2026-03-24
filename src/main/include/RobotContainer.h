// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#define NOCLIMB

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <numbers>

#include <frc/PowerDistribution.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/shooter/Shooter.h"
#include "subsystems/Feeder.h"
#include "subsystems/ROSBridge.h"
#include "subsystems/OperatorInterface.h"
#include "subsystems/Intake.h"
#include "subsystems/Climb.h"
#include "AutoBuilder.h"
#include "subsystems/LEDSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetDisabledCommand();
  frc2::Command* GetAutonomousCommand();

public:
  // The robot's subsystems are defined here...
  Drivetrain m_swerve;
  ROSBridge m_ros;
  OperatorInterface m_oi;
  Shooter m_shooter;
  Feeder m_feederTop{Feeder::Type::Top};
  Feeder m_feederBottom{Feeder::Type::Bottom};
  rev::spark::SparkFlex m_followerFeederBottom;
  Intake m_intake; 
  LEDSubsystem m_ledSubsystem;

  frc::PowerDistribution m_pdh{1, frc::PowerDistribution::ModuleType::kRev};
 
#ifndef NOCLIMB
  Climb m_climb;
#endif

  // other state
  bool m_isRed;
  frc::SendableChooser<AutoBuilder::Trajectory_t> m_chooser;

  // visualization objects, units in approximate feet
  frc::Mechanism2d m_visualization;

  // load depot auto by default
  frc2::CommandPtr m_loaded_auto{AutoBuilder::DepotAuto(*this)};

public:
  void ConfigureBindings();
  void ConfigureDashboard();
  void ConfigureVisualization();
  void ConfigureAuto();
  void ReloadAuto();
  void ConfigureContinuous();

public:
  frc2::CommandPtr TopFeederShooting();
  frc2::CommandPtr AutoAim();

public:
  bool IsRed() {return m_isRed;}
  void CheckAlliance();
};