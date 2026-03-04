#pragma once

#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

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


// #pragma GCC diagnostic ignored
//  DO NOT DO THIS, this is a quick comopetition fix
//  namespace TEMP_COMP_VARIABLES {
//  bool s_climkExtended{false};
//  }

class OperatorInterface {

public:
  OperatorInterface();
  double throttle();
  double boolean_slowdown();
  units::meters_per_second_t strafe();
  units::meters_per_second_t fwd();
  units::meters_per_second_t alt_fwd();
  units::revolutions_per_minute_t rot();

  //Co-Pilot = COP
  std::function<double()> getIntakeArmSpeedCOP{[this] 
    {return m_copilotController.GetLeftY();}};
  std::function<double()> getFeederSpeedCOP{[this] 
    {return m_copilotController.GetRightX();}};
  
  std::function<double()> getBottomFeederSpeed{[this]
    {return m_swerveController.GetRightTriggerAxis();}};

  frc2::Trigger ZeroHeadingTrigger = m_swerveController.Start();
  frc2::Trigger RobotRelativeToggleTrigger = m_swerveController.Back();

  frc2::CommandPtr RumbleController(units::second_t time, double intensity);

  frc2::Trigger RetractHoldArm = m_swerveController.RightBumper();
  frc2::Trigger AutoAim = m_swerveController.Y(); //from any place
  frc2::Trigger HUBAim = m_swerveController.A(); //HUB
  frc2::Trigger TowerAim = m_swerveController.X(); //Ladder
  frc2::Trigger BottomFeeder{
    [this] {return std::abs(getBottomFeederSpeed()) > 0.1;}
  };
  frc2::Trigger OutTake = m_swerveController.B();
  frc2::Trigger LiftArm = m_swerveController.LeftBumper();
  frc2::Trigger ClimbUp{
    [this] {return m_swerveController.GetLeftTriggerAxis
      () > 0.3;}
  };
  frc2::Trigger ClimbDown = m_swerveController.POVRight();
  frc2::Trigger ClimbLift = m_swerveController.POVDown();

  frc2::Trigger ArmDownAndIntake = m_copilotController.POVDown();
  frc2::Trigger ArmDown = m_copilotController.POVRight();
  frc2::Trigger ArmRetract = m_copilotController.POVLeft();
  frc2::Trigger ArmLifted = m_copilotController.POVUp();
  frc2::Trigger ArmIntakeManual{
    [this] {return std::abs(getIntakeArmSpeedCOP()) > 0.1;}
  };

  frc2::Trigger ClimbUpManual = m_copilotController.A();
  frc2::Trigger ClimbDownManual = m_copilotController.B();

  frc2::Trigger ShouldRStickMove{[this] {return std::abs(getFeederSpeedCOP()) > 0.15;}};
  frc2::Trigger MakeRStickBottomFeederAndIntake = ShouldRStickMove && m_copilotController.LeftBumper() && !m_copilotController.RightBumper();
  frc2::Trigger MakeRStickBottomAndTopFeeder = ShouldRStickMove && !m_copilotController.LeftBumper() && m_copilotController.RightBumper();
  frc2::Trigger MakeRStickBottomAndTopFeederOpposite = ShouldRStickMove && m_copilotController.LeftBumper() && m_copilotController.RightBumper();
  frc2::Trigger MakeRStickIntakeOnly = ShouldRStickMove && !m_copilotController.LeftBumper() && !m_copilotController.RightBumper();


  frc2::Trigger HoodRaise = m_copilotController.Y();
  frc2::Trigger HoodLower = m_copilotController.X();
  //TODO: Make right trigger enter shooter mode
  frc2::Trigger PitReset = m_copilotController.Start(); 

public:
  frc2::CommandXboxController m_swerveController;
  frc2::CommandXboxController m_copilotController;
  
  bool IsRed();
};