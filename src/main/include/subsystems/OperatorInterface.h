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

namespace OperatorConstants {

} // namespace OperatorConstants

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

  frc2::Trigger ZeroHeadingTrigger = m_swerveController.Start();
  frc2::Trigger RobotRelativeToggleTrigger = m_swerveController.Back();

  frc2::CommandPtr RumbleController(units::second_t time, double intensity);

  //TODO: Default command intakes
  //TODO: Defualt command for shooter is hood down.
  frc2::Trigger RetractHoldArm = m_swerveController.RightBumper();
  frc2::Trigger AutoAim = m_swerveController.Y(); //from any place
  frc2::Trigger HUBAim = m_swerveController.A(); //HUB
  frc2::Trigger TowerAim = m_swerveController.X(); //Ladder
  //TODO: Trigger to set speed of the bottom feeder
  frc2::Trigger OutTake = m_swerveController.B();
  frc2::Trigger LiftArm = m_swerveController.LeftBumper();
  //TODO: Make climbUp left trigger
  // frc2::Trigger ClimbUp = m_swerveController.POVUp();
  frc2::Trigger ClimbDown = m_swerveController.POVDown();

  frc2::Trigger ArmDownAndIntake = m_copilotController.POVDown();
  frc2::Trigger ArmDown = m_copilotController.POVRight();
  frc2::Trigger ArmRetract = m_copilotController.POVLeft();
  frc2::Trigger ArmLifted = m_copilotController.POVUp();
  //TODO: Make both intake and OI command for controlling the velocity of the arm with left stick Y
  frc2::Trigger ClimbUpManual = m_copilotController.A();
  frc2::Trigger ClimbDownManual = m_copilotController.B();
  //TODO: Make R stick x and y independently controll the intake and bottom feeder
  frc2::Trigger TopFeederInManual = m_copilotController.RightBumper();
  frc2::Trigger TopFeederOutManual = m_copilotController.LeftBumper();
  frc2::Trigger HoodRaise = m_copilotController.Y();
  frc2::Trigger HoodLower = m_copilotController.X();
  //TODO: Make right trigger enter shooter mode
  frc2::Trigger PitReset = m_copilotController.Start(); 

public:
  frc2::CommandXboxController m_swerveController;
  frc2::CommandXboxController m_copilotController;
  
  bool IsRed();
};