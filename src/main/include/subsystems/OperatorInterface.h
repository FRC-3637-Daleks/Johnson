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

private:
  frc2::CommandXboxController m_swerveController;
  frc2::CommandXboxController m_copilotController;
  
  bool IsRed();
};