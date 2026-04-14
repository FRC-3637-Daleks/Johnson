#include "swerve/PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include <iostream>
#include <numbers>
#include <random>

namespace PathFollowerConstants {

constexpr frc::Pose2d kCloseEnough{0.25_m, 0.25_m, 25_deg};

}

PathFollower::PathFollower(trajectory_t trajectory, SwerveChassis& swerve,
  EndConditionType end_type)
  : m_trajectory{std::move(trajectory)}
  , m_swerve{swerve}
  , m_end_type{end_type} {
  AddRequirements(&m_swerve);
}

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_swerve.GetField().GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
}

void PathFollower::Execute() {
  auto currentTime = m_timer.Get();
  if (auto desiredState =
    m_trajectory.SampleAt(currentTime, /* mirror */ false)) {
    auto desiredPose = desiredState->GetPose();
    auto feedForward = desiredState->GetChassisSpeeds();
    auto currentPose = m_swerve.GetPose();
    auto futurePos = currentPose.Translation() + frc::Translation2d{feedForward.vx*20_ms, feedForward.vy*20_ms};
    auto currentError = currentPose.Translation() - desiredPose.Translation();
    auto futureError = futurePos - desiredPose.Translation();

    m_swerve.DriveToPose(desiredPose, feedForward,
      {0.0_m, 0.0_m, 0_deg});
    
    /* Most of the time we should be on top of the desired state due to feedforward
     * If we fall behind too much, pause the timer so that the robot can catch up
     * to the desired state.
     * We also have to check that we're not in front of the desired-state before
     * clock is paused
     */
    if (!m_swerve.AtPose(desiredState->GetPose(), PathFollowerConstants::kCloseEnough)
        && currentError.Norm() > futureError.Norm())
        m_timer.Stop();
    else
      m_timer.Start();
  }
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  // teleport the trajectory "off-screen" once the command is complete
  m_swerve.GetField().GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetFinalPose();
  if (m_end_type == EndConditionType::TIMER)
    return m_timer.Get() >= m_trajectory.GetTotalTime();
  else
    return finalPose.has_value()
      && m_swerve.AtPose(finalPose.value(), {0.03_m, 0.03_m, 2_deg})
      && m_timer.Get() > m_trajectory.GetTotalTime()/2
      && (m_end_type == EndConditionType::NEAR_DEST || m_swerve.IsStopped())
    ;
}

frc2::Trigger PathFollower::GetEventTrigger(std::string_view event_name) {
  return frc2::Trigger{[this, events = m_trajectory.GetEvents(event_name)] {
    if (!m_timer.IsRunning()) return false;
    for (const auto& e : events) {
      if (frc::IsNear(e.timestamp, m_timer.Get(), 0.03_s)) return true;
    }

    return false;
  }};
}