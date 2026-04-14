#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>

#include <choreo/trajectory/Trajectory.h>

#include "swerve/SwerveChassis.h"

class PathFollower: public frc2::CommandHelper<frc2::Command, PathFollower> {
public:
    enum class EndConditionType {STOP_AT_DEST, NEAR_DEST, TIMER};

public:
    using pose_supplier_t = std::function<frc::Pose2d()>;

    using swerve_state_modifier_t =
        std::function<void(std::array<frc::SwerveModuleState, 4>)>;

    using trajectory_t = choreo::Trajectory<choreo::SwerveSample>;

    /**
     * Constructor
     *
     * @param trajectory The trajectory you want to follow
     * @param desiredPoseSupplier A function that returns the desired pose
     * @param subsystem The subsystem used by this command.
     */
    PathFollower(trajectory_t trajectory, SwerveChassis& subsystem,
        EndConditionType end_type = EndConditionType::STOP_AT_DEST);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    frc2::Trigger GetEventTrigger(std::string_view event_name);

private:
    trajectory_t m_trajectory;
    SwerveChassis& m_swerve;
    frc::Timer m_timer;
    EndConditionType m_end_type;
};
