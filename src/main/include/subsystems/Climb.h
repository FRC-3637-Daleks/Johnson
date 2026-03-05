#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <units/base.h>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>

#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

class ClimbSim;

class Climb : public frc2::SubsystemBase {
public:
    Climb();
    ~Climb() noexcept;

    void Periodic() override;

    enum class Height : int{Bottom = 0, Climbed, Top};

    bool IsAtHeight(Height height);

    units::centimeter_t GetPosition();

    frc2::CommandPtr GoToHeight(Height goal);
    frc2::CommandPtr Deploy();
    frc2::CommandPtr LiftBot();
    frc2::CommandPtr Retract();

public:
    void InitVisualization(frc::MechanismRoot2d* climb_base);

private:
    ctre::phoenix6::hardware::TalonFX m_climbMotor;
    Height m_targetHeight{Height::Bottom};

    void UpdateDashboard();
    void UpdateVisualization();

    bool IsAtHeight(units::centimeter_t pos);

    void SetGoalHeight(units::centimeter_t length);
    void SetGoalHeight(Height height);

    units::angle::turn_t HeightToRotorTurns(const units::centimeter_t height) const;
    units::centimeter_t RotorTurnsToHeight(const units::angle::turn_t turns) const;

private:  // visualization
    frc::MechanismLigament2d *m_hook;

private:  // simulation
    friend class ClimbSim;
    std::unique_ptr<ClimbSim> m_sim_state;
    void SimulationPeriodic() override;
};