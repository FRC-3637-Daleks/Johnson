#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <units/base.h>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>

class Climb : public frc2::SubsystemBase {
public:
    Climb();
    ~Climb() noexcept;

    void Periodic() override;
    void UpdateDashboard();
    void UpdateVisualization();

    enum Height : int{Bottom = 0, Top};

    bool IsAtHeight(units::centimeter_t pos);
    bool IsAtHeight(Height height);

    void SetGoalHeight(units::centimeter_t length);
    void SetGoalHeight(Height height);
    
    units::centimeter_t GetHeight();

    frc2::CommandPtr GoToHeight(Height goal);

private:
    ctre::phoenix6::hardware::TalonFX m_climbMotor;

    units::angle::turn_t heightToRotorTurns(const units::centimeter_t height);
    units::centimeter_t RotorTurnsToheight(const units::angle::turn_t turns);
};