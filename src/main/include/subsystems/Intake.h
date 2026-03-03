#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include "Feeder.h"

#include <ctre/phoenix6/TalonFX.hpp>

#include <rev/SparkFlex.h>


class IntakeSim;

class Intake : public frc2::SubsystemBase {
public:

    Intake();
    ~Intake();

public:
    void Periodic() override;

public:
    // Smoothly extends arm to the extended position using closed loop control on rotor position
    frc2::CommandPtr Extend();
    
    // Smoothly retracts the arm to the starting position using closed loop control on rotor position
    frc2::CommandPtr Retract();

    // Smoothly lifts arm (expected but not nessiary) from Exteneded Pos
    frc2::CommandPtr Lift();

    //
    frc2::CommandPtr ManuallyControlArm(std::function<double()> input);

    frc2::CommandPtr ManuallyCotrolIntake(std::function<double()> input);

    // Extends arm at a constant velocity for a duration that should roughly extend it fully
    frc2::CommandPtr BlindExtend();

    // Brings arm in at a constant velocity for a duration that should roughly retract it fully
    frc2::CommandPtr BlindRetract();

    // Drives arm slowly but firmly in until it senses resistance and re-zeroes the arm at that position.
    frc2::CommandPtr HomeArm();

    // Intakes fuel indefinitely
    frc2::CommandPtr IntakeFuel();
    
    // Spits out fuel indefinitely
    frc2::CommandPtr OutakeFuel();

    // Executes a specific motion profile for coaxing the fuel into the shooter
    frc2::CommandPtr ScoreFuel(units::second_t duration = 3_s);

public:
    bool IsArmOut();
    std::function<bool()> ArmOut{[this] {return IsArmOut();}};
    bool IsArmIn();
    std::function<bool()> ArmIn{[this] {return IsArmIn();}};
    
private:
    // Returns arm angle relative to horizontal
    units::angle::turn_t GetArmPos();
    // Applies a bit of pressure at the extents to help keep the intake secure
    void HoldExtended(); void HoldRetracted(); void HoldLift();

private:
    void UpdateDashboard();
    void UpdateVisualization();

private:
    ctre::phoenix6::hardware::TalonFX m_armMotor;
    Feeder m_intakeMotor;
    rev::spark::SparkFlex m_feederIntakeFollower;

    bool m_armZeroed;
    std::function<bool()> Zeroed{[this] {return m_armZeroed;}};

private:
    friend class IntakeSim;
    void SimulationPeriodic() override;

    frc::Mechanism2d m_mechIntake{3, 3};
    frc::MechanismRoot2d* m_root = m_mechIntake.GetRoot("intake", 2, 0);

    frc::MechanismLigament2d *m_arm{}, *m_wheel{};
    std::unique_ptr<IntakeSim> m_sim_state;

};