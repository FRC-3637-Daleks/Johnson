#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/Mechanism2d.h>

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
    
    // positive for intake, negative for outake, 0 to stop
    void SetIntakeSpeed(units::turns_per_second_t speed);

    // Applies a bit of pressure at the extents to help keep the intake secure
    void HoldExtended(); void HoldRetracted();

private:
    void UpdateDashboard();
    void UpdateVisualization();

private:
    ctre::phoenix6::hardware::TalonFX m_armMotor;
    rev::spark::SparkFlex m_intakeMotor;

    bool m_armZeroed;
    std::function<bool()> Zeroed{[this] {return m_armZeroed;}};

private:
    friend class IntakeSim;
    std::unique_ptr<IntakeSim> m_sim_state;
    void SimulationPeriodic() override;

    frc::Mechanism2d m_mechIntake{3, 3};
    frc::MechanismRoot2d* m_root = m_mechIntake.GetRoot("intake", 2, 0);

    frc::MechanismLigament2d *m_arm{}, *m_wheel{};
};