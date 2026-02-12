#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/Mechanism2d.h>


class IntakeSim;

class Intake : public frc2::SubsystemBase {
public:

    Intake();
    ~Intake();

public:
    frc2::CommandPtr GoArmOut();

    frc2::CommandPtr GoArmIn();

    //RunEnd
    frc2::CommandPtr IntakeFuel();
    
    //RunEnd
    frc2::CommandPtr OutakeFuel();

    bool IsArmOut(); 
    bool IsArmIn();

private:

    units::angle::turn_t GetArmPos();
    
    void IntakeIn();
    void IntakeOut();
    void IntakeStop();
    
    void ArmIn();
    void ArmOut();

private:

    ctre::phoenix6::CANBus m_CANBusInstance;
    ctre::phoenix6::hardware::TalonFX m_armMotor;
    ctre::phoenix6::hardware::TalonFX m_intakeMotor;
    units::angle::turn_t m_goal{};
private:
    friend class IntakeSim;
    std::unique_ptr<IntakeSim> m_sim_state;
    void SimulationPeriodic() override;

    frc::Mechanism2d m_mechIntake{3, 3};
    frc::MechanismRoot2d* m_root = m_mechIntake.GetRoot("intake", 2, 0);

    frc::MechanismLigament2d *m_arm{}, *m_wrist{};
};