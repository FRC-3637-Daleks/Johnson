#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkFlex.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/DigitalInput.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <wpi/sendable/SendableBuilder.h>

#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/Mechanism2d.h>

class FeederSim; //forward declarations

class Feeder : public frc2::SubsystemBase{
public:

    enum class Type {
        Top,
        Bottom,
        Intake
    };
    std::vector<std::string> TypeStrs = {"Top", "Bottom", "Intake"};

    Feeder(Type type); //get the motor ID from constants and pass it in here
    ~Feeder();

public:
    void InitializeDashboard();
    void UpdateDashboard();
    void Periodic() override;

public:

    int getMotorIDforFollower();
    frc2::CommandPtr ManuallySetMotor(std::function<double()> point);
    frc2::CommandPtr setRPM(units::turns_per_second_t speed);
    frc2::CommandPtr setRPMEnd(units::turns_per_second_t speed);
    frc2::CommandPtr setRPMUntilThere(units::turns_per_second_t speed);
    units::turns_per_second_t getRPM();    

private:
    void stop();
    void setVelocity(units::turns_per_second_t speed);
    bool isAtRPM(units::turns_per_second_t speed);

private:

    rev::spark::SparkFlex m_feederMotor;
    rev::spark::SparkClosedLoopController m_pidController;

    units::turns_per_second_t targetSpeed = 0_tps;

// simulation
private:
    friend class FeederSim;
    std::unique_ptr<FeederSim> m_sim_state;

    void SimulationPeriodic() override;

    std::string thisMotorStr{};
    
    frc::Mechanism2d m_mech{2, 2};
    frc::MechanismRoot2d* m_root = m_mech.GetRoot("Feeder", 1, 1);
    frc::MechanismLigament2d* m_MotorLine =
        m_root->Append<frc::MechanismLigament2d>("Angle", 1, 0_deg);
    units::degree_t motorDegState = 0_deg;
};