#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkFlex.h>
#include <frc/DigitalInput.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>


class ShooterSim;

class Shooter : public frc2::SubsystemBase{
public:
    Shooter();
    ~Shooter();

public:
    void InitializeDashboard();

public:
    frc2::CommandPtr SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity);

    frc2::CommandPtr FeederBottomIn();
    frc2::CommandPtr FeederBottomOut();
    frc2::CommandPtr FeederTopIn();
    frc2::CommandPtr FeederTopOut();


private:

    void SetFlywheelSpeedNRM(units::angular_velocity::turns_per_second_t velocity);
    units::angular_velocity::turns_per_second_t targetVelocity{}; //read only
    units::angular_velocity::turns_per_second_t GetCurrentFlywheelSpeed();
    bool isAtCorrectSpeed();

    bool IsBBBroken(); 

    //NRM = Non-returing member
    void FeederBottomInNRM();
    void FeederBottomOutNRM();
    void FeederBottomStopNRM();
    void FeederTopInNRM();
    void FeederTopOutNRM();
    void FeederTopStopNRM();

private:
    ctre::phoenix6::hardware::TalonFX m_flyWheelLeadMotor;
    ctre::phoenix6::hardware::TalonFX m_flyWheelFollowMotor;

    frc::DigitalInput m_feederBreakBeam;
    rev::spark::SparkFlex m_feederBottomMotor, m_feederTopMotor;

// simulation
private:
    friend class ShooterSim;
    std::unique_ptr<ShooterSim> m_sim_state;

    void SimulationPeriodic() override;
};