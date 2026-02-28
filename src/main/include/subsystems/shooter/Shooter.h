#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkFlex.h>
#include <frc/DigitalInput.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <wpi/sendable/SendableBuilder.h>

#include "subsystems/shooter/LinearActuator.h"


class ShooterSim;

class Shooter : public frc2::SubsystemBase{
public:
    Shooter();
    ~Shooter();

public:
    void InitializeDashboard();
    void UpdateDashboard();

    void Periodic() override;

public:
    frc2::CommandPtr SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity);

    //Point in mm
    frc2::CommandPtr SetHoodPosition(double point);
    //Point in mm
    frc2::CommandPtr SetHoodPositionUntilThere(double point);
    frc2::CommandPtr SetHoodPositionMin();
    bool isHoodAtPos();

private:

    void SetFlywheelSpeedNRM(units::angular_velocity::turns_per_second_t velocity);
    units::angular_velocity::turns_per_second_t targetVelocity{}; //read only
    units::angular_velocity::turns_per_second_t GetCurrentFlywheelSpeed();
    bool isAtCorrectSpeed();

    bool IsBBBroken(); 

    LinearActuator m_hoodActuator;

private:
    ctre::phoenix6::hardware::TalonFX m_flyWheelLeadMotor;
    ctre::phoenix6::hardware::TalonFX m_flyWheelFollowMotor;

    frc::DigitalInput m_feederBreakBeam;



// simulation
private:
    friend class ShooterSim;
    std::unique_ptr<ShooterSim> m_sim_state;

    void SimulationPeriodic() override;
};