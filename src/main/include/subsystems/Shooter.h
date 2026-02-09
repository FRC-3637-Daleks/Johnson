#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <frc/DigitalInput.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>


class ShooterSim;

class Shooter {
public:
    Shooter();
    ~Shooter();

private:

    bool IsBBBroken(); 

    void FeederIn();
    void FeederOut();


    

    void SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity);
    units::angular_velocity::turns_per_second_t targetVelocity{}; //read only
    units::angular_velocity::turns_per_second_t GetCurrentFlywheelSpeed();
    bool isAtCorrectSpeed();

private:

    ctre::phoenix6::CANBus m_CANBusInstance;

    ctre::phoenix6::hardware::TalonFX m_flyWheelLeadMotor;
    ctre::phoenix6::hardware::TalonFX m_flyWheelFollowMotor;

    frc::DigitalInput m_feederBreakBeam;
    ctre::phoenix6::hardware::TalonFX m_feederMotor;
};