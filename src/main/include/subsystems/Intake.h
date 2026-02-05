#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

class Intake : public frc2::SubsystemBase {
public:

    Intake();
    ~Intake();

public:
    //Hangs until compleated
    //Uses const voltage once out to keep out
    frc2::CommandPtr GoArmOut();

    //Hangs until compleated
    frc2::CommandPtr GoArmIn();

    //RunEnd
    frc2::CommandPtr IntakeFuel();
    
    //RunEnd
    frc2::CommandPtr OutakeFuel();

private:

    units::angle::turn_t GetArmPos();
    
    bool IsArmOut(); 
    bool IsArmIn();

    void IntakeIn();
    void IntakeOut();
    void IntakeStop();
    void IntakeConstVoltage();
    
    void ArmIn();
    void ArmOut();
    void ArmStop();

private:

    ctre::phoenix6::CANBus m_CANBusInstance;
    ctre::phoenix6::hardware::TalonFX m_armMotor;
    ctre::phoenix6::hardware::TalonFX m_intakeMotor;

};