#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>

class Intake {
public:

    Intake();
    ~Intake();

private:

    units::angle::turn_t GetArmPos();
    
    enum class ArmPos {
        In = 0,
        Out,
    }
    //void SetArmPos();
    
    bool IsArmOut(); 
    bool IsArmIn();
//Make a wrapper**
    void IntakeIn();
    void IntakeOut();
    void IntakeStop();
    
    void ArmIn();
    void ArmOut();
    void ArmStop();

private:

    ctre::phoenix6::CANBus m_CANBusInstance;
    ctre::phoenix6::hardware::TalonFX m_armMotor;
    ctre::phoenix6::hardware::TalonFX m_intakeMotor;

};