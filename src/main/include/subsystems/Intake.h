#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANBus.hpp>


class Intake {
public:

    Intake();
    ~Intake();

private:

    void IntakeIn();
    void IntakeOut();

    
    void ArmIn();
    void ArmOut();


private:

    ctre::phoenix6::CANBus m_CANBusInstance;
    ctre::phoenix6::hardware::TalonFX m_armMotor;
    ctre::phoenix6::hardware::TalonFX m_intakeMotor;

};