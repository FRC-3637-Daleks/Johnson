#include "subsystems/Intake.h"

namespace IntakeConstants {

    int kIntakeMotorID = 3;
    int kArmMotorID = 2;

    double kP = 2.4;
    double kI = 0;
    double kD = 0.1;    


    units::volt_t intakeFowardVoltage = 6_V;
    units::volt_t intakeBackwardsVoltage = -7_V;
    units::volt_t armFowardVoltage = 6_V;
    units::volt_t armBackwardsVoltage = -7_V;
}

Intake::Intake() :
    m_CANBusInstance{"Drivebase"},
    m_armMotor{IntakeConstants::kArmMotorID, m_CANBusInstance},
    m_intakeMotor{IntakeConstants::kIntakeMotorID, m_CANBusInstance} 
{
    
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = IntakeConstants::kP; 
    slot0Configs.kI = IntakeConstants::kI; 
    slot0Configs.kD = IntakeConstants::kD; 

    m_armMotor.GetConfigurator().Apply(slot0Configs);    
}

Intake::~Intake() {
}



void Intake::IntakeIn() {
    m_intakeMotor.SetVoltage(IntakeConstants::intakeFowardVoltage);
}

void Intake::IntakeOut() {
    m_intakeMotor.SetVoltage(IntakeConstants::intakeBackwardsVoltage);
}

void Intake::IntakeStop() {
    m_intakeMotor.SetVoltage(0_V);
}

void Intake::ArmIn() {
    m_armMotor.SetVoltage(IntakeConstants::armBackwardsVoltage);
}

void Intake::ArmOut() {
    m_armMotor.SetVoltage(IntakeConstants::armBackwardsVoltage);
}

void Intake::ArmStop() {
    m_armMotor.SetVoltage(0_V);
}
