#include "subsystems/Intake.h"

namespace IntakeConstants {

    int kArmMotorID = 2;
    int kIntakeMotorID = 3;

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
}

Intake::~Intake() {
}

void Intake::IntakeIn() {
    m_intakeMotor.SetVoltage(IntakeConstants::intakeFowardVoltage);
}

void Intake::IntakeOut() {
    m_intakeMotor.SetVoltage(IntakeConstants::intakeBackwardsVoltage);
}

void Intake::ArmIn() {
    m_intakeMotor.SetVoltage(IntakeConstants::armBackwardsVoltage);
}

void Intake::ArmOut() {
    m_intakeMotor.SetVoltage(IntakeConstants::armBackwardsVoltage);
}


