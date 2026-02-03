#include "subsystems/Intake.h"

namespace IntakeConstants {

    int kIntakeMotorID = 3;
    int kArmMotorID = 2;

    double kP = 2.4;
    double kI = 0.0;
    double kD = 0.1;   
    double kG = 0.0;
    double kS = 0.0;
    double kV = 0.0;
    
    units::angle::turn_t armOutPos = 10_tr;
    units::angle::turn_t armInPos = 0_tr;
    units::angle::turn_t tolerance = 0.5_tr;
    
    units::angle::turn_t armPositions[] = {0_tr, 10_tr};
    std::string_view armPositionNames[] = {"In", "Out"};

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
    ctre::phoenix6::configs::TalonFXConfiguration m_armConfig;

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = IntakeConstants::kP; 
    slot0Configs.kI = IntakeConstants::kI; 
    slot0Configs.kD = IntakeConstants::kD; 
    slot0Configs.kG = IntakeConstants::kG; 
    slot0Configs.kS = IntakeConstants::kS; 
    slot0Configs.kV = IntakeConstants::kV;

    m_armConfig.WithSlot0(slot0Configs);

    auto &motionMagicConfigs = m_armConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
      units::angular_velocity::turns_per_second_t{65};
    motionMagicConfigs.MotionMagicAcceleration =
      units::angular_acceleration::turns_per_second_squared_t{200};

    m_armMotor.GetConfigurator().Apply(m_armConfig);  
    
}

Intake::~Intake() {
}

units::angle::turn_t Intake::GetArmPos() {
    return m_armMotor.GetPosition().GetValue();
}

bool Intake::IsArmOut() {
    return IntakeConstants::armOutPos - IntakeConstants::tolerance < GetArmPos()
        && IntakeConstants::armOutPos + IntakeConstants::tolerance > GetArmPos();
}

bool Intake::IsArmIn() {

    return IntakeConstants::armInPos - IntakeConstants::tolerance < GetArmPos()
        && IntakeConstants::armInPos + IntakeConstants::tolerance > GetArmPos();

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
