#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>


namespace IntakeConstants {

    int kIntakeMotorID = 10;
    int kArmMotorID = 11;

    double kP = 3.0;
    double kI = 0.0;
    double kD = 0.1;   
    double kG = 0.3;
    double kS = 0.0;
    double kV = 0.0;

    double GearRatio = 100;
    
    units::angle::turn_t armOutPos = 0_tr;
    units::angle::turn_t armInPos = 0.25_tr;
    units::angle::turn_t tolerance = 0.05_tr;
    
    units::volt_t intakeFowardVoltage = 6_V;
    units::volt_t intakeBackwardsVoltage = -7_V;
    units::volt_t armFowardVoltage = 6_V;
    units::volt_t armBackwardsVoltage = -7_V;
}

class IntakeSim {
public:
    friend class Intake;

public:
    IntakeSim(Intake &in);

public:
    ctre::phoenix6::sim::TalonFXSimState m_ArmMotorState, m_IntakeMotorState;

    frc::sim::SingleJointedArmSim m_armPhysics;

};

Intake::Intake() :
    m_CANBusInstance{"Drivebase"},
    m_armMotor{IntakeConstants::kArmMotorID, m_CANBusInstance},
    m_intakeMotor{IntakeConstants::kIntakeMotorID, m_CANBusInstance},
    m_arm{m_root->Append<frc::MechanismLigament2d>("intake", 1, 90_deg, 6, frc::Color8Bit{frc::Color::kBlue})},
    m_wrist{m_arm->Append<frc::MechanismLigament2d>(
            "wrist", 0.5, 90_deg, 6, frc::Color8Bit{frc::Color::kPurple})},
    m_sim_state{new IntakeSim{*this}}
{
    ctre::phoenix6::configs::TalonFXConfiguration m_armConfig;

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = IntakeConstants::kP; 
    slot0Configs.kI = IntakeConstants::kI; 
    slot0Configs.kD = IntakeConstants::kD; 
    slot0Configs.kG = IntakeConstants::kG; 
    slot0Configs.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
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

frc2::CommandPtr Intake::GoArmOut() {
    return Run([this]() {ArmOut();});
}

frc2::CommandPtr Intake::GoArmIn() {
    return Run([this]() {ArmIn();});
}

frc2::CommandPtr Intake::IntakeFuel() {
    return RunEnd([this]() {IntakeIn();},
    [this]() {IntakeStop();});
}

frc2::CommandPtr Intake::OutakeFuel() {
    return RunEnd([this]() {IntakeOut();},
    [this]() {IntakeStop();});
}


//**************************** Private Members ****************************/

units::angle::turn_t Intake::GetArmPos() {
    return m_armMotor.GetPosition().GetValue();
}

bool Intake::IsArmOut() {
    return IntakeConstants::armOutPos - IntakeConstants::tolerance < GetArmPos()/IntakeConstants::GearRatio
        && IntakeConstants::armOutPos + IntakeConstants::tolerance > GetArmPos()/IntakeConstants::GearRatio;
}

bool Intake::IsArmIn() {
    return IntakeConstants::armInPos - IntakeConstants::tolerance < GetArmPos()/IntakeConstants::GearRatio
        && IntakeConstants::armInPos + IntakeConstants::tolerance > GetArmPos()/IntakeConstants::GearRatio;
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
    m_goal = IntakeConstants::armInPos * IntakeConstants::GearRatio;
    std::cout << m_goal.value() << std::endl;
    m_armMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{m_goal}
        .WithSlot(0));
}

void Intake::ArmOut() {
    m_goal = IntakeConstants::armOutPos * IntakeConstants::GearRatio;
    std::cout << m_goal.value() << std::endl;
    m_armMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{m_goal}
        .WithSlot(0));
}

//**************************** Simulation ****************************/

IntakeSim::IntakeSim(Intake& in) :
    m_armPhysics{
        frc::DCMotor::KrakenX60(1), // DCMotor
        100.0,                      // Gearing
        1.0_kg_sq_m,                // Moment of Inertia
        0.3_m,                      // Arm Length
        units::radian_t{0_tr},      // Min Angle
        units::radian_t{0.25_tr},   // Max Angle
        true,                       // Simulate Gravity
        units::radian_t{0.25_tr}       // Stating angle
    }, 
    m_ArmMotorState{in.m_armMotor},
    m_IntakeMotorState{in.m_intakeMotor}   
{}

void Intake::SimulationPeriodic() {
    if (!m_sim_state) return;

    //does not command motor, just available enegery
    m_sim_state->m_ArmMotorState.SetSupplyVoltage(12_V); 

    //update arm physics sim
    auto& ssArmPhys = m_sim_state->m_armPhysics;
    ssArmPhys.SetInputVoltage(
        m_sim_state->m_ArmMotorState.GetMotorVoltage());
    ssArmPhys.Update(20_ms);

    //update arm motor + visual sim
    m_sim_state->m_ArmMotorState.SetRawRotorPosition(ssArmPhys.GetAngle() * IntakeConstants::GearRatio);
    m_sim_state->m_ArmMotorState.SetRotorVelocity(ssArmPhys.GetVelocity() * IntakeConstants::GearRatio);
    m_arm->SetAngle(ssArmPhys.GetAngle());

    //update intake motor
    double delta = (double)m_intakeMotor.Get() * 30;
    auto newAngle =units::angle::degree_t(m_wrist->GetAngle() + delta);
    m_wrist->SetAngle(newAngle);

    frc::SmartDashboard::PutNumber("Intake/Delta", delta);
    frc::SmartDashboard::PutNumber("Intake/NewAngle", newAngle.value());
    frc::SmartDashboard::PutNumber("Intake/PhysSimArmVelocity", ssArmPhys.GetVelocity().value());
    
    frc::SmartDashboard::PutData("Intake", &m_mechIntake);
}