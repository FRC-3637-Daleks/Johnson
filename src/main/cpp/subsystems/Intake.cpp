#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/RobotController.h>
#include <frc/simulation/RoboRioSim.h>

#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

#include <rev/config/SparkFlexConfig.h>
#include <rev/sim/SparkFlexSim.h>

#include <iostream>
#include <random>

namespace IntakeConstants {

    constexpr auto kCanBus = ctre::phoenix6::CANBus{"Drivebase"};
    constexpr int kIntakeMotorID = 10;
    constexpr int kArmMotorID = 11;

    constexpr auto kP = 3.0;
    constexpr auto kI = 0.0;
    constexpr auto kD = 0.1;   
    constexpr auto kG = 0.3;
    constexpr auto kS = 0.0;
    constexpr auto kV = 0.0;

    constexpr auto armGearing = 20.0;
    constexpr auto intakeGearing = 1.0;
    
    constexpr auto armOutPos = 0_tr;
    constexpr auto armInPos = 0.25_tr;
    constexpr auto tolerance = 0.05_tr;
    
    constexpr auto intakeFowardVoltage = 6_V;
    constexpr auto intakeBackwardsVoltage = -7_V;

    constexpr auto armInRequest = 
        ctre::phoenix6::controls::PositionVoltage{armGearing*armInPos}
        .WithSlot(0)
    ;
    constexpr auto armOutRequest = 
        ctre::phoenix6::controls::PositionVoltage{armGearing*armOutPos}
        .WithSlot(0)
    ;

    constexpr auto armLength = 0.3_m;
    constexpr auto armMass = 15_lb;
    constexpr auto armMOI = armMass * armLength * armLength;

    constexpr auto wheelMOI = 0.001_kg_sq_m;
}

class IntakeSim {
public:
    friend class Intake;

public:
    IntakeSim(Intake &in);

public:
    frc::DCMotor m_intakeMotorModel;
    frc::sim::SingleJointedArmSim m_armPhysics;
    frc::sim::FlywheelSim m_wheelPhysics;

    ctre::phoenix6::sim::TalonFXSimState m_ArmMotorState;
    rev::spark::SparkFlexSim m_IntakeMotorState;
};

Intake::Intake() :
    m_armMotor{IntakeConstants::kArmMotorID, IntakeConstants::kCanBus},
    m_intakeMotor{IntakeConstants::kIntakeMotorID, rev::spark::SparkFlex::MotorType::kBrushless},
    m_arm{m_root->Append<frc::MechanismLigament2d>(
        "intake", 1, 90_deg, 20, frc::Color8Bit{frc::Color::kBlue})},
    m_wheel{m_arm->Append<frc::MechanismLigament2d>(
        "wheel", 0.1, 90_deg, 6, frc::Color8Bit{frc::Color::kPurple})},
    m_sim_state{new IntakeSim{*this}}
{
    frc::SmartDashboard::PutData("Mechanisms", &m_mechIntake);

    ctre::phoenix6::configs::TalonFXConfiguration armConfig;

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = IntakeConstants::kP;
    slot0Configs.kI = IntakeConstants::kI;
    slot0Configs.kD = IntakeConstants::kD;
    slot0Configs.kG = IntakeConstants::kG;
    slot0Configs.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    slot0Configs.kS = IntakeConstants::kS; 
    slot0Configs.kV = IntakeConstants::kV;

    armConfig.WithSlot0(slot0Configs);

    auto &motionMagicConfigs = armConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
      units::angular_velocity::turns_per_second_t{65};
    motionMagicConfigs.MotionMagicAcceleration =
      units::angular_acceleration::turns_per_second_squared_t{200};

    m_armMotor.GetConfigurator().Apply(armConfig);

    rev::spark::SparkFlexConfig wheelConfig;
    m_intakeMotor.Configure(wheelConfig,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kPersistParameters);
}

Intake::~Intake() {
}

void Intake::Periodic() {
    UpdateDashboard();
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
    return m_armMotor.GetPosition().GetValue()/IntakeConstants::armGearing;
}

bool Intake::IsArmOut() {
    return frc::IsNear(IntakeConstants::armOutPos, GetArmPos(), IntakeConstants::tolerance);
}

bool Intake::IsArmIn() {
    return frc::IsNear(IntakeConstants::armInPos, GetArmPos(), IntakeConstants::tolerance);
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
    m_armMotor.SetControl(IntakeConstants::armInRequest);
}

void Intake::ArmOut() {
    m_armMotor.SetControl(IntakeConstants::armOutRequest);
}

void Intake::UpdateDashboard() {
    UpdateVisualization();
}

void Intake::UpdateVisualization() {
    m_arm->SetAngle(GetArmPos());
    m_wheel->SetAngle(m_intakeMotor.GetEncoder().GetPosition()*1_tr);
}

//**************************** Simulation ****************************/

IntakeSim::IntakeSim(Intake& in) :
    m_intakeMotorModel{frc::DCMotor::NeoVortex(1)},
    m_armPhysics{
        frc::DCMotor::KrakenX60FOC(1),
        IntakeConstants::armGearing,
        IntakeConstants::armMOI,
        IntakeConstants::armLength,
        IntakeConstants::armOutPos,  // Min Angle
        IntakeConstants::armInPos,   // Max Angle
        true,                        // Simulate Gravity
        IntakeConstants::armInPos   // Starting angle
    },
    m_wheelPhysics{
        frc::LinearSystemId::FlywheelSystem(
            m_intakeMotorModel,
            IntakeConstants::wheelMOI,
            IntakeConstants::intakeGearing),
        m_intakeMotorModel},
    m_ArmMotorState{in.m_armMotor},
    m_IntakeMotorState{&in.m_intakeMotor, &m_intakeMotorModel}
{
    /* This initializes the system to an "unknown" state which must be calibrated on boot
     * using some technique or sensor.
     */
    std::random_device rng;
    std::uniform_real_distribution dist{IntakeConstants::armOutPos.value(), IntakeConstants::armInPos.value()};
    m_armPhysics.SetState(dist(rng)*1_tr, 0_rpm);
    m_ArmMotorState.SetRawRotorPosition(m_armPhysics.GetAngle()*IntakeConstants::armGearing);
    in.m_armMotor.SetPosition(0.0_tr);
}

void Intake::SimulationPeriodic() {
    if (!m_sim_state) return;

    const auto supply_voltage = frc::RobotController::GetInputVoltage()*1_V;

    m_sim_state->m_ArmMotorState.SetSupplyVoltage(supply_voltage);

    //update arm physics sim
    auto& ssArmPhys = m_sim_state->m_armPhysics;
    ssArmPhys.SetInputVoltage(
        m_sim_state->m_ArmMotorState.GetMotorVoltage());
        
    const auto last_vel = ssArmPhys.GetVelocity();
    ssArmPhys.Update(20_ms);

    //update arm motor
    m_sim_state->m_ArmMotorState.SetRawRotorPosition(ssArmPhys.GetAngle() * IntakeConstants::armGearing);
    m_sim_state->m_ArmMotorState.SetRotorVelocity(ssArmPhys.GetVelocity() * IntakeConstants::armGearing);
    const auto accel = (ssArmPhys.GetVelocity() - last_vel)/20_ms;
    m_sim_state->m_ArmMotorState.SetRotorAcceleration(accel);

    //update intake motor
    m_sim_state->m_wheelPhysics.SetInputVoltage(m_sim_state->m_IntakeMotorState.GetAppliedOutput()*supply_voltage);
    m_sim_state->m_wheelPhysics.Update(20_ms);
    m_sim_state->m_IntakeMotorState.iterate(
        IntakeConstants::intakeGearing*units::revolutions_per_minute_t{
            m_sim_state->m_wheelPhysics.GetAngularVelocity()}.value(),
        supply_voltage.value(),
        0.02
    );
}