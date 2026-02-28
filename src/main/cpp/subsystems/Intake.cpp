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
    // Ports
    constexpr auto kCanBus = ctre::phoenix6::CANBus{"Drivebase"};
    constexpr int kArmMotorID = 11;

    // Arm
    // Physical constants
    constexpr auto armGearing = 120.0;
    
    constexpr auto armLength = 16_in;
    constexpr auto armMass = 15_lb;
    // 1/3*m*r^2
    constexpr auto armMOI = 1.0/3.0 * armMass * armLength * armLength;
    constexpr auto armWeight = armMass * units::standard_gravity_t{1.0};
    // center of mass halfway up arm
    constexpr units::newton_meter_t gravityTorque = armWeight*armLength/2;

    constexpr auto fuelMass = 0.2_kg;
    constexpr auto maxFuelOnIntake = 30;
    constexpr auto maxFuelMass = fuelMass*maxFuelOnIntake;
    constexpr auto fuelTorque = maxFuelMass*units::standard_gravity_t{1.0}*armLength/2;
    
    // Configurations
    constexpr auto tolerance = 0.005_tr;
    constexpr auto armOutPos = 0.25_tr;
    constexpr auto armLiftPos = 0.1_tr;
    constexpr auto armInPos = 0.0_tr;
    constexpr auto armRange = units::math::abs(armOutPos - armInPos);
    constexpr auto extendTime = 1_s;
    constexpr auto retractTime = 1_s;
    constexpr auto accelTime = 0.2_s;
    constexpr auto slowRetractTime = 3_s;
    constexpr auto extendVel = (armOutPos - armInPos)/extendTime;
    constexpr auto retractVel = (armInPos - armOutPos)/retractTime;
    constexpr auto slowRetractVel = (armInPos - armOutPos)/slowRetractTime;

    // Gains/Limits
    constexpr auto armMotor = frc::DCMotor::KrakenX60FOC(1).WithReduction(armGearing);

    constexpr auto gravityTorqueCurrent = armMotor.Current(gravityTorque);

    constexpr units::ampere_t kG = gravityTorqueCurrent;
    
    // These constants seem to need vibe-tuning
    constexpr ctre::unit::amperes_per_turn_per_second_squared_t kA = 
        0.75*armGearing*armMOI/armMotor.Kt/1_tr;
    
    // Higher results in slower acceleration
    constexpr ctre::unit::volts_per_turn_per_second_squared_t kA_profile{7.5};
    
    // Scaled relative to the weight of the intake
    constexpr auto peakExtendTorque = 4*gravityTorque;
    constexpr auto peakForwardCurrent = armMotor.Current(peakExtendTorque);
    constexpr auto peakRetractTorque = 2*(gravityTorque + fuelTorque);
    constexpr auto peakReverseCurrent = armMotor.Current(-peakRetractTorque);

    constexpr auto currentLimits = ctre::phoenix6::configs::CurrentLimitsConfigs{}
        .WithSupplyCurrentLimit(40_A)  // never allow over this amount
        .WithSupplyCurrentLowerLimit(10_A)  // limit to this if over for 100_ms
        .WithSupplyCurrentLowerTime(100_ms)
    ;

    constexpr auto mmConfig = ctre::phoenix6::configs::MotionMagicConfigs{}
        .WithMotionMagicAcceleration(extendVel/accelTime)
        .WithMotionMagicExpo_kV(1.0/armMotor.Kv)
        .WithMotionMagicExpo_kA(kA_profile)
    ;

    // kP defined in terms of the kG and the maximum error
    constexpr auto positionGains = ctre::phoenix6::configs::Slot0Configs{}
        .WithKG(kG.value())
        .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        .WithGravityArmPositionOffset(armOutPos)
        .WithKA(kA.value())
        .WithKP(10*(kG/units::turn_t{armRange}).value())
        .WithKD(5*(kG/units::turn_t{armRange}).value())
    ;

    // kP defined in terms of kG and the maximum error
    constexpr auto velocityGains = ctre::phoenix6::configs::Slot1Configs{}
        .WithKG(kG.value())
        .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        .WithGravityArmPositionOffset(armOutPos)
        .WithKA(kA.value())
        .WithKP(5*(kG/units::turns_per_second_t{units::math::abs(retractVel)}).value())
        .WithKI(5*(kG/units::turn_t{armRange}).value())
    ;

    // No kG since the cosine cant be calculated appropriately
    constexpr auto blindGains = ctre::phoenix6::configs::Slot2Configs{}
        .WithKA(kA.value())
        .WithKP((2*kG/units::turns_per_second_t{units::math::abs(retractVel)}).value())
    ;

    constexpr auto armInRequest = 
        ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{armInPos}
        .WithSlot(0)
    ;
    constexpr auto armLiftRequest = 
        ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{armLiftPos}
        .WithSlot(0)
    ;
    constexpr auto armOutRequest = 
        ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{armOutPos}
        .WithSlot(0)
    ;

    constexpr auto blindExtendRequest = 
        ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC{extendVel}
        .WithSlot(2)
        .WithIgnoreSoftwareLimits(true)
    ;
    constexpr auto blindRetractRequest =
        ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC{retractVel}
        .WithSlot(2)
        .WithIgnoreSoftwareLimits(true)
    ;

    constexpr auto homeArmRequest =
        ctre::phoenix6::controls::DutyCycleOut{-0.1}
        .WithIgnoreSoftwareLimits(true)
    ;

    constexpr auto holdExtendRequest = 
        ctre::phoenix6::controls::TorqueCurrentFOC{gravityTorqueCurrent}
        .WithIgnoreSoftwareLimits(true)
    ;

    constexpr auto holdRetractRequest = 
        ctre::phoenix6::controls::TorqueCurrentFOC{-gravityTorqueCurrent}
        .WithIgnoreSoftwareLimits(true)
    ;

    constexpr auto scoreArmRequest = 
        ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC{slowRetractVel}
        .WithSlot(1)
        .WithIgnoreSoftwareLimits(true)
        .WithFeedForward(-armMotor.Current(fuelTorque))
    ;

    // Intake Rollers
    constexpr auto intakeGearing = 1.0;
    constexpr auto intakeWheelDiameter = 2_in;
    constexpr auto intakeWheelCircum = intakeWheelDiameter * std::numbers::pi;

    // Has to be moving faster than the robot lest we push away the balls
    constexpr auto intakingSurfaceSpeed = 3_mps;
    constexpr auto intakingWheelVelocity = 1_tr*intakingSurfaceSpeed/intakeWheelCircum;
    constexpr auto outakingSurfaceSpeed = 1_mps;
    constexpr auto outakingWheelVelocity = 1_tr*outakingSurfaceSpeed/intakeWheelCircum;

    constexpr auto wheelMOI = 0.001_kg_sq_m;
}

class IntakeSim {
public:
    friend class Intake;

public:
    IntakeSim(Intake &in);

public:
    frc::sim::SingleJointedArmSim m_armPhysics;

    ctre::phoenix6::sim::TalonFXSimState m_ArmMotorState;
};

Intake::Intake() :
    m_armMotor{IntakeConstants::kArmMotorID, IntakeConstants::kCanBus},
    m_intakeMotor{Feeder::Type::Intake},
    m_armZeroed{false},
    m_arm{m_root->Append<frc::MechanismLigament2d>(
        "intake", 1, 90_deg, 20, frc::Color8Bit{frc::Color::kBlue})},
    m_wheel{m_arm->Append<frc::MechanismLigament2d>(
        "wheel", 0.1, 90_deg, 6, frc::Color8Bit{frc::Color::kPurple})},
    m_sim_state{new IntakeSim{*this}}
{
    auto put_cmd = [this] (std::string_view name, frc2::CommandPtr&& cmd) {
        frc::SmartDashboard::PutData(fmt::format("Intake/{}", name),
            std::move(cmd).WithName(name).Unwrap().release());
    };

    frc::SmartDashboard::PutData("Mechanisms", &m_mechIntake);
    put_cmd("Extend", Extend());
    put_cmd("Retract", Retract());
    put_cmd("BlindExtend", BlindExtend());
    put_cmd("BlindRetract", BlindRetract());
    put_cmd("HomeArm", HomeArm());
    put_cmd("IntakeFuel", IntakeFuel());
    put_cmd("OutakeFuel", OutakeFuel());
    put_cmd("ScoreFuel", ScoreFuel());

    using namespace ctre::phoenix6;
    configs::TalonFXConfiguration armConfig;

    armConfig.WithMotorOutput(configs::MotorOutputConfigs{}
        .WithNeutralMode(signals::NeutralModeValue::Brake)
        .WithInverted(signals::InvertedValue::Clockwise_Positive)
    );

    armConfig.WithTorqueCurrent(configs::TorqueCurrentConfigs{}
        .WithPeakForwardTorqueCurrent(
            IntakeConstants::armMotor.Current(IntakeConstants::peakExtendTorque))
        .WithPeakReverseTorqueCurrent(
            -IntakeConstants::armMotor.Current(IntakeConstants::peakRetractTorque)
        )
    );

    armConfig.WithHardwareLimitSwitch(configs::HardwareLimitSwitchConfigs{}
        .WithReverseLimitAutosetPositionEnable(true)
        .WithReverseLimitAutosetPositionValue(0_tr)
    );

    armConfig.WithClosedLoopGeneral(configs::ClosedLoopGeneralConfigs{}
        .WithContinuousWrap(false)
    );

    armConfig.WithCurrentLimits(IntakeConstants::currentLimits);

    armConfig.WithFeedback(configs::FeedbackConfigs{}
        .WithSensorToMechanismRatio(IntakeConstants::armGearing)
    );

    armConfig.WithSlot0(IntakeConstants::positionGains);
    armConfig.WithSlot1(IntakeConstants::velocityGains);
    armConfig.WithSlot2(IntakeConstants::blindGains);
    armConfig.WithMotionMagic(IntakeConstants::mmConfig);

    m_armMotor.GetConfigurator().Apply(armConfig);

}

Intake::~Intake() {
}

void Intake::Periodic() {
    UpdateDashboard();
}

frc2::CommandPtr Intake::Extend() {
    return 
        HomeArm().Unless(Zeroed)  // attempt to HomeArm first if needed
        .AndThen(
            frc2::cmd::Either(
                Run([this] {m_armMotor.SetControl(IntakeConstants::armOutRequest);})
                    .WithTimeout(IntakeConstants::extendTime*1.5) // safety factor
                    .Until(ArmOut)
                    .Unless(ArmOut),  // dont do anything if arm is already out
                BlindExtend(),  // if we failed to home, need a fallback
                Zeroed))
        .AndThen(RunOnce([this] {HoldExtended();}))
    ;
}

frc2::CommandPtr Intake::Retract() {
    return 
        frc2::cmd::Either(
            Run([this] {m_armMotor.SetControl(IntakeConstants::armInRequest);})
                .Until(ArmIn)
                .Unless(ArmIn)  // dont do anything if arm is already in
                .AndThen(HomeArm())  // rehome arm upon successful retract to prevent drift,
                .WithTimeout(IntakeConstants::retractTime*1.5), // safety factor
            BlindRetract(),  // if we never zeroed, need a fallback
            Zeroed)
        .AndThen(RunOnce([this] {HoldRetracted();}))
    ;
}

frc2::CommandPtr Intake::Lift() {
    return 
        Run([this] {m_armMotor.SetControl(IntakeConstants::armLiftRequest);})
        .OnlyIf(Zeroed);
}

frc2::CommandPtr Intake::BlindExtend() {
    return 
        Run([this] {m_armMotor.SetControl(IntakeConstants::blindExtendRequest);})
            .WithTimeout(IntakeConstants::extendTime)
        .AndThen(RunOnce([this] {HoldExtended();}))
    ;
}

frc2::CommandPtr Intake::BlindRetract() {
    return
        Run([this] {m_armMotor.SetControl(IntakeConstants::blindRetractRequest);})
            .WithTimeout(IntakeConstants::retractTime)
        .AndThen(RunOnce([this] {HoldRetracted();}))
    ;
}

frc2::CommandPtr Intake::HomeArm() {
    // detects when current spikes and motor isnt moving for a 10th of a second
    auto hard_stopped = frc2::Trigger{[this] {
        return frc::IsNear(0_tps, m_armMotor.GetVelocity().GetValue(), 1_tps)
            && m_armMotor.GetStatorCurrent().GetValue() > 10_A;
    }}.Debounce(0.1_s);

    return
        Run([this, hard_stopped] {
            // Drives motor gently backwards
            // Autosets rotor to 0 when hard_stopped is triggered
            auto req = IntakeConstants::homeArmRequest;
            m_armMotor.SetControl(req
//                .WithLimitReverseMotion(hard_stopped.Get())
            );

            // this flag is set once per boot-cycle
            //m_armZeroed = hard_stopped.Get();
            if (hard_stopped.Get()) {
                m_armZeroed = true;
                m_armMotor.SetPosition(0_tr);
            }
        })
        .Until([this, hard_stopped] {return hard_stopped.Get();})
        .WithTimeout(0.5_s)
        .FinallyDo([this] {HoldRetracted();})
    ;
}

frc2::CommandPtr Intake::IntakeFuel() {
    return 
        Extend()
        .AndThen(
            m_intakeMotor.setRPMEnd(
                IntakeConstants::intakingWheelVelocity)
        )
    ;
}

frc2::CommandPtr Intake::OutakeFuel() {
    return 
        Extend()
        .AndThen(
           m_intakeMotor.setRPMEnd(
                IntakeConstants::outakingWheelVelocity)
        )
    ;
}

frc2::CommandPtr Intake::ScoreFuel(units::second_t duration) {
    auto req = IntakeConstants::scoreArmRequest;
    req.WithVelocity(-IntakeConstants::armRange/duration);
    return
        Extend()
        .AndThen(
            Run([this, req] {m_armMotor.SetControl(req);})
                .WithTimeout(duration)
        )
        .AndThen(Retract())
    ;
}

//**************************** Private Members ****************************/

units::angle::turn_t Intake::GetArmPos() {
    return m_armMotor.GetPosition().GetValue();
}

bool Intake::IsArmOut() {
    return m_armZeroed && frc::IsNear(IntakeConstants::armOutPos, GetArmPos(), IntakeConstants::tolerance);
}

bool Intake::IsArmIn() {
    return m_armZeroed && frc::IsNear(IntakeConstants::armInPos, GetArmPos(), IntakeConstants::tolerance);
}

void Intake::HoldExtended() {
    m_armMotor.SetControl(IntakeConstants::holdExtendRequest);
}

void Intake::HoldRetracted() {
    m_armMotor.SetControl(IntakeConstants::holdRetractRequest);
}

void Intake::UpdateDashboard() {
    frc::SmartDashboard::PutBoolean("Intake/Zeroed", m_armZeroed);
    frc::SmartDashboard::PutNumber("Intake/Arm Current (A)", 
        m_armMotor.GetTorqueCurrent().GetValue().value());
    frc::SmartDashboard::PutBoolean("Intake/Deployed", IsArmOut());
    frc::SmartDashboard::PutBoolean("Intake/Retracted", IsArmIn());
    UpdateVisualization();
}

void Intake::UpdateVisualization() {
    m_arm->SetAngle(0.25_tr - GetArmPos());
}

//**************************** Simulation ****************************/

IntakeSim::IntakeSim(Intake& in) :
    m_armPhysics{
        frc::DCMotor::KrakenX60FOC(1),
        IntakeConstants::armGearing,
        IntakeConstants::armMOI,
        IntakeConstants::armLength,
        IntakeConstants::armInPos,  // Min Angle
        IntakeConstants::armOutPos,   // Max Angle
        true,                        // Simulate Gravity
        IntakeConstants::armInPos   // Starting angle
    },
    m_ArmMotorState{in.m_armMotor}
{
    /* This initializes the system to an "unknown" state which must be calibrated on boot
     * using some technique or sensor.
     */
    std::random_device rng;
    std::uniform_real_distribution dist{IntakeConstants::armInPos.value(), IntakeConstants::armOutPos.value()};
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
}