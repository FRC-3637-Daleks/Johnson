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
    constexpr auto kArmCanBus = ctre::phoenix6::CANBus{"Drivebase"};
    constexpr auto kRollerCanBus = ctre::phoenix6::CANBus::RoboRIO();
    constexpr int kArmMotorID = 11;
    constexpr int kRollerMotorID = 12;
    constexpr int kFollowerIntakeMotorID = 13;

    // Physical constants
    constexpr auto armGearing = 120.0;
    constexpr auto intakeGearing = 2.0;
    
    constexpr auto armLength = 18_in;
    constexpr auto armMass = 12_lb;
    // 1/3*m*r^2
    constexpr auto armMOI = 1.0/3.0 * armMass * armLength * armLength;
    constexpr auto armWeight = armMass * units::standard_gravity_t{1.0};
    // center of mass halfway up arm
    constexpr units::newton_meter_t gravityTorque = armWeight*armLength;

    constexpr auto fuelMass = 0.2_kg;
    constexpr auto maxFuelOnIntake = 40;
    constexpr auto maxFuelMass = fuelMass*maxFuelOnIntake;
    constexpr auto fuelTorque = maxFuelMass*units::standard_gravity_t{1.0}*armLength/2;
    
    // Configurations
    constexpr auto tolerance = 0.01_tr;
    constexpr auto armOutPos = 0.21_tr;
    constexpr auto armLiftPos = 0.15_tr;
    constexpr auto armInPos = 0.0_tr;
    constexpr auto armRange = units::math::abs(armOutPos - armInPos);
    constexpr auto extendTime = 0.75_s;
    constexpr auto retractTime = 0.75_s;
    constexpr auto accelTime = 0.1_s;
    constexpr auto slowRetractTime = 3_s;
    constexpr auto extendVel = (armOutPos - armInPos)/extendTime;
    constexpr auto retractVel = (armInPos - armOutPos)/retractTime;
    constexpr auto slowRetractVel = (armInPos - armOutPos)/slowRetractTime;

    // Gains/Limits
    constexpr auto armMotor = frc::DCMotor::KrakenX60FOC(1).WithReduction(armGearing);
    constexpr auto rollerMotor = frc::DCMotor::KrakenX44FOC(2).WithReduction(intakeGearing);

    constexpr auto gravityTorqueCurrent = armMotor.Current(gravityTorque);

    constexpr units::ampere_t kG = gravityTorqueCurrent;
    
    // These constants seem to need vibe-tuning
    constexpr ctre::unit::amperes_per_turn_per_second_squared_t kA = 
        0.5*armGearing*armMOI/armMotor.Kt/1_tr;
    
    // Higher results in slower acceleration
    constexpr ctre::unit::volts_per_turn_per_second_squared_t kA_profile{20};
    
    // motion magic cruise velocity 0.3


    // Scaled relative to the weight of the intake
    constexpr auto peakExtendTorque = 4*gravityTorque;
    constexpr auto peakForwardCurrent = armMotor.Current(peakExtendTorque);
    constexpr auto peakRetractTorque = 2*(gravityTorque + fuelTorque);
    constexpr auto peakReverseCurrent = armMotor.Current(-peakRetractTorque);

    constexpr auto currentLimits = ctre::phoenix6::configs::CurrentLimitsConfigs{}
        .WithSupplyCurrentLimit(40_A)  // never allow over this amount
        .WithSupplyCurrentLowerLimit(10_A)  // limit to this if over for 100_ms
        .WithSupplyCurrentLowerTime(100_ms)
        .WithStatorCurrentLimit(units::math::max(IntakeConstants::peakForwardCurrent, -IntakeConstants::peakReverseCurrent))
    ;

    constexpr auto rollerCurrentLimits = ctre::phoenix6::configs::CurrentLimitsConfigs{}
        .WithSupplyCurrentLimit(60_A)  // never allow over this amount
        .WithSupplyCurrentLowerLimit(30_A)  // limit to this if over for 100_ms
        .WithSupplyCurrentLowerTime(500_ms)
        .WithStatorCurrentLimit(80_A)
    ;

    constexpr auto mmConfig = ctre::phoenix6::configs::MotionMagicConfigs{}
        .WithMotionMagicAcceleration(extendVel/accelTime)
        .WithMotionMagicCruiseVelocity(2*extendVel)
        .WithMotionMagicExpo_kV(1.0/armMotor.Kv)
        .WithMotionMagicExpo_kA(kA_profile)
    ;

    // kP defined in terms of the kG and the maximum error
    constexpr auto positionGains = ctre::phoenix6::configs::Slot0Configs{}
        .WithKG(kG.value())
        .WithGravityType(ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine)
        .WithGravityArmPositionOffset(armOutPos)
        .WithKA(kA.value())
        .WithKP(20*(kG/units::turn_t{armRange}).value())
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

    constexpr auto rollerGains = ctre::phoenix6::configs::Slot0Configs{}
        .WithKP(0.12)
        .WithKI(0)
        .WithKD(0)
        .WithKV(ctre::unit::volts_per_turn_per_second_t{1.0/rollerMotor.Kv}.value())
    ;

    // No kG since the cosine cant be calculated appropriately
    constexpr auto blindGains = ctre::phoenix6::configs::Slot2Configs{}
        .WithKA(kA.value())
        .WithKP((2*kG/units::turns_per_second_t{units::math::abs(retractVel)}).value())
    ;

    constexpr auto armInRequest = 
        ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{armInPos}
        .WithSlot(0)
        .WithOverrideCoastDurNeutral(true)
    ;
    constexpr auto armLiftRequest = 
        ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{armLiftPos}
        .WithSlot(0)
        .WithOverrideCoastDurNeutral(true)
    ;
    constexpr auto armOutRequest = 
        ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC{armOutPos}
        .WithSlot(0)
        .WithOverrideCoastDurNeutral(true)
    ;

    constexpr auto blindExtendRequest = 
        ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC{extendVel}
        .WithSlot(2)
        .WithIgnoreSoftwareLimits(true)
        .WithOverrideCoastDurNeutral(true)
    ;
    constexpr auto blindRetractRequest =
        ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC{retractVel}
        .WithSlot(2)
        .WithIgnoreSoftwareLimits(true)
        .WithOverrideCoastDurNeutral(true)
    ;

    constexpr auto homeArmRequest =
        ctre::phoenix6::controls::DutyCycleOut{-0.4}
        .WithIgnoreSoftwareLimits(true)
    ;

    constexpr auto holdExtendRequest = 
        ctre::phoenix6::controls::TorqueCurrentFOC{gravityTorqueCurrent/10}
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
        .WithOverrideCoastDurNeutral(true)
        .WithFeedForward(-armMotor.Current(fuelTorque))
    ;

    constexpr auto manualArmRequest =
        ctre::phoenix6::controls::DutyCycleOut{0.0}
        .WithOverrideBrakeDurNeutral(true)
    ;

    constexpr auto rollerRequest =
        ctre::phoenix6::controls::VelocityVoltage{0.0_tps}
        .WithSlot(0)
        .WithIgnoreSoftwareLimits(true)
    ;

    // Intake Rollers
    constexpr auto intakeWheelDiameter = 2_in;
    constexpr auto intakeWheelCircum = intakeWheelDiameter * std::numbers::pi;

    // Has to be moving faster than the robot lest we push away the balls
    constexpr auto intakingSurfaceSpeed = 4_mps;
    constexpr auto intakingWheelVelocity = units::turns_per_second_t{
        1_tr*intakingSurfaceSpeed/intakeWheelCircum
    };
    constexpr auto outakingSurfaceSpeed = -4_mps;
    constexpr auto outakingWheelVelocity = 1_tr*outakingSurfaceSpeed/intakeWheelCircum;

    constexpr auto scoreWheelVelocity = 5_tps;

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
    m_armMotor{IntakeConstants::kArmMotorID, IntakeConstants::kArmCanBus},
    m_intakeMotor{IntakeConstants::kRollerMotorID, IntakeConstants::kRollerCanBus},
    m_feederIntakeFollower{IntakeConstants::kFollowerIntakeMotorID, IntakeConstants::kRollerCanBus},
    m_armZeroed{false},
    m_sim_state{new IntakeSim{*this}}
{
    //Config follower
    m_feederIntakeFollower.SetControl(ctre::phoenix6::controls::Follower{
        IntakeConstants::kRollerMotorID, true
    });

    auto put_cmd = [this] (std::string_view name, frc2::CommandPtr&& cmd) {
        frc::SmartDashboard::PutData(fmt::format("Intake/{}", name),
            std::move(cmd).WithName(name).Unwrap().release());
    };

    put_cmd("Extend", Extend());
    put_cmd("Retract", Retract());
    put_cmd("BlindExtend", BlindExtend());
    put_cmd("BlindRetract", BlindRetract());
    put_cmd("HomeArm", HomeArm());
    put_cmd("IntakeFuel", IntakeFuel());
    put_cmd("OutakeFuel", OutakeFuel());
    put_cmd("ScoreFuel", ScoreFuel());

    using namespace ctre::phoenix6;
    configs::TalonFXConfiguration armConfig, rollerConfig;

    armConfig.WithMotorOutput(configs::MotorOutputConfigs{}
        .WithNeutralMode(signals::NeutralModeValue::Brake)
        .WithInverted(signals::InvertedValue::Clockwise_Positive)
    );

    rollerConfig.WithMotorOutput(configs::MotorOutputConfigs{}
        .WithNeutralMode(signals::NeutralModeValue::Brake)
        .WithInverted(signals::InvertedValue::Clockwise_Positive)
        .WithDutyCycleNeutralDeadband(0.05)
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
    rollerConfig.WithCurrentLimits(IntakeConstants::currentLimits);

    armConfig.WithFeedback(configs::FeedbackConfigs{}
        .WithSensorToMechanismRatio(IntakeConstants::armGearing)
    );

    rollerConfig.WithFeedback(configs::FeedbackConfigs{}
        .WithSensorToMechanismRatio(IntakeConstants::intakeGearing)
    );

    armConfig.WithSlot0(IntakeConstants::positionGains);
    armConfig.WithSlot1(IntakeConstants::velocityGains);
    armConfig.WithSlot2(IntakeConstants::blindGains);
    armConfig.WithMotionMagic(IntakeConstants::mmConfig);

    rollerConfig.WithSlot0(IntakeConstants::rollerGains);

    m_armMotor.GetConfigurator().Apply(armConfig);
    m_intakeMotor.GetConfigurator().Apply(rollerConfig);
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

frc2::CommandPtr Intake::ManuallyControlArm(std::function<double()> input) {
    return RunEnd(
        [this, input] {
            auto req = IntakeConstants::manualArmRequest;
            m_armMotor.SetControl(req.WithOutput(input()*0.5));},
        [this] {m_armMotor.SetControl(IntakeConstants::manualArmRequest);}
    );
}

frc2::CommandPtr Intake::ManuallyCotrolIntake(std::function<double()> input, double scaler) {
    return m_rollerSubsystem.RunEnd(
        [this, input, scaler] {m_intakeMotor.Set(input()*scaler);},
        [this] {m_intakeMotor.StopMotor();}
    );
}

frc2::CommandPtr Intake::SpinRoller(units::turns_per_second_t vel) {
    return m_rollerSubsystem.RunEnd(
        [this, vel] {SetRollerVelocity(vel);},
        [this] {m_intakeMotor.StopMotor();}
    );
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
            && m_armMotor.GetStatorCurrent().GetValue() > 15_A;
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

frc2::CommandPtr Intake::SeedArm(units::turn_t pos) {
    return RunOnce([this, pos] {if (!m_armZeroed) m_armMotor.SetPosition(pos); m_armZeroed = true;});
}

frc2::CommandPtr Intake::IntakeFuel() {
    return 
        Extend()
        .AndThen(
            SpinRoller(IntakeConstants::intakingWheelVelocity)
        )
    ;
}

frc2::CommandPtr Intake::OutakeFuel() {
    return 
        Extend()
        .AndThen(
           SpinRoller(IntakeConstants::outakingWheelVelocity)
        )
    ;
}

frc2::CommandPtr Intake::ScoreFuel(units::second_t duration) {
    auto req = IntakeConstants::scoreArmRequest;
    req.WithVelocity(-IntakeConstants::armRange/duration);
    return
        SpinRoller(IntakeConstants::intakingWheelVelocity).RaceWith(
                Run([this, req] {m_armMotor.SetControl(req);}).WithTimeout(duration)
                .AndThen(Extend().WithTimeout(duration))
            ).WithTimeout(duration*3)
    ;
}

//**************************** Private Members ****************************/

units::angle::turn_t Intake::GetArmPos() {
    return m_armMotor.GetPosition().GetValue();
}

bool Intake::IsArmOut() {
    return m_armZeroed && GetArmPos() > IntakeConstants::armOutPos - IntakeConstants::tolerance;
}

bool Intake::IsArmIn() {
    return m_armZeroed && GetArmPos() < IntakeConstants::armInPos + IntakeConstants::tolerance;
}

void Intake::HoldExtended() {
    m_armMotor.SetControl(IntakeConstants::holdExtendRequest);
}

void Intake::HoldRetracted() {
    m_armMotor.SetControl(IntakeConstants::holdRetractRequest);
}

void Intake::SetRollerVelocity(units::turns_per_second_t vel) {
    auto req = IntakeConstants::rollerRequest;
    m_intakeMotor.SetControl(req.WithVelocity(vel));
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
    if (m_arm) {
        if (m_armZeroed) {
            m_arm->SetAngle(0.25_tr - GetArmPos());
            m_arm->SetColor(frc::Color::kWhite);
        } else {
            m_arm->SetAngle(0.25_tr);
            m_arm->SetColor(frc::Color::kDimGray);
        }
    }
}

void Intake::InitVisualization(frc::MechanismRoot2d* intake_pivot) {
    m_arm = intake_pivot->Append<frc::MechanismLigament2d>(
        "intake", 1.5, 90_deg, 15, frc::Color8Bit{frc::Color::kDimGray});
    
    //m_intakeMotor.InitVisualization(m_arm);
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