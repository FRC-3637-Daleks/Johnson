#include "subsystems/shooter/Shooter.h"

#include <frc/system/plant/LinearSystemId.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc2/command/ProxyCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

#include <rev/sim/SparkFlexSim.h>

namespace ShooterConstants {
    int kfeederBreakBeamID = 1;

    int kShooterFlywheelLeaderID = 20;
    int kShooterFlywheelFollowerID = 21;

    //Voltage
    units::volt_t kPeakForwardVoltage = 12_V;
    units::volt_t kReverseForwardVoltage = 0_V;

    //Torque
    units::ampere_t PeakForwardTorqueCurrent = 80_A;
    units::ampere_t PeakReverseTorqueCurrent = -10_A;

    //used just in bool isAtCorrectSpeed()
    units::angular_velocity::turns_per_second_t kSpeedTolerance = 0.3_tps;

    constexpr auto launcherGearing = 26.0/18.0;
    constexpr auto launcherMOI = 0.01_kg_sq_m;
    constexpr auto launcherMotor = frc::DCMotor::KrakenX60FOC(2).WithReduction(launcherGearing);

    constexpr auto feederGearing = 1.0;
    constexpr auto feederMOI = 0.001_kg_sq_m;

    //PID
    double kP = 10.0;
    double kI = 0.000;
    double kD = 0.0;
    constexpr auto kS = 2.0_A;
    constexpr ctre::unit::amperes_per_turn_per_second_squared_t kA = //0.6
        1.68*launcherMOI/launcherMotor.Kt/1_tr;
    constexpr auto kMotionMagicAcc = 50_tr_per_s_sq;

    constexpr ctre::phoenix6::CANBus canBus{"Drivebase"};
}

std::unique_ptr<ShooterSim> create_shooter_sim(Shooter& shooter);

Shooter::Shooter() : 
    m_flyWheelLeadMotor{ShooterConstants::kShooterFlywheelLeaderID, ShooterConstants::canBus},
    m_flyWheelFollowMotor{ShooterConstants::kShooterFlywheelFollowerID, ShooterConstants::canBus},
    m_feederBreakBeam{ShooterConstants::kfeederBreakBeamID},
    m_sim_state{create_shooter_sim(*this)}
{
    //Shooter PID config
    ctre::phoenix6::configs::TalonFXConfiguration PIDConfig;
    ctre::phoenix6::configs::MotionMagicConfigs MMConfig;

    MMConfig.MotionMagicAcceleration = ShooterConstants::kMotionMagicAcc;


    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = ShooterConstants::kP; 
    slot0Configs.kI = ShooterConstants::kI; 
    slot0Configs.kD = ShooterConstants::kD; 
    slot0Configs.kA = ShooterConstants::kA.value();

    PIDConfig.WithSlot0(slot0Configs);

    m_flyWheelLeadMotor.GetConfigurator().Apply(PIDConfig);  
    m_flyWheelLeadMotor.GetConfigurator().Apply(MMConfig);
    m_flyWheelFollowMotor.SetControl(ctre::phoenix6::controls::Follower
        {ShooterConstants::kShooterFlywheelLeaderID, true});

    //Shooter Voltage config
    ctre::phoenix6::configs::VoltageConfigs VoltConfig;
    VoltConfig.PeakForwardVoltage = 12_V;
    VoltConfig.PeakReverseVoltage = 0_V;
    m_flyWheelLeadMotor.GetConfigurator().Apply(VoltConfig);

    //Shooter Torque config
    ctre::phoenix6::configs::TorqueCurrentConfigs TorqueConfig;
    TorqueConfig.PeakForwardTorqueCurrent = ShooterConstants::PeakForwardTorqueCurrent;
    TorqueConfig.PeakReverseTorqueCurrent = ShooterConstants::PeakReverseTorqueCurrent;
    m_flyWheelLeadMotor.GetConfigurator().Apply(TorqueConfig);

    InitializeDashboard();
}

Shooter::~Shooter() {

}

void Shooter::InitializeDashboard() {
    auto put_cmd = [this] (std::string_view name, frc2::CommandPtr&& cmd) {
        frc::SmartDashboard::PutData(fmt::format("Shooter/{}", name),
            std::move(cmd).WithName(name).Unwrap().release()
        );
    };

    frc::SmartDashboard::PutNumber("Shooter/SetLauncherRPM", 0.0);
    put_cmd("SetLauncher", Run([this] {
        const auto dashboard_rpm = 
            frc::SmartDashboard::GetNumber("Shooter/SetLauncherRPM", 0.0)*1.0_rpm;
        SetFlywheelSpeedNRM(dashboard_rpm);
    }));
}

void Shooter::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Shooter/LauncherRPM",
        units::revolutions_per_minute_t{GetCurrentFlywheelSpeed()}.value()
    );
    
    frc::SmartDashboard::PutBoolean("Shooter/SpunUp",
        isAtCorrectSpeed()
    );
}

void Shooter::Periodic() {
    UpdateDashboard();
}

frc2::CommandPtr Shooter::SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity) {
    return RunEnd(
        [this, velocity] {SetFlywheelSpeedNRM(velocity);},
        [this] {m_flyWheelLeadMotor.StopMotor();});
}

//RunEnd
frc2::CommandPtr Shooter::SetFlywheelSpeedAndHoodPosParallel(
    units::angular_velocity::turns_per_second_t velocity,
    double point) {
    return SetFlywheelSpeed(velocity).AlongWith(SetHoodPosition(point));
}

frc2::CommandPtr Shooter::SetHoodPosition(double point) {
    return m_hoodActuator.SetPosition(point);}

frc2::CommandPtr Shooter::SetHoodPositionUntilThere(double point) {
    return m_hoodActuator.SetPositionUntilThere(point);}

frc2::CommandPtr Shooter::SetHoodPositionMin() {
    return m_hoodActuator.SetPosition(0);
}

frc2::CommandPtr Shooter::SetHoodPositionRelative(double change) {
    return m_hoodActuator.RelativePositionChange(change);
}

bool Shooter::isHoodAtPos() {
    return m_hoodActuator.isLinearActuatorAtPos();}

//********************** Private **********************/

//side-effect of setting targetVelocity (read only)
void Shooter::SetFlywheelSpeedNRM(units::angular_velocity::turns_per_second_t velocity) {
    // m_flyWheelLeadMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage(velocity).WithSlot(0)); //Voltage
    m_flyWheelLeadMotor.SetControl(ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC(velocity).WithSlot(0)); //Torque
    targetVelocity = velocity;
}

units::angular_velocity::turns_per_second_t Shooter::GetCurrentFlywheelSpeed() {
    return m_flyWheelLeadMotor.GetVelocity().GetValue();
}


bool Shooter::isAtCorrectSpeed() {
    return targetVelocity - ShooterConstants::kSpeedTolerance < GetCurrentFlywheelSpeed() &&
        targetVelocity + ShooterConstants::kSpeedTolerance > GetCurrentFlywheelSpeed();
}

bool Shooter::IsBBBroken() {return m_feederBreakBeam.Get();}

    //**************************** Simulation ****************************/

class ShooterSim {
public:
    ShooterSim(Shooter &shooter);

public:
    // physics models
    frc::sim::FlywheelSim m_launcherPhysics;

    enum {kLower = 0, kUpper = 1};
    frc::sim::FlywheelSim m_feederPhysics[2];

    // sim state objects
    ctre::phoenix6::sim::TalonFXSimState m_launcherMotorState;
};

std::unique_ptr<ShooterSim> create_shooter_sim(Shooter &shooter) {
    if constexpr (frc::RobotBase::IsSimulation()) {
        return std::make_unique<ShooterSim>(shooter);
    } else {
        return nullptr;
    }
}

auto make_feeder_sim() {
    return frc::sim::FlywheelSim{
        frc::LinearSystemId::FlywheelSystem(
            frc::DCMotor::NeoVortex(1),
            ShooterConstants::feederMOI,
            ShooterConstants::feederGearing),
        frc::DCMotor::NeoVortex(1)
    };
}

ShooterSim::ShooterSim(Shooter& shooter) :
    m_launcherPhysics{
        frc::LinearSystemId::FlywheelSystem(
            frc::DCMotor::KrakenX60FOC(2),
            ShooterConstants::launcherMOI,
            ShooterConstants::launcherGearing),
        frc::DCMotor::KrakenX60FOC(2)},
    m_feederPhysics{make_feeder_sim(), make_feeder_sim()},
    m_launcherMotorState{shooter.m_flyWheelLeadMotor}
{
}

void Shooter::SimulationPeriodic() {
    if (!m_sim_state) return;

    const auto supply_voltage = frc::RobotController::GetInputVoltage()*1_V;

    m_sim_state->m_launcherMotorState.SetSupplyVoltage(supply_voltage);

    //update flywheel physics sim
    m_sim_state->m_launcherPhysics.SetInputVoltage(
        m_sim_state->m_launcherMotorState.GetMotorVoltage());
        
    m_sim_state->m_launcherPhysics.Update(20_ms);

    //update launcher motor
    m_sim_state->m_launcherMotorState.SetRotorVelocity(
        m_sim_state->m_launcherPhysics.GetAngularVelocity() * ShooterConstants::launcherGearing);
    m_sim_state->m_launcherMotorState.SetRotorAcceleration(m_sim_state->m_launcherPhysics.GetAngularAcceleration());
}