#include "subsystems/Shooter.h"

namespace ShooterConstants {
    int kfeederBreakBeamID = 14;
    int kFeederBottomMotorID = 12;
    int kFeederTopMotorID = 13;

    units::volt_t kFeederInV = 6_V;
    units::volt_t kFeederOutV = -6_V;

    int kShooterFlywheelLeaderID = 10;
    int kShooterFlywheelFollowerID = 11;

    double kP = 2.0;
    double kI = 0.001;
    double kD = 0.1;
    double kV = 1;

    //used just in bool isAtCorrectSpeed()
    units::angular_velocity::turns_per_second_t kSpeedTolerance = 0.3_tps;
}

Shooter::Shooter() : 
    m_CANBusInstance{"Drivebase"},
    m_flyWheelLeadMotor{ShooterConstants::kShooterFlywheelLeaderID, m_CANBusInstance},
    m_flyWheelFollowMotor{ShooterConstants::kShooterFlywheelFollowerID, m_CANBusInstance},
    m_feederBreakBeam{ShooterConstants::kfeederBreakBeamID},
    m_feederBottomMotor{ShooterConstants::kFeederBottomMotorID, m_CANBusInstance},
    m_feederTopMotor{ShooterConstants::kFeederTopMotorID, m_CANBusInstance}    
{
    //Shooter PID config
    ctre::phoenix6::configs::TalonFXConfiguration PIDConfig;

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = ShooterConstants::kP; 
    slot0Configs.kI = ShooterConstants::kI; 
    slot0Configs.kD = ShooterConstants::kD; 
    slot0Configs.kV = ShooterConstants::kV;

    PIDConfig.WithSlot0(slot0Configs);

    m_flyWheelLeadMotor.GetConfigurator().Apply(PIDConfig);  
    m_flyWheelFollowMotor.SetControl(ctre::phoenix6::controls::Follower
        {ShooterConstants::kShooterFlywheelLeaderID, false});

    //Shooter Voltage config
    ctre::phoenix6::configs::VoltageConfigs VoltConfig;

    VoltConfig.PeakForwardVoltage = 12_V;
    VoltConfig.PeakReverseVoltage = 0_V;
}

Shooter::~Shooter() {

}

frc2::CommandPtr Shooter::SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity) {
    return Run([this, velocity] {SetFlywheelSpeedNRM(velocity);});
}

frc2::CommandPtr Shooter::FeederBottomIn() {
    return RunEnd([this] {FeederBottomInNRM();}, [this] {FeederBottomStopNRM();});}

frc2::CommandPtr Shooter::FeederBottomOut() {
    return RunEnd([this] {FeederBottomInNRM();}, [this] {FeederBottomStopNRM();});}

frc2::CommandPtr Shooter::FeederTopIn() {
    return RunEnd([this] {FeederBottomInNRM();}, [this] {FeederBottomStopNRM();});}

frc2::CommandPtr Shooter::FeederTopOut() {
    return RunEnd([this] {FeederBottomInNRM();}, [this] {FeederBottomStopNRM();});}

//********************** Private **********************/

//side-effect of setting targetVelocity (read only)
void Shooter::SetFlywheelSpeedNRM(units::angular_velocity::turns_per_second_t velocity) {
    m_flyWheelLeadMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage(velocity).WithSlot(0));
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

void Shooter::FeederBottomInNRM() {
    m_feederBottomMotor.SetVoltage(ShooterConstants::kFeederInV);}

void Shooter::FeederBottomOutNRM() {
    m_feederBottomMotor.SetVoltage(ShooterConstants::kFeederOutV);}

void Shooter::FeederBottomStopNRM() {
    m_feederBottomMotor.SetVoltage(0_V);}

void Shooter::FeederTopInNRM() {
    m_feederTopMotor.SetVoltage(ShooterConstants::kFeederInV);}

void Shooter::FeederTopOutNRM() {
    m_feederTopMotor.SetVoltage(ShooterConstants::kFeederOutV);}

void Shooter::FeederTopStopNRM() {
    m_feederTopMotor.SetVoltage(0_V);}