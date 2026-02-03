#include "subsystems/Shooter.h"

namespace ShooterConstants {
    int kFeederMotorID = 2;

    units::volt_t kFeederInV = 6_V;
    units::volt_t kFeederOutV = -6_V;

    int kShooterFlywheelLeaderID = 3;
    int kShooterFlywheelFollowerID = 4;

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
    m_flyWheelFollowMotor{ShooterConstants::kShooterFlywheelFollowerID, m_CANBusInstance} ,
    m_feederMotor{ShooterConstants::kFeederMotorID, m_CANBusInstance}    
{
    ctre::phoenix6::configs::TalonFXConfiguration m_armConfig;

    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kP = ShooterConstants::kP; 
    slot0Configs.kI = ShooterConstants::kI; 
    slot0Configs.kD = ShooterConstants::kD; 
    slot0Configs.kV = ShooterConstants::kV;

    m_armConfig.WithSlot0(slot0Configs);

    m_flyWheelLeadMotor.GetConfigurator().Apply(m_armConfig);  
    m_flyWheelFollowMotor.SetControl(ctre::phoenix6::controls::Follower
        {ShooterConstants::kShooterFlywheelLeaderID, false});
}

Shooter::~Shooter() {

}

void Shooter::FeederIn() {
    m_feederMotor.SetVoltage(ShooterConstants::kFeederInV);
}

void Shooter::FeederOut() {
    m_feederMotor.SetVoltage(ShooterConstants::kFeederOutV);
}

//side-effect of setting targetVelocity
void Shooter::SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity) {
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