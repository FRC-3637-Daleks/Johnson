#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkFlex.h>
#include <frc/DigitalInput.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <wpi/sendable/SendableBuilder.h>

#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include <wpi/interpolating_map.h>
#include <frc/EigenCore.h>

#include "subsystems/shooter/LinearActuator.h"

struct ShooterSetpoint: frc::Vectord<2> {
    using shooter_velocity_t = units::turns_per_second_t;
    using hood_t = units::millimeter_t;

    ShooterSetpoint(shooter_velocity_t vel, hood_t hood):
        frc::Vectord<2>{vel.value(), hood.value()} 
    {}

    shooter_velocity_t Velocity() const {
        return shooter_velocity_t{(*this)[0]};
    }
    
    hood_t Hood() const {
        return hood_t{(*this)[1]};
    }
};

class ShooterSim;

class Shooter : public frc2::SubsystemBase{
public:
    Shooter();
    ~Shooter();

public:
    void InitializeDashboard();
    void UpdateDashboard();
    void InitVisualization(frc::MechanismRoot2d* shooter_base);

    void Periodic() override;

public:
    frc2::CommandPtr AimFromHUB();
    frc2::CommandPtr AimFromTrench();
    frc2::CommandPtr AimFromTower();

    // Spins up to the lowest setpoint to save time
    frc2::CommandPtr SpinUp();

    // Cuts power to motor to save energy
    frc2::CommandPtr SpinDown();

    // Retracts hood to allow passing under bump
    frc2::CommandPtr RetractHood() {return SetHoodPositionMin();};

    frc2::CommandPtr CycleHopper();

    frc2::CommandPtr AutoAdjust(std::function<units::inch_t()> distanceFunc);

public:
    frc2::CommandPtr SetFlywheelSpeedAndHoodPosParallel(
        const ShooterSetpoint &setpoint
    );
    
    frc2::CommandPtr SetFlywheelSpeed(units::angular_velocity::turns_per_second_t velocity);
    frc2::CommandPtr SetFlywheelSpeed(std::function<units::inch_t()> distanceFunc);

    //Point in mm
    frc2::CommandPtr SetHoodPosition(double point);
    frc2::CommandPtr SetHoodPosition(std::function<units::inch_t()> distanceFunc);

    //Point in mm
    frc2::CommandPtr SetHoodPositionUntilThere(double point);
    frc2::CommandPtr SetHoodPositionMin();
    frc2::CommandPtr SetHoodPositionRelative(double change);
    bool isHoodAtPos();
    bool isAtCorrectSpeed();
    bool readyToFire() {return isHoodAtPos() && isAtCorrectSpeed();}


private:

    wpi::interpolating_map<units::inch_t, ShooterSetpoint> m_distance_to_shots{};


private:

    void SetFlywheelSpeedNRM(units::angular_velocity::turns_per_second_t velocity);
    units::angular_velocity::turns_per_second_t targetVelocity{}; //read only
    units::angular_velocity::turns_per_second_t GetCurrentFlywheelSpeed();

    bool IsBBBroken(); 

    LinearActuator m_hoodActuator;

private:
    ctre::phoenix6::hardware::TalonFX m_flyWheelLeadMotor;
    ctre::phoenix6::hardware::TalonFX m_flyWheelFollowMotor;

    frc::DigitalInput m_feederBreakBeam;

private: // visualization
    frc::MechanismLigament2d *m_wheel, *m_hood;

// simulation
private:
    friend class ShooterSim;
    std::unique_ptr<ShooterSim> m_sim_state;

    void SimulationPeriodic() override;
};