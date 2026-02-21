#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkFlex.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/DigitalInput.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <wpi/sendable/SendableBuilder.h>

class FeederSim; //forward declarations

class Feeder : public frc2::SubsystemBase{
public:

    enum class Type {
        Top,
        Bottom
    };

    Feeder(Type type); //get the motor ID from constants and pass it in here
    ~Feeder();

public:
    void InitializeDashboard();
    void UpdateDashboard();
    void Periodic() override;

public:

    frc2::CommandPtr setRPM(double RPM);
    frc2::CommandPtr setRPMUntilThere(double RPM);
    double getRPM();

private:

    void setVelocity(double RPM);
    bool isAtRPM(double RPM);

private:

    rev::spark::SparkFlex m_feederMotor;
    rev::spark::SparkClosedLoopController m_pidController;

    double targetRPM = 0;

// simulation
private:
    friend class FeederSim;
    std::unique_ptr<FeederSim> m_sim_state;

    void SimulationPeriodic() override;

    inline static int classIndex = 0; //should only be init once
};