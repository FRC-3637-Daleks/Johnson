#include <frc/DigitalInput.h>
#include <frc/servo.h>
#include <frc/Timer.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <wpi/sendable/SendableBuilder.h>

class LinearActuator  : public frc2::SubsystemBase {
public:
    LinearActuator();
    ~LinearActuator();

public:
    void InitializeDashboard();
    void UpdateDashboard();

    void Periodic() override;

public:
    //percent as range of 0-1 were zero and one are within limits
    frc2::CommandPtr SetPosition(double point);

    frc2::CommandPtr SetPositionUntilThere(double point);

    bool isLinearActuatorAtPos();

private:
    int ClampPoint(double point);

    //percent as range of 0-1 were zero and one are within limits
    void SetPos(double point);

    void CalculateTravelPeriodic();

private:
    frc::Servo actuator;    

    //State managing stuff
    double positionTarget = 0;
    double positionEstimate = 0;
    
};