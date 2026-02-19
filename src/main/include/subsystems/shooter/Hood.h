#include <frc/DigitalInput.h>
#include <frc/servo.h>
#include <frc/Timer.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Subsystem.h>
#include <wpi/sendable/SendableBuilder.h>

class Hood  : public frc2::SubsystemBase {
public:
    Hood();
    ~Hood();

public:
    void InitializeDashboard();
    void UpdateDashboard();

    void Periodic() override;

public:
    //percent as range of 0-1 were zero and one are within limits
    frc2::CommandPtr SetPosPercent(double percent);

    frc2::CommandPtr SetPosPercentUntilThere(double percent);

    bool isArmAtPos();

private:
    //percent as range of 0-1 were zero and one are within limits
    void SetPos(double percent);

    void CalculateTravelPeriodic();

private:
    frc::Servo actuator;    

    //State managing stuff
    bool isMoving = false;
    units::time::second_t lastTime = 0_s;
    frc::Timer m_timer;
    double percentTarget = 0;
    double percentEstimate = 0;
    
};