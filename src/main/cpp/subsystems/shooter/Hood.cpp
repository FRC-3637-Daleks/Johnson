#include "subsystems/shooter/Hood.h"
#include <algorithm>

namespace HoodConstants {
    int LinearActuatorID = 15;

    units::time::microsecond_t max =            2.0_ms;
    units::time::microsecond_t deadbandMax =    1.9_ms;
    units::time::microsecond_t center =         1.5_ms;
    units::time::microsecond_t deadbandMin =    1.1_ms;
    units::time::microsecond_t Min =            1.0_ms;

    //Percent of allowed movement that it moves a sec
    double movementPerSecond = 0.1; 

    double tolerance = 0.01; //As a percentage
}

Hood::Hood() :
    actuator(HoodConstants::LinearActuatorID)
{
    actuator.SetBounds(HoodConstants::max, HoodConstants::deadbandMax, HoodConstants::center,
        HoodConstants::deadbandMin, HoodConstants::Min);
        
    m_timer.Start();
    lastTime = m_timer.Get();
}

Hood::~Hood() {}

void Hood::Periodic() {
    CalculateTravelPeriodic();
}

frc2::CommandPtr Hood::SetPosPercent(double percent) {
    return RunOnce([this, percent] {SetPos(percent);});
}

frc2::CommandPtr Hood::SetPosPercentUntilThere(double percent) {
    return RunOnce([this, percent] { SetPos(percent); })
           .AndThen(Run([this]{ /* Waiting */ }).Until([this] { return !isMoving; }));
}

bool Hood::isHoodAtPos() {
    return std::abs(percentTarget - percentEstimate) < HoodConstants::tolerance;
}

//********************** Private **********************/

void Hood::SetPos(double percent) {
    isMoving = true && !isHoodAtPos(); //prevents race condition were this runs before periodic and make .until hang

    percentTarget = std::clamp(percent, 0.0, 1.0); 
    actuator.Set(percentTarget);
}


void Hood::CalculateTravelPeriodic() {
    units::time::second_t newTime = m_timer.Get();
    units::time::second_t timeGap = newTime - lastTime;
    lastTime = newTime;

    if (isMoving) {
        double distMoved = HoodConstants::movementPerSecond * timeGap.value();
        int direction = (percentTarget - percentEstimate >= 0) ? 1 : -1;
        double vectorMoved = distMoved * direction;

        if (distMoved >= std::abs(percentTarget - percentEstimate)) { //prevent overshooting
            percentEstimate = percentTarget; //eliminate error
            isMoving = false;
        } else {
            percentEstimate += vectorMoved;
        }
    }

    //redundent/backup
    if (isHoodAtPos()) {
            isMoving = false;
            percentEstimate = percentTarget; //eliminate error build up
    }
}