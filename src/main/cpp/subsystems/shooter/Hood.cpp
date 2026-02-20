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
    double movementPerSecond = 0.1; //MUST: (Speed * 0.02) < Tolerance

    double tolerance = 0.01; //As a percentage of 0-1
}

Hood::Hood() :
    actuator(HoodConstants::LinearActuatorID)
{
    actuator.SetBounds(HoodConstants::max, HoodConstants::deadbandMax, HoodConstants::center,
        HoodConstants::deadbandMin, HoodConstants::Min);
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
           .AndThen(Run([this]{ /* Waiting */ }).Until([this] { return isHoodAtPos(); }));
}

bool Hood::isHoodAtPos() {
    return std::abs(percentTarget - percentEstimate) < HoodConstants::tolerance;
}

//********************** Private **********************/

void Hood::SetPos(double percent) {
    //in the future, math here if it needs to be scaled/clamped more than 0-1
    percentTarget = std::clamp(percent, 0.0, 1.0); 
    actuator.Set(percentTarget);
}


void Hood::CalculateTravelPeriodic() {
    //Assumes/Ignores loop overuns, runs @ 20ms
    if (isHoodAtPos()) {
        percentEstimate = percentTarget;
    } else if (percentEstimate < percentTarget) {
        percentEstimate += HoodConstants::movementPerSecond * 0.02;
    } else { //percentEstimate > percentTarget
        percentEstimate -= HoodConstants::movementPerSecond * 0.02;
    }
}