#include "subsystems/shooter/LinearActuator.h"
#include <algorithm>

#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/length.h>

namespace LinearActuatorConstants {
    int LinearActuatorID = 0;

    units::time::microsecond_t max =            2.0_ms;
    units::time::microsecond_t deadbandMax =    1.9_ms;
    units::time::microsecond_t center =         1.5_ms;
    units::time::microsecond_t deadbandMin =    1.1_ms;
    units::time::microsecond_t Min =            1.0_ms;

    //mm
    double length = 55;
    //mm/s
    double speed = 30;

    // Clamps Actuator to not go beyond these points
    double minLength = 05;
    double maxLength = 55; //from 0

    double tolerance = 0.01; //As a percentage of 0-1
}

LinearActuator::LinearActuator() :
    actuator(LinearActuatorConstants::LinearActuatorID)
{
    actuator.SetBounds(LinearActuatorConstants::max, LinearActuatorConstants::deadbandMax, 
        LinearActuatorConstants::center, LinearActuatorConstants::deadbandMin, LinearActuatorConstants::Min);
    SetPos(LinearActuatorConstants::minLength);
    InitializeDashboard();
}

LinearActuator::~LinearActuator() {}

void LinearActuator::Periodic() {
    CalculateTravelPeriodic();
    UpdateDashboard();
}

//RunEnd
frc2::CommandPtr LinearActuator::SetPosition(double point) {
    return RunEnd([this, point] {SetPos(point);},
                  [this] {SetPos(LinearActuatorConstants::minLength);});
}

frc2::CommandPtr LinearActuator::SetPosition(std::function<double()> positionFunc){
    return RunEnd([this, positionFunc] {SetPos(positionFunc());},
                  [this] {SetPos(LinearActuatorConstants::minLength);});
}

//RunOnce
frc2::CommandPtr LinearActuator::SetPositionUntilThere(double point) {
    return RunOnce([this, point] { SetPos(point); })
           .AndThen(Run([this]{ /* Waiting */ }).Until([this] { return isLinearActuatorAtPos(); }));
}

frc2::CommandPtr LinearActuator::RelativePositionChange(double change) {
    return RunOnce([this, change] {
        this->positionTarget += change;
    }).AndThen(SetPosition(positionTarget));
}

bool LinearActuator::isLinearActuatorAtPos() {
    return std::abs(positionEstimate - positionTarget) < LinearActuatorConstants::tolerance; 
}

//********************** Private **********************/

int LinearActuator::ClampPoint(double point) {
    return std::clamp(point, LinearActuatorConstants::minLength, LinearActuatorConstants::maxLength);
}

void LinearActuator::SetPos(double point) {
    //This logic makes sense you you consider its a servo (pulses) not a motor
    positionTarget = ClampPoint(point);
    actuator.SetSpeed((positionTarget/LinearActuatorConstants::length *2)-1);
}

#include <iostream>
void LinearActuator::CalculateTravelPeriodic() {
    //above going down
    if (positionEstimate > positionTarget + (LinearActuatorConstants::speed * 0.02)) { 
        positionEstimate -= LinearActuatorConstants::speed * 0.02;
    } else if (positionEstimate < positionTarget - (LinearActuatorConstants::speed * 0.02)) { //below and up
        positionEstimate += LinearActuatorConstants::speed * 0.02;
    } else { //near pos
        positionEstimate = positionTarget;
    }
}

//Dirty Trick because lazy
namespace { //anonomous namespace accessable only in .cpp, dont make multip
    frc::Mechanism2d m_mech{1, 2};
    frc::MechanismRoot2d* m_rootReal = m_mech.GetRoot("LinearActuatorReal", 1, 0);
    frc::MechanismRoot2d* m_rootTarget = m_mech.GetRoot("LinearActuatorTarget", 0, 0);
    frc::MechanismLigament2d* m_TargetLine =
        m_rootReal->Append<frc::MechanismLigament2d>("TargetLine", 1, 90_deg, 
            6, frc::Color8Bit{frc::Color::kPurple});
    frc::MechanismLigament2d* m_RealLine =
        m_rootTarget->Append<frc::MechanismLigament2d>("RealLine", 1, 90_deg);
}

void LinearActuator::InitializeDashboard() {
    auto put_cmd = [this] (std::string_view name, frc2::CommandPtr&& cmd) {
        frc::SmartDashboard::PutData(fmt::format("Shooter/Hood/{}", name),
            std::move(cmd).WithName(name).Unwrap().release()
        );
    };

    frc::SmartDashboard::PutNumber("Shooter/Hood/SetHoodAngle", 0.0);
    put_cmd("SetHood", Run([this] {
        const units::millimeter_t dashboard_angle{
            frc::SmartDashboard::GetNumber("Shooter/Hood/SetHoodAngle", 0.0)};
        SetPos(dashboard_angle.value());
    }));

    frc::SmartDashboard::PutData("Shooter/LinearActuator", &m_mech);
    frc::SmartDashboard::PutNumber
    ("Shooter/LinearActuator/PosEstimate", -1);
    frc::SmartDashboard::PutNumber
    ("Shooter/LinearActuator/PosTarget", -1);
}

void LinearActuator::UpdateDashboard() {
    m_RealLine->SetLength(positionEstimate/LinearActuatorConstants::length);
    m_TargetLine->SetLength(positionTarget/LinearActuatorConstants::length);

    frc::SmartDashboard::PutNumber
    ("Shooter/LinearActuator/PosEstimate", positionEstimate);
    frc::SmartDashboard::PutNumber
    ("Shooter/LinearActuator/PosTarget", positionTarget);
    frc::SmartDashboard::PutBoolean
    ("Shooter/LinearActuator/IsAtTarget", isLinearActuatorAtPos());
}

