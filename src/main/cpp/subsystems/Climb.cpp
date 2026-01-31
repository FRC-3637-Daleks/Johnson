#include "subsystems/Climb.h"
#include <frc/smartdashboard/SmartDashboard.h>

namespace ClimbConstants{

    int kMotorID = 1;
    
    constexpr auto kSprocketTeeth = 22;
    constexpr auto kDistancePerChainLink = 0.25_in; 
    constexpr auto kSprocketCircum = kSprocketTeeth * kDistancePerChainLink;
    constexpr auto kGearReduction = 62.0 / 10.0 * 30.0 / 22.0;

    constexpr auto kMinHeight = 1_in;
    
    constexpr units::length::centimeter_t kBottom = 0_in;
    constexpr units::length::centimeter_t kTop = 10_in;

    constexpr units::length::centimeter_t goal_heights[] = {kBottom, kTop};
    constexpr std::string_view goal_names[] = {"Bottom", "Top"};

    constexpr units::length::centimeter_t kTolerance = 1_in;

    constexpr ctre::phoenix6::CANBus kBus{"Drivebase"};
}

Climb::Climb()
    : m_climbMotor{ClimbConstants::kMotorID, ClimbConstants::kBus}  
{}

Climb::~Climb(){}

void Climb::Periodic(){UpdateDashboard();}

void Climb::UpdateDashboard(){
  frc::SmartDashboard::PutNumber("Climb/Height (in)",
                                 units::inch_t{GetHeight()}.value());
  frc::SmartDashboard::PutNumber(
      "Climb/Height Setpoint (in)",
      units::inch_t{RotorTurnsToheight(units::turn_t{
                        m_climbMotor.GetClosedLoopReference().GetValue()})}
          .value());
  frc::SmartDashboard::PutNumber(
      "Climb/Output Voltage (V)",
      m_climbMotor.GetMotorVoltage().GetValue().value());

  frc::SmartDashboard::PutNumber(
      "Climb/Stator Current (A)",
      m_climbMotor.GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber(
      "Climb/Supply Current (A)",
      m_climbMotor.GetSupplyCurrent().GetValue().value());
}

units::angle::turn_t Climb::heightToRotorTurns(const units::centimeter_t height){
  return (height - ClimbConstants::kMinHeight) / 
          (3 * ClimbConstants::kSprocketCircum) *
          ClimbConstants::kGearReduction * 1_tr;
}

units::centimeter_t Climb::RotorTurnsToheight(const units::angle::turn_t turns){
  
  return ((turns/ ClimbConstants::kGearReduction) * 
          (3 * ClimbConstants::kSprocketCircum / 1_tr) + ClimbConstants::kMinHeight);

}

units::centimeter_t Climb::GetHeight(){
  return RotorTurnsToheight(m_climbMotor.GetPosition().GetValue());
}


void Climb::SetGoalHeight(const units::centimeter_t length){
  auto request = ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{heightToRotorTurns(length)}};
  m_climbMotor.SetControl(request);
} 

void Climb::SetGoalHeight(Climb::Height height){
  frc::SmartDashboard::PutString("Climb/Target Level",
                                  ClimbConstants::goal_names[height]);
  SetGoalHeight(ClimbConstants::goal_heights[height]);
}

bool Climb::IsAtHeight(units::length::centimeter_t height){
    return (units::math::abs((height - GetHeight())) <=
          (ClimbConstants::kTolerance));
};

bool Climb::IsAtHeight(Climb::Height height){
    return IsAtHeight(ClimbConstants::goal_heights[height]);
};


frc2::CommandPtr Climb::GoToHeight(Height goal) {
  return Run([this, goal] { SetGoalHeight(goal); }).Until([this, goal] {
    return IsAtHeight(ClimbConstants::goal_heights[goal]);
  });
}