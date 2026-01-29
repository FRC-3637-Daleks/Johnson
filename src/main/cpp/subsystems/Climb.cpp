#include "subsystems/Climb.h"




namespace ClimbConstants{

    int kMotorID = 1;
    

    constexpr auto kGearReduction = 62.0 / 10.0 * 30.0 / 22.0;
    
    constexpr units::length::centimeter_t kBottom = 0_in;
    constexpr units::length::centimeter_t kTop = 10_in;

    constexpr units::length::centimeter_t goal_heights[] = {kBottom, kTop};
    constexpr std::string_view goal_names[] = {"Bottom", "Top"};

    constexpr units::length::centimeter_t kTolerance = 1_in;

}



Climb::Climb()
    : m_climbMotor{ClimbConstants::kMotorID, "DriveBase"}  // CANBus instance should be passed here
{

}

Climb::~Climb(){}

void Climb::Periodic(){

}

void Climb::SetGoalHeight(const units::centimeter_t length){

}

void Climb::SetGoalHeight(Climb::Height height){

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