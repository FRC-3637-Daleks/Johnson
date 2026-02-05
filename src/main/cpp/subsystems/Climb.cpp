#include "subsystems/Climb.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/RobotController.h>
#include <numbers>

namespace ClimbConstants{

    int kMotorID = 14;
    
    constexpr auto kSprocketTeeth = 22;
    constexpr auto kDistancePerChainLink = 0.25_in; 
    constexpr auto kSprocketCircum = kSprocketTeeth * kDistancePerChainLink;
    constexpr auto kGearReduction = 62.0 / 10.0 * 30.0 / 22.0;

    constexpr auto kMinHeight = 0_in;
    constexpr auto kMaxHeight = 10_in;
    constexpr auto kFirstStageLength =
    (kMaxHeight - kMinHeight) / 3;
  
    constexpr units::length::centimeter_t goal_heights[] = {kMinHeight, kMaxHeight};
    constexpr std::string_view goal_names[] = {"Bottom", "Top"};

    constexpr units::length::centimeter_t kTolerance = 1_in;

    constexpr auto kMassEffective = 21.0_kg;

    constexpr ctre::phoenix6::CANBus kBus{"Drivebase"};
}

class ClimbSim{
public:
  friend class Climb;

  ClimbSim(Climb  &climb);

  frc::sim::ElevatorSim m_climbModel;

  ctre::phoenix6::sim::TalonFXSimState m_simState;
};




Climb::Climb()
    : m_climbMotor{ClimbConstants::kMotorID, ClimbConstants::kBus}  
{}

Climb::~Climb(){}

void Climb::Periodic(){UpdateDashboard();}

void Climb::UpdateDashboard(){
  frc::SmartDashboard::PutNumber("Climb/Height (in)",
                                 units::inch_t{GetPosition()}.value());
  frc::SmartDashboard::PutNumber(
      "Climb/Height Setpoint (in)",
      units::inch_t{RotorTurnsToHeight(units::turn_t{
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

units::angle::turn_t Climb::HeightToRotorTurns(const units::centimeter_t height) const{
  return (height - ClimbConstants::kMinHeight) / 
          (ClimbConstants::kSprocketCircum) *
          ClimbConstants::kGearReduction * 1_tr;
}

units::centimeter_t Climb::RotorTurnsToHeight(const units::angle::turn_t turns) const{
  
  return ((turns/ ClimbConstants::kGearReduction) * 
          (ClimbConstants::kSprocketCircum / 1_tr) + ClimbConstants::kMinHeight);

}

units::centimeter_t Climb::GetPosition(){
  return RotorTurnsToHeight(m_climbMotor.GetPosition().GetValue());
}


void Climb::SetGoalHeight(const units::centimeter_t length){
  auto request = ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{HeightToRotorTurns(length)}};
  m_climbMotor.SetControl(request);
} 

void Climb::SetGoalHeight(Climb::Height height){
   m_targetHeight = height;
  frc::SmartDashboard::PutString("Climb/Target Level",
                                  ClimbConstants::goal_names[static_cast<int> (height)]);
  SetGoalHeight(ClimbConstants::goal_heights[static_cast<int>(height)]);
}

bool Climb::IsAtHeight(units::length::centimeter_t height){
    return (units::math::abs((height - GetPosition())) <=
          (ClimbConstants::kTolerance));
};

bool Climb::IsAtHeight(Climb::Height height){
    return IsAtHeight(ClimbConstants::goal_heights[static_cast<int>(height)]);
};

//Holds Function Until Climb is at Correct Height
frc2::CommandPtr Climb::GoToHeight(Height goal) {
  return Run([this, goal] { SetGoalHeight(goal); }).Until([this, goal] {
    return IsAtHeight(ClimbConstants::goal_heights[static_cast<int>(goal)]);
  });
}
 
// Does NOT Hold Function Until Climb is at Position
frc2::CommandPtr Climb::ToggleHeight(){ 
    return 
      RunOnce([this]{
        if (m_targetHeight == Climb::Height::Top){
          m_targetHeight = Climb::Height::Bottom;
        }
        else{
          m_targetHeight = Climb::Height::Top;
        }
      }).AndThen(GoToHeight(m_targetHeight));
}

frc2::CommandPtr Climb::Deploy(){
  return GoToHeight(Height::Top);
}

frc2::CommandPtr Climb::LiftBot(){
  return GoToHeight(Height::Bottom);
}

frc2::CommandPtr Climb::Retract(){
  return GoToHeight(Height::Bottom);
}


//********************SIMULATION********************* 

ClimbSim::ClimbSim(Climb &climb):
      m_climbModel{
        frc::DCMotor::KrakenX60FOC(2),
        ClimbConstants::kGearReduction,
        ClimbConstants::kMassEffective,
        ClimbConstants::kSprocketCircum/(2*std::numbers::pi),  // drum radius
        0_m,    // min height
        ClimbConstants::kFirstStageLength,
        true,   // simulate gravity
        0_m     // starting height
      },
      m_simState{climb.m_climbMotor}{}

void Climb::SimulationPeriodic() {
  if(!m_sim_state){}
    return;

  //reduce code clutter
  auto &m_climbModel = m_sim_state->m_climbModel;
  auto &m_simState = m_sim_state->m_simState;
    
  const auto supply_voltage = frc::RobotController::GetBatteryVoltage();
  m_simState.SetSupplyVoltage(supply_voltage);

  //set inputs into model
  m_climbModel.SetInputVoltage(-m_simState.GetMotorVoltage());

  //Simulate model over next 20ms
  m_climbModel.Update(20_ms);

  // The motor turns kGearReduction times to turn the spool once
  // This raises the elevator one spool circumference up
  constexpr auto rotor_turns_per_climb_height =
    ClimbConstants::kGearReduction * 1_tr /
    ClimbConstants::kSprocketCircum;


  // Feed simulated outputs of model back into user program
  const auto position = m_climbModel.GetPosition();
  const units::turn_t rotor_turns = position * rotor_turns_per_climb_height;

  const auto velocity = m_climbModel.GetVelocity();
  const units::turns_per_second_t rotor_velocity =
    velocity * rotor_turns_per_climb_height;

  // mechanically linked, though we should never read this value
  m_simState.SetRawRotorPosition(-rotor_turns);
  m_simState.SetRotorVelocity(-rotor_velocity);

  // Publishing data to NetworkTables
  frc::SmartDashboard::PutNumber("Climb/Sim Position (m)",
                                units::meter_t{position}.value());
  frc::SmartDashboard::PutNumber("Climb Voltage", 1);
}