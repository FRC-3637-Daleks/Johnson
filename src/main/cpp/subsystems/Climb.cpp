#include "subsystems/Climb.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/RobotController.h>
#include <numbers>

namespace ClimbConstants{

    int kMotorID = 14;

    constexpr auto kSprocketCircum = std::numbers::pi*1_in;
    constexpr auto kGearReduction = 3*4*4;

    constexpr auto kMinHeight = 22_in;
    constexpr auto kMaxHeight = 29_in;
    constexpr auto kClimbHeight = 24_in;
    constexpr auto kFirstStageLength =
      (kMaxHeight - kMinHeight);
  
    constexpr units::length::centimeter_t goal_heights[] = {kMinHeight, kClimbHeight, kMaxHeight};
    constexpr std::string_view goal_names[] = {"Bottom", "Climbed", "Top"};

    constexpr units::length::centimeter_t kTolerance = 1_in;

    constexpr auto kMassEffective = 21.0_kg;

    constexpr auto kBus = ctre::phoenix6::CANBus::RoboRIO();

    constexpr auto kStatorCurrentLimit = 40_A;
    constexpr auto kSupplyCurrentLimit = 30_A;
}

class ClimbSim{
public:
  friend class Climb;

  ClimbSim(Climb  &climb);

  frc::sim::ElevatorSim m_climbModel;

  ctre::phoenix6::sim::TalonFXSimState m_simState; //Climb motor
};




Climb::Climb()
    : m_climbMotor{ClimbConstants::kMotorID, ClimbConstants::kBus},
    m_sim_state{new ClimbSim{*this}}
{
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  
  config.Slot0.kP = 12.0;  
  config.Slot0.kI = 0.0;
  config.Slot0.kD = 0.1;
  
  //config.Slot0.kG = 0.3;  
  config.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  
  config.Feedback.SensorToMechanismRatio = 1.0;

  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = ClimbConstants::kStatorCurrentLimit;
  config.CurrentLimits.SupplyCurrentLimit = ClimbConstants::kSupplyCurrentLimit;

  //needed for sim, dont know the effect on sim
  config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

  config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HeightToRotorTurns(ClimbConstants::goal_heights[static_cast<int>(Height::Top)]);
  config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0_tr;

  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.DutyCycleNeutralDeadband = 0.05;
  
  m_climbMotor.GetConfigurator().Apply(config);

  m_climbMotor.SetPosition(0_tr);
}

Climb::~Climb(){}

void Climb::Periodic(){UpdateDashboard();}

void Climb::InitVisualization(frc::MechanismRoot2d *climb_base) {
  m_hook = climb_base
    ->Append<frc::MechanismLigament2d>("climb_base",
      units::foot_t{ClimbConstants::kMinHeight - 4_in}.value(), 90_deg, 40, frc::Color::kGray)
    ->Append<frc::MechanismLigament2d>("extender",
      0, 0_deg, 30, frc::Color::kSilver);
  
  m_hook->Append<frc::MechanismLigament2d>("hook",
    0.2, 90_deg, 15, frc::Color::kTeal);
}

void Climb::UpdateDashboard(){
  if (m_hook) {
    m_hook->SetLength(units::foot_t{GetPosition() - ClimbConstants::kMinHeight}.value());
  }

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

frc2::CommandPtr Climb::Deploy(){
  return GoToHeight(Height::Top);
}

frc2::CommandPtr Climb::LiftBot(){
  return GoToHeight(Height::Climbed);
}

frc2::CommandPtr Climb::Retract(){
  return BlindDown().Until([this] {return units::math::abs(m_climbMotor.GetVelocity().GetValue()) < 5_tps;});
}

frc2::CommandPtr Climb::BlindUp() {
  return RunEnd(
    [this] {m_climbMotor.Set(0.5);},
    [this] {m_climbMotor.StopMotor();}
  );
}

frc2::CommandPtr Climb::BlindDown() {
  return RunEnd(
    [this] {m_climbMotor.Set(-0.5);},
    [this] {m_climbMotor.StopMotor();}
  );
}


//********************SIMULATION********************* 

ClimbSim::ClimbSim(Climb &climb):
      m_climbModel{
        frc::DCMotor::KrakenX60FOC(1),
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
  if(!m_sim_state) return;

  auto &m_climbModel = m_sim_state->m_climbModel;
  auto &m_simState = m_sim_state->m_simState;
    
  m_simState.SetSupplyVoltage(12_V); //Prob gana be 12_V but should use frc::RobotController::GetBatteryVoltage()

  m_climbModel.SetInputVoltage(-m_simState.GetMotorVoltage());

  m_climbModel.Update(20_ms);

  constexpr auto rotor_turns_per_climb_height =
    ClimbConstants::kGearReduction * 1_tr / 
    ClimbConstants::kSprocketCircum;

  const auto position = m_climbModel.GetPosition();
  const units::turn_t rotor_turns = position * rotor_turns_per_climb_height;

  const auto velocity = m_climbModel.GetVelocity();
  const units::turns_per_second_t rotor_velocity = 
    velocity * rotor_turns_per_climb_height;
  
  m_simState.SetRawRotorPosition(-rotor_turns);
  m_simState.SetRotorVelocity(-rotor_velocity);
  
  // Publishing data to NetworkTables
  frc::SmartDashboard::PutNumber("Climb/Sim Position (m)",
                                units::meter_t{position}.value());
  frc::SmartDashboard::PutNumber("Climb Voltage", 1);
}