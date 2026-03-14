#include "swerve/SwerveModule.h"

//#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/filter/LinearFilter.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <iostream>
#include <numbers>
#include <random>

namespace ModuleConstants {

using namespace ctre::phoenix6;

// Motor outputs under 4% will just be cut to 0 (brake)
constexpr double kNeutralDeadband = 0.04;

// Current Limit configs
constexpr auto kDriveMotorCurrentLimit = 70_A;
constexpr auto kDriveMotorSustainedCurrentLimit = 30_A;
constexpr auto kDriveMotorCurrentLimitTime = 0.25_s;
constexpr auto kSlipCurrent = 80_A;
constexpr auto kSteerMotorStatorCurrentLimit = 30_A;
// Can exceed limit for 40ms seconds
constexpr auto kCurrentLimitPeriod = 40_ms;

constexpr auto kWheelDiameter = 4_in;

constexpr double kDriveEncoderReduction = 5.9;     // reduction in drive motor
// Linear distance per revolution of motor
constexpr auto kDriveEncoderDistancePerRevolution = 
  kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;

  // calculated based on a weight of 70lbs
constexpr auto kWheelMoment = .0101_kg_sq_m;
constexpr auto kMotorSpeedChoreo = 5104_rpm; // choreo value
constexpr auto kMotorSpeed = 6000_rpm;       // Website value
constexpr auto kDriveMaxAcceleration = 500_tr_per_s_sq;
constexpr auto kDriveTargetAcceleration = 200_tr_per_s_sq;
constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 18.75;

// Reduced to near 0-mass for smooth sim driving
constexpr auto kSteerMoment = 0.0001_kg_sq_m;

// Measured empirically, rough guess
constexpr auto kSteerAcceleration = 135.7_tr_per_s_sq * 2;
constexpr auto kSteerSpeed = kMotorSpeed / kSteerGearReduction;

constexpr auto kDriveV = 12.0_V/units::turns_per_second_t{kPhysicalMaxSpeed/kDistanceToRotations};
constexpr auto kDriveGains = configs::Slot0Configs{}
  .WithKP(0)
  .WithKI(0.0)
  .WithKD(0)
  .WithKV(kDriveV.value())
;

constexpr auto kSteerV = 12.0_V/kSteerSpeed;
constexpr auto kSteerA = 12.0_V/kSteerAcceleration;
constexpr auto kSteerGains = configs::Slot0Configs{}
  .WithKP(120)
  .WithKI(0)
  .WithKD(0.24)
  .WithKS(0.36)
  .WithKV(kSteerV.value())
  .WithKA(kSteerA.value())
;

const auto MotorModel = [](int N = 1) { return frc::DCMotor::KrakenX60FOC(N); };

constexpr ctre::phoenix6::CANBus kBus{"Drivebase"};

} // namespace ModuleConstants

using namespace ModuleConstants;

namespace p6 = ctre::phoenix6;

class SwerveModuleSim {
public:
  SwerveModuleSim(SwerveModule& module)
    : m_driveSim(std::move(module.m_driveMotor.GetSimState())),
    m_steerSim(std::move(module.m_steerMotor.GetSimState())),
    m_encoderSim(std::move(module.m_absoluteEncoder.GetSimState())),
    m_wheelModel(frc::LinearSystemId::DCMotorSystem(
      MotorModel(), kWheelMoment, kDriveEncoderReduction),
      MotorModel()),
    m_swivelModel(frc::LinearSystemId::DCMotorSystem(
      MotorModel(), kSteerMoment, kSteerGearReduction),
      MotorModel()) {
    static std::random_device rng;
    std::uniform_real_distribution dist(-0.5, 0.5);

    // randomize starting positions
    m_swivelModel.SetState(dist(rng) * 1_tr, 0_rpm);
  }

  void update();

private:
  // hooks to hardware abstraction layer

  ctre::phoenix6::sim::TalonFXSimState m_driveSim, m_steerSim;
  ctre::phoenix6::sim::CANcoderSimState m_encoderSim;

  // tracks the simulation state for each wheel
  frc::sim::DCMotorSim m_wheelModel, m_swivelModel;
};

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
  const int steerMotorId, const int absoluteEncoderId)
  : m_name{name}, m_CANBusInstance("Drivebase"), m_driveMotor(driveMotorId, m_CANBusInstance),
  m_steerMotor(steerMotorId, m_CANBusInstance),
  m_absoluteEncoder(absoluteEncoderId, m_CANBusInstance),
  m_signals{m_driveMotor.GetPosition(), m_driveMotor.GetVelocity(),
            m_steerMotor.GetPosition(), //< FusedCANCoder
            m_steerMotor.GetVelocity()},
  m_sim_state(new SwerveModuleSim(*this)) {

  // Reduce clutter in this function
  using namespace ctre::phoenix6;
  using namespace units;

  configs::TalonFXConfiguration steerConfig, driveConfig;

  steerConfig.WithMotorOutput(configs::MotorOutputConfigs{}
    .WithNeutralMode(signals::NeutralModeValue::Brake)
    .WithInverted(true)
  );

  driveConfig.WithMotorOutput(configs::MotorOutputConfigs{}
    .WithNeutralMode(signals::NeutralModeValue::Brake)
    .WithDutyCycleNeutralDeadband(kNeutralDeadband)
  );
  
  driveConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
    .WithStatorCurrentLimit(kSlipCurrent)
    .WithSupplyCurrentLimit(kDriveMotorCurrentLimit)
    .WithSupplyCurrentLowerLimit(kDriveMotorSustainedCurrentLimit)
    .WithSupplyCurrentLowerTime(kDriveMotorCurrentLimitTime)
  );

  steerConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
    .WithStatorCurrentLimit(kSteerMotorStatorCurrentLimit)
  );

  driveConfig.WithSlot0(kDriveGains);
  steerConfig.WithSlot0(kSteerGains);

  driveConfig.WithMotionMagic(configs::MotionMagicConfigs{}
    .WithMotionMagicAcceleration(kDriveTargetAcceleration)
  );

  steerConfig.WithMotionMagic(configs::MotionMagicConfigs{}
    .WithMotionMagicCruiseVelocity(kSteerSpeed)
    .WithMotionMagicAcceleration(kSteerAcceleration)
    .WithMotionMagicExpo_kV(kSteerV)
    .WithMotionMagicExpo_kA(kSteerA)
  );

  steerConfig.WithClosedLoopGeneral(configs::ClosedLoopGeneralConfigs{}
    .WithContinuousWrap(true)
  );

  steerConfig.WithFeedback(configs::FeedbackConfigs{}
    .WithFeedbackSensorSource(
      signals::FeedbackSensorSourceValue::FusedCANcoder)
    .WithFeedbackRemoteSensorID(m_absoluteEncoder.GetDeviceID())
    .WithRotorToSensorRatio(kSteerGearReduction)
    .WithSensorToMechanismRatio(1.0)
  );

  /* Sometimes configuration fails, so we check the return code
   * and retry if needed.
   */
  int retries = 4;
  while (auto ret =
    m_driveMotor.GetConfigurator().Apply(driveConfig, 500_ms)) {
    if (retries-- == 0) {
      // when ret is non-zero, that means there's an error
      std::cerr << "ERROR Applying Drive Motor Configs for " << m_name
        << std::endl;
      std::cerr << "Talon ID: " << driveMotorId << ", Error: " << ret
        << std::endl;
      break;
    }
  }

  retries = 4;
  while (auto ret =
    m_steerMotor.GetConfigurator().Apply(steerConfig, 500_ms)) {
    if (retries-- == 0) {
      std::cerr << "ERROR Applying Steer Motor Configs for " << m_name
        << std::endl;
      std::cerr << "Talon ID: " << steerMotorId << ", Error: " << ret
        << std::endl;
      break;
    }
  }

  frc::DataLogManager::Log(
    fmt::format("Finished initializing {} swerve module", m_name));
}

SwerveModule::~SwerveModule() {}

void SwerveModule::RefreshSignals() {
  /* Refreshes all this modules signals at once.
   * This should improve performance
   */
  ctre::phoenix6::BaseStatusSignal::RefreshAll(
    m_signals.m_drivePosition, m_signals.m_driveVelocity,
    m_signals.m_steerPosition, m_signals.m_steerVelocity);
}

units::meter_t SwerveModule::SignalGroup::GetModuleDistance() {
  const auto position =
    ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      m_drivePosition, m_driveVelocity);
  return position * kDistanceToRotations;
}

units::meters_per_second_t SwerveModule::SignalGroup::GetModuleVelocity() {
  return m_driveVelocity.GetValue() * kDistanceToRotations;
}

frc::Rotation2d SwerveModule::SignalGroup::GetModuleHeading() {
  const auto position = p6::BaseStatusSignal::GetLatencyCompensatedValue(
    m_steerPosition, m_steerVelocity);
  return position.convert<units::degree>();
}

frc::SwerveModulePosition SwerveModule::SignalGroup::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::SignalGroup::GetState() {
  return {GetModuleVelocity(), GetModuleHeading()};
}

void SwerveModule::CoastMode(bool coast) {
  if (coast) {
    m_steerMotor.SetNeutralMode(p6::signals::NeutralModeValue::Coast);
    m_driveMotor.SetNeutralMode(p6::signals::NeutralModeValue::Coast);
  } else {
    m_steerMotor.SetNeutralMode(p6::signals::NeutralModeValue::Brake);
    m_driveMotor.SetNeutralMode(p6::signals::NeutralModeValue::Brake);
  }
}

void SwerveModule::SyncEncoders() {
  m_steerMotor.SetPosition(
    m_absoluteEncoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::SetDesiredState(
  const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to prevent the module turning >90 degrees.
  auto state = referenceState;
  state.Optimize(GetModuleHeading());
  state.speed *= (state.angle - GetModuleHeading()).Cos();

  m_driveMotor.SetControl(p6::controls::MotionMagicVelocityVoltage{
      state.speed / kDistanceToRotations}
      .WithEnableFOC(true)
    .WithSlot(0));

  m_steerMotor.SetControl(
    p6::controls::MotionMagicExpoVoltage{state.angle.Radians()}
    .WithEnableFOC(true)
    .WithSlot(0));
}

void SwerveModule::UpdateDashboard() {
  const auto state = GetState();

  frc::SmartDashboard::PutNumber(
    fmt::format("Swerve/{}/heading (degrees)", m_name),
    state.angle.Degrees().value());

  frc::SmartDashboard::PutNumber(fmt::format("Swerve/{}/speed (mps)", m_name),
    state.speed.convert<units::mps>().value());
  
  // frc::SmartDashboard::PutNumber(fmt::format("Swerve/{}/drive supply current (amps)", m_name),
  //   m_driveMotor.GetSupplyCurrent().GetValue().value());
  // frc::SmartDashboard::PutNumber(fmt::format("Swerve/{}/steer supply current (amps)", m_name),
  //   m_steerMotor.GetSupplyCurrent().GetValue().value());
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return m_absoluteEncoder.GetAbsolutePosition().GetValue();
}

// Simulation
void SwerveModule::SimulationPeriodic() {
  if (m_sim_state)
    m_sim_state->update();
}

void SwerveModuleSim::update() {
  m_driveSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_steerSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_encoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  // Simulate the wheel swiveling
  m_swivelModel.SetInputVoltage(m_steerSim.GetMotorVoltage());
  m_swivelModel.Update(20_ms);
  // cancoder is on mechanism and is inverted from the falcon's rotor
  m_encoderSim.SetRawPosition(-m_swivelModel.GetAngularPosition());
  m_encoderSim.SetVelocity(-m_swivelModel.GetAngularVelocity());
  m_steerSim.SetRawRotorPosition(m_swivelModel.GetAngularPosition() *
    kSteerGearReduction);
  m_steerSim.SetRotorVelocity(m_swivelModel.GetAngularVelocity() *
    kSteerGearReduction);
  m_steerSim.SetRotorAcceleration(m_swivelModel.GetAngularAcceleration() *
    kSteerGearReduction);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorVoltage());
  m_wheelModel.Update(20_ms);

  m_driveSim.SetRawRotorPosition(m_wheelModel.GetAngularPosition() *
    kDriveEncoderReduction);
  m_driveSim.SetRotorVelocity(m_wheelModel.GetAngularVelocity() *
    kDriveEncoderReduction);
  m_driveSim.SetRotorAcceleration(m_wheelModel.GetAngularAcceleration() *
    kDriveEncoderReduction);
}