#include "subsystems/shooter/Feeder.h"
#include <rev/config/SparkMaxConfig.h>
#include <rev/sim/SparkFlexSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc/RobotBase.h>


namespace FeederConstants {
    int kTopMotorID = 21;
    int kBottomMotorID = 22;

    double kToleranceRPM = 100; //only used for setVelocityUntilSpeed

    //Top Motor
    units::volt_t TopMotorVoltageComp = 12_V;
    units::ampere_t TopMotorSmartCurrentLimit = 40_A;
    double TopMotor_P = 1.0;
    double TopMotor_I = 0.0001;
    double TopMotor_D = 0.0;
    double TopMotor_FF = 0.1; 

    //Bottom Motor
    units::volt_t BottomMotorVoltageComp = 12_V;
    units::ampere_t BottomMotorSmartCurrentLimit = 40_A;
    double BottomMotor_P = 1.0;
    double BottomMotor_I = 0.0001;
    double BottomMotor_D = 0.0;
    double BottomMotor_FF = 0.1; 

    //For cleaner initialization of feeder class
    struct Perams {
        int motorID;
        units::volt_t VoltComp;
        units::ampere_t SmartCurrLim;
        double kP, kI, kD, kFF;
        
        //True = Top Motor config, False = Bottom Motor config
        Perams(bool isTopMotor) {
            if (isTopMotor) {motorID = kTopMotorID; 
                VoltComp = TopMotorVoltageComp; SmartCurrLim = TopMotorSmartCurrentLimit; 
                kP = TopMotor_P; kI = TopMotor_I; kD = TopMotor_D; kFF = TopMotor_FF;
            } else { motorID = kBottomMotorID; 
                VoltComp = BottomMotorVoltageComp; SmartCurrLim = BottomMotorSmartCurrentLimit; 
                kP = BottomMotor_P; kI = BottomMotor_I; kD = BottomMotor_D; kFF = BottomMotor_FF;
            }
        }
    };

    constexpr auto feederGearing = 1.0;
    constexpr auto feederMOI = 0.001_kg_sq_m;

}

std::unique_ptr<FeederSim> create_feeder_sim(Feeder& feeder);

Feeder::Feeder(FeederConstants::Perams peramConfig) :
    m_feederMotor(peramConfig.motorID, rev::spark::SparkFlex::MotorType::kBrushless),
    m_pidController(m_feederMotor.GetClosedLoopController()),
    m_sim_state{create_feeder_sim(*this)}
{
    classIndex++;

    //velocty control (voltage but with some compensation)
    rev::spark::SparkMaxConfig feederConfig = rev::spark::SparkMaxConfig();

    feederConfig.VoltageCompensation(peramConfig.VoltComp.value());
    feederConfig.SmartCurrentLimit(peramConfig.SmartCurrLim.value());

    feederConfig.closedLoop
    .P(peramConfig.kP)
    .I(peramConfig.kI)
    .D(peramConfig.kD)
    .feedForward.kS(peramConfig.kFF); //maybe add KV?

    m_feederMotor.Configure(feederConfig, 
                rev::ResetMode::kNoResetSafeParameters, 
                rev::PersistMode::kNoPersistParameters);


    InitializeDashboard();

}

Feeder::~Feeder() {}

void Feeder::Periodic() {
    UpdateDashboard();
}

frc2::CommandPtr Feeder::setRPM(double RPM) {
    return Run([this, RPM] {setVelocity(RPM);});
}

frc2::CommandPtr Feeder::setRPMUntilThere(double RPM) {
    return setRPM(RPM).Until([this, RPM]() -> bool{return isAtRPM(RPM);});
}

double Feeder::getRPM() {
    return m_feederMotor.GetEncoder().GetVelocity();
}

// ************************* Private ************************* //
void Feeder::setVelocity(double RPM) {
    targetRPM = RPM;
    m_pidController.SetSetpoint(RPM, rev::spark::SparkFlex::ControlType::kVelocity);
}

bool Feeder::isAtRPM(double RPM_Target) {
    return RPM_Target - FeederConstants::kToleranceRPM < getRPM() &&
            RPM_Target + FeederConstants::kToleranceRPM > getRPM();
}



// ************************* Sim/Dashboard ************************* //

class FeederSim {
public:
    FeederSim(Feeder &feeder);

public:
    frc::sim::FlywheelSim m_feederPhysics;
    frc::DCMotor m_feederMotor;
    rev::spark::SparkFlexSim m_feederState;
};

// Fixing my lazyness is left as an excersize to the reader
namespace { //Wont leave the .cpp
    frc::Mechanism2d m_mech{2, 2};
    frc::MechanismRoot2d* m_root = m_mech.GetRoot("LinearActuator", 1, 1);
    frc::MechanismLigament2d* m_MotorLine =
        m_root->Append<frc::MechanismLigament2d>("RealLine", 1, 0_deg);
    units::degree_t motorDegState = 0_deg;
}

void Feeder::InitializeDashboard() {
    frc::SmartDashboard::PutData("Shooter/Feeder"+(char)classIndex, &m_mech);
}

void Feeder::UpdateDashboard() {
    if (m_sim_state) {
        frc::SmartDashboard::PutNumber("Shooter/Feeder/Velocity"+(char)classIndex, 
            FeederConstants::feederGearing*units::revolutions_per_minute_t{
            m_sim_state->m_feederPhysics.GetAngularVelocity()}.value());
    }
}

std::unique_ptr<FeederSim> create_feeder_sim(Feeder &feeder) {
    if constexpr (frc::RobotBase::IsSimulation()) {
        return std::make_unique<FeederSim>(feeder);
    } else {
        return nullptr;
    }
}

auto make_feeder_phys_sim() {
    return frc::sim::FlywheelSim{
        frc::LinearSystemId::FlywheelSystem(
            frc::DCMotor::NeoVortex(1),
            FeederConstants::feederMOI,
            FeederConstants::feederGearing),
        frc::DCMotor::NeoVortex(1)
    };
}

FeederSim::FeederSim(Feeder &feeder) :
    m_feederMotor{frc::DCMotor::NeoVortex(1)},
    m_feederState{&feeder.m_feederMotor, &m_feederMotor},
    m_feederPhysics{make_feeder_phys_sim()}
{
}

void Feeder::SimulationPeriodic() {
    if (!m_sim_state) return;

    const auto supply_voltage = frc::RobotController::GetInputVoltage()*1_V;

    auto &phys = m_sim_state->m_feederPhysics;
    auto &state = m_sim_state->m_feederState;

    phys.SetInputVoltage(state.GetAppliedOutput()*supply_voltage);
    phys.Update(20_ms);
    state.iterate(
            FeederConstants::feederGearing*units::revolutions_per_minute_t{
                phys.GetAngularVelocity()}.value(),
            supply_voltage.value(),
            0.02
        );

    //only for visualizatoin
    motorDegState += units::degree_t(phys.GetAngularVelocity().value() *0.02 /*20ms*/ * 57.3 /*to deg*/); 
}