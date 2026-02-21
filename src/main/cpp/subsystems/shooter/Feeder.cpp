#include "subsystems/shooter/Feeder.h"
#include <rev/config/SparkMaxConfig.h>
#include <rev/sim/SparkFlexSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc/RobotBase.h>


namespace FeederConstants {
    int kTopMotorID = 21;
    int kBottomMotorID = 22;

    double kToleranceRPM = 100; //only used for setVelocityUntilSpeed

    struct Perams {
        units::volt_t VoltComp;
        units::ampere_t SmartCurrLim;
        double kP, kI, kD, kFF;
    };

    Perams TopMotor {
    /*units::volt_t VoltComp*/          12_V,
    /*units::ampere_t SmartCurrLim*/    40_A,
    /*double P*/                        1.0,
    /*double I*/                        0.0001,
    /*double D*/                        0.0,
    /*double FF*/                       0.1 
    };

    Perams BottomMotor {
    /*units::volt_t VoltComp*/          12_V,
    /*units::ampere_t SmartCurrLim*/    40_A,
    /*double P*/                        1.0,
    /*double I*/                        0.0001,
    /*double D*/                        0.0,
    /*double FF*/                       0.1 
    };

    constexpr auto feederGearing = 1.0;
    constexpr auto feederMOI = 0.001_kg_sq_m;

    int getMotorID(Feeder::Type id) {
        if (id == Feeder::Type::Top) return kTopMotorID;
        if (id == Feeder::Type::Bottom) return kBottomMotorID;
        return 12345; //clear error
    }
}

std::unique_ptr<FeederSim> create_feeder_sim(Feeder& feeder);

Feeder::Feeder(Type type) :
    m_feederMotor(FeederConstants::getMotorID(type), rev::spark::SparkFlex::MotorType::kBrushless),
    m_pidController(m_feederMotor.GetClosedLoopController()),
    m_sim_state{create_feeder_sim(*this)}
{
    classIndex++;
    thisClassesIndex = classIndex;
    FeederConstants::Perams peramConfig{};
    if (type == Type::Top) peramConfig = FeederConstants::TopMotor;
    else if (type == Type::Bottom) peramConfig = FeederConstants::BottomMotor;

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

void Feeder::InitializeDashboard() {
    frc::SmartDashboard::PutData("Shooter/Feeder"+ std::to_string(thisClassesIndex), &m_mech);
}

void Feeder::UpdateDashboard() {
    if (m_sim_state) {
        frc::SmartDashboard::PutNumber("Shooter/Feeder"+std::to_string(thisClassesIndex)+"/Velocity", 
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