#include "subsystems/Feeder.h"
#include <rev/config/SparkMaxConfig.h>
#include <rev/sim/SparkFlexSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc/RobotBase.h>


namespace FeederConstants {
    int kTopMotorID = 23;
    int kBottomMotorID = 22;
    constexpr int kIntakeMotorID = 10;

    units::turns_per_second_t kToleranceRPM = 2_tps; //only used for setVelocityUntilSpeed

    struct Perams {
        units::volt_t VoltComp;
        units::ampere_t SmartCurrLim;
        double kP, kI, kD, kFF;
    };

    Perams TopMotor {
    /*units::volt_t VoltComp*/          12_V,
    /*units::ampere_t SmartCurrLim*/    40_A,
    /*double P*/                        0,
    /*double I*/                        0.0,
    /*double D*/                        0.0,
    /*double FF*/                       0.1 
    };

    Perams BottomMotor {
    /*units::volt_t VoltComp*/          12_V,
    /*units::ampere_t SmartCurrLim*/    40_A,
    /*double P*/                        0.0,
    /*double I*/                        0.0,
    /*double D*/                        0.0,
    /*double FF*/                       0.1 
    };

    Perams IntakeMotor {
    /*units::volt_t VoltComp*/          12_V,
    /*units::ampere_t SmartCurrLim*/    40_A,
    /*double P*/                        0.0,
    /*double I*/                        0.0,
    /*double D*/                        0.0,
    /*double FF*/                       0.1 
    };

    constexpr auto feederGearing = 1.0;
    constexpr auto feederMOI = 0.001_kg_sq_m;
    constexpr auto VelocityConversionFactor = 1.0/60; //RPM -> RPS
    constexpr auto feederMotor = frc::DCMotor::NeoVortex(1).WithReduction(feederGearing);

    int getMotorID(Feeder::Type id) {
        if (id == Feeder::Type::Top) return kTopMotorID;
        if (id == Feeder::Type::Bottom) return kBottomMotorID;
        if (id == Feeder::Type::Intake) return kIntakeMotorID;
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
    else if (type == Type::Intake) peramConfig = FeederConstants::IntakeMotor;

    //velocty control (voltage but with some compensation)
    rev::spark::SparkMaxConfig feederConfig = rev::spark::SparkMaxConfig();

    feederConfig.VoltageCompensation(peramConfig.VoltComp.value());
    feederConfig.SmartCurrentLimit(peramConfig.SmartCurrLim.value());

    constexpr auto Kv = (FeederConstants::feederMotor.Kv * 1_V);
    feederConfig.closedLoop
    .P(peramConfig.kP)
    .I(peramConfig.kI)
    .D(peramConfig.kD)
    .feedForward.kV(
        Kv.value());


    feederConfig.encoder.VelocityConversionFactor(FeederConstants::VelocityConversionFactor);

    m_feederMotor.Configure(feederConfig, 
                rev::ResetMode::kNoResetSafeParameters, 
                rev::PersistMode::kNoPersistParameters);


    InitializeDashboard();

}

Feeder::~Feeder() {}

void Feeder::Periodic() {
    UpdateDashboard();
}

frc2::CommandPtr Feeder::ManuallySetMotor(std::function<double()> input) {
    return Run([this, input] {m_feederMotor.Set(input()*0.5/*Scaler*/);});
}

frc2::CommandPtr Feeder::setRPM(units::turns_per_second_t speed) {
    return Run([this, speed] {setVelocity(speed);});
}

frc2::CommandPtr Feeder::setRPMEnd(units::turns_per_second_t speed) {
    return RunEnd([this, speed] {setVelocity(speed);}, [this] {setVelocity(0_tps);});
}

frc2::CommandPtr Feeder::setRPMUntilThere(units::turns_per_second_t speed) {
    return setRPM(speed).Until([this, speed]() -> bool{return isAtRPM((speed));});
}

units::turns_per_second_t Feeder::getRPM() {
    return units::turns_per_second_t(m_feederMotor.GetEncoder().GetVelocity());
}

// ************************* Private ************************* //
void Feeder::setVelocity(units::turns_per_second_t speed) {
    targetSpeed = speed;
    m_pidController.SetSetpoint(speed.value(), rev::spark::SparkFlex::ControlType::kVelocity);
}

bool Feeder::isAtRPM(units::turns_per_second_t RPM_Target) {
    return RPM_Target - FeederConstants::kToleranceRPM < getRPM() &&
            RPM_Target + FeederConstants::kToleranceRPM > getRPM();
}



// ************************* Sim/Dashboard ************************* //

class FeederSim {
public:
    FeederSim(Feeder &feeder);

public:
    frc::DCMotor m_feederMotor;
    rev::spark::SparkFlexSim m_feederState;
    frc::sim::FlywheelSim m_feederPhysics;
};

void Feeder::InitializeDashboard() {
    frc::SmartDashboard::PutData("Feeder"+ std::to_string(thisClassesIndex), &m_mech);
    
    auto put_cmd = [this] (std::string_view name, frc2::CommandPtr&& cmd) {
        frc::SmartDashboard::PutData(fmt::format("Feeder{}/{}", thisClassesIndex, name),
            std::move(cmd).WithName(name).Unwrap().release()
        );
    };

    frc::SmartDashboard::PutNumber("Feeder"+ std::to_string(thisClassesIndex)+"/SetFeederSpeedTPS", 0.0);
    put_cmd("SetFeeder"+ std::to_string(thisClassesIndex), 
    setRPM(frc::SmartDashboard::GetNumber(
        "Feeder"+ std::to_string(thisClassesIndex)+"/SetFeederSpeedTPS", 0.0)*1.0_tps));
}

void Feeder::UpdateDashboard() {
    if (m_sim_state) {
        frc::SmartDashboard::PutNumber("Feeder"+std::to_string(thisClassesIndex)+"/Velocity", 
            FeederConstants::feederGearing*units::revolutions_per_minute_t{
            m_sim_state->m_feederPhysics.GetAngularVelocity()}.value());

        m_MotorLine->SetAngle(motorDegState);
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
    motorDegState += units::degree_t(phys.GetAngularVelocity() *0.02_s /*20ms*/ * 57.3 /*to deg*/); 
}