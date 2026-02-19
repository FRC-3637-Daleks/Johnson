#include "subsystems/shooter/Feeder.h"
#include <rev/config/SparkMaxConfig.h>

namespace FeederConstants {
    int kTopMotorID = 21;
    int kBottomMotorID = 22;

    double kToleranceRPM = 100; //only used for setVelocityUntilSpeed

    //Top Motor
    units::volt_t TopMotorVoltageComp = 12_V;
    units::ampere_t TopMotorSmartCurrentLimit = 40_A;
    int TopMotor_P = 1.0;
    int TopMotor_I = 0.0001;
    int TopMotor_D = 0.0;
    int TopMotor_FF = 0.1; 

    //Bottom Motor
    units::volt_t BottomMotorVoltageComp = 12_V;
    units::ampere_t BottomMotorSmartCurrentLimit = 40_A;
    int BottomMotor_P = 1.0;
    int BottomMotor_I = 0.0001;
    int BottomMotor_D = 0.0;
    int BottomMotor_FF = 0.1; 

    //For cleaner initialization of feeder class
    struct Perams {
        int motorID;
        units::volt_t VoltComp;
        units::ampere_t SmartCurrLim;
        int kP, kI, kD, kFF;
        
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

}

Feeder::Feeder(FeederConstants::Perams peramConfig) :
    m_feederMotor(peramConfig.motorID, rev::spark::SparkFlex::MotorType::kBrushless),
    m_pidController(m_feederMotor.GetClosedLoopController())    
{
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

}

Feeder::~Feeder() {}

void Feeder::Periodic() {}

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