#pragma once

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc2/command/SubsystemBase.h>

namespace LEDConst{

}

class LED : public frc2::SubsystemBase{
 public:
    LED();
    void Periodic() override;

 private:
    static constexpr int kPort{0};

    frc::AddressableLED m_led{kPort};

};