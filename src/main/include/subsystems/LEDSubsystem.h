#pragma once

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>

namespace LEDConstants {
    enum class TEAM {
        NONE,
        RED,
        BLUE,
    };

    enum class PHASE {
        TRANSITION_SHIFT,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME,
    };
}

class LEDSubsystem : public frc2::SubsystemBase{
public:
    LEDSubsystem();

    void TeleopInit();

    void Periodic() override;

private:
    frc::AddressableLED m_led;

    std::vector<frc::AddressableLED::LEDData> m_ledBuffer;

    LEDConstants::TEAM m_firstInactive;

    LEDConstants::TEAM m_team;

    frc::Timer m_timer;

    uint32_t m_nextTimestampIndex;
};