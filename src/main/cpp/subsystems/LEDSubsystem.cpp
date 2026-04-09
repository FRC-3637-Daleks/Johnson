#include "subsystems/LEDSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/DriverStation.h>

#include <iostream>

namespace LEDConstants {
    constexpr int kPort = 1;
    constexpr int kNumLED = 41;

    constexpr auto kTransitionShiftTimestamp = 10_s;
    constexpr auto kShift1Timestamp = 35_s;
    constexpr auto kShift2Timestamp = 60_s;
    constexpr auto kShift3Timestamp = 85_s;
    constexpr auto kShift4Timestamp = 110_s;
    constexpr auto kEndGameTimestamp = 140_s;

    constexpr std::array<units::time::second_t, 6> kMatchTimestamps{kTransitionShiftTimestamp, 
                                                                    kShift1Timestamp,
                                                                    kShift2Timestamp,
                                                                    kShift3Timestamp,
                                                                    kShift4Timestamp,
                                                                    kEndGameTimestamp};

    const auto kStevensStateMachineIsntWorkingPattern = frc::LEDPattern::Rainbow(255, 150); //Rainbow
    const auto kActivePattern = frc::LEDPattern::Solid(frc::Color{0.0, 1.0, 0.0});          // Green
    const auto kInactivePattern = frc::LEDPattern::Solid(frc::Color{1.0, 0.0, 0.0});        // Red
    const auto kFirstWarningPattern = frc::LEDPattern::Solid(frc::Color{1.0, 1.0, 0.0});    // Yellow
    const auto kSecondWarningPattern = frc::LEDPattern::Solid(frc::Color{1.0, 0.5, 0.0});   // Orange
    const auto kActivationWarningPattern = kFirstWarningPattern;
    const auto kAutonPattern = kActivePattern;
    const auto kEndPattern = kActivePattern;

    // Time before the shift where it will change to the first warning color
    constexpr auto kFirstWarningCountdownTime = 10_s;

    // Time before the shift where it will change to the second warning color
    constexpr auto kSecondWarningCountdownTime = 3_s;

    // Time before active period where it will change to the second warning color
    constexpr auto kActivationWarningCountdownTime = 3_s;
}

LEDSubsystem::LEDSubsystem() : m_led{LEDConstants::kPort}, m_ledBuffer(LEDConstants::kNumLED), m_nextTimestampIndex{0U} {
    m_led.SetLength(LEDConstants::kNumLED);
    LEDConstants::kAutonPattern.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}

void LEDSubsystem::TeleopInit()
{
    m_timer.Restart();
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        m_team = LEDConstants::TEAM::RED;
    } else {
        m_team = LEDConstants::TEAM::BLUE;
    }
}

void LEDSubsystem::Periodic() {
    if (!m_timer.IsRunning()) {
        return;
    }

    // Determine which team will be inactive during the first shift
    if (m_firstInactive == LEDConstants::TEAM::NONE) {
        const std::string message = frc::DriverStation::GetGameSpecificMessage();

        if (message.starts_with("R")) {
            m_firstInactive = LEDConstants::TEAM::RED;
        } else if (message.starts_with("B")) {
            m_firstInactive = LEDConstants::TEAM::BLUE;
        }
    }

    const auto timerVal = m_timer.Get();
    bool hubActive{true};

    // Stage Changes
    if (m_nextTimestampIndex < LEDConstants::kMatchTimestamps.size() && timerVal > LEDConstants::kMatchTimestamps.at(m_nextTimestampIndex)) {
        m_nextTimestampIndex++;
    }
    if (m_nextTimestampIndex >= LEDConstants::kMatchTimestamps.size()) {
        LEDConstants::kEndPattern.ApplyTo(m_ledBuffer);
        m_led.SetData(m_ledBuffer);
        return;
    }

    // If we are in shift 1 or 3
    if (m_nextTimestampIndex == 1 || m_nextTimestampIndex == 3) {
        if (m_team == m_firstInactive) {
            hubActive = false;
        }
    } else if (m_nextTimestampIndex == 2 || m_nextTimestampIndex == 4) {
        if (m_team != m_firstInactive) {
            hubActive = false;
        }
    }

    const auto timeUntilChange = LEDConstants::kMatchTimestamps.at(m_nextTimestampIndex) - timerVal;

    // Set LED Color
    if (hubActive) {
        if (timeUntilChange < LEDConstants::kSecondWarningCountdownTime) {
            LEDConstants::kSecondWarningPattern.ApplyTo(m_ledBuffer);
        } else if (timeUntilChange < LEDConstants::kFirstWarningCountdownTime) {
            LEDConstants::kFirstWarningPattern.ApplyTo(m_ledBuffer);
        } else {
            LEDConstants::kActivePattern.ApplyTo(m_ledBuffer);
        }
    } else {
        if (timeUntilChange < LEDConstants::kActivationWarningCountdownTime) {
            LEDConstants::kActivationWarningPattern.ApplyTo(m_ledBuffer);
        } else {
            LEDConstants::kInactivePattern.ApplyTo(m_ledBuffer);
        }
    }

    //Overide in logic for auto and if were not getting team color to make it
    //return green because if we don't know out team color, no useful data can be displayed
    if (m_team == LEDConstants::TEAM::NONE) {
        LEDConstants::kStevensStateMachineIsntWorkingPattern.ApplyTo(m_ledBuffer);
    }
    
    // Applies The Pattern
    m_led.SetData(m_ledBuffer);
}