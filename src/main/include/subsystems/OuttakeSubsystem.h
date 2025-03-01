#pragma once

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/AddressableLED.h>
#include "frc/DigitalInput.h"

#include "Constants.h"

using namespace OuttakeConstants;

class OuttakeSubsystem : public frc2::SubsystemBase {
public:
    OuttakeSubsystem();
    void SetOuttakeMotors(bool spinning);
    bool GamePieceDetected();
    void SetColorLEDCoralDetected(int R, int G, int B);
    void SetColorLEDIntakeTargetDetected(int R, int G, int B);
    void SetColorLEDOuttakeTargetDetected(int R, int G, int B);
    bool usingColorSensor = false; // True: Color sensor. False: Limit switch(es)

private:
    //bool GamePieceDetectedByColor();
    bool GamePieceDetectedBySwitch();

    // Motor control

    rev::spark::SparkMax m_LeftOuttakeMotor{LeftIntakeCANID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder m_LeftEncoder =
        m_LeftOuttakeMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_LeftOuttakePIDController =
        m_LeftOuttakeMotor.GetClosedLoopController();

    rev::spark::SparkMax m_RightOuttakeMotor{RightIntakeCANID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder m_RightEncoder =
        m_RightOuttakeMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_RightOuttakePIDController =
        m_RightOuttakeMotor.GetClosedLoopController();

    // Game piece detection

    /*
    rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
    rev::ColorMatch m_colorMatcher;
    frc::Color kGamePiece = frc::Color(0.0, 0.0, 0.0);
    frc::Color kBackGround = frc::Color(0.99, 0.99, 0.99);
    */
    frc::DigitalInput LimitSwitch = frc::DigitalInput(1);
   
    // LEDs
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    // Index order: OuttakeTargetDetected, IntakeTargetDetected, CoralDetected

    static constexpr int startIndexLEDOuttakeTargetDetected = 0;
    static constexpr int startIndexLEDIntakeTargetDetected = 30;
    static constexpr int startIndexLEDCoralDetected = 60;
    static constexpr int totalLEDLength = 90;

    static const int kLengthLEDOuttakeTargetDetected = startIndexLEDIntakeTargetDetected - startIndexLEDOuttakeTargetDetected;
    static const int kLengthLEDIntakeTargetDetected = startIndexLEDCoralDetected - startIndexLEDIntakeTargetDetected;
    static const int kLengthLEDCoralDetected = totalLEDLength - startIndexLEDCoralDetected;

    frc::AddressableLED m_led_OuttakeTargetDetected{0};
    std::array<frc::AddressableLED::LEDData, kLengthLEDOuttakeTargetDetected> m_ledBuffer_OuttakeTargetDetected;
    frc::AddressableLED m_led_IntakeTargetDetected{0};
    std::array<frc::AddressableLED::LEDData, kLengthLEDIntakeTargetDetected> m_ledBuffer_IntakeTargetDetected;
    frc::AddressableLED m_led_CoralDetected{0};
    std::array<frc::AddressableLED::LEDData, kLengthLEDCoralDetected> m_ledBuffer_CoralDetected;
};