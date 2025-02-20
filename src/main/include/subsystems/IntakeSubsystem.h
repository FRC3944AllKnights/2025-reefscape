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

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();
    void SetIntakeMotorSpeed(double speed);
    bool GamePieceDetected();
    void SetColorLED(int R, int G, int B);
    bool usingColorSensor = false; // True: Color sensor. False: Limit switch(es)

private:
    //bool GamePieceDetectedByColor();
    bool GamePieceDetectedBySwitch();

    // Motor control

    rev::spark::SparkMax m_IntakeMotor{20, rev::spark::SparkMax::MotorType::kBrushless};  // Replace '20' with the CAN ID of the Spark MAX
    rev::spark::SparkRelativeEncoder m_Encoder =
        m_IntakeMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_IntakePIDController =
        m_IntakeMotor.GetClosedLoopController();

    // Game piece detection

    /*
    rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
    rev::ColorMatch m_colorMatcher;
    frc::Color kGamePiece = frc::Color(0.0, 0.0, 0.0);
    frc::Color kBackGround = frc::Color(0.99, 0.99, 0.99);
    */
    frc::DigitalInput LimitSwitch = frc::DigitalInput(1);
   
    static constexpr int kLength = 46;
// PWM port 9
    // Must be a PWM header, not MXP or DIO
    frc::AddressableLED m_led{0};
    std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
    // Store what the last hue of the first pixel is
    int firstPixelHue = 0;

};