#include "subsystems/IntakeSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"
#include "Configs.h"
#include <frc/SmartDashboard/SmartDashboard.h>


IntakeSubsystem::IntakeSubsystem() {
    m_IntakeMotor.Configure(Configs::IntakeConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
}

void IntakeSubsystem::SetIntakeMotorSpeed(double speed) {
        m_IntakeMotor.Set(speed);
        if(GamePieceDetected()){
            SetColorLED (245, 157, 5);
        }
        else {
            SetColorLED (0, 0, 0);
        }
}

bool IntakeSubsystem::GamePieceDetected(){
    return GamePieceDetectedBySwitch();
    /*
    if (usingColorSensor) {
        return GamePieceDetectedByColor();
    }
    else {
        return GamePieceDetectedBySwitch();
    }
    */
}
/*
bool IntakeSubsystem::GamePieceDetectedByColor() {
    double confidence = 0.1;
    frc::Color detectedColor = m_colorSensor.GetColor();
    frc::SmartDashboard::PutNumber("Color R", detectedColor.red);
    frc::SmartDashboard::PutNumber("Color G", detectedColor.green);
    frc::SmartDashboard::PutNumber("Color B", detectedColor.blue);

    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    frc::SmartDashboard::PutNumber("MATCHED R", matchedColor.red);
    frc::SmartDashboard::PutNumber("MATCHED G", matchedColor.green);
    frc::SmartDashboard::PutNumber("MATCHED B", matchedColor.blue);

    if (matchedColor == kGamePiece){
        return true;
    }
    else {
    return false;
    }
}
*/
bool IntakeSubsystem::GamePieceDetectedBySwitch() {
    return LimitSwitch.Get();
}

void IntakeSubsystem::SetColorLED(int R, int G, int B){
    for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(R, G, B);
    }

    m_led.SetData(m_ledBuffer);


}