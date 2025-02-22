#include "subsystems/IntakeSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"
#include "Configs.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem() {
    m_LeftIntakeMotor.Configure(Configs::LeftIntakeConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
    m_RightIntakeMotor.Configure(Configs::RightIntakeConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
}

void IntakeSubsystem::SetIntakeMotors(bool spinning) {
    // spinning: true = motors moving, false = motors stopped
    if (spinning) {
        m_LeftIntakeMotor.Set(IntakeSpeed);
    }
    else {
        m_LeftIntakeMotor.Set(0.0);
    }
    if(GamePieceDetected()){
        SetColorLEDCoralDetected (0, 0, 255);
    }
    else {
        SetColorLEDCoralDetected (0, 0, 0);
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


void IntakeSubsystem::SetColorLEDOuttakeTargetDetected(int R, int G, int B){
    for (int i = startIndexLEDOuttakeTargetDetected; i < startIndexLEDOuttakeTargetDetected + kLengthLEDOuttakeTargetDetected; i++) {
        m_ledBuffer_OuttakeTargetDetected[i].SetRGB(R, G, B);
    }
    m_led_CoralDetected.SetData(m_ledBuffer_OuttakeTargetDetected);
}
void IntakeSubsystem::SetColorLEDIntakeTargetDetected(int R, int G, int B){
    for (int i = startIndexLEDIntakeTargetDetected; i < startIndexLEDIntakeTargetDetected + kLengthLEDIntakeTargetDetected; i++) {
        m_ledBuffer_IntakeTargetDetected[i].SetRGB(R, G, B);
    }
    m_led_CoralDetected.SetData(m_ledBuffer_IntakeTargetDetected);
}
void IntakeSubsystem::SetColorLEDCoralDetected(int R, int G, int B){
    for (int i = startIndexLEDCoralDetected; i < startIndexLEDCoralDetected + kLengthLEDCoralDetected; i++) {
        m_ledBuffer_CoralDetected[i].SetRGB(R, G, B);
    }
    m_led_CoralDetected.SetData(m_ledBuffer_CoralDetected);
}