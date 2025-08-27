#include "subsystems/OuttakeSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"
#include "Configs.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace OuttakeConstants;

OuttakeSubsystem::OuttakeSubsystem() {
    m_colorMatcher.AddColorMatch(kGamePiece);
    m_colorMatcher.AddColorMatch(kBackGround);
    
    m_LeftOuttakeMotor.Configure(Configs::OuttakeSubsystem::LeftOuttakeConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
    m_RightOuttakeMotor.Configure(Configs::OuttakeSubsystem::RightOuttakeConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
}

void OuttakeSubsystem::SetOuttakeMotors(bool spinning) {
    // spinning: true = motors moving, false = motors stopped
    if (spinning) {
        m_LeftOuttakeMotor.Set(OuttakeSpeed);
    }
    else {
        m_LeftOuttakeMotor.Set(0.0);
    }
    
}

void OuttakeSubsystem::IntakeCoral() {
    frc::SmartDashboard::PutBoolean("GamePieceDetected", GamePieceDetected());
    if(GamePieceDetected()){
        m_LeftOuttakeMotor.Set(IntakeSpeed);
    }
    else {
        m_LeftOuttakeMotor.Set(0.0);
    }
}

bool OuttakeSubsystem::GamePieceDetected(){
    return false;
    //double IRval = m_colorSensor.GetIR();
    //frc::SmartDashboard::PutNumber("IR Sensor", IRval); 
    //return (IRval > 15.0);
}

bool OuttakeSubsystem::GamePieceDetectedByColor(){
    return false;
    //double IRval = m_colorSensor.GetIR();
    //frc::SmartDashboard::PutNumber("IR Sensor", IRval); 
    //return (IRval > 15.0);
    /*
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
        // Check IR sensor too
        frc::SmartDashboard::PutNumber("IR Sensor", m_colorSensor.GetIR()); 
        return (m_colorSensor.GetIR() > 15.0);
    }
    else { 
        return false;
    }
    */
    
}

bool OuttakeSubsystem::GamePieceDetectedBySwitch() {
    return true; //LimitSwitch.Get();
}

/*
void OuttakeSubsystem::SetColorLEDOuttakeTargetDetected(int R, int G, int B){
    for (int i = startIndexLEDOuttakeTargetDetected; i < startIndexLEDOuttakeTargetDetected + kLengthLEDOuttakeTargetDetected; i++) {
        m_ledBuffer_OuttakeTargetDetected[i].SetRGB(R, G, B);
    }
    m_led_CoralDetected.SetData(m_ledBuffer_OuttakeTargetDetected);
}
void OuttakeSubsystem::SetColorLEDIntakeTargetDetected(int R, int G, int B){
    for (int i = startIndexLEDIntakeTargetDetected; i < startIndexLEDIntakeTargetDetected + kLengthLEDIntakeTargetDetected; i++) {
        m_ledBuffer_IntakeTargetDetected[i].SetRGB(R, G, B);
    }
    m_led_CoralDetected.SetData(m_ledBuffer_IntakeTargetDetected);
}
void OuttakeSubsystem::SetColorLEDCoralDetected(int R, int G, int B){
    for (int i = startIndexLEDCoralDetected; i < startIndexLEDCoralDetected + kLengthLEDCoralDetected; i++) {
        m_ledBuffer_CoralDetected[i].SetRGB(R, G, B);
    }
    m_led_CoralDetected.SetData(m_ledBuffer_CoralDetected);
}
*/