#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"
#include "Configs.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace ElevatorConstants;


ElevatorSubsystem::ElevatorSubsystem() {
    m_LeftElevatorMotor.Configure(Configs::ElevatorSubsystem::LeftElevatorConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
    m_RightElevatorMotor.Configure(Configs::ElevatorSubsystem::RightElevatorConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
}

void ElevatorSubsystem::raiseElevatorSimple(double speed) {
    // Speed: [0, 1]
    if (speed > 1.0) {
        speed = 1.0;
    }
    if (speed < 0.0) {
        speed = 0.0;
    }
    targetHeight += speed * ElevatorMaxSpeed;
    if (targetHeight > encoderTiers[6]) {
        // Set maximum height
        targetHeight = encoderTiers[6];
    }
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
    resetTargetLevel();
    frc::SmartDashboard::PutNumber("Elevator Target Level", targetLevel);
    frc::SmartDashboard::PutNumber("Elevator Target Height %", 100.0 * targetHeight / encoderTiers[5]);
}
void ElevatorSubsystem::lowerElevatorSimple(double speed) {
    // Speed: [0, 1]
    if (speed > 1.0) {
        speed = 1.0;
    }
    if (speed < 0.0) {
        speed = 0.0;
    }
    targetHeight -= speed * ElevatorMaxSpeed;
    if (targetHeight < encoderTiers[0]) {
        // Set minimum height
        targetHeight = encoderTiers[0];
    }
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
    resetTargetLevel();
    frc::SmartDashboard::PutNumber("Elevator Target Level", targetLevel);
    frc::SmartDashboard::PutNumber("Elevator Target Height %", 100.0 * targetHeight / encoderTiers[6]);
}
void ElevatorSubsystem::stopElevatorSimple() {
    targetHeight = m_LeftEncoder.GetPosition();
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
}

void ElevatorSubsystem::raiseElevatorTiered() {
    setElevatorLevel(targetLevel + 1);
}
void ElevatorSubsystem::lowerElevatorTiered() {
    int nextLowestLevel;
    if (targetHeight <= encoderTiers[1]) {
        nextLowestLevel = 0;
    }
    else if (targetHeight <= encoderTiers[2]) {
        nextLowestLevel = 1;
    }
    else if (targetHeight <= encoderTiers[3]) {
        nextLowestLevel = 2;
    }
    else if (targetHeight <= encoderTiers[4]) {
        nextLowestLevel = 3;
    }
    else if (targetHeight <= encoderTiers[5]){
        nextLowestLevel = 4;
    }
    else if (targetHeight <= encoderTiers[6]){
        nextLowestLevel = 5;
    }
    else {
        nextLowestLevel = 6;
    }
    setElevatorLevel(nextLowestLevel);
}

int ElevatorSubsystem::getLevel() {
// returns the level of elevator (0-5)
    double encoderPosition = m_LeftEncoder.GetPosition();
    if (encoderPosition < encoderTiers[1]) {
        return 0;
    }
    else if (encoderPosition < encoderTiers[2]) {
        return 1;
    }
    else if (encoderPosition < encoderTiers[3]) {
        return 2;
    }
    else if (encoderPosition < encoderTiers[4]) {
        return 3;
    }
    else if (encoderPosition < encoderTiers[5]){
        return 4;
    }
    else {
        return 5;
    }
}

double ElevatorSubsystem::getHeight() {
    return m_LeftEncoder.GetPosition();
}

bool ElevatorSubsystem::isAtTop() {
    return getHeight() > 0.98 * encoderTiers[6];
}

int ElevatorSubsystem::getTargetLevel() {
    return targetLevel;
}

void ElevatorSubsystem::resetTargetLevel() {
if (targetHeight < encoderTiers[1]) {
        targetLevel = 0;
    }
    else if (targetHeight < encoderTiers[2]) {
        targetLevel = 1;
    }
    else if (targetHeight < encoderTiers[3]) {
        targetLevel = 2;
    }
    else if (targetHeight < encoderTiers[4]) {
        targetLevel = 3;
    }
    else if (targetHeight < encoderTiers[5]){
        targetLevel = 4;
    }
    else if (targetHeight < encoderTiers[6]){
        targetLevel = 5;
    }
    else {
        targetLevel = 6;
    }
}

void ElevatorSubsystem::setElevatorLevel(int level) {
    if (level > 6) {
        targetLevel = 6;
    }
    else if (level < 0) {
        targetLevel = 0;
    }
    else {
        targetLevel = level;
    }
    targetHeight = encoderTiers[targetLevel];
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
    frc::SmartDashboard::PutNumber("Elevator Target Level", targetLevel);
    frc::SmartDashboard::PutNumber("Elevator Target Height %", 100.0 * targetHeight / encoderTiers[6]);
}

void ElevatorSubsystem::disableElevator() {
    m_LeftElevatorMotor.Set(0.0);
}

void ElevatorSubsystem::resetElevatorEncoder() {
    m_LeftEncoder.SetPosition(0.25);
    m_RightEncoder.SetPosition(0.25);
}