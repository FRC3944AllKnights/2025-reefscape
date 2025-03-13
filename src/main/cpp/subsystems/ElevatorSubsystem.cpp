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
    if (targetHeight > encoderTiers[5]) {
        // Set maximum height
        targetHeight = encoderTiers[5];
    }
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
    targetLevel = getLevel();
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
    targetLevel = getLevel();
}
void ElevatorSubsystem::stopElevatorSimple() {
    targetHeight = m_LeftEncoder.GetPosition();
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
}

void ElevatorSubsystem::raiseElevatorTiered() {
    setElevatorLevel(targetLevel + 1);
}
void ElevatorSubsystem::lowerElevatorTiered() {
    setElevatorLevel(targetLevel - 1);
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

void ElevatorSubsystem::setElevatorLevel(int level) {
    frc::SmartDashboard::PutBoolean("Elevator Target Level", level);
    if (level > 5) {
        targetLevel = 5;
    }
    else if (level < 0) {
        targetLevel = 0;
    }
    else {
        targetLevel = level;
    }
    targetHeight = encoderTiers[targetLevel];
    m_LeftElevatorPIDController.SetReference(targetHeight, SparkMax::ControlType::kMAXMotionPositionControl);
}
