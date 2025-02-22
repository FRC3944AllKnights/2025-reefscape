#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"
#include "Configs.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace ElevatorConstants;


ElevatorSubsystem::ElevatorSubsystem() {
    m_LeftElevatorMotor.Configure(Configs::LeftElevatorConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
    m_RightElevatorMotor.Configure(Configs::RightElevatorConfig(),
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
    m_LeftElevatorMotor.Set(speed * ElevatorMaxSpeed);
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
    m_LeftElevatorMotor.Set(-1.0 * speed * ElevatorMaxSpeed);
    targetLevel = getLevel();
}
void ElevatorSubsystem::stopElevatorSimple() {
    m_LeftElevatorMotor.Set(0.0);
}

void ElevatorSubsystem::raiseElevatorTiered() {
    setElevatorLevel(targetLevel + 1);
}
void ElevatorSubsystem::lowerElevatorTiered() {
    setElevatorLevel(targetLevel - 1);
}

int ElevatorSubsystem::getLevel() {
// returns the level of elevator (0-5)
    double encoderPosition = this->m_Encoder.GetPosition();
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
    if (level > 5) {
        targetLevel = 5;
    }
    else if (level < 0) {
        targetLevel = 0;
    }
    else {
        targetLevel = level;
    }
    double encoderPosition = encoderTiers[targetLevel];
    m_ElevatorPIDController.SetReference(encoderPosition, SparkMax::ControlType::kPosition);
}
