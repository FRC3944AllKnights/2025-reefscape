#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"
#include "Configs.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace ElevatorConstants;


ElevatorSubsystem::ElevatorSubsystem() {
    m_ElevatorMotor.Configure(Configs::ElevatorConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);
}

void ElevatorSubsystem::raiseElevatorSimple(double speed) {
        m_ElevatorMotor.Set(speed);
}
void ElevatorSubsystem::lowerElevatorSimple(double speed) {
        m_ElevatorMotor.Set(speed);
}

void ElevatorSubsystem::raiseElevatorTiered() {
    if (targetLevel >= 5) {
        return;
    }

    targetLevel++;

    double encoderPosition = ElevatorConstants::encoderTiers[targetLevel];
    this->m_ElevatorPIDController.SetReference(encoderPosition, SparkMax::ControlType::kPosition);
}
void ElevatorSubsystem::lowerElevatorTiered() {
   if (targetLevel <= 0) {
        return;
    }

    targetLevel--;

    double encoderPosition = ElevatorConstants::encoderTiers[targetLevel];
    this->m_ElevatorPIDController.SetReference(encoderPosition, SparkMax::ControlType::kPosition);
}

int ElevatorSubsystem::getLevel() {
// returns the level of elevator (1-4)
    double encoderPosition = this->m_Encoder.GetPosition();
    if (encoderPosition < ElevatorConstants::encoderTiers[1]) {
        return 0;
    }
    else if (encoderPosition < ElevatorConstants::encoderTiers[2]) {
        return 1;
    }
    else if (encoderPosition < ElevatorConstants::encoderTiers[3]) {
        return 2;
    }
    else if (encoderPosition < ElevatorConstants::encoderTiers[4]) {
        return 3;
    }
    else if (encoderPosition < ElevatorConstants::encoderTiers[5]){
        return 4;
    }
    else {
        return 5;
    }
}

void ElevatorSubsystem::setElevatorLevel(int level) {
    targetLevel = level;
    double encoderPosition = ElevatorConstants::encoderTiers[targetLevel];
    this->m_ElevatorPIDController.SetReference(encoderPosition, SparkMax::ControlType::kPosition);
}
