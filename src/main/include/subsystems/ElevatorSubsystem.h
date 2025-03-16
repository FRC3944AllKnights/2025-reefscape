#pragma once

#include <rev/ColorSensorV3.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"

using namespace ElevatorConstants;

class ElevatorSubsystem : public frc2::SubsystemBase {
public:
    ElevatorSubsystem();
    void raiseElevatorSimple(double speed);
    void lowerElevatorSimple(double speed);
    void stopElevatorSimple();
    void raiseElevatorTiered();
    void lowerElevatorTiered();
    int getLevel();
    void setElevatorLevel(int level);
    void resetTargetLevel();

private:
    int targetLevel = 0;
    double targetHeight = 0;

    rev::spark::SparkMax m_LeftElevatorMotor{LeftElevatorCANID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder m_LeftEncoder =
        m_LeftElevatorMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_LeftElevatorPIDController =
        m_LeftElevatorMotor.GetClosedLoopController();

    rev::spark::SparkMax m_RightElevatorMotor{RightElevatorCANID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder m_RightEncoder =
        m_LeftElevatorMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_RightElevatorPIDController =
        m_LeftElevatorMotor.GetClosedLoopController();
    
};