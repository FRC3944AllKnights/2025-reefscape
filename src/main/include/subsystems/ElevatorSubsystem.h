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
    double getHeight();
    bool isAtTop();
    int getTargetLevel();
    void setElevatorLevel(int level);
    void resetTargetLevel();

    /**
     * A software e-stop that manually sets the elevator speed to zero. Can also be used to retract it.
     */
    void disableElevator();

    /**
     * Sets the elevator encoders to the position naturally reached by the elevator falling while disabled. Intended to be used to recover from a brown-out condition.
     */
    void resetElevatorEncoder();

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