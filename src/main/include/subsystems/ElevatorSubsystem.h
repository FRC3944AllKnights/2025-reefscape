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

private:
    int targetLevel = 0;

    rev::spark::SparkMax m_LeftElevatorMotor{LeftElevatorCANID, rev::spark::SparkMax::MotorType::kBrushless};  // Replace '20' with the CAN ID of the Spark MAX
    rev::spark::SparkMax m_RightElevatorMotor{RightElevatorCANID, rev::spark::SparkMax::MotorType::kBrushless};  // Replace '20' with the CAN ID of the Spark MAX
    
    rev::spark::SparkRelativeEncoder m_Encoder =
        m_LeftElevatorMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_ElevatorPIDController =
        m_LeftElevatorMotor.GetClosedLoopController();
};