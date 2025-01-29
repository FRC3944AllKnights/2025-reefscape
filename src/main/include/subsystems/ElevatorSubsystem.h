#pragma once

#include <rev/ColorSensorV3.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
public:
    ElevatorSubsystem();
    void raiseElevatorSimple(double speed);
    void lowerElevatorSimple(double speed);
    void raiseElevatorTiered(); 
    void lowerElevatorTiered();
    int getLevel();

private:
    rev::spark::SparkMax m_IntakeMotor{20, rev::spark::SparkMax::MotorType::kBrushless};  // Replace '20' with the CAN ID of the Spark MAX

    rev::spark::SparkRelativeEncoder m_Encoder =
        m_IntakeMotor.GetEncoder();
    rev::spark::SparkClosedLoopController m_IntakePIDController =
        m_IntakeMotor.GetClosedLoopController();
};