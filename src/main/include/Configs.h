#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace rev::spark;

namespace Configs {
class MAXSwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig() {
    static SparkMaxConfig drivingConfig{};

    // Use module constants to calculate conversion factors and feed forward
    // gain.
    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;
    double drivingVelocityFeedForward =
        1 / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    drivingConfig.encoder
        .PositionConversionFactor(drivingFactor)          // meters
        .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    drivingConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(0.04, 0, 0)
        .VelocityFF(drivingVelocityFeedForward)
        .OutputRange(-1, 1);

    return drivingConfig;
  }

  static SparkMaxConfig& TurningConfig() {
    static SparkMaxConfig turningConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    turningConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    turningConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(true)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    turningConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return turningConfig;
  }
};
class OuttakeSubsystem {
 public:
  static SparkMaxConfig& LeftOuttakeConfig() {
    static SparkMaxConfig leftOuttakeConfig{};

    leftOuttakeConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    leftOuttakeConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1);

    return leftOuttakeConfig;
  }

    static SparkMaxConfig& RightOuttakeConfig() {
    static SparkMaxConfig rightOuttakeConfig{};

    rightOuttakeConfig.Follow(OuttakeConstants::LeftOuttakeCANID, true);
    
    // Copied from left outtake config - KEEP UPDATED

    rightOuttakeConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    rightOuttakeConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1);

    return rightOuttakeConfig;
  }
};

class ElevatorSubsystem {
 public:
  
  static SparkMaxConfig& LeftElevatorConfig() {
    static SparkMaxConfig leftElevatorConfig{};

    leftElevatorConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    //leftElevatorConfig.Inverted(true);
    leftElevatorConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .P(0.5)
        .I(0)
        .D(0.05)
        .OutputRange(-1, 1)
        .P(0.0001, ClosedLoopSlot::kSlot1)
        .I(0, ClosedLoopSlot::kSlot1)
        .D(0, ClosedLoopSlot::kSlot1)
        .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
        .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);
    leftElevatorConfig.closedLoop
        .maxMotion
        .MaxVelocity(1000)
        .MaxAcceleration(1000)
        .AllowedClosedLoopError(1);

    return leftElevatorConfig;
  }

  static SparkMaxConfig& RightElevatorConfig() {
    static SparkMaxConfig rightElevatorConfig{};
    
    rightElevatorConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    rightElevatorConfig.Follow(ElevatorConstants::LeftElevatorCANID, true);

    return rightElevatorConfig;
  }
};
}  // namespace Configs