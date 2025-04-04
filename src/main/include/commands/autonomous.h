#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/OuttakeSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "LimelightHelpers.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"

namespace autos {
    frc2::CommandPtr DriveForward(DriveSubsystem* drive);
    frc2::CommandPtr DriveForwardAndScore(DriveSubsystem* drive, ElevatorSubsystem* elevator, OuttakeSubsystem* outtake);
    frc2::CommandPtr RaiseLevel4AndScore(ElevatorSubsystem* elevator, OuttakeSubsystem* outtake);
}