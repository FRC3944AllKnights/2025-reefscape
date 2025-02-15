#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
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
    frc2::CommandPtr RaiseLevel4AndScore(ElevatorSubsystem* elevator, IntakeSubsystem* intake);
    frc2::CommandPtr AlignChute(DriveSubsystem* drive);
}