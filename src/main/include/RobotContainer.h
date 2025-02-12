// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/DriverStation.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "commands/autonomous.h"
#include "subsystems/ElevatorSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* getAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...
  bool isRed;
  double DegreeToRad(double degree);

  // The robot's subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_IntakeSubsystem;
  ClimberSubsystem m_ClimberSubsystem;
  ElevatorSubsystem m_ElevatorSubsystem;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  frc::SendableChooser<frc2::Command*> autoChooser;

  frc::PIDController translationPID{0.0125, 1.0e-3, 0.0};
  frc::PIDController rotationPID{0.01, 5.5e-3, 0.0};

  void ConfigureButtonBindings();
};
