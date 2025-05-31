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
#include "subsystems/OuttakeSubsystem.h"
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
  double absoluteFieldOffset;

  frc2::Command* getAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::GenericHID m_stick{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...
  double DegreeToRad(double degree);

  struct velocity2D {double x; double y; double theta;};
  velocity2D SnapToCoral(std::string direction);

  // The robot's subsystems
  DriveSubsystem m_drive;
  OuttakeSubsystem m_OuttakeSubsystem;
  ClimberSubsystem m_ClimberSubsystem;
  ElevatorSubsystem m_ElevatorSubsystem;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  //frc::SendableChooser<frc2::CommandPtr> autoChooser;

  bool thetaGood = false; //error < tolerance;
  double posTheta;
  frc2::CommandPtr m_DriveForward = autos::DriveForward(&m_drive);
  frc2::CommandPtr m_DriveForwardAndScore = autos::DriveForwardAndScore(&m_drive, &m_ElevatorSubsystem, &m_OuttakeSubsystem);
  
  frc::PIDController yTranslationPID{0.06, 0.0, 0.0005};
  frc::PIDController xTranslationPID{0.01, 0.0, 0.0005};
  frc::PIDController rotationPID{0.01, 0.0, 0.0005};

  std::map<int, double> coralAngles 
    {
        {18, 180.0},
        {10, 180.0},
        {17, 240.0},
        {11, 240.0},
        {22, 300.0},
        {6, 300.0},
        {21, 0.0},
        {7, 0.0},
        {20, 60.0},
        {8, 60.0},
        {9, 120.0},
        {19, 120.0}
    };

  std::map<std::string, double> coralXOffset 
    {
        {"LEFT", 0.0}, // TODO: Tune
        {"RIGHT", 0.0}  // TODO: Tune

    };

  double speedFactor = 1.0;

  void ConfigureButtonBindings();

  bool stupidTest = true;
};
