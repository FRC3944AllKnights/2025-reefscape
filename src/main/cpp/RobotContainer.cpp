// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include <units/angle.h>
#include <units/velocity.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "LimelightHelpers.h"


using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {
    frc::SmartDashboard::PutData("auto modes", &m_chooser);
     
  // Initialize all of your commands and subsystems here

  // Configure the button bindingsR
  ConfigureButtonBindings();

 // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // Get controller inputs
        double x = -frc::ApplyDeadband(m_driverController.GetLeftY(), OIConstants::kDriveDeadband);
        double y = -frc::ApplyDeadband(m_driverController.GetLeftX(), OIConstants::kDriveDeadband);
        double theta = -frc::ApplyDeadband(m_driverController.GetRightX(), OIConstants::kDriveDeadband);

        // Check for Limelight hijacking
        int POVReading = m_driverController.GetPOV();
        if (POVReading == 180) {
          // Command: Auto-align intake-side
          double xComponent = 0.0;
          double yComponent = 0.0;
          double translationTheta = (isRed) ? 60 : 300;
          if(LimelightHelpers::getTX("limelight-intake") != 0)
          {
              xComponent = translationPID.Calculate(LimelightHelpers::getTX(""), 0.0)*cos(DegreeToRad(translationTheta));
              yComponent = -translationPID.Calculate(LimelightHelpers::getTX(""), 0.0)*sin(DegreeToRad(translationTheta));
          }
          if(LimelightHelpers::getTY("limelight-intake") != 0)
          {
              xComponent += translationPID.Calculate(LimelightHelpers::getTY("limelight-intake"), desiredPosYIntake)*sin(DegreeToRad(translationTheta));
              yComponent += translationPID.Calculate(LimelightHelpers::getTY("limelight-intake"), desiredPosYIntake)*cos(DegreeToRad(translationTheta));
          }
          x = xComponent;
          y = yComponent;
          rotationPID.EnableContinuousInput(0,360);
          theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), translationTheta);
        }
        else if (POVReading == 90 || POVReading == 135 || POVReading == 45) {
          // Command: Auto-align outtake-side (right)
          double xComponent = 0.0;
          double yComponent = 0.0;
          double translationTheta = (isRed) ? 60 : 300;
          if(LimelightHelpers::getTX("limelight-intake") != 0)
          {
              xComponent = translationPID.Calculate(LimelightHelpers::getTX("limelight-outtake"), desiredPosXOuttakeRight)*cos(DegreeToRad(translationTheta));
              yComponent = -translationPID.Calculate(LimelightHelpers::getTX("limelight-outtake"), desiredPosXOuttakeRight)*sin(DegreeToRad(translationTheta));
          }
          if(LimelightHelpers::getTY("limelight-intake") != 0)
          {
              xComponent += translationPID.Calculate(LimelightHelpers::getTY("limelight-outtake"), desiredPosYOuttake)*sin(DegreeToRad(translationTheta));
              yComponent += translationPID.Calculate(LimelightHelpers::getTY("limelight-outtake"), desiredPosYOuttake)*cos(DegreeToRad(translationTheta));
          }
          x = xComponent = 0.0;
          y = yComponent = 0.0;
          rotationPID.EnableContinuousInput(0,360);
          theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), translationTheta);
        }
        else if (POVReading == 270 || POVReading == 225 || POVReading == 315) {
          // Command: Auto-align outtake-side (left)
          double xComponent = 0.0;
          double yComponent = 0.0;
          double translationTheta = (isRed) ? 60 : 300;
          if(LimelightHelpers::getTX("limelight-intake") != 0)
          {
              xComponent = translationPID.Calculate(LimelightHelpers::getTX("limelight-outtake"), desiredPosXOuttakeLeft)*cos(DegreeToRad(translationTheta));
              yComponent = -translationPID.Calculate(LimelightHelpers::getTX("limelight-outtake"), desiredPosXOuttakeLeft)*sin(DegreeToRad(translationTheta));
          }
          if(LimelightHelpers::getTY("limelight-intake") != 0)
          {
              xComponent += translationPID.Calculate(LimelightHelpers::getTY("limelight-outtake"), desiredPosYOuttake)*sin(DegreeToRad(translationTheta));
              yComponent += translationPID.Calculate(LimelightHelpers::getTY("limelight-outtake"), desiredPosYOuttake)*cos(DegreeToRad(translationTheta));
          }
          x = xComponent;
          y = yComponent;
          rotationPID.EnableContinuousInput(0,360);
          theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), translationTheta);
        }
        
        // Apply calculated velocities (drive)
        m_drive.Drive(units::meters_per_second_t{x}, units::meters_per_second_t{y}, units::radians_per_second_t{theta}, true, true);
      },
      {&m_drive}));

    
    // Build an auto chooser. This will use frc2::cmd::None() as the default option.
  autoChooser = AutoBuilder::buildAutoChooser();

  // Another option that allows you to specify the default auto by its name
  // autoChooser = AutoBuilder::buildAutoChooser("My Default Auto");

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  
  // Register Named Commands. You must pass either a CommandPtr rvalue or a shared_ptr to the command, not the command directly.
  NamedCommands::registerCommand("RaiseLevel4AndScore", std::move(autos::RaiseLevel4AndScore(&m_ElevatorSubsystem, &m_IntakeSubsystem)));
    

}

void RobotContainer::ConfigureButtonBindings() {
    // Spin intake - A button
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kA)
       .WhileFalse(new frc2::RunCommand([this] { m_IntakeSubsystem.SetIntakeMotorSpeed(0);})).WhileTrue(new frc2::RunCommand([this] {m_IntakeSubsystem.SetIntakeMotorSpeed(-.6);}));

    // Raise climber - Y button
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kY)
        .OnTrue(new frc2::InstantCommand([this] { m_ClimberSubsystem.raiseClimber();}));

    //Lower climber - X button
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kX)
        .OnTrue(new frc2::InstantCommand([this] { m_ClimberSubsystem.lowerClimber();}));

    // Raise elevator (simple) - right trigger
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Axis::kRightTrigger)
        .WhileTrue(new frc2::InstantCommand([this] { m_ElevatorSubsystem.raiseElevatorSimple(0.1 * m_driverController.GetRightTriggerAxis());}));
        //.GetRightTriggerAxis

         // lower elevator (simple) - left trigger
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Axis::kLeftTrigger)
        .WhileTrue(new frc2::InstantCommand([this] { m_ElevatorSubsystem.raiseElevatorSimple(-0.1 * m_driverController.GetLeftTriggerAxis());}));
}
    

frc2::Command* RobotContainer::getAutonomousCommand() {
  // Returns a frc2::Command* that is freed at program termination
  return autoChooser.GetSelected();
}

double RobotContainer::DegreeToRad(double degree){
    return degree*3.14159/180;
}
