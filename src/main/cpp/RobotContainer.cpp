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
  
  m_chooser.SetDefaultOption("Drive Forward", m_DriveForward.get());
  m_chooser.AddOption("Drive Forward And Score", m_DriveForwardAndScore.get());
  m_chooser.AddOption("One Coral Center Automatic", m_OneCoralCenterAutomatic.get());
  frc::SmartDashboard::PutData("auto modes", &m_chooser);
     
  // Initialize all of fyour commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();



 // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // Get controller inputs
        double x = -frc::ApplyDeadband(m_driverController.GetLeftY(), OIConstants::kDriveDeadband) * speedFactor;
        double y = -frc::ApplyDeadband(m_driverController.GetLeftX(), OIConstants::kDriveDeadband) * speedFactor;
        double theta = -frc::ApplyDeadband(m_driverController.GetRightX(), OIConstants::kDriveDeadband) * speedFactor;
        
        m_ElevatorSubsystem.ReportMotors();
        //frc::SmartDashboard::PutNumber("Speed Factor", speedFactor);
        // Set Limelight LEDs
        /*
        if (LimelightHelpers::getTV("limelight-intake") >= 1.0) { // Target detected intake-side
          m_OuttakeSubsystem.SetColorLEDIntakeTargetDetected(0, 0, 255);
        }
        else {
          m_OuttakeSubsystem.SetColorLEDIntakeTargetDetected(0, 0, 0);
        }
        if (LimelightHelpers::getTV("limelight-outtake") >= 1.0) { // Target detected outtake-side
          m_OuttakeSubsystem.SetColorLEDOuttakeTargetDetected(0, 0, 255);
        }
        else {
          m_OuttakeSubsystem.SetColorLEDOuttakeTargetDetected(0, 0, 0);
        }
        */
       //calculate theta error
        int ID = LimelightHelpers::getFiducialID("limelight-intake");
        posTheta = coralAngles[ID];// - absoluteFieldOffset; //convert from absolute field angles to angles relative to the robot starting pose
        posTheta -= absoluteFieldOffset; //convert from absolute field angles to angles relative to the robot starting pose
        if (posTheta < 0.0) {
          posTheta += 360;
        }
        /*
        double allowedError = 0.005;
        double tolerance = 3;
        double error = abs(posTheta - m_drive.GetNormalizedHeading());
        bool thetaGood = error < tolerance;

        frc::SmartDashboard::PutBoolean("Limelight: thetaGood", thetaGood);
        frc::SmartDashboard::PutNumber("Limelight Theta Error", error);
        */
        
       // TODO: Re-enable these dashboard values
        //frc::SmartDashboard::PutNumber("tx", LimelightHelpers::getTX("limelight-intake"));
        //frc::SmartDashboard::PutNumber("ty", LimelightHelpers::getTY("limelight-intake"));
        //frc::SmartDashboard::PutNumber("tv", LimelightHelpers::getTV("limelight-intake"));


        // Check for Limelight hijacking
        int POVReading = m_driverController.GetPOV();
        bool intakeButton = m_driverController.GetBButton();
        if (POVReading == 90 || POVReading == 135 || POVReading == 45) {
          // Command: Auto-align outtake-side (right)
          if (intakeButton) {
            rotationPID.EnableContinuousInput(0,360);
            theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), 54);
            /*
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {
              theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), 54);
            }
            else {
              theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), 234);
            }
            */
          }
          // if a valid apriltag is detected, run SnapToCoral logic
          else if (LimelightHelpers::getTV("limelight-intake")) {
              velocity2D velocities = SnapToCoral("RIGHT");
              x = velocities.x;
              y = velocities.y;
              theta = velocities.theta;
          }
        }

        else if (POVReading == 270 || POVReading == 225 || POVReading == 315) {
          // Command: Auto-align outtake-side (left)
          if (intakeButton) {
            rotationPID.EnableContinuousInput(0,360);
            theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), 306);
            /*
            if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue) {
              theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), 306);
            }
            else {
              theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), 126);
            }
            */
          }
          else if (LimelightHelpers::getTV("limelight-intake")) {
            velocity2D velocities = SnapToCoral("LEFT");
            x = velocities.x;
            y = velocities.y;
            theta = velocities.theta;
          }
        }

        else if (POVReading == 0) {
          // Command: Drive straight forward, robot-relative
          DriveSubsystem::velocity2D velocities = m_drive.DriveStraightForward();
          x = velocities.x;
          y = velocities.y;
          theta = velocities.theta;
        }
        
        // Apply calculated velocities (drive)
        if (m_driverController.GetXButton()) {
          m_drive.SetX();
        }
        else {
          m_drive.Drive(units::meters_per_second_t{x}, units::meters_per_second_t{y}, units::radians_per_second_t{theta}, true, true);
        }
      },
      {&m_drive}));

    
    // Build an auto chooser. This will use frc2::cmd::None() as the default option.
  //autoChooser = AutoBuilder::buildAutoChooser();

  // Another option that allows you to specify the default auto by its name
  // autoChooser = AutoBuilder::buildAutoChooser("My Default Auto");

  //frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  
  // Register Named Commands. You must pass either a CommandPtr rvalue or a shared_ptr to the command, not the command directly.
  //NamedCommands::registerCommand("RaiseLevel4AndScore", std::move(autos::RaiseLevel4AndScore(&m_ElevatorSubsystem, &m_OuttakeSubsystem)));
    
    m_ElevatorSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // Raise elevator (simple) - right trigger
        // If game piece detected don't raise unless safety mode = True
        if (m_driverController.GetRightTriggerAxis() > 0.0 && (!m_OuttakeSubsystem.GamePieceDetected() || !m_ElevatorSubsystem.getSafetyMode())) {
          m_ElevatorSubsystem.raiseElevatorSimple(m_driverController.GetRightTriggerAxis());
        }

    // lower elevator (simple) - left trigger
    // If game piece detected don't lower unless safety mode = True
        if (m_driverController.GetLeftTriggerAxis() > 0.0 && (!m_OuttakeSubsystem.GamePieceDetected() || !m_ElevatorSubsystem.getSafetyMode())) {
          m_ElevatorSubsystem.lowerElevatorSimple(m_driverController.GetLeftTriggerAxis());
        }

        // Dashboard
        //frc::SmartDashboard::PutNumber("Elevator Encoder", m_ElevatorSubsystem.getHeight());

        // Set speedFactor
        if (m_ElevatorSubsystem.getLevel() > 0 && m_ElevatorSubsystem.getSafetyMode()) {
          speedFactor = 0.25;
        }
        else {
          speedFactor = 1.0;
        }
      },
    {&m_ElevatorSubsystem}));
}

RobotContainer::velocity2D RobotContainer::SnapToCoral(std::string direction) {
    velocity2D velocities;

    if (LimelightHelpers::getTV("limelight-intake") == 0) {
      return velocities;
    }
    
    velocities.x += xTranslationPID.Calculate(LimelightHelpers::getTX("limelight-intake"), coralXOffset[direction]) * sin(DegreeToRad(posTheta));
    velocities.y += xTranslationPID.Calculate(LimelightHelpers::getTX("limelight-intake"), coralXOffset[direction]) * cos(DegreeToRad(posTheta));
    velocities.x += -yTranslationPID.Calculate(LimelightHelpers::getTY("limelight-intake"), desiredPosYOuttake)*cos(DegreeToRad(posTheta));
    velocities.y += -yTranslationPID.Calculate(LimelightHelpers::getTY("limelight-intake"), desiredPosYOuttake) *sin(DegreeToRad(posTheta));
    rotationPID.EnableContinuousInput(0,360);
    velocities.theta = rotationPID.Calculate(m_drive.GetNormalizedHeading(), posTheta);
    
    return velocities;
}

void RobotContainer::ConfigureButtonBindings() {
    // Spin outtake - A button
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kA)
       .WhileTrue(new frc2::RunCommand([this] {m_OuttakeSubsystem.SetOuttakeMotors(true);})).OnFalse(new frc2::RunCommand([this] { m_OuttakeSubsystem.SetOuttakeMotors(false);}));

    // Intake gamepiece - B button
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kB)
       .WhileTrue(new frc2::RunCommand([this] {
        if (m_ElevatorSubsystem.getTargetLevel() != 0) {
          if (!m_OuttakeSubsystem.GamePieceDetected() || m_ElevatorSubsystem.getSafetyMode()) {
            m_ElevatorSubsystem.setElevatorLevel(0);
          }
        }
        m_OuttakeSubsystem.IntakeCoral();
        })).OnFalse(new frc2::RunCommand([this] { m_OuttakeSubsystem.SetOuttakeMotors(false);}));

    // Leave safety mode - Y button
 
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kY)
        .WhileTrue(new frc2::InstantCommand([this] {m_ElevatorSubsystem.setSafetyMode(false);
          frc::SmartDashboard::PutBoolean("Safety Mode", false);}))
        .WhileFalse(new frc2::InstantCommand([this] {m_ElevatorSubsystem.setSafetyMode(true);
          frc::SmartDashboard::PutBoolean("Safety Mode", true);}));

    //Lower climber - X button
    /*
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kX)
        .OnTrue(new frc2::InstantCommand([this] { m_drive.SetX();}));
    */
   
    // Raise elevator (tiered) - right bumper
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kRightBumper)
        .OnTrue(new frc2::InstantCommand([this] {
          if (!m_ElevatorSubsystem.getSafetyMode() || !m_OuttakeSubsystem.GamePieceDetected()) {
            // = 0.25; // set drivebase to quarter speed for safety and more control
            m_ElevatorSubsystem.raiseElevatorTiered();
          }
        }));

    // Lower elevator (tiered) - left bumper
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kLeftBumper)
        .OnTrue(new frc2::InstantCommand([this] {
          
          if (!m_ElevatorSubsystem.getSafetyMode() || !m_OuttakeSubsystem.GamePieceDetected()) {
            m_ElevatorSubsystem.lowerElevatorTiered();
          }
          /*
          if (m_ElevatorSubsystem.getTargetLevel() == 0) {
            speedFactor = 1.0;
          }
          */
        }));

    //e-stop elevator
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kBack)
        .WhileTrue(new frc2::RunCommand([this] {
          m_ElevatorSubsystem.disableElevator();
        }));

    //reset encoders after an e-stop or brown-out
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kStart)
        .OnTrue(new frc2::InstantCommand([this] {
          m_ElevatorSubsystem.resetElevatorEncoder();
        }));

    /*
      // Set swerve to X
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kX)
       .OnTrue(new frc2::RunCommand([this] {m_drive.SetX();}));
    */
}
    

frc2::Command* RobotContainer::getAutonomousCommand() {
  // Returns a frc2::Command* that is freed at program termination
  return m_chooser.GetSelected();
}

double RobotContainer::DegreeToRad(double degree){
    return degree*3.14159/180;
}
