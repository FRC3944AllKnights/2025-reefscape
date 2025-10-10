#include "commands/autonomous.h"

#include <frc/SmartDashboard/SmartDashboard.h>

using namespace AutoConstants;

frc2::CommandPtr autos::DriveForward(DriveSubsystem* drive) {
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg});},
            // onExecute: Drive forward
            [drive] {drive->Drive(0.15_mps, 0_mps, 0_rad_per_s, false, true);},
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // isFinished: Has it driven forward?
            [drive] {return drive->GetPose().X() >= 1_m;},
            // requirements: drive
            {drive}
        ).ToPtr()
    );
}

frc2::CommandPtr autos::OneCoralCenterAutomatic(DriveSubsystem* drive, ElevatorSubsystem* elevator, OuttakeSubsystem* outtake) {
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {
                drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); // Does this work if the robot is rotated?
                drive->m_autonTimer.Restart();  
            },
            // onExecute: Auto align to coral
            [drive] {
                DriveSubsystem::velocity2D velocities = drive->SnapToCoral("RIGHT");
                drive->Drive(
                    units::velocity::meters_per_second_t {velocities.x},
                    units::velocity::meters_per_second_t {velocities.y},
                    units::radians_per_second_t {velocities.theta},
                    true, true);
                },
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // isFinished: Has it driven forward?
            [drive] {
                return (drive->isSnappedToCoral("RIGHT") || drive->m_autonTimer.Get() > 7_s);
                },
            // requirements: drive
            {drive}
        ).ToPtr(),
    frc2::FunctionalCommand(
            // onInit: Raise elevator to level 4
            [elevator, drive] {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                elevator->setElevatorLevel(4);},
            // onExecute: None
            [elevator, drive] {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 4?
            [elevator] {return elevator->isAtTop();},
            // requirements: elevator
            {elevator}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {drive->poseOne = drive->GetPose();},
            // onExecute: Drive forward, robot-relative
            [drive] {
                drive->Drive(
                    units::velocity::meters_per_second_t {0.1_mps},
                    units::velocity::meters_per_second_t {0.0_mps},
                    units::radians_per_second_t {0.0_rad_per_s},
                    false, true);},
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // isFinished: Has it driven forward enough?
            [drive] {
                return drive->GetPose().X() >= drive->poseOne.X() + 0.080_m;
                },
            // requirements: drive
            {drive}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: set outtake motors to run
            [outtake] {
                outtake->SetOuttakeMotors(true);
                outtake->m_autonTimer.Restart();},
            // onExecute: None
            [outtake] {;},
            // onEnd: None
            [outtake](bool interrupted) {outtake->SetOuttakeMotors(false);},
            // isFinished: is the coral out of the robot?
            [outtake] {return outtake->m_autonTimer.Get() > 1.5_s;},
            // requirements: outtake
            {outtake}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: Lower elevator to level 0
            [elevator, drive] {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                elevator->setElevatorLevel(0);},
            // onExecute: None
            [elevator, drive] {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 0?
            [elevator] {return elevator->isAtBottom();},
            // requirements: elevator
            {elevator}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {drive->poseOne = drive->GetPose();},
            // onExecute: Drive backward, robot-relative
            [drive] {
                drive->Drive(
                    units::velocity::meters_per_second_t {-0.5_mps},
                    units::velocity::meters_per_second_t {0.0_mps},
                    units::radians_per_second_t {0.0_rad_per_s},
                    false, true);},
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // isFinished: Has it driven backward enough?
            [drive] {
                return drive->GetPose().X() <= drive->poseOne.X() - 0.3_m;
                },
            // requirements: drive
            {drive}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {
                int apriltagID = LimelightHelpers::getFiducialID("limelight-intake");
                if (apriltagID == 9 || apriltagID == 22) {
                    // Right auton (from driver view), align to right side
                    drive->autonTargetAngle = 54.0;
                }
                else if (apriltagID == 11 || apriltagID == 20) {
                    // Left auton (from driver view), align to left side
                    drive->autonTargetAngle = 306.0;
                }
                else {
                    // Center auton, remain head-on
                    drive->autonTargetAngle = 0.0;
                }
                drive->poseOne = drive->GetPose();
                drive->m_autonTimer.Restart();
            },
            // onExecute: Snap robot heading to coral station
            [drive] {
                if (drive->autonTargetAngle == 0.0) {
                    // Don't rotate if doing center auton
                    return;
                }
                drive->rotationPID.EnableContinuousInput(0,360);
                
                double theta = drive->rotationPID.Calculate(drive->GetNormalizedHeading(), drive->autonTargetAngle);
                drive->Drive(
                    units::velocity::meters_per_second_t {0.0_mps},
                    units::velocity::meters_per_second_t {0.0_mps},
                    units::radians_per_second_t {theta},
                    false, true);},
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
            // isFinished: Has it been aligning for half a second?
            [drive] {return drive->m_autonTimer.Get() > 0.5_s;},
            // requirements: drive
            {drive}
        ).ToPtr()
    );
}
