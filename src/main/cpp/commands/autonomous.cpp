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

frc2::CommandPtr autos::DriveForwardAndScore(DriveSubsystem* drive, ElevatorSubsystem* elevator, OuttakeSubsystem* outtake) {
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg});},
            // onExecute: Drive forward
            [drive] {drive->Drive(0.15_mps, 0_mps, 0_rad_per_s, false, true);
            frc::SmartDashboard::PutBoolean("Driving in Auto", true);},
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
            frc::SmartDashboard::PutBoolean("Driving in Auto", false);},
            // isFinished: Has it driven forward?
            [drive] {return drive->GetPose().X() >= 0.5_m;}, // 2.235_m
            // requirements: drive
            {drive}
        ).ToPtr(),
    frc2::FunctionalCommand(
            // onInit: Raise elevator to level 4
            [elevator] {elevator->setElevatorLevel(4);},
            // onExecute: None
            [elevator] {;},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 4?
            [elevator] {return elevator->isAtTop();},
            // requirements: elevator
            {elevator}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: set outtake motors to run
            [outtake] {outtake->SetOuttakeMotors(true);},
            // onExecute: None
            [outtake] {;},
            // onEnd: None
            [outtake](bool interrupted) {;},
            // isFinished: is the coral out of the robot?
            [outtake] {return true;},
            // requirements: intake
            {outtake}
        ).ToPtr()
    );
}
/*
frc2::CommandPtr autos::LockOnAndScore(DriveSubsystem* drive, ElevatorSubsystem* elevator, OuttakeSubsystem* outtake) {
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg});},
            // onExecute: Drive forward
            [drive] {drive->Drive(0.15_mps, 0_mps, 0_rad_per_s, false, true);
            frc::SmartDashboard::PutBoolean("Driving in Auto", true);},
            // onEnd: Stop driving
            [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
            frc::SmartDashboard::PutBoolean("Driving in Auto", false);},
            // isFinished: Has it driven forward?
            [drive] {return drive->GetPose().X() >= 2_m;}, // 2.235_m
            // requirements: drive
            {drive}
        ).ToPtr(),
    frc2::FunctionalCommand(
            // onInit: Raise elevator to level 4
            [elevator] {elevator->setElevatorLevel(4);},
            // onExecute: None
            [elevator] {;},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 4?
            [elevator] {return elevator->isAtTop();},
            // requirements: elevator
            {elevator}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: set outtake motors to run
            [outtake] {outtake->SetOuttakeMotors(true);},
            // onExecute: None
            [outtake] {;},
            // onEnd: None
            [outtake](bool interrupted) {;},
            // isFinished: is the coral out of the robot?
            [outtake] {return true;},
            // requirements: intake
            {outtake}
        ).ToPtr()
    );
}
*/
frc2::CommandPtr autos::RaiseLevel4AndScore(ElevatorSubsystem* elevator, OuttakeSubsystem* outtake) {
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: Raise elevator to level 4
            [elevator] {elevator->setElevatorLevel(4);},
            // onExecute: None
            [elevator] {;},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 4?
            [elevator] {return elevator->getLevel() == 4;},
            // requirements: elevator
            {elevator}
        ).ToPtr(),
        frc2::FunctionalCommand(
            // onInit: set motor speed to 100
            [outtake] {outtake->SetOuttakeMotors(true);},
            // onExecute: None
            [outtake] {;},
            // onEnd: None
            [outtake](bool interrupted) {outtake->SetOuttakeMotors(false);},
            // isFinished: is the coral out of the robot?
            [outtake] {return !outtake->GamePieceDetected();},
            // requirements: intake
            {outtake}
        ).ToPtr(),
      frc2::FunctionalCommand(
            // onInit: Lower elevator to level 0
            [elevator] {elevator->setElevatorLevel(0);},
            // onExecute: None
            [elevator] {;},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 0?
            [elevator] {return elevator->getLevel() == 0;},
            // requirements: elevator
            {elevator}
        ).ToPtr()
    );
}

/*
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: None
            [drive] {;},
            // onExecute: Move towards intake-side AprilTag
            [drive] {;}, // TODO
            // onEnd: None
            [drive](bool interrupted) {;},
            // isFinished: Is robot at chute?
            [drive] {return true;}, // TODO
            // requirements: drive
            {drive}
        ).ToPtr()
    );
    */