#include "commands/autonomous.h"

#include <frc/SmartDashboard/SmartDashboard.h>

using namespace AutoConstants;

frc2::CommandPtr autos::RaiseLevel4AndScore(ElevatorSubsystem* elevator, IntakeSubsystem* intake) {
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
            [intake] {intake->SetIntakeMotors(true);},
            // onExecute: None
            [intake] {;},
            // onEnd: None
            [intake](bool interrupted) {intake->SetIntakeMotors(false);},
            // isFinished: is the coral out of the robot?
            [intake] {return !intake->GamePieceDetected();},
            // requirements: intake
            {intake}
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

frc2::CommandPtr autos::AlignChute(DriveSubsystem* drive) {
    
    
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