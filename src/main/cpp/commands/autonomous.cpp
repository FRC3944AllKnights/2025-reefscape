#include "commands/autonomous.h"

#include <frc/SmartDashboard/SmartDashboard.h>

using namespace AutoConstants;

frc2::CommandPtr autos::RaiseLevel4(ElevatorSubsystem* elevator) {
    return frc2::cmd::Sequence(
        frc2::FunctionalCommand(
            // onInit: None
            [elevator] {;},
            // onExecute: Raise elevator to level 4
            [elevator] {elevator->setElevatorLevel(4);},
            // onEnd: None
            [elevator](bool interrupted) {;},
            // isFinished: Is elevator at level 4?
            [elevator] {return elevator->getLevel() == 4;},
            // requirements: elevator
            {elevator}
        ).ToPtr()
    );
}