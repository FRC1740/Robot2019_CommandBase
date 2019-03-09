#include "CommandBase.h"
#include <frc/commands/Scheduler.h>

// Initialize a single static pointer/instance of all of your subsystems to NULL
OI* CommandBase::oi = NULL;
MecanumDriveSystem* CommandBase::mecanumDriveSystem = NULL;
GamePieceManipulator* CommandBase::gamePieceManipulator = NULL;
HABClimber* CommandBase::habClimber = NULL;
Utility* CommandBase::utility = NULL;
double CommandBase::visionOffset = 0.0;
bool CommandBase::visionEnabled = false;

CommandBase::CommandBase(char const *name): frc::Command(name) {}

CommandBase::CommandBase(): frc::Command() {}

void CommandBase::init()
{
    oi = new OI();
    mecanumDriveSystem = new MecanumDriveSystem();
    gamePieceManipulator = new GamePieceManipulator();
    habClimber = new HABClimber();
    utility = new Utility();
}
