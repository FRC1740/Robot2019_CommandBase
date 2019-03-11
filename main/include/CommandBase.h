#ifndef COMMANDBASE_H
#define COMMANDBASE_H

//#include <string>
#include <frc/commands/Command.h>
#include <frc/WPILib.h>
#include "OI.h"
#include "subsystems/MecanumDriveSystem.h"
#include "subsystems/GamePieceManipulator.h"
#include "subsystems/HABClimber.h"
#include "subsystems/Utility.h"


/**
 * The base for all commands. All atomic commands should subclass CommandBase.
 * CommandBase stores creates and stores each control system. To access a
 * subsystem elsewhere in your code in your code use CommandBase.examplesubsystem
 */
class CommandBase: public frc::Command
{
public:
	CommandBase(char const *name);
	CommandBase();
	static void init();
	// Create a single static instance of all of your subsystems
	static OI *oi;
	static MecanumDriveSystem *mecanumDriveSystem;
    static GamePieceManipulator *gamePieceManipulator;
	static HABClimber *habClimber;
	static Utility *utility;
	static double visionOffset;
	static bool visionEnabled;
};
#endif