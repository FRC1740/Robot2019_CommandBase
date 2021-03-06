/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "commands/DriveBy.h"
#include "subsystems/GamePieceManipulator.h"

DriveBy::DriveBy() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(gamePieceManipulator);
}

// Called just before this Command runs the first time
void DriveBy::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveBy::Execute() {
    gamePieceManipulator->DriveByActivate();
    printf("DriveByActivate\n");
}

// Make this return true when this Command no longer needs to run execute()
bool DriveBy::IsFinished() { return false; }

// Called once after isFinished returns true
void DriveBy::End() {
    gamePieceManipulator->DriveByDeactivate();
    printf("DriveByDeactivate\n");
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveBy::Interrupted() {
    gamePieceManipulator->DriveByDeactivate();
    printf("DriveByDeactivate\n");
}
