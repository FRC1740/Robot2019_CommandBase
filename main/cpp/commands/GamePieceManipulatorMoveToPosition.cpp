/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/GamePieceManipulatorMoveToPosition.h"
#include "subsystems/GamePieceManipulator.h"
#include <frc/smartdashboard/SmartDashboard.h>

constexpr double posStow          = 120.0; // A
constexpr double posHatchLoad     =  90.0; // B
constexpr double posGround        =   0.0; // C
constexpr double posCargoLoad     =  90.0; // D
constexpr double posDeliverRocket =  60.0; // E
constexpr double posDeliverShip   =  90.0; // F
constexpr double posPartialStow   = 100.0; // G

GamePieceManipulatorMoveToPosition::GamePieceManipulatorMoveToPosition() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(gamePieceManipulator);
}

// Called just before this Command runs the first time
void GamePieceManipulatorMoveToPosition::Initialize() {
  m_setpoint = posStow;
  //gamePieceManipulator->EnablePIDLoop();  // Enable this after PID is working
}

// Called repeatedly when this Command is scheduled to run
void GamePieceManipulatorMoveToPosition::Execute() {
  /*
  from driver_x, driver_y, codriver_x, codriver_y get named position
  if named position is different from saved
    set setpoint
  save named position
  */
}

// Make this return true when this Command no longer needs to run execute()
bool GamePieceManipulatorMoveToPosition::IsFinished() { return false; }

// Called once after isFinished returns true
void GamePieceManipulatorMoveToPosition::End() {
  gamePieceManipulator->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GamePieceManipulatorMoveToPosition::Interrupted() {
  gamePieceManipulator->Stop();
}
