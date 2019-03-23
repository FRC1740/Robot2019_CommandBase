/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/GamePieceManipulatorManual.h"
#include "subsystems/GamePieceManipulator.h"
#include <frc/smartdashboard/SmartDashboard.h>

GamePieceManipulatorManual::GamePieceManipulatorManual() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(gamePieceManipulator);
}

// Called just before this Command runs the first time
void GamePieceManipulatorManual::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void GamePieceManipulatorManual::Execute() {

  double velocity;
  
  // Left trigger minus right trigger will provide input to move from range -1 to 1
  velocity = oi->m_XboxCoDriver->GetRawAxis(HINGE_LOWER_INPUT_AXIS) - oi->m_XboxCoDriver->GetRawAxis(HINGE_RAISE_INPUT_AXIS);
#define GPM_DEADBAND 0.15
  if ((velocity < GPM_DEADBAND) && (velocity > -GPM_DEADBAND)) {
    velocity = oi->m_XboxDriver->GetRawAxis(HINGE_LOWER_INPUT_AXIS) - oi->m_XboxDriver->GetRawAxis(HINGE_RAISE_INPUT_AXIS); 
  }

  gamePieceManipulator->Move(velocity);
  // Display arm/hinge position on the dashboard
  frc::SmartDashboard::PutNumber("Left Hinge", gamePieceManipulator->GetLPosition());
  frc::SmartDashboard::PutNumber("Right Hinge", gamePieceManipulator->GetRPosition());

}

// Make this return true when this Command no longer needs to run execute()
bool GamePieceManipulatorManual::IsFinished() { return false; }

// Called once after isFinished returns true
void GamePieceManipulatorManual::End() {
  gamePieceManipulator->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GamePieceManipulatorManual::Interrupted() {
  gamePieceManipulator->Stop();
}
