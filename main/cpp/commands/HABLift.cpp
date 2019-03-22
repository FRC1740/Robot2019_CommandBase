/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/HABLift.h"
#include "subsystems/HABClimber.h"
#include "OI.h"

#define DEADBAND .15

HABLift::HABLift() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(habClimber);
}

// Called just before this Command runs the first time
void HABLift::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HABLift::Execute() {
  // Front Lifter
#define USE_DPAD
#ifdef USE_DPAD
  int dpad = oi->m_XboxDriver->GetPOV();
  if (dpad >= 0) {
    if ((dpad >= 135) && (dpad <= 225)) {
      habClimber->LiftFront();
      habClimber->LiftRear();
    }
    else if ((dpad >= 0) && (dpad <= 90)) {
      habClimber->LowerFront();
    }
  }
  else {
    habClimber->StopFront();
    habClimber->StopRear();
  }
  if (oi->m_XboxCoDriver->GetBButton()) {
    habClimber->LowerRear();
  }
  else if (dpad < 0) {
    habClimber->StopFront();
    habClimber->StopRear();
  }
  double habSpeed = oi->m_XboxDriver->GetRawAxis(5);
  if (habSpeed > DEADBAND) {
    habClimber->DriveFwd(habSpeed);
  }
  else if (habSpeed < -DEADBAND) {
    habClimber->DriveRev(-habSpeed);
  }
  else {
    habClimber->DriveStop();
  }
#else // USE_DPAD
  // Negate input because FORWARD on Y-Axis IS NEGATIVE
  if (-oi->m_XboxCoDriver->GetRawAxis(1) > DEADBAND) {
    habClimber->LiftFront();
  }
  // Negate input because FORWARD on Y-Axis IS NEGATIVE
  else if (-oi->m_XboxCoDriver->GetRawAxis(1) < -DEADBAND) {
    habClimber->LowerFront();
  }
  else {
     habClimber->StopFront();
  }
#endif // USE_DPAD  // Rear Lifter  
#ifdef USE_DPAD
#else // USE_DPAD
  // Negate input because FORWARD on Y-Axis IS NEGATIVE
  if (-oi->m_XboxCoDriver->GetRawAxis(5) > DEADBAND) {
    habClimber->LiftRear();
  }
  // Negate input because FORWARD on Y-Axis IS NEGATIVE
  else if (-oi->m_XboxCoDriver->GetRawAxis(5) < -DEADBAND) {
    habClimber->LowerRear();
  }
  else {
    habClimber->StopRear();
  }
#endif // USE_DPAD
}

// Make this return true when this Command no longer needs to run execute()
bool HABLift::IsFinished() { return false; }

// Called once after isFinished returns true
void HABLift::End() {
  habClimber->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void HABLift::Interrupted() {
  habClimber->Stop();
}
