/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MecanumDriveCommand.h"
#include "OI.h"

MecanumDriveCommand::MecanumDriveCommand(bool g) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  // Requires mecanumDriveSystem();
  useGyro = g;
  mecanumDriveSystem->GyroReset();
}

// Called just before this Command runs the first time
void MecanumDriveCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MecanumDriveCommand::Execute() {
    // Check Start button
#if 1
  if (oi->m_XboxDriver->GetStartButton()) {
		mecanumDriveSystem->GyroReset();
  }
#else
  if (oi->m_XboxDriver->GetStartButtonPressed()) {
		ToggleUseGyro();
  }
  if (oi->m_XboxDriver->GetBackButtonPressed()) {
		ToggleDriveSide();
  }
#endif
  // TBD: the visionOffset needs a scale and a max limit
  if (oi->m_XboxDriver->GetAButton()) {
    CommandBase::visionEnabled = true;
    mecanumDriveSystem->Go(0, 0.5 * CommandBase::visionOffset, 0);
  }
  else {
    CommandBase::visionEnabled = false;
    CommandBase::visionOffset = 0.0;
    double x;
    double y;
    if (driveSideways) {
      x = GetInvertedY();
      y = GetX();
    }
    else {
      x = GetX();
      y = GetInvertedY();
    }
    if (useGyro) {
      mecanumDriveSystem->Saucer(x, y, this->GetTwist());
    }
    else {
      mecanumDriveSystem->Go(x, y, this->GetTwist());
    }
  }
}

// Make this return true when this Command no longer needs to run execute()
bool MecanumDriveCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void MecanumDriveCommand::End() {
  mecanumDriveSystem->Stop();  
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MecanumDriveCommand::Interrupted() {
  mecanumDriveSystem->Stop();
}

double MecanumDriveCommand::GetX()
{
	return Deadband(oi->m_XboxDriver->GetRawAxis(0)); // Left stick, X axis
}
double MecanumDriveCommand::GetY()
{
	return Deadband(oi->m_XboxDriver->GetRawAxis(1)); // Left Stick, Y axis
}

double MecanumDriveCommand::GetInvertedY()
{
	return -Deadband(oi->m_XboxDriver->GetRawAxis(1)); // Left Stick, Y axis, Inverted
}

double MecanumDriveCommand::GetTwist()
{
	return Deadband(oi->m_XboxDriver->GetRawAxis(4)); // Right stick, X axis
}

void MecanumDriveCommand::ToggleUseGyro() {
  useGyro = !useGyro;
}

void MecanumDriveCommand::ToggleDriveSide() {
  driveSideways = !driveSideways;
}

double MecanumDriveCommand::Deadband(double val)
{
  double newVal;
  if (val > -DEADBAND && val < DEADBAND) {
    return 0.0;
  }
  if (val != 0) {
   newVal = pow(val, 2);
  } 
  if (val >= 0) {
    return newVal;
  }
  else {
    return -newVal;
  }
}