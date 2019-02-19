/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "CommandBase.h"

// Example Commands & Subsystem
#include "commands/ExampleCommand.h"
#include "commands/MyAutoCommand.h"
#include "subsystems/ExampleSubsystem.h"

// Drive commands & Subsystem
#include "commands/MecanumDriveCommand.h"
#include "subsystems/MecanumDriveSystem.h"

// Game Piece Manipulator commands & Subsystem
#include "commands/GamePieceManipulatorManual.h"
#include "commands/GamePieceManipulatorMoveToPosition.h"
#include "commands/HatchPanelEject.h"
#include "subsystems/GamePieceManipulator.h"

#include "commands/HABLift.h"
#include "subsystems/HABClimber.h"

// Utility subsystem and commands
#include "commands/SignalLight.h"
#include "subsystems/Utility.h"

#include "OI.h"

class Robot : public frc::TimedRobot {
 public:
  // Subsystems
  static ExampleSubsystem m_subsystem;
  // static OI m_oi;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;


 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;
	frc::Command *m_teleopCommand = nullptr;
  frc::Command *m_gamePieceCommand = nullptr;
  frc::Command *m_gamePieceCommandPID = nullptr;
  frc::Command *m_habClimbCommand = nullptr;
  ExampleCommand m_defaultAuto;
  MyAutoCommand m_myAuto;
  frc::SendableChooser<frc::Command*> m_chooser;
};
