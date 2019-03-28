/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "CommandBase.h"

#define DEADBAND 0.1
#define XDEADBAND 0.1
#define YDEADBAND 0.1
#define TWISTDEADBAND 0.1

class MecanumDriveCommand : public CommandBase {
 public:
  MecanumDriveCommand(bool);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
  void ToggleUseGyro();
  void ToggleDriveSide();
 private:
  double GetX();
  double GetY();
  double GetInvertedY();
  double GetTwist();
  double Deadband(double); // Also Square inputs
  bool useGyro; // Relative vs. absolute steering using gyro
  bool driveSideways;
};
