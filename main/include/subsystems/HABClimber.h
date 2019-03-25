/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>

class HABClimber : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  WPI_TalonSRX *liftMotorR; // Right
  WPI_TalonSRX *liftMotorL; // Left
  WPI_TalonSRX *liftMotorF; // Front
  WPI_TalonSRX *habDriveMotor; // Climber Drive Fwd/Rev
  // String Pots
  // AnalogInput *liftMotorRpot; 
  // AnalogInput *liftMotorLpot; 
  // AnalogInput *liftMotorFpot; 

 public:
  HABClimber();
  void Lift();
  void Lower();
  void Stop();
  /* */
  void LiftFront();
  void LowerFront();
  void StopFront();
  /* */
  void LiftRear();
  void LowerRear();
  void StopRear();
  /* */
  void DriveFwd(double v);
  void DriveRev(double v);
  void DriveStop();
  /* */
  void InitDefaultCommand() override;
};
