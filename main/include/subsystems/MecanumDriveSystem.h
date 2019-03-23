/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/drive/MecanumDrive.h>
#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h> // Required by WPI_TalonSRX class
#include <rev/CANSparkMax.h>
#include <AHRS.h>

class MecanumDriveSystem : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

  // CTRE/TalonSRX controllers
  //WPI_TalonSRX *fl; // Front Left motor
	//WPI_TalonSRX *fr; // Front Right motor
	//WPI_TalonSRX *rl; // Rear left motor
	//WPI_TalonSRX *rr; // Rear Right motor

  // Neo motor controllers
  rev::CANSparkMax fl{2, rev::CANSparkMax::MotorType::kBrushless}; //was 2
  rev::CANSparkMax rl{5, rev::CANSparkMax::MotorType::kBrushless}; //was 5
  rev::CANSparkMax fr{4, rev::CANSparkMax::MotorType::kBrushless}; // should be 4
  rev::CANSparkMax rr{1, rev::CANSparkMax::MotorType::kBrushless}; // should be 1

  // Drive encoders
  rev::CANEncoder fl_encoder = fl.GetEncoder();
  rev::CANEncoder rl_encoder = rl.GetEncoder();
  rev::CANEncoder fr_encoder = fr.GetEncoder();
  rev::CANEncoder rr_encoder = rr.GetEncoder();

//  AHRS gyro{SPI::Port::kMXP};
  AHRS *gyro;

 public:
  MecanumDriveSystem();
  void InitDefaultCommand() override;
  void Go(double, double, double);
  void Saucer(double, double, double);
  void Stop();
  void GyroReset();
  void WriteDashboard();
	frc::MecanumDrive *m_robotDrive;
};
