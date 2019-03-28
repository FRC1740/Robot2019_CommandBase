/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/MecanumDriveSystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

MecanumDriveSystem::MecanumDriveSystem() : Subsystem("MecanumDriveSystem") {}

void MecanumDriveSystem::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
	//fl = new WPI_TalonSRX(FRONT_LEFT_MOTOR_ID); // Front Left
	//rl = new WPI_TalonSRX(REAR_LEFT_MOTOR_ID); // Rear Left
	//fr = new WPI_TalonSRX(FRONT_RIGHT_MOTOR_ID); // Front Right
	//rr = new WPI_TalonSRX(REAR_RIGHT_MOTOR_ID); // Rear Right

	m_robotDrive = new frc::MecanumDrive(fl, rl, fr, rr);

#define OLRR 0.2
  fl.SetOpenLoopRampRate(OLRR);
  rl.SetOpenLoopRampRate(OLRR);
  fr.SetOpenLoopRampRate(OLRR);
  rr.SetOpenLoopRampRate(OLRR);

	m_robotDrive->SetExpiration(0.5);
	m_robotDrive->SetSafetyEnabled(false);
  gyro = new AHRS(SPI::Port::kMXP);

}
void MecanumDriveSystem::Saucer(double x, double y, double twist) {
  m_robotDrive->DriveCartesian(x, y, twist, -gyro->GetAngle());
	WriteDashboard();
}
void MecanumDriveSystem::Go(double x, double y, double twist) {
  m_robotDrive->DriveCartesian(x, y, twist, 0.0);
	WriteDashboard();
}
void MecanumDriveSystem::Stop() {
  m_robotDrive->DriveCartesian(0.0, 0.0, 0.0, 0.0);
	WriteDashboard();
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
void MecanumDriveSystem::GyroReset() {
	gyro->Reset();
	WriteDashboard();
}

void MecanumDriveSystem::WriteDashboard() {
  frc::SmartDashboard::PutNumber("FL Velo:", fl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("RL Velo:", rl_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("FR Velo:", -fr_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("RR Velo:", -rr_encoder.GetVelocity());

  frc::SmartDashboard::PutNumber("FL Pos:", fl_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("RL Pos:", rl_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("FR Pos:", -fr_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("RR Pos:", -rr_encoder.GetPosition());

}
