/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/HABClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define HAB_DRIVE_MOTOR 2
#define LIFT_MOTOR_F 3
#define LIFT_MOTOR_L 4
#define LIFT_MOTOR_R 5

#define UP 1.0
#define DOWN -UP
#define FWD 1.0
#define REV -FWD
#define FRONT_RATIO 0.4

HABClimber::HABClimber() : Subsystem("HABClimber") {
  liftMotorF = new WPI_TalonSRX(LIFT_MOTOR_F);
  liftMotorL = new WPI_TalonSRX(LIFT_MOTOR_L);
  liftMotorR = new WPI_TalonSRX(LIFT_MOTOR_R);
  habDriveMotor = new WPI_TalonSRX(HAB_DRIVE_MOTOR);
}

void HABClimber::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
/*
* Lift/Lower/Stop - ALL Motors
*/
void HABClimber::Lift() {
  liftMotorF->Set(UP * FRONT_RATIO);
  liftMotorR->Set(UP);
  liftMotorL->Set(UP);
}
void HABClimber::Lower() {
  liftMotorF->Set(DOWN * FRONT_RATIO);
  liftMotorR->Set(DOWN);
  liftMotorL->Set(DOWN);
}
/*
* Stop ALL Climber Motors
*/
void HABClimber::Stop() {
    liftMotorF->Set(0.0);
    liftMotorR->Set(0.0);
    liftMotorL->Set(0.0);
    habDriveMotor->Set(0.0);
}
/*
* Lift/Lower FRONT Motors
*/
void HABClimber::LiftFront() {
    liftMotorF->Set(UP * FRONT_RATIO);
}
void HABClimber::LowerFront(){
    liftMotorF->Set(DOWN * FRONT_RATIO);
}
void HABClimber::StopFront(){
    liftMotorF->Set(0.0);
}
/*
*  Lift/Lower/Stop Rear climb motors
*/
void HABClimber::LiftRear() {
    liftMotorR->Set(UP);
    liftMotorL->Set(UP);  
}

void HABClimber::LowerRear() {
    liftMotorR->Set(DOWN);
    liftMotorL->Set(DOWN);  
}
void HABClimber::StopRear() {
    liftMotorR->Set(0.0);
    liftMotorL->Set(0.0);  
}
/*
* Forward/Reverse/Stop Drive
*/
void HABClimber::DriveFwd(double v) {
    habDriveMotor->Set(FWD * v);
}
void HABClimber::DriveRev(double v) {
    habDriveMotor->Set(REV * v);
}
void HABClimber::DriveStop() {
    habDriveMotor->Set(0.0);  
}
