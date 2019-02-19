/*----------------------------------m_gamePieceCommand->Start();------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/GamePieceManipulator.h"
#include <frc/smartdashboard/SmartDashboard.h>

// Move these to the appropriate location
constexpr double hingeMaxLeft  = 0.7; // was 4.3; measured on Armada
constexpr double hingeMinLeft  = 4.7; // was 0.3;
constexpr double hingeMaxRight = 0.8; // was 4.3;
constexpr double hingeMinRight = 4.8; // was 0.538;
constexpr double hingeLeftKp   = 1.0;
constexpr double hingeLeftKi   = 0.05;
constexpr double hingeLeftKd   = 0.75;
constexpr double hingeRightKp  = hingeLeftKp;
constexpr double hingeRightKi  = hingeLeftKi;
constexpr double hingeRightKd  = hingeLeftKd;

GamePieceManipulator::GamePieceManipulator() : frc::Subsystem("GamePieceManipulator") {

  // Pneumatic Hatch Panel Eject
  hatchPanel = new frc::DoubleSolenoid(0,1); // PCM Ports
  // Cargo Ball Intake/Eject Motor
  ballMotor = new WPI_TalonSRX(1); // CAN ID
  // Hinge Raise/Lower Motor
  hingeMotorL = new WPI_TalonSRX(6); // CAN ID
  hingeMotorR = new WPI_TalonSRX(7); // CAN ID
  hingeMotorL->EnableCurrentLimit(true);
  hingeMotorR->EnableCurrentLimit(true);

  hingePotL = new frc::AnalogInput(0);
  hingePotR = new frc::AnalogInput(1);
  hingePotL->SetOversampleBits(4);
  hingePotR->SetOversampleBits(4);
  int bitsL = hingePotL->GetOversampleBits();
  int bitsR = hingePotR->GetOversampleBits();
  hingePotL->SetAverageBits(2);
  hingePotR->SetAverageBits(2);
  bitsL = hingePotL->GetAverageBits();
  bitsR = hingePotR->GetAverageBits();

  hingeInL = new HingePIDSource(hingePotL,
    hingeMinLeft, hingeMaxLeft - hingeMinLeft);
  hingeInR = new HingePIDSource(hingePotR,
    hingeMinRight, hingeMaxRight - hingeMinRight);
  hingeOutL = new HingePIDOutput(hingeMotorL, hingePotL,
    hingeMinLeft, hingeMaxLeft - hingeMinLeft);
  hingeOutR = new HingePIDOutput(hingeMotorR, hingePotR,
    hingeMinRight, hingeMaxRight - hingeMinRight);

  hingePIDL = new frc::PIDController(hingeLeftKp, hingeLeftKi, hingeLeftKd,
    *hingeInL, *hingeOutL);
  hingePIDR = new frc::PIDController(hingeRightKp, hingeRightKi, hingeRightKd,
    *hingeInR, *hingeOutR);
  frc::SmartDashboard::PutData("Hinge PID Left", hingePIDL);
  frc::SmartDashboard::PutData("Minge PID Right", hingePIDR);
  
  hingePIDL->SetInputRange(0.0, 1.0);  // [120,0] (here::MoveTo) <- [0,1] (PID) <- [4.7,0.7] (here)
  hingePIDL->SetOutputRange(-1.0, 1.0);  // velocity
  hingePIDR->SetInputRange(0.0, 1.0);
  hingePIDR->SetOutputRange(-1.0, 1.0);
}

void GamePieceManipulator::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  }

// Put methods for controlling this subsystem
// here. Call these from Commands.
/*******************************
 * Pneumatic Hatch Panel Methods
********************************/

void GamePieceManipulator::HatchEject() {
    hatchPanel->Set(frc::DoubleSolenoid::Value::kForward);
  }

void GamePieceManipulator::HatchInject() {
    hatchPanel->Set(frc::DoubleSolenoid::Value::kReverse);
  }

/*******************************
    Arm Raise & Lower Methods
********************************/
//v = velocity
#define GP_DEADBAND 0.25
void GamePieceManipulator::Move(double v) {
 
    double positionL = hingePotL->GetVoltage();

    if ((v > GP_DEADBAND && positionL > HINGE_MIN_LEFT)
        || (v < -GP_DEADBAND && positionL < HINGE_MAX_LEFT)) {
        //v *= 10.0;
        hingeMotorL->Set(v);
    }
    else {
        hingeMotorL->Set(0.0);
    }

    double positionR = hingePotR->GetVoltage();

    if ((v > GP_DEADBAND && positionR > HINGE_MIN_RIGHT)
        || (v < -GP_DEADBAND && positionR < HINGE_MAX_RIGHT)) {
        //v *= 10.0;
        hingeMotorR->Set(v);
    }
    else {
        hingeMotorR->Set(0.0);
    }
}

void GamePieceManipulator::MoveTo(double p) {
  // PID takes a position in the range [0,1]
  hingePIDL->SetSetpoint(p);
  hingePIDR->SetSetpoint(p);
  EnablePIDLoop();
}

void GamePieceManipulator::EnablePIDLoop() {
  hingePIDL->Enable();
  hingePIDR->Enable();
}

void GamePieceManipulator::DisablePIDLoop() {
  hingePIDL->Disable();
  hingePIDR->Disable();
}

void GamePieceManipulator::Stop() {
    DisablePIDLoop();
    //hingeMotorL->Set(0.0); redundant
    //hingeMotorR->Set(0.0);
}


double GamePieceManipulator::GetLPosition() {
    return hingePotL->GetVoltage();
}
double GamePieceManipulator::GetRPosition() {
    return hingePotR->GetVoltage();
}

// Helpers for hinge PIDController -------------------------------------------
HingePIDSource::HingePIDSource(frc::AnalogInput *pot, double min, double range)
  : frc::PIDSource() {
  m_min = min;
  m_range = range;
  m_pot = pot;
  m_pidSource = frc::PIDSourceType::kDisplacement;
}
double HingePIDSource::PIDGet() {
  double v = m_pot->GetVoltage();
  // Scale [4.7,0.7] to [0, 1] for PID
  v = (v - m_min) / m_range;
  return v;
}
void HingePIDSource::SetPIDSourceType(frc::PIDSourceType pidSource) {
  // No-op (do not change from default)
}

HingePIDOutput::HingePIDOutput(WPI_TalonSRX *motor, frc::AnalogInput *pot, 
  double min, double range)
  : frc::PIDOutput() {
  m_motor = motor;
  m_pot = pot;
  m_min = min;
  m_range = range;
}
void HingePIDOutput::PIDWrite(double d) {
  double v = m_pot->GetVoltage();
  v = (v - m_min) / m_range;
  if (((v < 0.0) && (d < 0.0)) || ((v > 1.0) && (d > 0.0))) {
    // Software limit switch
    m_motor->Set(0.0);
  }
  else {
    m_motor->Set(d);
  }
}

// ----------------------------------------------------------------------------

/*******************************
    Cargo Ball Methods
********************************/
void GamePieceManipulator::CargoLoad() {
    ballMotor->Set(1.0);
}
void GamePieceManipulator::CargoEject() {
    ballMotor->Set(-1.0);
}
void GamePieceManipulator::CargoStop() {
    ballMotor->Set(0.0);
}
