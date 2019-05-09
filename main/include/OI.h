/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/XboxController.h>
#include <frc/buttons/JoystickButton.h>

class OI {
 public:
  OI();
  frc::XboxController *m_XboxDriver;
  frc::XboxController *m_XboxCoDriver;
  frc::JoystickButton *btnEjectHatchPanel;
  frc::JoystickButton *btnDriveBy;
  frc::JoystickButton *btnIntakeCargoBall;
  frc::JoystickButton *btnEjectCargoBall;
  frc::JoystickButton *btnHABLift;
  frc::JoystickButton *btnHABRetractFront;
  frc::JoystickButton *btnHABRetractRear;
  frc::JoystickButton *btnHABForward;
  frc::JoystickButton *btnHABReverse;
  frc::JoystickButton *btnSignalLight;
  frc::JoystickButton *btnGyroReset;
};
