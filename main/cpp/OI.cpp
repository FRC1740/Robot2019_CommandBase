/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/WPILib.h>
#include "OI.h"
#include "commands/HatchPanelEject.h"
#include "commands/SignalLight.h"
#include "commands/CargoBallEject.h"
#include "commands/CargoBallIntake.h"

/* ****
 *
 * 	xBox Controller button mapping:
 * 	Button 1 - A - 
 * 	Button 2 - B - 
 * 	Button 3 - X - 
 * 	Button 4 - Y - 
 * 	Button 5 - Left Bumper - 
 * 	Button 6 - Right Bumper - 
 * 	Button 7 - Back - 
 * 	Button 8 - Start - 
 * 	Button 9 - Left Stick
 * 	Button 10 - Right Stick
 */

OI::OI() {
  // Process operator interface input here.
  m_XboxDriver = new frc::XboxController(0); // Driver in USB slot zero
  m_XboxCoDriver = new frc::XboxController(1); // Driver in USB slot one

  // A button: Pneumatic Panel Eject
  btnEjectHatchPanel = new frc::JoystickButton(m_XboxDriver, 6);
  btnEjectHatchPanel->WhileHeld(new HatchPanelEject);

  // B button: Signal light to indicate Cargo/Hatch Panel
  btnSignalLight = new frc::JoystickButton(m_XboxDriver, 2);
  btnSignalLight->WhileHeld(new SignalLight);

  //btnGyroReset = new frc::JoystickButton(m_XboxDriver, 8);

  // Co-Driver Left/Right Bumpers control cargo ball
  btnIntakeCargoBall = new frc::JoystickButton(m_XboxCoDriver, 5);
  btnIntakeCargoBall->WhileHeld(new CargoBallIntake);

  btnEjectCargoBall = new frc::JoystickButton(m_XboxDriver, 6);
  btnEjectCargoBall->WhileHeld(new CargoBallEject);
  
}
