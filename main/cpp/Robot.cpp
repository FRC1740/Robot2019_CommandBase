/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//On error, create env.h from env-default.h and modify ROBOT_VERSION_STRING
#include "env.h"

#include "Robot.h"
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

// Following four lines for vision processing
#include "vision/VisionPipeline.h"
#include "vision/VisionRunner.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

ExampleSubsystem Robot::m_subsystem;
//OI Robot::m_oi;

#define no_RAW_CAMERA
#ifndef RAW_CAMERA

static void VisionThread()
{
    cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
    // Reduce Exposure when using monochrome/brightness only
    camera.SetExposureManual(25);
//    camera.SetExposureManual(100);
    camera.SetFPS(15);
    camera.SetResolution(640, 480);
    camera.SetWhiteBalanceManual(5000);
    cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
    cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
    cv::Mat source;
    cv::Mat output;
    cv::Mat mask;
    cv::Mat draw;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    while(true) {
      //frc::SmartDashboard::PutBoolean("visionEnabled:", CommandBase::visionEnabled);
        cvSink.GrabFrame(source);
//        if (1) {
        if (CommandBase::visionEnabled) {
        cvSink.GrabFrame(mask);
        cvSink.GrabFrame(draw);
        if (!source.empty()) {
          // Filter based on brightness ONLY
//          cvtColor(source, output, cv::COLOR_BGR2GRAY);
          int lowH = 80; // Yellow end of green
          int highH = 160; // Cyan end of green
          int lowS = 75; // Percent saturation
          int highS = 100; // Fully saturated
          int lowV = 85; // Darkest value
          int highV = 100; // Brighest value
           // Filter based on COLOR
          cvtColor(source, output, cv::COLOR_BGR2HSV);
          // Filter the image based on our HSV low/high threshold values
          inRange(output, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask);

          // inRange selects only the brightest values in the image frame
          // inRange(output, cv::Scalar(128, 128, 128), cv::Scalar(255, 255, 255), mask);
          // findCounters() locates the reflective shapes
          findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
          // We don't know how to deal with more than two targets, so abort if > 2
          if (contours.size() <= 2) { // One or two targets we can handle
            double avgCenterX=0;
            double avgCenterY=0;
            for (size_t i=0; i<contours.size(); i++) {
              // drawCountours() below will outline the countours in magenta
              drawContours(draw, contours, i, cv::Scalar(255, 0, 255), 3, 8, hierarchy, 0, cv::Point() );
              // find the bounding rectangle of the blob
              cv::Rect boundRect = boundingRect(contours[i]);
              // Outline the bounding rectangle in yellow
              rectangle(draw, boundRect, cv::Scalar(0, 255, 255), 3, 8, 0);
              // Determine the center X, Y of the blob
              double centerX = boundRect.x + (boundRect.width / 2);
              double centerY = boundRect.y + (boundRect.height / 2);
              // Determine the AVERAGE center X, Y of all blobs
              // Determine the AVERAGE center X, Y of all blobs
              avgCenterX += centerX;
              avgCenterY += centerY;
              }
            avgCenterX /= contours.size();
            avgCenterY /= contours.size();
            circle(draw, cv::Point(avgCenterX, avgCenterY), 5, cv::Scalar(250, 250, 0), 3, 8, 0);
            CommandBase::visionOffset = avgCenterX / 320.0 - 1.0;
            frc::SmartDashboard::PutNumber("visionOffset:", CommandBase::visionOffset);
          }
        }
        else {
          cvSink.GrabFrame(draw);
        }
        if (!draw.empty()) {
          outputStreamStd.PutFrame(draw);
        }
      } //visionEnabled
      else {
        if (!source.empty()) {
          cvtColor(source, output, cv::COLOR_BGR2GRAY);
          resize(output, draw, cvSize(320, 240));
          outputStreamStd.PutFrame(output);

        }
      }
    }
}
#endif

void Robot::RobotInit() {
  // Instantiate all subsystems objects
  CommandBase::init();

// Launch vision thread
#ifndef RAW_CAMERA
  std::thread visionThread(VisionThread);
  visionThread.detach();
#else  // RAW_CAMERA
  cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
  cs::UsbCamera camera0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera1.SetResolution(640, 480);
  camera0.SetResolution(640, 480);
  camera1.SetExposureManual(35);
  camera0.SetExposureManual(35);
#endif

  m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
  m_chooser.AddOption("My Auto", &m_myAuto);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutString("Code Version", ROBOT_VERSION_STRING);

  frc::SmartDashboard::PutNumber("Left Hinge", CommandBase::gamePieceManipulator->GetLPosition());
  frc::SmartDashboard::PutNumber("Right Hinge", CommandBase::gamePieceManipulator->GetRPosition());
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
  // std::string autoSelected = frc::SmartDashboard::GetString(
  //     "Auto Selector", "Default");
  // if (autoSelected == "My Auto") {
  //   m_autonomousCommand = &m_myAuto;
  // } else {
  //   m_autonomousCommand = &m_defaultAuto;
  // }

  m_autonomousCommand = m_chooser.GetSelected();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Start();
  }
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  // To use Field-Centric steering (Saucer Mode), pass a TRUE to the command.
  // FALSE will use Robot-Centric (relative) steering
  // This value should come from the sendable chooser/dashboard
  m_teleopCommand = new MecanumDriveCommand(false);
  m_teleopCommand->Start();
#define noUSE_PID
#ifdef USE_PID
  m_gamePieceCommand = new GamePieceManipulatorMoveToPosition();
  //m_gamePieceCommandPID->Start();
#else // USE_PID
  m_gamePieceCommand = new GamePieceManipulatorManual();
#endif // USE_PID
  m_gamePieceCommand->Start();
  m_habClimbCommand = new HABLift();
  m_habClimbCommand->Start();
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
