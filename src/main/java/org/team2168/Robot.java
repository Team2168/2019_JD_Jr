/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import org.team2168.subsystems.Drivetrain;

import org.team2168.utils.Debouncer;
import org.team2168.utils.consoleprinter.ConsolePrinter;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project
 */
public class Robot extends TimedRobot
 {
  // Operator Interface
  public static OI oi;

  // Subsystems
  public static Drivetrain drivetrain;

    
  // gyro calibration
  static boolean autoMode;
  private static boolean matchStarted = false;
  public static int gyroReinits;
  private double lastAngle;
  private Debouncer gyroDriftDetector = new Debouncer(1.0);
  public static boolean gyroCalibrating = false;
  private boolean lastGyroCalibrating = false;
  private double curAngle = 0.0;

  public static boolean isClimbEnabled = false;

  @Override
  public void robotInit()
  {

    try
    {
      // Stop all WPILib 2018 Telementry
      LiveWindow.disableAllTelemetry();

      ConsolePrinter.init();
      ConsolePrinter.setRate(RobotMap.CONSOLE_PRINTER_LOG_RATE_MS);

      // Instantiate the subsystems
      drivetrain = Drivetrain.getInstance();
   

      drivetrain.calibrateGyro();

      oi = OI.getInstance();

      System.out.println("Robot Initialization Complete!!");

    }
  }
  

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }


  public void teleopInit()
  {
    // callArduino();
    autoMode = false;
    matchStarted = true;
    drivetrain.stopGyroCalibrating();
  }

    /**
   * @return true if the robot is in auto mode
   */
  public static boolean isAutoMode()
  {
    return autoMode;

  }

    /**
   * @return true if the robot is in climb mode
   */
  public static boolean isClimbMode()
  {
    return isClimbEnabled;

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

    /**
   * Method which checks to see if gyro drifts and resets the gyro. Call this in a
   * loop.
   */
  private void gyroReinit()
  {
    // Check to see if the gyro is drifting, if it is re-initialize it.
    // Thanks FRC254 for orig. idea.
    curAngle = drivetrain.getHeading();
    gyroCalibrating = drivetrain.isGyroCalibrating();

    if (lastGyroCalibrating && !gyroCalibrating)
    {
      // if we've just finished calibrating the gyro, reset
      gyroDriftDetector.reset();
      curAngle = drivetrain.getHeading();
      System.out.println("Finished auto-reinit gyro");
    }
    else if (gyroDriftDetector.update(Math.abs(curAngle - lastAngle) > (0.75 / 50.0)) && !matchStarted
        && !gyroCalibrating)
    {
      // && gyroReinits < 3) {
      gyroReinits++;
      System.out.println("!!! Sensed drift, about to auto-reinit gyro (" + gyroReinits + ")");
      drivetrain.calibrateGyro();
    }

    lastAngle = curAngle;
    lastGyroCalibrating = gyroCalibrating;

  }
}
