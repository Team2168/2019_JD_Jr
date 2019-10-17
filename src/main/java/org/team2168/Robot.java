/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import org.team2168.commands.autos.DoNothing;
import org.team2168.subsystems.VacuumClimberLift;
import org.team2168.subsystems.VacuumClimberPump;
import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Operator Interface
  public static OI oi;

  //climber
  public static VacuumClimberLift vacuumClimberLift;
  public static VacuumClimberPump vacuumClimberPump;

  // Variables for initializing and calibrating the Gyro
  static boolean autoMode;
  private static boolean matchStarted = false;
  public static int gyroReinits;
  private double lastAngle;
  private Debouncer gyroDriftDetector = new Debouncer(1.0);
  public static boolean gyroCalibrating = false;
  private boolean lastGyroCalibrating = false;
  private double curAngle = 0.0;

 // PDP Instance
  public static PowerDistribution pdp;

  // Driverstation Instance
  public static DriverStation driverstation;

  // Driver Joystick Chooser
  static int controlStyle;
  static int throttleStyle;
  static Command autonomousCommand;
  public static SendableChooser<Command> autoChooser;
  public static SendableChooser<Number> controlStyleChooser;
  public static SendableChooser<Number> throttleVibeChooser;

  //Variable to track blue alliance vs red alliance
  private static boolean blueAlliance = false;

  
  // Keep track of time
  double runTime = Timer.getFPGATimestamp();


  //

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    try
    {
      m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
      m_chooser.addOption("My Auto", kCustomAuto);
      SmartDashboard.putData("Auto choices", m_chooser);

      ConsolePrinter.init();
      ConsolePrinter.setRate(RobotMap.CONSOLE_PRINTER_LOG_RATE_MS)

      vacuumClimberLift = VacuumClimberLift.getInstance();
      vacuumClimberPump = VacuumClimberPump.getInstance();

      drivetrain.calibrateGyro();
      driverstation = DriverStation.getInstance();

      // Starting PDP
      pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
      pdp.startThread();

          /*******************************************************
       *                    
       ******************************************************/

      // Start Operator Interface
      oi = OI.getInstance();

      // enable compressor
      new StartCompressor();

      // Initialize Autonomous Selector Choices
      autoSelectInit();
      controlStyleSelectInit();
      throttleVibeSelectInit();
      
      ConsolePrinter.startThread();
      ConsolePrinter.putSendable("Control Style Chooser", () -> {return Robot.controlStyleChooser;}, true, false);
      ConsolePrinter.putSendable("Autonomous Mode Chooser", () -> {return Robot.autoChooser;}, true, false);
      ConsolePrinter.putSendable("Throttle Vibe Chooser", () -> {return Robot.throttleVibeChooser;}, true, false);
      ConsolePrinter.putString("AutoName", () -> {return Robot.getAutoName();}, true, false);
      ConsolePrinter.putString("Control Style Name", () -> {return Robot.getControlStyleName();}, true, false);
      ConsolePrinter.putNumber("gameClock", () -> {return driverstation.getMatchTime();}, true, false);
      ConsolePrinter.putNumber("Robot Pressure", () -> {return Robot.pneumatics.getPSI();}, true, false);
      
      
        System.out.println("Robot Initialization Complete!!");
    }

    catch (Throwable throwable)
    {
      Throwable cause = throwable.getCause();
      if (cause != null)
        throwable = cause;

      System.err.println("Bad things occured, testing using our own stach trace catch");
      System.err.println("Implement Logging function here");
      System.err.flush();

      // Show Stack Trace on Driverstration like before
      DriverStation.reportError("Unhandled exception instantiating robot" + throwable.toString(),
          throwable.getStackTrace());
      DriverStation.reportWarning("Robots should not quit, but yours did!", false);
      DriverStation.reportError("Could not instantiate robot!", false);
      System.exit(1);
      return;
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
//
  /**
   * This method is called once each time the robot enters Disabled mode. You can
   * use it to reset any subsystem information you want to clear when the robot is
   * disabled.
   */
  public void disabledInit()
  {
    autoMode = false;
    matchStarted = false;

    // If we are not in a match allow Gyro to be recalibrated in Disabled even if a
    // previous
    // calibration was performed, we disable this in a match so that if we ever die
    // in a match,
    // we don't try to recalibrate a moving robot.
    if (driverstation.isFMSAttached())
      drivetrain.startGyroCalibrating();

    drivetrain.calibrateGyro();
    
    drivetrain.limelightPosController.Pause();
  }

  public void disabledPeriodic()
  {

    // Keep track of Gunstyle Controller Variables

    // callArduino();
    getControlStyleInt();
    controlStyle = (int) controlStyleChooser.getSelected();
    throttleStyle = (int) throttleVibeChooser.getSelected();
    autonomousCommand = (Command) autoChooser.getSelected();

    Scheduler.getInstance().run();
    Drivetrain.getInstance().limelight.setPipeline(8);

    // Check to see if the gyro is drifting, if it is re-initialize it.
    gyroReinit();
  }

  public void autonomousInit()
  {
    autoMode = true;

    matchStarted = true;
    drivetrain.stopGyroCalibrating();
    drivetrain.resetGyro();

    autonomousCommand = (Command) autoChooser.getSelected();

    // schedule the autonomous command
    if (autonomousCommand != null)
      autonomousCommand.start();
  }

  /**
   * This function is called periodically during autonomous
   */
  public void autonomousPeriodic()
  {
    autoMode = true;
    Scheduler.getInstance().run();
  }

  /**
   * This function called prior to robot entering Teleop Mode
   */
  public void teleopInit()
  {
    // callArduino();
    autoMode = false;
    matchStarted = true;
    drivetrain.stopGyroCalibrating();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (autonomousCommand != null) autonomousCommand.cancel();

    // Select the control style
    controlStyle = (int) controlStyleChooser.getSelected();
    runTime = Timer.getFPGATimestamp();
  }

  /**
   * This function is called periodically during operator control
   */
  public void teleopPeriodic()
  {
    if((int) throttleVibeChooser.getSelected() == 0) 
    {
      Robot.oi.driverJoystick.setRumble(RumbleType.kLeftRumble, Math.abs(Robot.oi.getGunStyleYValue()));
    }

    SmartDashboard.putNumber("TeleopLoopTime", Timer.getFPGATimestamp() - runTime);
    runTime = Timer.getFPGATimestamp();

    autoMode = false;
    Scheduler.getInstance().run();

    controlStyle = (int) controlStyleChooser.getSelected();
    throttleStyle = (int) throttleVibeChooser.getSelected();
    
  }

  /************************************************************
   *
   * HELPER FUNCTIONS FOR ENTIRE ROBOT
   * 
   ************************************************************/
  /**
   * Get the name of a contron style.
   * 
   * @return the name of the control style.
   */
  public static String getControlStyleName()
  {
    String retVal = "";

    switch (controlStyle)
    {
    case 0:
      retVal = "Tank Drive";
      break;
    case 1:
      retVal = "Gun Style";
      break;
    case 2:
      retVal = "Arcade Drive";
      break;
    case 3:
      retVal = "GTA Drive";
      break;
    case 4:
      retVal = "New Gun Style";
      break;
    default:
      retVal = "Invalid Control Style";
    }

    return retVal;
  }

  /**
   * Adds control styles to the selector
   */
  public void controlStyleSelectInit()
  {
    controlStyleChooser = new SendableChooser<>();
    controlStyleChooser.addOption("Tank Drive", 0);
    controlStyleChooser.setDefaultOption("Gun Style Controller", 1);
    controlStyleChooser.addOption("Arcade Drive", 2);
    controlStyleChooser.addOption("GTA Drive", 3);
    controlStyleChooser.setDefaultOption("New Gun Style", 4);
  }

  /**
   * Method which determines which control (joystick) style was selected returns
   * int which is a enumeration for the control style to be implemented. The int
   * is positive only.
   */
  public static int getControlStyleInt()
  {
    return (int) controlStyleChooser.getSelected();
  }

  public static int getTrottleVibeInt()
  {
    return (int) throttleVibeChooser.getSelected();
  }

  public void throttleVibeSelectInit()
  {
    throttleVibeChooser = new SendableChooser<>();
    throttleVibeChooser.addOption("Throttle Vibe ON", 0);
    throttleVibeChooser.setDefaultOption("Throttle Vibe OFF", 1);
  }

  /**
   * Adds the autos to the selector
   */
  public void autoSelectInit()
  {
    autoChooser = new SendableChooser<Command>();
    // autoChooser.addDefault("Drive Straight", new DriveStraight(8.0));
    autoChooser.addObject("Do Nothing", new DoNothing());
    // autoChooser.addObject("Center Auto 3 Cube", new AutoStartCenter3Cube());
  }

  /**
   * Get the name of an autonomous mode command.
   * 
   * @return the name of the auto command.
   */
  public static String getAutoName()
  {
    if (autonomousCommand != null)
      return autonomousCommand.getName();
    else
      return "None";
  }

  /**
   * @return true if the robot is in auto mode
   */
  public static boolean isAutoMode()
  {
    return autoMode;

  }


  public static boolean onBlueAlliance() {
		return driverstation.getAlliance() == DriverStation.Alliance.Blue;

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
