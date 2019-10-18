/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.*;
import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.controllers.PIDSpeed;
import org.team2168.PID.sensors.ADXRS453Gyro;
import org.team2168.PID.sensors.AverageEncoder;
import org.team2168.PID.sensors.IMU;
import org.team2168.utils.consoleprinter.ConsolePrinter;
import org.team2168.PID.sensors.Limelight;
import org.team2168.utils.TCPSocketSender;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax leftMotor1;
  private CANSparkMax leftMotor2;
  private CANSparkMax rightMotor1;
  private CANSparkMax rightMotor2;

  private ADXRS453Gyro _gyroSPI;
  private AverageEncoder drivetrainLeftEncoder;
  private AverageEncoder drivetrainRightEncoder;
  public IMU imu;

  //declare position controllers
  public PIDPosition driveTrainPosController;
  public PIDPosition rotateController;
  public PIDPosition rotateDriveStraightController;

  public PIDPosition rightPosController;
  public PIDPosition leftPosController;

  // declare speed controllers
  public PIDSpeed rightSpeedController;
  public PIDSpeed leftSpeedController;

  public Limelight limelight;
  public PIDPosition limelightPosController;

  //dclare TCP servers, remove before comps, only use for debugging
  TCPSocketSender TCPdrivePosController;
  TCPSocketSender TCPrightSpeedController;
  TCPSocketSender TCPleftSpeedController;
  TCPSocketSender TCProtateController;
  TCPSocketSender TCPleftPosController;
  TCPSocketSender TCPrightPosController;
  TCPSocketSender TCPlimelightPosController;

  public volatile double leftMotor1Voltage;
  public volatile double leftMotor2Voltage;
  public volatile double rightMotor1Voltage;
  public volatile double rightMotor2Voltage;

  double runTime = Timer.getFPGATimestamp();

  
  private static Drivetrain instance = null;

  private Drivetrain() {
    leftMotor1 = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
    leftMotor2 = new CANSparkMax(RobotMap.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);
    rightMotor1 = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
    rightMotor2 = new CANSparkMax(RobotMap.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);

    double rightMotor1FPS;
    double rightMotor2FPS;
    double leftMotor1FPS;
    double lefttMotor2FPS;

    drivetrainRightEncoder = new AverageEncoder(RobotMap.RIGHT_DRIVE_ENCODER_A, RobotMap.RIGHT_DRIVE_ENCODER_B,
        RobotMap.DRIVE_ENCODER_PULSE_PER_ROT, RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
        RobotMap.RIGHT_DRIVE_TRAIN_ENCODER_REVERSE, RobotMap.DRIVE_ENCODING_TYPE, RobotMap.DRIVE_SPEED_RETURN_TYPE,
        RobotMap.DRIVE_POS_RETURN_TYPE, RobotMap.DRIVE_AVG_ENCODER_VAL);

    drivetrainLeftEncoder = new AverageEncoder(RobotMap.LEFT_DRIVE_ENCODER_A, RobotMap.LEFT_DRIVE_ENCODER_B,
        RobotMap.DRIVE_ENCODER_PULSE_PER_ROT, RobotMap.DRIVE_ENCODER_DIST_PER_TICK,
        RobotMap.LEFT_DRIVE_TRAIN_ENCODER_REVERSE, RobotMap.DRIVE_ENCODING_TYPE, RobotMap.DRIVE_SPEED_RETURN_TYPE,
        RobotMap.DRIVE_POS_RETURN_TYPE, RobotMap.DRIVE_AVG_ENCODER_VAL);

    leftMotor1.setSmartCurrentLimit(60);
    leftMotor2.setSmartCurrentLimit(60);
    rightMotor1.setSmartCurrentLimit(60);
    rightMotor2.setSmartCurrentLimit(60);

    //control frame every 20ms
    leftMotor1.setControlFramePeriodMs(20);
    leftMotor2.setControlFramePeriodMs(20);
    rightMotor1.setControlFramePeriodMs(20);
    rightMotor2.setControlFramePeriodMs(20);

     //status frame every 500ms
    leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
    leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
    rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);     
    rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);

     //status frame every 500ms
    leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
    leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
    rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
    rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);

     //status frame every 500ms
    leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
    leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
    rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
    rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
    
    _gyroSPI = new ADXRS453Gyro();
    _gyroSPI.startThread();

    imu = new IMU(drivetrainLeftEncoder, drivetrainRightEncoder, RobotMap.WHEEL_BASE);

    limelight = new Limelight();
    limelight.setCamMode(1);
    limelight.setPipeline(7);
      
    // DriveStraight Controller
  rotateController = new PIDPosition(
      "RotationController", 
      RobotMap.ROTATE_POSITION_P, 
      RobotMap.ROTATE_POSITION_I,
      RobotMap.ROTATE_POSITION_D, 
      _gyroSPI, 
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

  
  rotateDriveStraightController = new PIDPosition(
      "RotationStraightController",
      RobotMap.ROTATE_POSITION_P_Drive_Straight, 
      RobotMap.ROTATE_POSITION_I_Drive_Straight,
      RobotMap.ROTATE_POSITION_D_Drive_Straight, 
      _gyroSPI, 
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

  driveTrainPosController = new PIDPosition(
      "driveTrainPosController", 
      RobotMap.DRIVE_TRAIN_RIGHT_POSITION_P,
      RobotMap.DRIVE_TRAIN_RIGHT_POSITION_I, 
      RobotMap.DRIVE_TRAIN_RIGHT_POSITION_D, 
      imu,
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

  // Spawn new PID Controller
  rightSpeedController = new PIDSpeed(
      "rightSpeedController", 
      RobotMap.DRIVE_TRAIN_RIGHT_SPEED_P,
      RobotMap.DRIVE_TRAIN_RIGHT_SPEED_I, 
      RobotMap.DRIVE_TRAIN_RIGHT_SPEED_D, 
      1, 
      drivetrainRightEncoder,
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

  leftSpeedController = new PIDSpeed(
      "leftSpeedController", 
      RobotMap.DRIVE_TRAIN_LEFT_SPEED_P,
      RobotMap.DRIVE_TRAIN_LEFT_SPEED_I, 
      RobotMap.DRIVE_TRAIN_LEFT_SPEED_D, 
      1, 
      drivetrainLeftEncoder,
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

    // Spawn new PID Controller
  rightPosController = new PIDPosition(
      "rightPosController", 
      RobotMap.DRIVE_TRAIN_RIGHT_POSITION_P,
      RobotMap.DRIVE_TRAIN_RIGHT_POSITION_I, 
      RobotMap.DRIVE_TRAIN_RIGHT_POSITION_D, 
      1, 
      drivetrainRightEncoder,
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

  leftPosController = new PIDPosition(
      "leftPosController", 
      RobotMap.DRIVE_TRAIN_LEFT_POSITION_P,
      RobotMap.DRIVE_TRAIN_LEFT_POSITION_I, 
      RobotMap.DRIVE_TRAIN_LEFT_POSITION_D, 
      1, 
      drivetrainLeftEncoder,
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

            // Limelight Controller
 limelightPosController = new PIDPosition(
      "limelightPosController",
      RobotMap.LIMELIGHT_POSITION_P,
      RobotMap.LIMELIGHT_POSITION_I,
      RobotMap.LIMELIGHT_POSITION_D,
      limelight,
      RobotMap.DRIVE_TRAIN_PID_PERIOD);

    rightSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    leftSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    rightPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    leftPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    driveTrainPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    rotateController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    rotateDriveStraightController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    limelightPosController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);
    // start controller threads
    rightSpeedController.startThread();
    leftSpeedController.startThread();
    rightPosController.startThread();
    leftPosController.startThread();
    driveTrainPosController.startThread();
    rotateController.startThread();
    rotateDriveStraightController.startThread();
    limelightPosController.startThread();  

 // start TCP Servers for DEBUGING ONLY
    TCPdrivePosController = new TCPSocketSender(RobotMap.TCP_SERVER_DRIVE_TRAIN_POS, driveTrainPosController);
    TCPdrivePosController.start();

    TCPrightSpeedController = new TCPSocketSender(RobotMap.TCO_SERVER_RIGHT_DRIVE_TRAIN_SPEED, rightSpeedController);
    TCPrightSpeedController.start();

    TCPleftSpeedController = new TCPSocketSender(RobotMap.TCP_SERVER_LEFT_DRIVE_TRAIN_SPEED, leftSpeedController);
    TCPleftSpeedController.start();

    TCPrightPosController = new TCPSocketSender(RobotMap.TCP_SERVER_RIGHT_DRIVE_TRAIN_POSITION, rightPosController);
    TCPrightPosController.start();

    TCPleftPosController = new TCPSocketSender(RobotMap.TCP_SERVER_LEFT_DRIVE_TRAIN_POSITION, leftPosController);
    TCPleftPosController.start();

    TCProtateController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER, rotateController);
    TCProtateController.start();

    TCProtateController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER_STRAIGHT,rotateDriveStraightController);
    TCProtateController.start();

    TCPlimelightPosController = new TCPSocketSender(RobotMap.TCP_SERVER_ROTATE_CONTROLLER_WITH_CAMERA,limelightPosController);
    TCPlimelightPosController.start();

    leftMotor1Voltage = 0;
    leftMotor2Voltage = 0;

    rightMotor1Voltage = 0;
    rightMotor2Voltage = 0;

      // Log sensor data
  ConsolePrinter.putNumber("Left Encoder Distance", () -> {return Robot.drivetrain.getLeftPosition();}, true, false);
  ConsolePrinter.putNumber("Right Encoder Distance:", () -> {return Robot.drivetrain.getRightPosition();}, true, false);
  ConsolePrinter.putNumber("Average Drive Encoder Distance", () -> {return Robot.drivetrain.getAverageDistance();}, true, false);
  ConsolePrinter.putNumber("Right Drive Encoder Rate", () -> {return Robot.drivetrain.getRightEncoderRate();}, true, false);
  ConsolePrinter.putNumber("Left Drive Encoder Rate", () -> {return Robot.drivetrain.getLeftEncoderRate();}, true, false);
  ConsolePrinter.putNumber("Average Drive Encoder Rate", () -> {return Robot.drivetrain.getAverageEncoderRate();}, true, false);

  ConsolePrinter.putNumber("Gyro Angle:", () -> {return Robot.drivetrain.getHeading();}, true, false);	
  ConsolePrinter.putNumber("Gunstyle X Value", () -> {return Robot.oi.getGunStyleXValue();}, true, false);
  ConsolePrinter.putNumber("Gunstyle Y Value", () -> {return Robot.oi.getGunStyleYValue();}, true, false);
  ConsolePrinter.putNumber("DTLeft1MotorVoltage", () -> {return Robot.drivetrain.getleftMotor1Voltage();}, true, false);
  ConsolePrinter.putNumber("DTLeft2MotorVoltage", () -> {return Robot.drivetrain.getleftMotor2Voltage();}, true, false);

  ConsolePrinter.putNumber("DTRight1MotorVoltage", () -> {return Robot.drivetrain.getrightMotor1Voltage();}, true, false);
  ConsolePrinter.putNumber("DTRight2MotorVoltage", () -> {return Robot.drivetrain.getrightMotor2Voltage();}, true, false);
  ConsolePrinter.putNumber("DTRight1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP);}, true, false);
  ConsolePrinter.putNumber("DTRight2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP);}, true, false);
  ConsolePrinter.putNumber("DTLeft1MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP);}, true, false);
  ConsolePrinter.putNumber("DTLeft2MotorCurrent", () -> {return Robot.pdp.getChannelCurrent(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP);}, true, false);
  ConsolePrinter.putNumber("PID right motor 1 voltage", () -> {return this.PIDVoltagefeedRightMotor1();}, true, false);
  ConsolePrinter.putNumber("PID right motor 2 voltage", () -> {return this.PIDVoltagefeedRightMotor2();}, true, false);
  ConsolePrinter.putNumber("PID left motor 1 voltage", () -> {return this.PIDVoltagefeedLeftMotor1();}, true, false);
  ConsolePrinter.putNumber("PID left motor 2 voltage", () -> {return this.PIDVoltagefeedLeftMotor2();}, true, false);
  ConsolePrinter.putNumber("GYRO Driftrate:", () -> {return Robot.drivetrain._gyroSPI.driftRate;}, true, false);
  ConsolePrinter.putNumber("GYRO Rate:", () -> {return Robot.drivetrain._gyroSPI.getRate();}, true, false);
  ConsolePrinter.putNumber("GYRO Angle SPI:", () -> {return Robot.drivetrain._gyroSPI.getAngle();}, true, false);
  ConsolePrinter.putNumber("GYRO reInits:", () -> {return (double) Robot.gyroReinits;}, true, false);
  ConsolePrinter.putBoolean("Gyro Cal Status", () -> {return !Robot.gyroCalibrating;}, true, false);
  ConsolePrinter.putNumber("GYRO Status:", () -> {return (double) Robot.drivetrain._gyroSPI.getStatus();}, true, false);
  ConsolePrinter.putNumber("GYRO Temp:", () -> {return Robot.drivetrain._gyroSPI.getTemp();}, true, false);
  
  
  ConsolePrinter.putBoolean("Left Motor One Trip", () -> {return !Robot.pdp.isLeftMotorOneTrip();}, true, false);
  ConsolePrinter.putBoolean("Left Motor Two Trip", () -> {return !Robot.pdp.isLeftMotorTwoTrip();}, true, false);
  ConsolePrinter.putBoolean("Right Motor One Trip", () -> {return !Robot.pdp.isRightMotorOneTrip();}, true, false);
  ConsolePrinter.putBoolean("Right Motor Two Trip", () -> {return !Robot.pdp.isRightMotorTwoTrip();}, true, false);
  
  
}

  public static Drivetrain getInstance() {
    if (instance == null)
      instance = new Drivetrain();
    return instance;
  }

  private void driveLeftMotor1(double speed) {
       // DT_REVERSE_XMOTOR dependant on hardware on robot, keeping because ctrl+x is
    // easier than than typing
    if (RobotMap.DT_REVERSE_LEFT1) {
      speed = -speed;
    }

    leftMotor1.set(speed);
    leftMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  private void driveLeftMotor2(double speed) {
    if (RobotMap.DT_REVERSE_LEFT2) {
      speed = -speed;
    }

    leftMotor2.set(speed);
    leftMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  private void driveRightMotor1(double speed) {
    if (RobotMap.DT_REVERSE_RIGHT1) {
      speed = -speed;
    }

    rightMotor1.set(speed);
    rightMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  private void driveRightMotor2(double speed) {
    if (RobotMap.DT_REVERSE_RIGHT2) {
      speed = -speed;
    }

    rightMotor2.set(speed);
    rightMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;

  }

  public void driveLeft(double speed) {
    driveLeftMotor1(speed);
    driveLeftMotor2(speed);
  }

  public void driveRight(double speed) {
    driveRightMotor1(speed);
    driveRightMotor2(speed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    if (!Robot.isAutoMode())
    {
      if (Robot.lift.getPotPos() > 50)
      {
        leftSpeed = leftSpeed;
        rightSpeed = rightSpeed;
      }
    }
    else
    {
      if (Robot.lift.getPotPos() > 30)
      {
        leftSpeed = leftSpeed * 0.3;
        rightSpeed = rightSpeed * 0.3;
      }
    }

    runTime = Timer.getFPGATimestamp();
    driveLeft(leftSpeed);
    driveRight(rightSpeed);
    SmartDashboard.putNumber("TankDriveSetCanTime", Timer.getFPGATimestamp() - runTime);
  }

  /**
   * returns total distance traveled by right side of drivetrain
   * 
   * @return double in feet of total distance traveled by right encoder
   */
  public double getRightPosition()
  {
    return drivetrainRightEncoder.getPos();
  }

  /**
   * returns total distance traveled by left side of drivetrain
   * 
   * @return double in feet of total distance traveled by left encoder
   */
  public double getLeftPosition()
  {
    return drivetrainLeftEncoder.getPos();
  }


  /**
   * returns total distance traveled by drivetrain
   * 
   * @return double in inches of average distance traveled by both encoders
   */
  public double getAverageDistance()
  {
    return imu.getPos();
  }

  /**
   * resets position of right encoder to 0 inches
   */
  public void resetRightPosition()
  {
    drivetrainRightEncoder.reset();
  }

  /**
   * resets position of left encoder to 0 inches
   */
  public void resetLeftPosition()
  {
    drivetrainLeftEncoder.reset();
  }

  /**
   * resets position of both Encoders to 0 inches
   */
  public void resetPosition()
  {
    resetLeftPosition();
    resetRightPosition();
  }

    /**
   * Returns the last commanded voltage of right Motor 1
   * 
   * @return Double in volts between 0 and 12
   */
  public double getrightMotor1Voltage()
  {
    return rightMotor1Voltage;
  }

  /**
   * Returns the last commanded voltage of right Motor 2
   * 
   * @return Double in volts between 0 and 12
   */
  public double getrightMotor2Voltage()
  {
    return rightMotor2Voltage;
  }

  /**
   * Returns the last commanded voltage of left Motor 1
   * 
   * @return Double in volts between 0 and 12
   */
  public double getleftMotor1Voltage()
  {
    return leftMotor1Voltage;
  }

  /**
   * Returns the last commanded voltage of left Motor 2
   * 
   * @return Double in volts between 0 and 12
   */
  public double getleftMotor2Voltage()
  {
    return leftMotor2Voltage;
  }

  public double getRightEncoderRate()
  {
    return drivetrainRightEncoder.getRate();
  }

  public double getLeftEncoderRate()
  {
    return drivetrainLeftEncoder.getRate();
  }

  public double getAverageEncoderRate()
  {
    return ((getRightEncoderRate() + getLeftEncoderRate()) / 2);
  }

  public double PIDVoltagefeedRightMotor1()
  {
    if (getRightEncoderRate() != 0)
      return this.getrightMotor1Voltage() / this.getRightEncoderRate();
    else
      return 0.0;
  }

  public double PIDVoltagefeedRightMotor2()
  {
    if (getRightEncoderRate() != 0)
      return this.getrightMotor2Voltage() / this.getRightEncoderRate();
    else
      return 0.0;
  }

  public double PIDVoltagefeedLeftMotor1()
  {
    if (getLeftEncoderRate() != 0)
      return this.getleftMotor1Voltage() / this.getLeftEncoderRate();
    else
      return 0.0;
  }

  public double PIDVoltagefeedLeftMotor2()
  {
    if (getLeftEncoderRate() != 0)
      return this.getleftMotor2Voltage() / this.getLeftEncoderRate();
    else
      return 0.0;
  }
  
  /**
   * returns heading of robot
   * 
   * @return double between 0 degrees and 360 degrees
   */
  public double getHeading()
  {
    return _gyroSPI.getPos();
  }

  /**
   * Reset robot heading to zero.
   */
  public void resetGyro()
  {
    _gyroSPI.reset();
  }

  /**
   * Calibrate gyro. This should only be called if the robot will be stationary
   * for the calibration period.
   */
  public void calibrateGyro()
  {
    _gyroSPI.calibrate();
  }

  /**
   * @return true if the gyro completed its previous calibration sequence.
   */
  public boolean isGyroCalibrated()
  {
    return _gyroSPI.hasCompletedCalibration();
  }

  /**
   * @return true if the gyro is being calibrated.
   */
  public boolean isGyroCalibrating()
  {
    return _gyroSPI.isCalibrating();
  }

  /**
   * Call to stop an active gyro calibration sequence.
   */
  public void stopGyroCalibrating()
  {
    _gyroSPI.stopCalibrating();
  }




  @Override
  public void initDefaultCommand() {
  //  setDefaultCommand(new DriveWithJoystick(0));
  }
}
