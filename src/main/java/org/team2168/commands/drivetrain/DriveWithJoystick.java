/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain;

import org.team2168.OI;
import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithJoystick extends Command {

    int ctrlStyle;
    /**
     * Controller Styles 0 = Tank Drive (Default) 1 = Gun Style 2 = Arcade Drive 3 =
     * GTA
     * 
     * @param inputStyle
     */

    private double distanceGoal;
    private double speed;
    private double endDistance;
    private boolean finished;
    private double angle;
    private double error = 0.1;

    private int intCounter = 0;

    double rightSpeed = 0;
    double leftSpeed = 0;

    static final double DIST_ERROR_TOLERANCE_INCH = 1;
    static final double TURN_ERROR_TOLERANCE_DEG = 1;

    double lastRotateOutput;

    public DriveWithJoystick(int inputStyle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    ctrlStyle = inputStyle;
    // TODO not sure where distanceGoal is used but for our tests we used a value of
    // 1
    this.distanceGoal = 1;
    this.speed = RobotMap.AUTO_NORMAL_SPEED;
    this.lastRotateOutput = 0;

    SmartDashboard.putNumber("Recordnumber", 0);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        intCounter = 0;
        ctrlStyle = Robot.getControlStyleInt();
        switch (ctrlStyle) {
        /**
         * Initialize driveStraightController for Gun style
         */
        case 1:
            finished = false;
            Robot.drivetrain.getInstance();
        
            // reset controller
                Robot.drivetrain.resetPosition();	
                Robot.drivetrain.imu.reset();
                Robot.drivetrain.driveTrainPosController.reset();
                Robot.drivetrain.rotateDriveStraightController.reset();

            // drivetrain.resetGyro();
            endDistance = Robot.drivetrain.getAverageDistance() + distanceGoal;
            angle = Robot.drivetrain.getHeading();

            // Robot.drivetrain.rotateDriveStraightController.setpGain(RobotMap.ROTATE_POSITION_P_Drive_Straight);
            // Robot.drivetrain.rotateDriveStraightController.setiGain(RobotMap.ROTATE_POSITION_I_Drive_Straight);
            // Robot.drivetrain.rotateDriveStraightController.setdGain(RobotMap.ROTATE_POSITION_D_Drive_Straight);
            Robot.drivetrain.driveTrainPosController.setSetPoint(endDistance);
            Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
            Robot.drivetrain.driveTrainPosController.setMinPosOutput(-speed);
            Robot.drivetrain.driveTrainPosController.setAcceptErrorDiff(error); // feet
            Robot.drivetrain.rotateDriveStraightController.setSetPoint(angle);

            Robot.drivetrain.driveTrainPosController.Enable();
            //Robot.drivetrain.rotateDriveStraightController.Enable();

            System.out.println("Initialize case ran");
        default:

            break;
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        double headingCorrection = 0.0;
        ctrlStyle = Robot.getControlStyleName();

        switch(ctrlStyle) {
        case 0:
                Robot.drivetrain.driveLeft(Robot.oi.driverJoystick.getLeftStickRaw_Y());
                Robot.drivetrain.driveRight(Robot.oi.driverJoystick.getRightStickRaw_Y());
            break;
            
        case 1:
            lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
            headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput());

            if ((Robot.oi.driverJoystick.getLeftStickRaw_X() < 0.1) && (Robot.oi.driverJoystick.getLeftStickRaw_X() > -0.1))
            {
                Robot.drivetrain.tankDrive(Robot.oi.getGunStyleYValue(), Robot.oi.getGunStyleYValue());	

            } 
            else {
                Robot.drivetrain.tankDrive(
                        (Robot.oi.getGunStyleYValue()) + Robot.oi.driverJoystick.getLeftStickRaw_X(),
                        (Robot.oi.getGunStyleYValue()) - Robot.oi.driverJoystick.getLeftStickRaw_X());
                Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
            }
            break;

        /**
         * Arcade Drive
         */
        case 2:
            Robot.drivetrain.driveLeft(
                    Robot.oi.driverJoystick.getLeftStickRaw_Y() + Robot.oi.driverJoystick.getRightStickRaw_X());
            Robot.drivetrain.driveRight(
                    Robot.oi.driverJoystick.getLeftStickRaw_Y() - Robot.oi.driverJoystick.getRightStickRaw_X());
            break;
        /**
         * GTA Drive
         */
        case 3:
            double fwdSpeed = Robot.oi.driverJoystick.getRightTriggerAxisRaw();
            double revSpeed = Robot.oi.driverJoystick.getLeftTriggerAxisRaw();
            double speed = fwdSpeed - revSpeed;
            double rotation = Robot.oi.driverJoystick.getRightStickRaw_X();

            // Adjusts angle while moving
            if (speed != 0 && rotation != 0) {
                Robot.drivetrain.driveLeft(rotation * speed);
                Robot.drivetrain.driveRight(-rotation * speed);
            }
            // Allows Robot to spin in place without needing to press in triggers
            else if (speed == 0 && rotation != 0) {
                Robot.drivetrain.driveLeft(rotation);
                Robot.drivetrain.driveRight(-rotation);
            }
            // Allows Robot to drive straight
            else if (speed != 0 && rotation == 0) {
                Robot.drivetrain.driveLeft(speed);
                Robot.drivetrain.driveRight(speed);
            }
            break;

        /**
         * New Gun Style Controller
         */
        case 4:
            lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
            headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput());

            if (Math.abs(Robot.oi.driverJoystick.getX(Hand.kLeft)) < 0.1) 
            {
                    //Drive straight
                    if(Robot.drivetrain.limelightPosController.isEnabled())
                    {
                        Robot.drivetrain.tankDrive(
                        -Robot.oi.driverJoystick.getY(Hand.kLeft) - Robot.drivetrain.limelightPosController.getControlOutput(),
                        -Robot.oi.driverJoystick.getY(Hand.kLeft) + Robot.drivetrain.limelightPosController.getControlOutput());
                    }
                    else {
                        Robot.drivetrain.tankDrive(-Robot.oi.driverJoystick.getY(Hand.kLeft),
                            -Robot.oi.driverJoystick.getY(Hand.kLeft));
                    }	
            } 
            else {
                    //Arcade drive
                    if(Robot.drivetrain.limelightPosController.isEnabled())
                    {
                        Robot.drivetrain.tankDrive(
                            Robot.oi.getGunStyleYValue() + Robot.oi.driverJoystick.getX(Hand.kLeft) - Robot.drivetrain.limelightPosController.getControlOutput(),
                            Robot.oi.getGunStyleYValue() - Robot.oi.driverJoystick.getX(Hand.kLeft) + Robot.drivetrain.limelightPosController.getControlOutput());
                        Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
                    }
                    else {
                        Robot.drivetrain.tankDrive(
                                Robot.oi.getGunStyleYValue() + Robot.oi.driverJoystick.getX(Hand.kLeft),
                                Robot.oi.getGunStyleYValue() - Robot.oi.driverJoystick.getX(Hand.kLeft));
                        Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
                    }
            }
            break;
        /**
         * Defaults to Tank Drive
         */
        default:
            Robot.drivetrain.driveLeft(Robot.oi.driverJoystick.getLeftStickRaw_Y());
            Robot.drivetrain.driveRight(Robot.oi.driverJoystick.getRightStickRaw_Y());
            break;
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    Robot.drivetrain.tankDrive(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
