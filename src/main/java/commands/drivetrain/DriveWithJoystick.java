/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import org.team2168.Robot;
import org.team2168.*;
import org.team2168.subsystems.Drivetrain;

public class DriveWithJoystick extends Command {
  public DriveWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intCounter = 0;
    ctrlStyle = Robot.getControlStyleName();
    switch(ctrlStyle) {
    case 0:
			Robot.drivetrain.driveLeft(Robot.oi.driverJoystick.getLeftStickRaw_Y());
			Robot.drivetrain.driveRight(Robot.oi.driverJoystick.getRightStickRaw_Y());
      break;
      
    case 1:
			{
			lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
			headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput());

			//
			double minStingerVoltage = 4.0/Robot.pdp.getBatteryVoltage();

			if(OI.getInstance().driverJoystick.isPressedButtonLeftBumper())
			{
				Robot.drivetrain.tankDrive(minStingerVoltage, 0.0);
				System.out.println("Left Speed:" + speed);
			}
			if(OI.getInstance().driverJoystick.isPressedButtonRightBumper())
			{
				Robot.drivetrain.tankDrive(0.0, minStingerVoltage);
				System.out.println("Right Speed:" + speed);
			}
			if(Robot.isClimbEnabled)
			{ 
				if (climbCounter < 25) //auto drive drivetrain for a small time 0.5 seconds 7*0.02 to help engage
				{
					double voltage = 1.0;
					double speed = voltage/Robot.pdp.getBatteryVoltage();
					Robot.drivetrain.tankDrive(speed,speed);
					climbCounter++;
				}
				else
					Robot.drivetrain.tankDrive(Robot.oi.getGunStyleYValue(), Robot.oi.getGunStyleYValue());
			}
			else if ((Robot.oi.driverJoystick.getLeftStickRaw_X() < 0.1) && (Robot.oi.driverJoystick.getLeftStickRaw_X() > -0.1))
			{
				Robot.drivetrain.tankDrive(Robot.oi.getGunStyleYValue(), Robot.oi.getGunStyleYValue());	
				climbCounter = 0;
				climbCounterReverse = 0;
			} 
			else {
				Robot.drivetrain.tankDrive(
						(Robot.oi.getGunStyleYValue()) + Robot.oi.driverJoystick.getLeftStickRaw_X(),
						(Robot.oi.getGunStyleYValue()) - Robot.oi.driverJoystick.getLeftStickRaw_X());
				Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
				climbCounter = 0;
				climbCounterReverse = 0;
						
			}
			
			
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
			//
			double minStingerVoltage = 4.0/Robot.pdp.getBatteryVoltage();

			if(OI.getInstance().driverJoystick.isPressedButtonLeftBumper())
			{
				Robot.drivetrain.tankDrive(0.0, minStingerVoltage);
				System.out.println("Right speed:" + minStingerVoltage);
				return;
			}

			if(OI.getInstance().driverJoystick.isPressedButtonRightBumper())
			{
				Robot.drivetrain.tankDrive(minStingerVoltage, 0.0);
				System.out.println("Left Speed:" + minStingerVoltage);
				return;
			}

			if(Robot.isClimbEnabled)
			{ 
				if( disengageDrivetrainCommand == null)
					disengageDrivetrainCommand = new DisengageDrivetrain();

				
				if (climbCounter < 25) //auto drive drivetrain for a small time 0.5 seconds 7*0.02 to help engage
				{
					double voltage = 2.0;
					double minspeed = voltage/Robot.pdp.getBatteryVoltage();
					Robot.drivetrain.tankDrive(minspeed,minspeed);
					System.out.println("Driving stinger slow");
					climbCounter++;
				}
				else
				{
					if(-Robot.oi.driverJoystick.getY(Hand.kLeft)>0.1)
						Robot.drivetrain.tankDrive(-Robot.oi.driverJoystick.getY(Hand.kLeft), -Robot.oi.driverJoystick.getY(Hand.kLeft));
					else //driving dt in reverse, lets not do that
					{
						if (climbCounterReverse < 15) //auto drive drivetrain for a small time 0.5 seconds 7*0.02 to help engage
						{
							if(!disengageDrivetrainCommand.isRunning())
								disengageDrivetrainCommand.start();

							double voltage = 1.0;
							double minspeed = voltage/Robot.pdp.getBatteryVoltage();
							Robot.drivetrain.tankDrive(minspeed,minspeed);
							System.out.println("Driving stinger slow");
							climbCounterReverse++;
						}
						else if (!Robot.isClimbEnabledLevel2)
						{
							if(Robot.drivetrain.getLeftStingerPosition()>0.0)
							{
								//if(Robot.oi.driverJoystick.getY(Hand.kLeft)>0.0)
								Robot.drivetrain.driveLeft(-Robot.oi.driverJoystick.getY(Hand.kLeft));
							}
							else
							{
								Robot.drivetrain.driveLeft(0.0);
							}

							if(Robot.drivetrain.getRightStingerPosition()>0.0)
							{
								//if(Robot.oi.driverJoystick.getY(Hand.kLeft)>0.0)
								Robot.drivetrain.driveRight(-Robot.oi.driverJoystick.getY(Hand.kLeft));
							}
							else
							{
								Robot.drivetrain.driveRight(0.0);
							}
						}
					}
					}
				}
			else if (Math.abs(Robot.oi.driverJoystick.getX(Hand.kLeft)) < 0.1) 
			{
					climbCounter = 0;
					climbCounterReverse = 0;
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
				} else {
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
