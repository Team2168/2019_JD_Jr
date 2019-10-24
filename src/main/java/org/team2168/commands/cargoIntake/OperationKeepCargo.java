package org.team2168.commands.cargoIntake;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OperationKeepCargo extends Command {

    public OperationKeepCargo() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cargoIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.cubeIntakeGripper.extendIntake(); 
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.cargoIntake.getRawIRVoltage()<3.1 && Robot.cargoIntake.getRawIRVoltage()>2.5)
        {
            Robot.cargoIntake.driveCargoIntakeMotor(RobotMap.CARGO_INTAKE_HOLDING_SPEED);
        }
        else if(Robot.cargoIntake.getRawIRVoltage()<2.5)
        {
            Robot.cargoIntake.driveCargoIntakeMotor(-1.0);
        }
        else
        {
            Robot.cargoIntake.driveCargoIntakeMotor(RobotMap.CARGO_INTAKE_MIN_SPEED); 
        }
    	
    	//Robot.i2c.write(8, 5);
    		 
    }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.oi.operatorJoystick.getLeftTriggerAxisRaw() > 0.1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.cargoIntake.driveCargoIntakeMotor(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.cargoIntake.driveCargoIntakeMotor(0.0);
    }
}
