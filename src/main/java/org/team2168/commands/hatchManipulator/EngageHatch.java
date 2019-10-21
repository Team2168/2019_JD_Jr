/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.hatchManipulator;

import edu.wpi.first.wpilibj.command.Command;
import org.team2168.Robot;
import org.team2168.subsystem.HatchManipulator;

public class EngageHatch extends Command {
  public EngageHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires (Robot.hatchManipulator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hatchManipulator.extend();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.hatchManipulator.isManipulatorExtended();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
