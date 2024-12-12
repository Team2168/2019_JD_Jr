/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.RobotMap;
import org.team2168.commands.pivotIntake.PivotIntakeWithJoystick;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class IntakePivot extends Subsystem {
  private static IntakePivot instance = null;
  private static DoubleSolenoid pivotRobot;

  public IntakePivot() 
  {
    pivotRobot = new DoubleSolenoid(RobotMap.PIVOT_EXTENDS_PCM, RobotMap.PIVOT_RETRACTS_PCM);
  }
  
  public static IntakePivot getInstance()
  {
    if (instance == null)
        instance = new IntakePivot();
    return instance;
  }
  
  /**
   * Extends the pivot position for the robot part
   */
  public void pivotIntakeUp()
  {
    pivotRobot.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Retracts the pivot postion for the robot part
   */
  public void pivotIntakeDown()
  {
    pivotRobot.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isPivotIntakeUp()
  {
    return pivotRobot.get() == Value.kForward;

  }
  
  public boolean isPivotIntakeDown()
  {
    return pivotRobot.get() == Value.kReverse;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new PivotIntakeWithJoystick());
  }
}
