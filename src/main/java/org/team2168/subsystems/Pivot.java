/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Pivot extends Subsystem {
  private static Pivot instance = null;
  private static DoubleSolenoid PivotRobot;
  
  public boolean isRobotPartExtended = false;
  public Pivot() 
  {
    PivotRobot = new DoubleSolenoid(RobotMap.PIVOT_THING_PCM, 0);
  }
  
  public static Pivot getinstance()
  {
    if (instance == null)
        instance = new Pivot();
    return instance;
  }

  /**
   * Extends the pivot position for the robot part
   */
  public void extendRobotPart()
  {
    PivotRobot.set(DoubleSolenoid.Value.kForward);
    this.isRobotPartExtended = true;
  }
  
  /**
   * Retracts the pivot postion for the robot part
   */
  public void retractRobotPart()
  {
    PivotRobot.set(DoubleSolenoid.Value.kReverse);
    this.isRobotPartExtended = false;
  }

  public boolean getRobotPartStatus()
  {
    return isRobotPartExtended;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
