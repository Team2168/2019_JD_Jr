/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Pivot extends Subsystem {
  private static Pivot instance = null;
  private static DoubleSolenoid PivotRobot;

  public Pivot() 
  {
    PivotRobot = new DoubleSolenoid(RobotMap.PIVOT_EXTENDS_PCM, RobotMap.PIVOT_RETRACTS_PCM);
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
  }

  /**
   * Retracts the pivot postion for the robot part
   */
  public void retractRobotPart()
  {
    PivotRobot.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isRobotPartExtended()
  {
    return PivotRobot.get() == Value.kForward;

  }
  
  public boolean isRobotPartRetracted()
  {
    return PivotRobot.get() == Value.kReverse;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
