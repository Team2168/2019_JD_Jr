/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team2168.RobotMap;
import org.team2168.commands.vacuumClimber.DriveVacuumClimberLiftWIthJoysticks;

/**
 * Add your docs here.
 */
public class VacuumClimberLift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Victor climberLiftMotor1;
  private Victor climberLiftMotor2;

  private static VacuumClimberLift _instance;

  private VacuumClimberLift()
  {
    climberLiftMotor1 = new Victor(RobotMap.CLIMBER_LIFT_MOTOR_1_PDP);
    climberLiftMotor2 = new Victor(RobotMap.CLIMBER_LIFT_MOTOR_2_PDP);

  }

  /**
   * Singleton constructor of the vacuum climber lift
   * 
   */

  public static VacuumClimberLift getInstance()
  {
    if (_instance == null)
      _instance = new VacuumClimberLift();
    return _instance;
  }

  public void driveClimberLiftMotor1(double speed)
  {
    if(RobotMap.CLIMBER_LIFT_MOTOR_1_REVERSE)
    {
      speed = -speed;
    }
    climberLiftMotor1.set(speed);

  }

  public void driveClimberLiftMotor2(double speed)
  {
    if(RobotMap.CLIMBER_LIFT_MOTOR_2_REVERSE)
    {
      speed = -speed;
    }
    climberLiftMotor2.set(speed);

  }

  public void driveClimberLiftMotors(double speed)
  {
    driveClimberLiftMotor1(speed);
    driveClimberLiftMotor2(speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveVacuumClimberLiftWIthJoysticks());
  }
}
