/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team2168.RobotMap;
import org.team2168.commands.vacuumClimber.DriveVacuumClimberPumpWithJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class VacuumClimberPump extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private VictorSPX climberPumpMotor;

  private static VacuumClimberPump _instance;

  private VacuumClimberPump()
  {
    climberPumpMotor = new VictorSPX(RobotMap.CLIMBER_PUMP_MOTOR_PDP);
  }

  /**
   * Singleton constructor of the vacuum climber pmmp
   * 
   */

  public static VacuumClimberPump getInstance()
  {
    if (_instance == null)
      _instance = new VacuumClimberPump();
    return _instance;
  }

  /**
   * IS positive suck???? TODO
   */
  public void driveClimberPumpMotor(double speed)
  {
    if(RobotMap.CLIMBER_PUMP_MOTOR_REVERSE)
    {
      speed = -speed;
    }
    climberPumpMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveVacuumClimberPumpWithJoystick());
  }
}
