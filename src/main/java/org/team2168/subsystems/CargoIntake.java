/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.RobotMap;
import org.team2168.PID.sensors.CanAnalogInput;
import org.team2168.commands.cargoIntake.DriveCargoIntakeWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;
/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CanAnalogInput _sharpIRSensor;
  public TalonSRX intakeMotor; //public so we can link the hatch intake limit switch to it
  public static CargoIntake instance;


  private CargoIntake() {
    _sharpIRSensor = new CanAnalogInput(intakeMotor, CanAnalogInput.kSCALE_3_3_VOLTS);
    intakeMotor = new TalonSRX(RobotMap.CARGO_INTAKE_MOTOR_PDP);
    ConsolePrinter.putNumber("Cargo Raw IR", () -> {return getRawIRVoltage();}, true, false);
    ConsolePrinter.putBoolean("isCargoPresent", () -> {return isCargoPresent();}, true, false);
  }

  public static CargoIntake getInstance()
	{
	 if (instance==null)
	 instance = new CargoIntake();
	 return instance;
  }

  public void driveCargoIntakeMotor(double speed){
    if (RobotMap.CARGO_INTAKE_MOTOR_REVERSE)
      speed = -speed;
    intakeMotor.set(ControlMode.PercentOutput,speed);
  }

  public boolean isCargoPresent()
  {
    return (getRawIRVoltage() >= RobotMap.CARGO_INTAKE_IR_THRESHOLD_MIN && getRawIRVoltage() <= RobotMap.CARGO_INTAKE_IR_THRESHOLD_MAX);
  }

  public double getRawIRVoltage()
  {
    return _sharpIRSensor.getVoltage();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveCargoIntakeWithJoystick());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

