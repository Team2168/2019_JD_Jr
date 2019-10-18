/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.commands.Lift.DriveLiftWithJoysticks;
import org.team2168.PID.sensors.AveragePotentiometer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * uses: 1 Talon SRX/1Victor SPX, Potentiameter/Encoder/Hall Effects (bruh which???),
 * 30 amp breaker?
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX liftMotor1;
  private VictorSPX liftMotor2;

  private static AveragePotentiometer liftPot;
  private static DigitalInput liftFullyUp; //hall effect sensor
  private static DigitalInput liftFullyDown; //hall effect sensor

  private static Lift _instance;


  private Lift(){
    liftMotor1=new TalonSRX(RobotMap.LIFT_MOTOR_1_PDP);
    liftMotor2=new VictorSPX(RobotMap.LIFT_MOTOR_2_PDP);
    liftPot=new AveragePotentiometer(liftMotor1,RobotMap.LIFT_POT_VOLTAGE_0,RobotMap.LIFT_POT_0_HEIGHT_INCHES,RobotMap.LIFT_POT_VOLTAGE_MAX,RobotMap.LIFT_POT_MAX_HEIGHT_INCHES, RobotMap.LIFT_AVG_ENCODER_VAL); 
    liftFullyUp=new DigitalInput(RobotMap.LIFT_FULLY_UP_LIMIT);
    liftFullyDown=new DigitalInput(RobotMap.LIFT_FULLY_DOWN_LIMIT);
  }
  public static Lift GetInstance(){
    if (_instance==null)
      _instance=new Lift();
    return _instance;
  }

  private void driveLiftMotor1(double speed){
    if (RobotMap.LIFT_MOTOR1_REVERSE)
      speed=-speed;
    liftMotor1.set(ControlMode.PercentOutput, speed);
    //liftMotor1Voltage=Robot.pdp.getBatteryVoltage() * speed;  //also what is this used for?
  }

  private void driveLiftMotor2(double speed){
    if (RobotMap.LIFT_MOTOR2_REVERSE)
      speed=-speed;
    liftMotor2.set(ControlMode.PercentOutput, speed);
    //liftMotor2Voltage=Robot.pdp.getBatteryVoltage() * speed;  //also what is this used for?
  }

  public void driveAllMotors(double speed){
    driveLiftMotor1(speed);
    driveLiftMotor2(speed);
  }

  public double getRawPot(){
    return liftPot.getRawPos();
  }

  public double getPotPos(){
    return liftPot.getPos();
  }

  public double getPotRate(){
    return liftPot.getRate();
  }

  public Boolean isLiftFullyUp(){
    return !liftFullyUp.get();
  }

  public Boolean isLiftFullyDown(){
    return !liftFullyDown.get();
  }


  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveLiftWithJoysticks());
  }
}
