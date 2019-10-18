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
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public PIDPosition liftPOTController;
  TCPSocketSender TCPLiftPOTController;

  private TalonSRX liftMotor1;
  private VictorSPX liftMotor2;

  private static AveragePotentiometer liftPot;
  private static DigitalInput liftFullyUp; //hall effect sensor
  private static DigitalInput liftFullyDown; //hall effect sensor

  private static Lift _instance;

  private boolean liftMotor1Fault=false;
  private boolean liftMotor2Fault=false;

  private boolean liftMotor1HighCurrent=false;
  private boolean liftMotor2HighCurrent=false;

  private boolean liftMotor1HighThenZeroCurrent=false;
  private boolean liftMotor2HighThenZeroCurrent=false;

  private boolean isLiftMotor1BreakerTrip=false;
  private boolean isLiftMotor2BreakerTrip=false;

  public volatile double liftMotor1Voltage;
  public volatile double liftMotor2Voltage;

  private boolean isSensorValid=true;



  private Lift(){
    liftMotor1=new TalonSRX(RobotMap.LIFT_MOTOR_1_PDP);
    liftMotor2=new VictorSPX(RobotMap.LIFT_MOTOR_2_PDP);
    liftPot=new AveragePotentiometer(liftMotor1,RobotMap.LIFT_POT_VOLTAGE_0,RobotMap.LIFT_POT_0_HEIGHT_INCHES,RobotMap.LIFT_POT_VOLTAGE_MAX,RobotMap.LIFT_POT_MAX_HEIGHT_INCHES, RobotMap.LIFT_AVG_ENCODER_VAL); 
    liftFullyUp=new DigitalInput(RobotMap.LIFT_FULLY_UP_LIMIT);
    liftFullyDown=new DigitalInput(RobotMap.LIFT_FULLY_DOWN_LIMIT);

    liftPOTController=new PIDPosition("LiftPOSController",RobotMap.LIFT_P,RobotMap.LIFT_I,RobotMap.LIFT_D,liftPot,RobotMap.LIFT_PID_PERIOD);
    liftPOTController.setSIZE(RobotMap.LIFT_PID_ARRAY_SIZE);
    liftPOTController.startThread();
    TCPLiftPOTController=new TCPSocketSender(RobotMap.TCP_SERVER_LIFT_POT_CONTROLLER, liftPOTController);
    TCPLiftPOTController.start();
    
    ConsolePrinter.putNumber("Lift Joystick value", () -> {return Robot.oi.getLiftJoystickValue();}, true,true);
    ConsolePrinter.putNumber("Lift motor 1 voltage", () -> {return liftMotor1Voltage;}, true, true);
    ConsolePrinter.putNumber("Lift motor 2 voltage", () -> {return liftMotor2Voltage;}, true, true);

    ConsolePrinter.putNumber("Lift motor 1 current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP);}, true, true);
    ConsolePrinter.putNumber("Lift motor 2 current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP);}, true, true);

    ConsolePrinter.putBoolean("Is lift fully up", () -> {return Robot.lift.isLiftFullyUp();}, true, false);
    ConsolePrinter.putBoolean("Is lift fully down", () -> {return Robot.lift.isLiftFullyDown();}, true, false);
    ConsolePrinter.putNumber("Lift raw pot", () -> {return getRawPot();}, true, false);
    ConsolePrinter.putNumber("Lift pot inches", () -> {return getPotPos();}, true, false);
    ConsolePrinter.putNumber("Lift pot rate", () -> {return getPotRate();}, true, false);
    ConsolePrinter.putBoolean("Lift is sensor valid", () -> {return isSensorValid();}, true, false);

    ConsolePrinter.putBoolean("Lift motor1_fault", () -> {return liftMotor1Fault;}, true, false);
    ConsolePrinter.putBoolean("Lift motor2_fault", () -> {return liftMotor2Fault;}, true, false);
    
    ConsolePrinter.putBoolean("Lift motor1_breaker_trip", () -> {return isLiftMotor1BreakerTrip;}, true, false);
    ConsolePrinter.putBoolean("Lift motor2_breaker_trip", () -> {return isLiftMotor2BreakerTrip;}, true, false);
    
  }



  public static Lift GetInstance(){
    if (_instance==null)

      _instance=new Lift();

    return _instance;
  }

  

  
  private void isLiftMotor1Failure(){
    double conditionLimitPercent=0.5;

    if(!this.liftMotor1Fault && this.liftMotor1Voltage >=RobotMap.LIFT_MIN_SPEED){
        this.liftMotor1Fault=((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)<=conditionLimitPercent*Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP)&&Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP)>2));

    }
  }




  private void isLiftMotor2Failure(){
    double conditionLimitPercent=0.5;

    if(!this.liftMotor2Fault && this.liftMotor1Voltage >=RobotMap.LIFT_MIN_SPEED){
      this.liftMotor2Fault=((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)<=conditionLimitPercent*Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)&&Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)>2));
    }
  }



  private void isLiftMotor1BreakerTrip(){
    //the motor is moving, and we are seeing if it overdrew current, tripping a breaker
    if(this.liftMotor1Voltage >= RobotMap.LIFT_MIN_SPEED){

      if(Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)>15)
        liftMotor1HighCurrent=true;

      if(liftMotor1HighCurrent&&Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)<1) //tests if the current stopped after exceeding the maximum before tripping a breaker
        liftMotor1HighThenZeroCurrent=true;

      if(this.liftMotor1HighCurrent&&liftMotor1HighThenZeroCurrent)
        this.isLiftMotor1BreakerTrip=Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)>3;

    }
  }



  private void isLiftMotor2BreakerTrip(){
    if(this.liftMotor2Voltage>=RobotMap.LIFT_MIN_SPEED){
      if(Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP)>15)
        liftMotor2HighCurrent=true;
      if(liftMotor2HighCurrent&&Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP)<1)
        liftMotor2HighThenZeroCurrent=true;
      if(this.liftMotor2HighCurrent&&liftMotor2HighThenZeroCurrent)
        this.isLiftMotor2BreakerTrip=Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP)>3;
    }
  }


  
  private void driveLiftMotor1(double speed){
    if (RobotMap.LIFT_MOTOR1_REVERSE)
      speed=-speed;
    liftMotor1.set(ControlMode.PercentOutput, speed);
    liftMotor1Voltage=Robot.pdp.getBatteryVoltage() * speed;
  }




  private void driveLiftMotor2(double speed){
    if (RobotMap.LIFT_MOTOR2_REVERSE)
      speed=-speed;
    liftMotor2.set(ControlMode.PercentOutput, speed);
    liftMotor2Voltage=Robot.pdp.getBatteryVoltage() * speed;
  }




  public void driveAllMotors(double speed){
    
    double stallLimit=35;
    //lift is stalling

    if((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)>stallLimit)||(Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) > stallLimit)){
      //i didn't include the timeCounter part because this lift has no break
      driveLiftMotor1(0.0);
      driveLiftMotor2(0.0);
    }

    else{
      if(RobotMap.ENABLE_LIFT_POT_SAFETY){ //uses potentiometer as another sensor for an additional check
        
        if((speed>RobotMap.LIFT_MIN_SPEED&&!isLiftFullyUp() && !liftPot.isAtUpperLimit()) ||
         ((speed<-RobotMap.LIFT_MIN_SPEED) && !isLiftFullyDown())){

        driveLiftMotor1(speed);
        driveLiftMotor2(speed);    
        }

        else{
          driveLiftMotor1(0.0);
          driveLiftMotor2(0.0);
        }

      }
      
    else{

      if(speed>RobotMap.LIFT_MIN_SPEED&&!isLiftFullyUp() || 
      ((speed<-RobotMap.LIFT_MIN_SPEED) && !isLiftFullyDown())){

        driveLiftMotor1(speed);
        driveLiftMotor2(speed);    
        }

        else{
          driveLiftMotor1(0.0);
          driveLiftMotor2(0.0);
        }

      }

      isLiftMotor1Failure();
      isLiftMotor2Failure();

      isLiftMotor1BreakerTrip();
      isLiftMotor2BreakerTrip();
      System.out.println("is this working?");
    }

  /* 
    //if the lift is driving, the potentiometer should reflect that
    if(Math.abs(liftPot.getRate())<1)
      isSensorValid=false;
    else
      isSensorValid=true;
 */



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



  public boolean isSensorValid(){
    return this.isSensorValid;
  }
  


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveLiftWithJoysticks());
  }
}