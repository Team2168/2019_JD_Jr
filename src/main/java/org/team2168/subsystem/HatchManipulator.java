package org.team2168.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team2168.Robot;
import org.team2168.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchManipulator extends Subsystem{
    private DoubleSolenoid _dSolenoidExtend;
    private DigitalInput _hatchCheck1;
    private DigitalInput _hatchCheck2;
    private DigitalInput _hatchCheck3;
    private DigitalInput _hatchCheck4;
}

private HatchManipulator()
{
    _dSolenoidExtend = new DoubleSolenoid(RobotMap.HATCH_EXTENSION_PCM, RobotMap.HATCH_RETRACTION_PCM);
    _hatchCheck1 = new DigitalInput(RobotMap.CHECK_ONE);
    _hatchCheck2 = new DigitalInput(RobotMap.CHECK_TWO);
    _hatchCheck3 = new DigitalInput(RobotMap.CHECK_THREE);
    _hatchCheck4 = new DigitalInput(RobotMap.CHECK_FOUR);
}
public void extend()
{
    _dSolenoidExtend.set(Value.kForward);
}

public void retract()
{
    _dSolenoidExtend.set(Value.kReverse);
}

public void checkOne(){
    if (_hatchCheck1.get())
    {
        output Signal.max(output, 0);
    else 
        output Signal.min(output, 0);
    }

    
public void checkTwo(){
    if (_hatchCheck2.get())
    {
        output Signal.max(output, 0);
    else 
        output Signal.min(output, 0);
    }

    
public void checkThree(){
    if (_hatchCheck3.get())
    {
        output Signal.max(output, 0);
    else 
        output Signal.min(output, 0);
    }

    
public void checkFour(){
    if (_hatchCheck4.get())
    {
        output Signal.max(output, 0);
    else 
        output Signal.min(output, 0);
    }
}

public 

public void initDefaultCommand(){

}
}
