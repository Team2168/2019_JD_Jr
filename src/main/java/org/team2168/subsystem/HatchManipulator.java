package org.team2168.subsystem;

import org.team2168.RobotMap;
import org.team2168.PID.sensors.CanDigitalInput;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchManipulator extends Subsystem {
    private DoubleSolenoid _dSolenoidExtend;
    private CanDigitalInput _limitSwitch;
    private static HatchManipulator instance = null;
   

  //  private DigitalInput _hatchCheck2;
  //  private DigitalInput _hatchCheck3;
  //  private DigitalInput _hatchCheck4;


private HatchManipulator()
{
    _dSolenoidExtend = new DoubleSolenoid(RobotMap.HATCH_EXTENSION_PCM, RobotMap.HATCH_RETRACTION_PCM);
    _limitSwitch = new CanDigitalInput(Robot.);
  //  _hatchCheck1 = new DigitalInput(RobotMap.CHECK_ONE);
  //  _hatchCheck2 = new DigitalInput(RobotMap.CHECK_TWO);
  //  _hatchCheck3 = new DigitalInput(RobotMap.CHECK_THREE);
  //  _hatchCheck4 = new DigitalInput(RobotMap.CHECK_FOUR);
  ConsolePrinter.putNumber("HatchManipulator Raw IR", () -> {return getRawIRVoltage();}, true, false);
  ConsolePrinter.putBoolean("Hatch Is Present", () -> {return isHatchPresent();}, true, false);
  ConsolePrinter.putBoolean("Manipulator is Extended", () -> {return isManipulatorExtended();}, true, false);
  ConsolePrinter.putBoolean("Is Hatch Present Limit", () -> {return isHatchPresent();}, true, false);
}
public void extend()
{
    _dSolenoidExtend.set(Value.kForward);
}

public void retract()
{
    _dSolenoidExtend.set(Value.kReverse);
}

public static HatchManipulator getInstance(){
    if (instance == null)
        instance = new HatchManipulator();
    return instance;
}

public boolean isHatchPresent()
{
return _limitSwitch.getForwardLimit();
}

public boolean isManipulatorExtended()
{
    return _dSolenoidExtend.get() == Value.kForward;
}

public boolean isManipulatorRetracted()
    {
        return _dSolenoidExtend.get() == Value.kReverse;
    }


public void initDefaultCommand(){

}
}
