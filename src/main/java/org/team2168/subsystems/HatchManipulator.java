package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.CanDigitalInput;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchManipulator extends Subsystem {
    private DoubleSolenoid _hatchPiston;
    private CanDigitalInput _limitSwitch;
    private static HatchManipulator instance = null;
   



    private HatchManipulator()
    {
        _hatchPiston = new DoubleSolenoid(RobotMap.HATCH_EXTENSION_PCM, RobotMap.HATCH_RETRACTION_PCM);
        _limitSwitch = new CanDigitalInput(Robot.cargoIntake.intakeMotor); //TO DO
        ConsolePrinter.putBoolean("Manipulator is Extended", () -> {return isManipulatorExtended();}, true, false);
        ConsolePrinter.putBoolean("Is Hatch Present", () -> {return isHatchPresent();}, true, false);
    }
    public void extend()
    {
        _hatchPiston.set(Value.kForward);
    }

    public void retract()
    {
        _hatchPiston.set(Value.kReverse);
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
        return _hatchPiston.get() == Value.kForward;
    }

    public boolean isManipulatorRetracted()
        {
            return _hatchPiston.get() == Value.kReverse;
        }


    public void initDefaultCommand(){

    }
}
