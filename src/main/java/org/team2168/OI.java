
package org.team2168;

import org.team2168.commands.drivetrain.PIDCommands.EnableLimelight;
import org.team2168.commands.drivetrain.PIDCommands.PauseLimelight;
import org.team2168.commands.hatchManipulator.DisengageHatch;
import org.team2168.commands.hatchManipulator.EngageHatch;
import org.team2168.commands.hatchManipulator.IntakeHatchPanel;
import org.team2168.commands.vacuumClimber.DriveVacuumClimberPumpWithConstant;
import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI
{
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	private static OI instance = null;

	public F310 driverJoystick = new F310(RobotMap.DRIVER_JOYSTICK);
	public F310 operatorJoystick = new F310(RobotMap.OPERATOR_JOYSTICK);

	private LinearInterpolator gunStyleInterpolator;
	private double[][] gunStyleArray = { { -1.0, -1.0},
										{ -0.15, 0.0},
										{  0.15, 0.0},
										{  1.0,  1.0}};

	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI() {
		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/

		driverJoystick.ButtonB().whenPressed(new EnableLimelight());
		driverJoystick.ButtonB().whenReleased(new PauseLimelight());
		driverJoystick.ButtonLeftStick().whenPressed(new EnableLimelight());
		driverJoystick.ButtonLeftStick().whenReleased(new PauseLimelight());

		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);

		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/
		
		operatorJoystick.ButtonDownDPad().whenPressed(new DriveVacuumClimberPumpWithConstant(-0.7)); //TODO totally a guess
		operatorJoystick.ButtonDownDPad().whenReleased(new DriveVacuumClimberPumpWithConstant(0.0));
		operatorJoystick.ButtonUpDPad().whenPressed(new DriveVacuumClimberPumpWithConstant(0.7));
		operatorJoystick.ButtonUpDPad().whenReleased(new DriveVacuumClimberPumpWithConstant(0.0));



		//Button X
		operatorJoystick.ButtonX().whileHeld(new IntakeHatchPanel());

		// Button A
		operatorJoystick.ButtonA().whenPressed(new DriveVacuumClimberPumpWithConstant(1.0));
		
		//Button Y
		operatorJoystick.ButtonY().whenPressed(new EngageHatch());
		
		//Button B
		operatorJoystick.ButtonB().whenPressed(new DisengageHatch());

	}
	

	/**
	 * Returns an instance of the Operator Interface.
	 * 
	 * @return is the current OI object
	 */
	public static OI getInstance()
	{
		if (instance == null)
			instance = new OI();

		return instance;
	}

	/*************************************************************************
	 * Lift *
	 *************************************************************************/

	public double getLiftJoystickValue()
	{

			return operatorJoystick.getLeftStickRaw_Y();
		}

	/*************************************************************************
	 * Cargo Intake *
	 *************************************************************************/

	public double getCargoIntakeJoystickValue()
	{

		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw();
	}

	/*************************************************************************
	 *Vacuum Climber Lift & Pump*
	*************************************************************************/
	public double getVacuumClimberLiftJoystickValue()
	{
		return -operatorJoystick.getRightStickRaw_Y(); //invert so pushing up goes up
	}

	public double getVacuumClimberPumpJoystickValue()
	{
		return operatorJoystick.getLeftStickRaw_Y();
	} 

	/*************************************************************************
	 * Drivetrain *
	 *************************************************************************/

	public double getGunStyleXValue()
	{
		return driverJoystick.getLeftStickRaw_X();
	}

	public double getGunStyleYValue()
	{

		return gunStyleInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
	}

	/**
	 * Method that sets that Left side of the drive train so that it drives with
	 * LeftStick Y
	 * 
	 * @author Krystina
	 */
	public double getDriveTrainLeftJoystick()
	{
		return driverJoystick.getLeftStickRaw_Y();
	}

	/**
	 * Method that sets that Right side of the drive train so that it drives with
	 * RightStick Y
	 * /
	 * @author Krystina
	 */
	public double getDriveTrainRightJoystick()
	{
		return driverJoystick.getRightStickRaw_Y();
	}

}