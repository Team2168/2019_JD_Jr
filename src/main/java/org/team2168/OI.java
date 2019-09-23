
package org.team2168;

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
		// driverJoystick.ButtonStart().whenPressed(new EngageStingers()); // add drivetrainshifter
		// driverJoystick.ButtonStart().whenPressed(new DisengageDrivetrain());
		// driverJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(63));

		// driverJoystick.ButtonA().whenPressed(new EngageDrivetrain());
		// driverJoystick.ButtonA().whenPressed(new DisengageStingers());

		// driverJoystick.ButtonX().whenPressed(new DriveMonkeyBarPivotPIDPathAutoClimb(63, 0, 3));
		// driverJoystick.ButtonX().whenPressed(new DriveStingerPIDPath(0,25,3.5));
		
		// driverJoystick.ButtonBack().whenPressed(new DisengageDrivetrain());
		// driverJoystick.ButtonBack().whenPressed(new DisengageStingers());

		// driverJoystick.ButtonB().whenPressed(new EnableLimelight());
		// driverJoystick.ButtonB().whenReleased(new PauseLimelight());
		// driverJoystick.ButtonLeftStick().whenPressed(new EnableLimelight());
		// driverJoystick.ButtonLeftStick().whenReleased(new PauseLimelight());

		// gunStyleInterpolator = new LinearInterpolator(gunStyleArray);

		
		// operatorJoystick.ButtonDownDPad().whenPressed(new MoveLiftToLvl1Position());
		// operatorJoystick.ButtonRightDPad().whenPressed(new MoveLiftToLvl2Position());
		// operatorJoystick.ButtonUpDPad().whenPressed(new MoveLiftToLvl3Position());
		// operatorJoystick.ButtonLeftDPad().whenPressed(new MoveLiftToCargoShipPosition());

		// operatorJoystick.ButtonRightBumper().whenPressed(new DriveMonkeyBarPivotWithConstant(0.7));
		// operatorJoystick.ButtonRightBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));
		// operatorJoystick.ButtonLeftBumper().whenPressed(new DriveMonkeyBarPivotWithConstant(-0.7));
		// operatorJoystick.ButtonLeftBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));

		//Button X
		// operatorJoystick.ButtonX().whenPressed(new ExtendHatchPlunger());
		// operatorJoystick.ButtonX().whileHeld(new IntakeHatchPanel());

		// Button A
		// operatorJoystick.ButtonA().whenPressed(new RetractHatchPlunger());
		
		//Button Y
		// operatorJoystick.ButtonY().whenPressed(new EngageHatchPanel());
		
		//Button B
		// operatorJoystick.ButtonB().whenPressed(new DisengageHatchPanel());

		// operatorJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(40));
		// operatorJoystick.ButtonBack().whenPressed(new DriveMonkeyBarPivotPIDPath(110));
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
	 * Hatch Probe Pivot *
	 *************************************************************************/
	public double getHatchProbePivotJoystickValue()
	{

			return operatorJoystick.getRightStickRaw_Y();
	}

	public double getCargoIntakeJoystickValue()
	{

		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw();
	}

	/*************************************************************************
	 *Monkey Bar Pivot *
	 *************************************************************************/
	public double getMonkeyBarPivotJoystickValue()
	{

			return 0;
	}

	public double getMonkeyBarIntakeJoystickValue()
	{
	
		return -operatorJoystick.getLeftTriggerAxisRaw() + operatorJoystick.getRightTriggerAxisRaw();
	}

	/*************************************************************************
	 *Hatch Floor Motor*
	*************************************************************************/
	public double getHatchFloorIntakeJoystickValue()
	{
		return 0;//operatorJoystick.getLeftStickRaw_X();
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

	public double getDriveWinchJoystickValue()
	{
		return operatorJoystick.getRightStickRaw_X();
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