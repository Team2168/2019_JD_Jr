package org.team2168.utils;

import java.util.TimerTask;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;


import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerDistribution {
	private java.util.Timer executor;
	private long period = 100; //ms

	private PowerDistributionPanel pdp;

	private volatile NPointAverager[] channelCurrent;
	private volatile double[] channelPower;
	private volatile int[] channelError;

	private volatile double batteryVoltage;
	private volatile double mainBreakerTemp;

	private volatile double totalCurrent;
	private volatile double totalEnergy;
	private volatile double totalPower;
	private volatile double temperature;

	public static final int NUM_OF_PDP_CHANNELS = 16;

	int currentTimeThreshold;

	private static PowerDistribution instance = null;

	public static PowerDistribution getInstance() {
		if (instance==null) {
			instance = new PowerDistribution();
		}

		return instance;
	}
	private PowerDistribution() {
		pdp = new PowerDistributionPanel(RobotMap.PDP_CAN_ID);

		// Calculate number of loops needed to meet time iteration
		currentTimeThreshold = (int) (RobotMap.CURRENT_LIMIT_TIME_THRESHOLD_SECONDS / (period * 1000));

		channelCurrent = new NPointAverager[NUM_OF_PDP_CHANNELS];

		// initialize NPoint Averager for each channel
		for (int i = 0; i < NUM_OF_PDP_CHANNELS; i++)
			channelCurrent[i] = new NPointAverager(currentTimeThreshold);

		channelPower = new double[NUM_OF_PDP_CHANNELS];
		channelError = new int[NUM_OF_PDP_CHANNELS];

		ConsolePrinter.putNumber("Battery Voltage", () -> {return getBatteryVoltage();}, true, false);
		ConsolePrinter.putNumber("totalCurrent", () -> {return getTotalCurrent();}, true, false);
		ConsolePrinter.putNumber("pcmCompressorCurrent", () -> {return getChannelCurrent(RobotMap.COMPRESSOR_PDP);
		}, true, false);

	}

	public void startThread() {
		this.executor = new java.util.Timer();
		this.executor.schedule(new PowerDistributionTask(this), 0L, this.period);

	}

	public int getChannelError(int channel) {
		if ((channel >= channelPower.length) || (channel < 0))
			return 0;

		return channelError[channel];
	}

	public double getChannelCurrent(int channel) {
		if ((channel >= channelPower.length) || (channel < 0))
			return 0;

		return channelCurrent[channel].getLatestValue();
	}

	public double getBatteryVoltage() {
		return batteryVoltage;
	}

	public double getChannelPower(int channel) {
		if ((channel >= channelPower.length) || (channel < 0))
			return 0;

		return channelPower[channel];
	}

	private void run() {
		batteryVoltage = pdp.getVoltage();

		for (int i = 0; i < NUM_OF_PDP_CHANNELS; i++) {

			channelCurrent[i].putData(pdp.getCurrent(i));
			channelPower[i] = channelCurrent[i].getLatestValue() * batteryVoltage;

			// calculate current average over last period of time and report error
			if (channelCurrent[i].getAverage() > RobotMap.STALL_CURRENT_LIMIT)
				channelError[i] = 2; // danger
			else if (channelCurrent[i].getAverage() > RobotMap.WARNING_CURRENT_LIMIT)
				channelError[i] = 1; // warning
			else
				channelError[i] = 0; // assume no error

		}

		totalCurrent = pdp.getTotalCurrent();
		totalEnergy = pdp.getTotalEnergy();
		temperature = pdp.getTemperature();
		totalPower = pdp.getTotalPower();

	}

	private class PowerDistributionTask extends TimerTask {
		private PowerDistribution console;

		private PowerDistributionTask(PowerDistribution printer) {
			if (printer == null) {
				throw new NullPointerException("PDP was null");
			}
			this.console = printer;
		}

		/**
		 * Called periodically in its own thread
		 */
		public void run() {
			console.run();
		}
	}

	/**
	 * Gets total Current
	 * 
	 * @return Total Current
	 */
	public double getTotalCurrent() {
		return totalCurrent;
	}

	/**
	 * Gets total Energy
	 * 
	 * @return Total Energy
	 */
	public double totalEnergy() {
		return totalEnergy;
	}

	/**
	 * Gets total Power
	 * 
	 * @return Total Power
	 */
	public double totalPower() {
		return totalPower;
	}
}
