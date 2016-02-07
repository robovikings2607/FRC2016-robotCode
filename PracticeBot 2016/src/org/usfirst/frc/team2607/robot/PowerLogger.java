package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerLogger extends Thread {


	private PowerDistributionPanel pdp;
	private PrintWriter logFile;
	private boolean running;
	private Robot theBot;
	
	public PowerLogger(Robot r) {
		pdp = new PowerDistributionPanel();
		theBot = r;
		running = false;
	}
	
	public void startLogging() {
		if (running) return;
		String fn = "";
		try {
			fn = "/home/lvuser/PowerLogger." + System.currentTimeMillis() + ".csv";
			logFile = new PrintWriter(new File(fn));
		} catch (Exception e) {
			System.out.println("ERROR:  Couldn't open PowerLogger file: ");
			System.out.println("\t" + fn);
			return;
		}
		logFile.println("Time,Y Stick,X Stick,PDP In,L Cmd,C1 Amps,C2 Amps,C3 Amps," +
						"R Cmd,C6 Amps,C4 Amps,C5 Amps,Total Drawn");
		running = true;
		start();
	}
	
	public void stopLogging() {
		if (!running) return;
		running = false;
	}
	
	private void logEntry() {
		StringBuffer sb = new StringBuffer();
		sb.append(System.currentTimeMillis()).append(",");		// timestamp
		sb.append(theBot.getDriverYIn()).append(",").append(theBot.getDriverXIn()).append(","); // joystick cmd
		sb.append(pdp.getVoltage()).append(",");  // PDP VIn
		sb.append(theBot.getLeftCmd()).append(","); // left motor cmd
		sb.append(pdp.getCurrent(1)).append(",").append(pdp.getCurrent(2)).append(",").append(pdp.getCurrent(3)).append(",");
		sb.append(theBot.getRightCmd()).append(",");  // right motor cmd
		sb.append(pdp.getCurrent(6)).append(",").append(pdp.getCurrent(4)).append(",").append(pdp.getCurrent(5)).append(",");
		sb.append(pdp.getTotalPower());
		logFile.println(sb);
		logFile.flush();
	}
	
	@Override
	public void run() {
		try {
			while (running) {
				logEntry();
				Thread.sleep(20);
			}
		} catch (Exception e) {
			running = false;
		}
		logFile.close();
		logFile = null;
	}

}
