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
		logFile.println("Time,Y Stick,X Stick,PDP In,L Cmd,L1 Amps,L2 Amps,L3 Amps," +
						"R Cmd,R1 Amps,R2 Amps,R3 Amps,Total Drawn");
		running = true;
		start();
	}
	
	public void stopLogging() {
		if (!running) return;
		running = false;
	}
	
	private void logEntry() {
		StringBuffer sb = new StringBuffer();
		sb.append(System.currentTimeMillis());		// timestamp
		sb.append(theBot.getDriverYIn()).append(theBot.getDriverXIn()); // joystick cmd
		sb.append(pdp.getVoltage());  // PDP VIn
		
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
