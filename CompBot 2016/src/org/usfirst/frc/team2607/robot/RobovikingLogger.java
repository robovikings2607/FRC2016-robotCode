package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;

public class RobovikingLogger extends Thread {

	private ArrayList<Loggable> itemsToLog;
	
	public RobovikingLogger() {
		itemsToLog = new ArrayList<>();
	}
	
	public void addItem(Loggable i) {
		itemsToLog.add(i);
	}
	
	@Override
	public void run() {
		PrintWriter log = null;
		try {
			log = new PrintWriter(new File("/home/lvuser/RobovikingLogger-" + System.currentTimeMillis() + ".csv"));
		} catch (Exception e) {}
		if (log == null) return;
		int numItems = itemsToLog.size() - 1;

		// write header
		log.print("Time,");
		for (int i = 0; i < numItems - 1; i++) {
			log.print(itemsToLog.get(i).logHeader());
			log.print(',');
		}
		log.println(itemsToLog.get(numItems).logHeader());
		log.flush();
		
		// write log entries
		while (true) {
			log.print(System.currentTimeMillis());
			log.print(',');
			for (int i = 0; i < numItems - 1; i++) {
				log.print(itemsToLog.get(i).logEntry());
				log.print(',');
			}
			log.println(itemsToLog.get(numItems).logEntry());
			log.flush();
			try { Thread.sleep(200); } catch (Exception e) {}
		}
	}
	 
}
