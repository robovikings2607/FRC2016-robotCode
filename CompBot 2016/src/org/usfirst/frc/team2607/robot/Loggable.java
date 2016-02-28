package org.usfirst.frc.team2607.robot;

public interface Loggable {
	
	// interface to implement for classes that get logged via RobovikingLogger
	
	// method to return comma-separated headings for the data the class provides for a log entry
	// (i.e. the header row in the log file)
	public String logHeader();
	
	// method to return comma-separated values for the data (i.e. a data row in the log file)
	public String logEntry();
}
