package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.DriverStation;

public class PIDLogger extends Thread {
	private PrintWriter logFile = null;
	private PrintWriter sameNameLogFile = null;
	private boolean loggingEnabled = false;
	String deviceName;
	
	Transmission theTrans;
	
	
	@Override
	public void run(){
		while (true){
			logEntry();
			try {Thread.sleep(10); } catch (Exception e) {}
		}
		
	}
	
	public PIDLogger(Transmission t){
		theTrans = t;					
	}
	
	 public void enableLogging(boolean enable) {
	    	if (enable && !loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			sameNameLogFile.close();
	    			logFile = null;
	    			sameNameLogFile = null;
	    		}
	    		try {
	    			String s = "/home/lvuser/" + "TuneFiles" + "." + System.currentTimeMillis() + ".csv";
	    			logFile = new PrintWriter(new File(s));
	    			String t = "/home/lvuser/" + "TuneFile" + "_" + theTrans.getName() + ".csv";
	    			sameNameLogFile = new PrintWriter(new File(t));
	    			sameNameLogFile.println("Time,SetPos,RealPos,SetVel,RealVel,SetHead,RealHead,Error,PCon,ICon,DCon,VCon,ACon");
	    			logFile.println("Time,SetPos,RealPos,SetVel,RealVel,SetHead,RealHead,Error,PCon,ICon,DCon,VCon,ACon");
	    		} catch (Exception e) {}
	    	} 
	    	
	    	if (!enable && loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			sameNameLogFile.close();
	    			logFile = null;
	    			sameNameLogFile = null;
	    		}
	    	}
	    	
	    	loggingEnabled = enable;
	    }
	 
	 public void logEntry() {
	        if (loggingEnabled) {
	        	logFile.println(System.currentTimeMillis() + "," +
	        					theTrans.pidLoop.getSetpoint()[0] + "," + 
	        					theTrans.enc.getDistance() + "," +
	        				    theTrans.pidLoop.getSetpoint()[1] + "," + 
	        					theTrans.enc.getRate() + "," + 
	        				    theTrans.pidLoop.getSetpoint()[3] + "," +
	        				    //(theTrans.gyro.getAngle() * (Math.PI/180.0)) + "," +
	        				    "0," +
	        				    theTrans.pidLoop.getError() + "," +
	        				    (theTrans.pidLoop.getError() * theTrans.pidLoop.getP()) + "," +
	        				    (theTrans.pidLoop.getError() * theTrans.pidLoop.getI()) + "," +
	        				    (theTrans.pidLoop.getError() * theTrans.pidLoop.getD()) + "," +
	        				    (theTrans.pidLoop.getSetpoint()[1] * theTrans.pidLoop.getV()) + "," +
	        				    (theTrans.pidLoop.getSetpoint()[2] * theTrans.pidLoop.getA()));
	        	logFile.flush();
	        	sameNameLogFile.println(System.currentTimeMillis() + "," +
    					theTrans.pidLoop.getSetpoint()[0] + "," + 
    					theTrans.enc.getDistance() + "," +
    				    theTrans.pidLoop.getSetpoint()[1] + "," + 
    					theTrans.enc.getRate() + "," + 
    				    theTrans.pidLoop.getSetpoint()[3] + "," +
    				    //(theTrans.gyro.getAngle() * (Math.PI/180.0)) + "," +
    				    "0," +
    				    theTrans.pidLoop.getError() + "," +
    				    (theTrans.pidLoop.getError() * theTrans.pidLoop.getP()) + "," +
    				    (theTrans.pidLoop.getError() * theTrans.pidLoop.getI()) + "," +
    				    (theTrans.pidLoop.getError() * theTrans.pidLoop.getD()) + "," +
    				    (theTrans.pidLoop.getSetpoint()[1] * theTrans.pidLoop.getV()) + "," +
    				    (theTrans.pidLoop.getSetpoint()[2] * theTrans.pidLoop.getA()));
	        	sameNameLogFile.flush();
	        }
	    }

}
