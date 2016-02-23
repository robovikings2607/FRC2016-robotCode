package org.usfirst.frc.team2607.robot;

import java.util.ArrayList;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Notifier;

public class RobovikingDriveTrainProfileDriver {
	
	private Transmission leftMotors, rightMotors;
	//private PIDController positionPID;
	private double dtSeconds;
	private Path path;
	private ArrayList<Double> leftVelPts, rightVelPts;
	private int numPoints;
	private Trajectory lt, rt;
	private boolean running = false, done = false;
	private long step;
	
	private class PeriodicRunnable implements java.lang.Runnable {
		private long startTime;
		private boolean firstTime;
		
		public PeriodicRunnable() {
			firstTime = true;
		}
		
		public void run() {
	    	if (firstTime) {
	    		firstTime = false;
	    		startTime = System.currentTimeMillis();
	    		running = true;
	    		done = false;
	    		leftMotors.enableVelPID();
	    		rightMotors.enableVelPID();
	    	}
	    	step = (System.currentTimeMillis() - startTime) / (long)(dtSeconds * 1000);
	    	try {
	    		double l = leftVelPts.get((int)step), r = rightVelPts.get((int)step);
	    		System.out.println("Step: " + step + " left SP: " + l + " right SP: " + r);
	    		leftMotors.setVelSP(l);
	    		rightMotors.setVelSP(r);	    		
	    	} catch (Exception e) {
	    		pointExecutor.stop();
	    		running = false;
	    		done = true;
	    		leftMotors.disableVelPID();
	    		rightMotors.disableVelPID();
	    	}
	    }
	}

	Notifier pointExecutor = new Notifier(new PeriodicRunnable());

	public RobovikingDriveTrainProfileDriver(Transmission leftMotors, Transmission rightMotors, 
										double dtSeconds, Path path) {
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;
		this.dtSeconds = dtSeconds;
		this.path = path;
		this.leftVelPts = new ArrayList<Double>();
		this.rightVelPts = new ArrayList<Double>();
		
		//store the velocity pts
		numPoints = path.getLeftWheelTrajectory().getNumSegments();
		lt = this.path.getLeftWheelTrajectory();
		rt = this.path.getRightWheelTrajectory();
		
		for (int i = 0; i < numPoints; i++) {
			leftVelPts.add(lt.getSegment(i).vel);
			rightVelPts.add(rt.getSegment(i).vel);
		}
	}

	public boolean isRunning() {
		return running;
	}
	
	public boolean isDone() {
		return done;
	}
	
	public void followPath() {
		pointExecutor.startPeriodic(dtSeconds / 2.0);
	}
	
}
