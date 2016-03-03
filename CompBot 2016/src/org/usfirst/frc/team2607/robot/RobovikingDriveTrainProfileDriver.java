package org.usfirst.frc.team2607.robot;

import java.util.ArrayList;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.Trajectory.Segment;

import edu.wpi.first.wpilibj.Notifier;

public class RobovikingDriveTrainProfileDriver {
	
	private Transmission leftMotors, rightMotors;
	//private PIDController positionPID;
	private double dtSeconds;
	//private Path path;
	private ArrayList<Segment> leftVelPts, rightVelPts;
	private int numPoints;
	private Trajectory lt, rt;
	private boolean running = false, done = false;
	private long step;
	private boolean runBACKWARDS = false;
	
	private class PeriodicRunnable implements java.lang.Runnable {
		private long startTime;
		private boolean firstTime;
		
		public PeriodicRunnable() {
			firstTime = true;
		}

		private Segment invertSegment(Segment s) {
			return new Segment(-s.pos, -s.vel, -s.acc, -s.jerk, s.heading, s.dt, s.x, s.y);
		}
		
		public void run() {
	    	if (firstTime) {
	    		firstTime = false;
	    		startTime = System.currentTimeMillis();
	    		running = true;
	    		done = false;
	    		leftMotors.enableVelPID();
	    		rightMotors.enableVelPID();
	    		leftMotors.resetEncoder();
	    		rightMotors.resetEncoder();
	    	}
	    	step = (System.currentTimeMillis() - startTime) / (long)(dtSeconds * 1000);
	    	try {
	    		if (runBACKWARDS){
	    			leftMotors.setSP(invertSegment(leftVelPts.get((int)step)));
		    		rightMotors.setSP(invertSegment(rightVelPts.get((int)step)));	
	    		} else {
		    		leftMotors.setSP(leftVelPts.get((int)step));
		    		rightMotors.setSP(rightVelPts.get((int)step));	
	    		}
	    	} catch (Exception e) {
	    		pointExecutor.stop();
	    		running = false;
	    		done = true;
	    		leftMotors.disableVelPID();
	    		rightMotors.disableVelPID();
	    		if (runBACKWARDS) runBACKWARDS = false;
	    	}
	    }
	}

	Notifier pointExecutor = new Notifier(new PeriodicRunnable());

	public RobovikingDriveTrainProfileDriver(Transmission leftMotors, Transmission rightMotors, Path path) {
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;
		//this.path = path;
		this.leftVelPts = new ArrayList<Segment>();
		this.rightVelPts = new ArrayList<Segment>();
		//store the velocity pts
		numPoints = path.getLeftWheelTrajectory().getNumSegments();
		lt = path.getLeftWheelTrajectory();
		rt = path.getRightWheelTrajectory();
		for (int i = 0; i < numPoints; i++) {
			leftVelPts.add(lt.getSegment(i));
			rightVelPts.add(rt.getSegment(i));
			if (i==0) dtSeconds = lt.getSegment(i).dt;
		}
	}

	public boolean isRunning() {
		return running;
	}
	
	public boolean isDone() {
		return done;
	}
	
	public void followPathBACKWARDS() {
		runBACKWARDS = true;
		pointExecutor.startPeriodic(dtSeconds / 2.0);
	}
	
	public void followPath() {
		runBACKWARDS = false;
		pointExecutor.startPeriodic(dtSeconds / 2.0);
	}
	
}
