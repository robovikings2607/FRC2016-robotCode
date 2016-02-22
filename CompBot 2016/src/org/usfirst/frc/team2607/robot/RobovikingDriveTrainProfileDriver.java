package org.usfirst.frc.team2607.robot;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.SRXProfileDriver.PeriodicRunnable;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotDrive;

public class RobovikingDriveTrainProfileDriver {
	
	private Transmission leftMotors, rightMotors;
	//private PIDController positionPID;
	private double driveTrainWidthInches;
	private int dtMS;
	private Path path;
	private ArrayList<Double> leftVelPts, rightVelPts;
	private int numPoints;
	
	private class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {      }
	}

	Notifier pointExecutor = new Notifier(new PeriodicRunnable());

	public RobovikingDriveTrainProfileDriver(Transmission leftMotors, Transmission rightMotors, 
										int dtMS, double driveTrainWidthInches, Path path) {
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;
		this.driveTrainWidthInches = driveTrainWidthInches;
		this.dtMS = dtMS;
		this.path = path;
		
		//store the velocity pts
		numPoints = path.getLeftWheelTrajectory().getNumSegments();
		Trajectory lt = path.getLeftWheelTrajectory(),
				   rt = path.getRightWheelTrajectory();
		
		for (int i = 0; i < numPoints; i++) {
			leftVelPts.add(lt.getSegment(i).vel);
			rightVelPts.add(rt.getSegment(i).vel);
		}
	}

	public void followPath() {
		
	}
	
}
