package org.usfirst.frc.team2607.robot;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.CANTalon;

public class SRXProfile {
		
	// sign of the following two must match and reflect desired travel direction
	private double maxSpeed;				// rotations/sec
	private double startPosition;			// the starting position (# rotations)
	private double travelDistance;			// the # of rotations to travel from the starting position
	
	private double dt, accelTime, decelTime;	//in ms
	
	public SRXProfile(double maxSpeed, double startPosition, double travelDistance, double accelTimeMS, double decelTimeMS, double dt) {
		this.maxSpeed = maxSpeed;
		this.startPosition = startPosition;
		this.travelDistance = travelDistance;
		this.accelTime = accelTimeMS;
		this.decelTime = decelTimeMS;
		this.dt = dt;
	}
		
	// this generates a profile using CTRE's method (second order filtering)
	// algorithm is ported from CTRE's spreadsheet
	public void generateAndPushProfile(CANTalon talonSRX) {

		CANTalon.TrajectoryPoint p = new CANTalon.TrajectoryPoint();
		p.timeDurMs = (int)dt;
		p.isLastPoint = false;
		p.profileSlotSelect = 0;
		p.velocityOnly = false;
		p.position = startPosition;
		if (p.position == 0.0) p.zeroPos = true; 
		else p.zeroPos = false;
		p.velocity = 0.0;
		
		// we've got the first point, let's push it
		int step = 1;
		if (talonSRX == null) {
			System.out.println(step + " POS: " + p.position + " VEL: " + p.velocity + " DT: " + p.timeDurMs);
		} else {
			talonSRX.pushMotionProfileTrajectory(p);
		}
		
		boolean input = false;
//		double time4 = (targetDistance / maxSpeed) * 1000;
		int filter1Length = (int)Math.ceil(accelTime / p.timeDurMs);
		int filter2Length = (int)Math.ceil(decelTime / p.timeDurMs);
		double impulseN = ((travelDistance / maxSpeed) * 1000) / p.timeDurMs;
		double filter1Sum = 0.0, filter2Sum = 0.0; 
		double filter1Buf[] = new double[filter2Length]; 	// Java language spec guarantees that array values are initialized to 0.0
		int bufPos = 0;
		filter1Buf[0] = filter1Sum;
		p.zeroPos = false;
		double prevPos, prevVel;

		while (!p.isLastPoint) {
			prevPos = p.position;
			prevVel = p.velocity;
			input = (++step < impulseN + 2);
			filter1Sum = Math.max(0.0, Math.min(1.0, filter1Sum + ((input) ? 1.0/filter1Length : -1.0/filter1Length)));
			// filter2Sum is the sum of up to the last X filter1Sums
			if (++bufPos >= filter2Length ) bufPos = 0;
			filter1Buf[bufPos] = filter1Sum;
			filter2Sum = 0.0;
			for (double d : filter1Buf) filter2Sum += d;
			p.velocity = ((filter1Sum + filter2Sum) / (filter2Length + 1.0)) * maxSpeed;
			p.position = ((((prevVel + p.velocity)/2.0) * dt) / 1000.0) + prevPos;
			
			// when filter1Sum and filter2Sum are both 0, we've reached the last point in the profile
			if (filter1Sum == 0.0 && filter2Sum == 0.0) {
				p.isLastPoint = true;
			}
			
			// push the point
			if (talonSRX == null) {
				System.out.println(step + " POS: " + p.position + " VEL: " + p.velocity + " DT: " + p.timeDurMs);
			} else {
				talonSRX.pushMotionProfileTrajectory(p);
			}
		}
		
		
	}

	
	
	
	
}
