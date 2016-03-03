package org.usfirst.frc.team2607.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.Robot;
import org.usfirst.frc.team2607.robot.RobovikingDriveTrainProfileDriver;

import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.trajectory.Path;

/**
 * @author Cerora
 *
 */
public class AutonomousManager {
	
	Robot robot;
	public ArrayList<AutonomousMode> modes = new ArrayList<AutonomousMode>();
	
	AutonomousManager(Robot robot){
		this.robot = robot;
		
		modes.add(new DoNothingFailsafe());
		modes.add(new DoNothing());
		modes.add(new BreachLowBarAndShoot(robot));
	}
	
	public AutonomousMode getModeByName (String name){
		for (AutonomousMode m : modes){
			if (m.getName().equals(name))
				return m;
		}
		
		try {
			throw new Exception();
		} catch (Exception e) {
			System.err.println("Mode not found");
			e.printStackTrace();
			return new DoNothingFailsafe();
		}
	}
	
	public AutonomousMode getModeByIndex (int index){
		try {
			return modes.get(index);
		} catch (IndexOutOfBoundsException e){
			System.err.println("Mode out of array bounds");
			e.printStackTrace();
			return new DoNothingFailsafe();
		}
	}
	
	public void rotateDegrees(double degrees, boolean zeroFirst){
		AHRS navx = robot.navX;
		if(zeroFirst) navx.zeroYaw();
		robot.rightMotors.setInverted(false); //Set to TRUE when done
		
		double kP = 0.001;
		double maxTurn = 0.2;
		double tolerance = 2;
		
		while(true){
			double error = navx.getYaw() - degrees;
			
			double calcTurn = kP * error;
			if (error <= 0){
				calcTurn = Math.max(-maxTurn, calcTurn);
			} else {
				calcTurn = Math.min(maxTurn, calcTurn);
			}
			
			if (navx.getYaw() > (degrees - tolerance) && navx.getYaw() < (degrees + tolerance)){
				break;
			} else {
				robot.rDrive.arcadeDrive(0, calcTurn);
			}
			
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				
			}
		}
		
		robot.rDrive.arcadeDrive(0, 0);
		robot.rightMotors.setInverted(true);
	}
	
	
	/*
	 * BEGIN AUTON MODE DECLARATIONS
	 * 
	 * You must add the mode to the array once you define its class
	 */
	

	public class BreachLowBarAndShoot extends AutonomousMode {

		BreachLowBarAndShoot(Robot r) {
			super(r);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void run() {
			
			Path p = getPathFromFile("/home/lvuser/breachLowBarAndShoot.txt");
			
			robot.navX.zeroYaw();
			
			if (!robot.arm.isArmEnabled()) {
				robot.arm.executeArmHomingSequence();
				while (!robot.arm.isArmEnabled()) {try { Thread.sleep(20); } catch (Exception e) {}}
			}
			
			RobovikingDriveTrainProfileDriver mp = new RobovikingDriveTrainProfileDriver(robot.leftMotors,robot.rightMotors, p);
			mp.followPath();
			
			while (!mp.isDone()) {try { Thread.sleep(20); } catch (Exception e) {}}
			
			rotateDegrees(180, false);
			
		}

		@Override
		public String getName() {
			return "BreachBarLow";
		}
		
	}
	
	public class DoNothing extends AutonomousMode {
		
		DoNothing(){
			
		}

		@Override
		public void run() {
			System.out.println("Explicitly told not to move");
		}

		@Override
		public String getName() {
			return "DoNothing";
		}
		
	}
	
	public class DoNothingFailsafe extends AutonomousMode {
		
		DoNothingFailsafe(){
			
		}

		@Override
		public void run() {
			System.out.println("This shouldn't be running - Mode 0 selected for some reason");
		}

		@Override
		public String getName() {
			return "DoNothingFailsafe";
		}
		
	}
}
