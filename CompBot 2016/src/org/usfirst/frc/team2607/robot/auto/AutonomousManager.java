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
		modes.add(new BreachLowBar(robot));
		modes.add(new BreachLowBarAndShoot(robot));
		modes.add(new TestAutonMode(robot));
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
		
		long timeoutMilli = 3000;
		long startTime = System.currentTimeMillis();
		
		double kP = 0.05;
		double maxTurn = 0.7;
		double tolerance = 0.5;
		robot.shifter.set(false);
		
		boolean keepLooping = true;
		while(keepLooping) {
			if (System.currentTimeMillis() > (startTime + timeoutMilli)) {
				keepLooping = false;
				break;
			}
			
			double error = navx.getYaw() - degrees;
			System.out.println("TurnAngleError: " + error);
			
			double calcTurn = kP * error;
			if (error <= 0){
				calcTurn = Math.max(-maxTurn, calcTurn - .4);
			} else {
				calcTurn = Math.min(maxTurn, calcTurn + .4);
			}
			
			if (navx.getYaw() > (degrees - tolerance) && navx.getYaw() < (degrees + tolerance)){
				keepLooping = false;
				break;
			} else {
				robot.rDrive.arcadeDrive(0, calcTurn);
				System.out.println("CommandedVoltage: " + calcTurn);
			}
			
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				System.out.println("rotateDegrees interrupted");
				keepLooping = false;
				break;
			}
		}
		
		robot.rDrive.arcadeDrive(0, 0);
		robot.rightMotors.setInverted(true);
		robot.shifter.set(true);
	}
	
	
	/*
	 * BEGIN AUTON MODE DECLARATIONS
	 * 
	 * You must add the mode to the array once you define its class
	 */
	
	public class TestAutonMode extends AutonomousMode {

		TestAutonMode(Robot r) {
			super(r);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void run() {
			robot.navX.zeroYaw();

			rotateDegrees(-130, false);
		}

		@Override
		public String getName() {
			return "TestAutonMode";
		}
		
	}
	

	public class BreachLowBarAndShoot extends AutonomousMode {

		BreachLowBarAndShoot(Robot r) {
			super(r);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void run() {
			
			new BreachLowBar(robot).run();

			robot.arm.shoot();		// go ahead and shoot	
			
		}

		@Override
		public String getName() {
			return "BreachLowAndShoot";
		}
		
	}
	
	public class BreachLowBar extends AutonomousMode {

		BreachLowBar(Robot r) {
			super(r);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void run() {
			
			Path p = getPathFromFile("/home/lvuser/breachLowBarAndShoot.txt");
			
			robot.navX.zeroYaw();
			robot.arm.toggleClaw(true);
			
			if (!robot.arm.isArmEnabled()) {
				robot.arm.executeArmHomingSequence();
				try {Thread.sleep(1500);} catch (Exception e) {}
//				while (!robot.arm.isArmEnabled()) {try { Thread.sleep(20); } catch (Exception e) {}}
			}
			
			RobovikingDriveTrainProfileDriver mp = new RobovikingDriveTrainProfileDriver(robot.leftMotors,robot.rightMotors, p);
			mp.followPathBACKWARDS();
			
			int counter = 0;
			while (!mp.isDone()) {
				try { 
					Thread.sleep(20);
					if (counter > 0 && counter < 5){
						robot.arm.rockAndRoll(-.5);
					} 
					if (counter > 5 && counter < 10) {
						robot.arm.rockAndRoll(0);
					} 
					if (counter > 10) {
						counter = 0;
					} 
					counter++;
				} catch (Exception e) {}
			}
			robot.arm.rockAndRoll(0);
			
//			robot.arm.rotateArmToPosition(-45.69);
			robot.arm.executeCheckAndRotate(-45.69);
			rotateDegrees(-126.6, false);
			
			while (!robot.arm.isArmWaiting()) {try { Thread.sleep(20); } catch (Exception e) {}}
			
			robot.arm.toggleClaw(false);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {}

			//robot.arm.shoot();		// go ahead and shoot	
			
		}

		@Override
		public String getName() {
			return "BreachLowBar";
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
