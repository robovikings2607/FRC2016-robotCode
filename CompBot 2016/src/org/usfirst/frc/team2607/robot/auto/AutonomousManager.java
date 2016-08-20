package org.usfirst.frc.team2607.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.Constants;
import org.usfirst.frc.team2607.robot.Robot;
import org.usfirst.frc.team2607.robot.RobovikingDriveTrainProfileDriver;

import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.trajectory.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		modes.add(new TestAutonMode(robot));
		modes.add(new CrossDefenseAndShoot(robot));
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
	
	public boolean rotateDegrees(double degrees, boolean zeroFirst, long timeoutMS){
		boolean interrupted = false;
		AHRS navx = robot.navX;
		boolean keepZeroing = true;
		while(zeroFirst && keepZeroing) { 
			navx.zeroYaw();
			double n = navx.getYaw();
			if (n > -1.0 && n < 1.0) {
				keepZeroing = false;
			} else {
				try {Thread.sleep(10); } catch (InterruptedException e) { return true; }
			}
			if (Thread.interrupted()) return true;
		}
		
		robot.rightMotors.setInverted(false); //Set to TRUE when done
		
		long timeoutMilli = timeoutMS;
		long startTime = System.currentTimeMillis();
		
		double kP = 0.053;
		double maxTurn = 0.7;
		double tolerance = 0.5; //0.5; 
		robot.shifter.set(false);
		
		boolean keepLooping = true;
		while(keepLooping) {
			if (Thread.interrupted()) {
				keepLooping = false;
				interrupted = true;
				break;
			}
			
			if (System.currentTimeMillis() > (startTime + timeoutMilli)) {
				keepLooping = false;
				break;
			}
			
			double error = navx.getYaw() - degrees;
			System.out.println("TurnAngleError: " + error);
			
			double calcTurn = kP * error;
			if (error <= 0){
				calcTurn = Math.max(-maxTurn, calcTurn - .36); // .4
			} else {
				calcTurn = Math.min(maxTurn, calcTurn + .36); // .4
			}
			
			if (navx.getYaw() > (degrees - tolerance) && navx.getYaw() < (degrees + tolerance)){
				keepLooping = false;
				robot.rDrive.arcadeDrive(0, 0);
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
				interrupted = true;
				break;
			}
		}
		
		robot.rDrive.arcadeDrive(0, 0);
		robot.rightMotors.setInverted(true);
		robot.shifter.set(true);
		return interrupted;
	}
	
	public class CrossDefenseAndShoot extends AutonomousMode {
		
		CrossDefenseAndShoot(Robot r) {
			super(r);
		}

		@Override
		public void run() {
			//close claw
			robot.arm.toggleClaw(true);

			//set the arm to the zero position
			if (!robot.arm.isArmEnabled()) {
				robot.arm.executeArmHomingSequence();
				try {Thread.sleep(1500);} catch (Exception e) {}
			}
			
			robot.leftMotors.setInverted(false);
			robot.rightMotors.setInverted(false);
			robot.shifter.set(false);	// shift to low gear

			//drive at half speed for 1 sec
			robot.rDrive.arcadeDrive(.5, 0);
			try { Thread.sleep(1000); } catch (Exception e) { robot.rDrive.arcadeDrive(0,0); return; }
			
			//wait until arm is at zero position
			robot.rDrive.arcadeDrive(0,0);
			while (!robot.arm.isArmEnabled()) {try { Thread.sleep(20); } catch (Exception e) {}}
			
			// take arm to crossing position
			robot.arm.executeCheckAndRotate(-43.25);
			while (!robot.arm.isArmWaiting()) {try { Thread.sleep(20); } catch (Exception e) {}}
			
			//drive full speed for 3 sec
			robot.rDrive.arcadeDrive(0.9, 0.0);
			try { Thread.sleep(3000); } catch (Exception e) { robot.rDrive.arcadeDrive(0,0); return; }
			robot.rDrive.arcadeDrive(0, 0);
			
		}

		@Override
		public String getName() {
			// TODO Auto-generated method stub
			return "CrossDefenseAndShoot";
		}

	
	
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
			robot.arm.toggleClaw(true);
			
			if (!robot.arm.isArmEnabled()) {
				robot.arm.executeArmHomingSequence();
				try {Thread.sleep(1500);} catch (Exception e) {}
//				while (!robot.arm.isArmEnabled()) {try { Thread.sleep(20); } catch (Exception e) {}}
			}
/*			
			robot.arm.executeCheckAndRotate(-45.69);
			rotateDegrees(-126.6, false, 3000);
*/			
			while (!robot.arm.isArmEnabled()) {try { Thread.sleep(20); } catch (Exception e) {}}
			try {Thread.sleep(200);} catch(InterruptedException e) { return; }
			double targetAngleInFOV = SmartDashboard.getNumber("targetAngleInFOV", 999);
			double degFromVision = SmartDashboard.getNumber("degToRotate", 999);
			while ((targetAngleInFOV == 999 || targetAngleInFOV == -999) || 
					(degFromVision == 999 || degFromVision == -999)){
				try {Thread.sleep(10); } catch(Exception e) {return;}
				targetAngleInFOV = SmartDashboard.getNumber("targetAngleInFOV", 999);
				degFromVision = SmartDashboard.getNumber("degToRotate", 999);
				if (Thread.interrupted()) {
					return;
				}
			}
			
			System.out.println("Angle used for Auton shot: " + targetAngleInFOV);
			System.out.println("degFromVision: " + degFromVision);
			
			double armPosition = Constants.getArmAngle(targetAngleInFOV);

			if (armPosition < 0 && armPosition > -67.0) {			
				robot.arm.executeCheckAndRotate(armPosition);
			}
			
			if (degFromVision != -999 && degFromVision != 999) {
				if (rotateDegrees(degFromVision, true, 3500)) {
					return;
				}
			}
			
			while (!robot.arm.isArmWaiting()) {try { Thread.sleep(10); } catch (InterruptedException e) { return; }}
			
			if (Thread.interrupted()) return;
			
			robot.arm.toggleClaw(false);
			
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) { return;}

					
			robot.arm.shoot();		// go ahead and shoot
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
			if (rotateDegrees(-126.6, false, 3000)) {
				// rotateDegrees returns true if thread interrupted
				return;
			}
			
			//while (!robot.arm.isArmWaiting()) {try { Thread.sleep(20); } catch (Exception e) {}}
			try {Thread.sleep(200);} catch(InterruptedException e) { return; }
			double targetAngleInFOV = SmartDashboard.getNumber("targetAngleInFOV", 999);
			double degFromVision = SmartDashboard.getNumber("degToRotate", 999);
			while ((targetAngleInFOV == 999 || targetAngleInFOV == -999) || 
					(degFromVision == 999 || degFromVision == -999)){
				try {Thread.sleep(10); } catch(Exception e) {return;}
				targetAngleInFOV = SmartDashboard.getNumber("targetAngleInFOV", 999);
				degFromVision = SmartDashboard.getNumber("degToRotate", 999);
				if (Thread.interrupted()) {
					return;
				}
			}
			
			System.out.println("Angle used for Auton shot: " + targetAngleInFOV);
			System.out.println("degFromVision: " + degFromVision);
			
			double armPosition = Constants.getArmAngle(targetAngleInFOV);

			if (armPosition < 0 && armPosition > -67.0) {			
				robot.arm.executeCheckAndRotate(armPosition);
			}
			
			if (degFromVision != -999 && degFromVision != 999) {
				if (rotateDegrees(degFromVision, true, 3500)) {
					return;
				}
			}
			
			while (!robot.arm.isArmWaiting()) {try { Thread.sleep(10); } catch (InterruptedException e) { return; }}
			
			if (Thread.interrupted()) return;
			
			robot.arm.toggleClaw(false);
			
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) { return;}

			//robot.arm.shoot();		// go ahead and shoot	
			
		}

		@Override
		public String getName() {
			return "BreachLowBarAndShoot";
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
