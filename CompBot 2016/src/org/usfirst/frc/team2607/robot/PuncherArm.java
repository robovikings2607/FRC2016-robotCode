package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DriverStation;

public class PuncherArm {
	
	private CANTalon punchWinder , armRotator , rollerz;
	private SRXProfileDriver armProfile;
	private Solenoid punchLock , santaClaw;
	private double armRotatorEncPos , armLastPosition;
	private final double armRotatorMaxSpeed = 18.0;  // cim rotations per second, for motion profiles

	// TODO:  add power logging thread
	
	private class PowerLogger extends Thread {

		private PrintWriter log;
		private DriverStation ds;
		@Override
		public void run() {
			int tick = 0;
			while (true && log != null) {
				if (ds.isEnabled()) tick = 0;
				if (++tick < 2) {
					log.println(System.currentTimeMillis() + "," +
						ds.isEnabled() + "," + 
						armRotator.getPosition() + "," +
						armRotator.getSpeed() + "," +
						armRotator.getBusVoltage() + "," + 
						armRotator.getOutputVoltage() + "," +
						armRotator.getOutputCurrent());
					log.flush();
				}
				try {Thread.sleep(10);} catch (Exception e) {}
			}
		}

		public PowerLogger() {
			ds = DriverStation.getInstance();
			try {
				String s = "/home/lvuser/PuncherArm." + System.currentTimeMillis() + ".csv";
				log = new PrintWriter(new File(s));
				log.println("Time,Enabled?,Pos,Vel,VIn,VOut,AmpOut");
			} catch (Exception e) { System.out.println("Can't start logging"); log = null;}
		}
	}
	
	public PuncherArm(){
		punchWinder = new CANTalon(Constants.puncherMotor);
		armRotator = new CANTalon(Constants.armMotor);
		rollerz = new CANTalon(Constants.rollersMotor);
		
		armProfile = new SRXProfileDriver(armRotator);
		new PowerLogger().start();
	
		punchLock = new Solenoid(1,Constants.puncherLock);
		santaClaw = new Solenoid(1,Constants.clawOpener);
		
		armRotator.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		armRotator.changeControlMode(TalonControlMode.MotionProfile);
    	armRotator.reverseSensor(true);
    	armRotator.setProfile(0);
    	armRotator.setPosition(0);
		armRotator.setForwardSoftLimit(0);
		armRotator.enableForwardSoftLimit(true);
		armRotator.setReverseSoftLimit(-82.6);
		//armRotator.setReverseSoftLimit(-50.0);
		armRotator.enableReverseSoftLimit(true);
    	armRotator.setF(0.003);
    	armRotator.setP(.03);
    	armRotator.setI(0.0001);
    	armRotator.setD(0);
    	armRotator.enableBrakeMode(true);
	}
	
	public void lock() {
		punchLock.set(true);
	}
	
	public void shoot() {
		punchLock.set(false);
	}
	
	//Basic method for setting the puncher winder motor to spin
	public void windPuncher(double jubbs) {
		punchWinder.set(jubbs);
	}
	
	//Basic method for setting the arm rotator motor to spin
	public void raiseArm() {

	}
	
	public void lowerArm() {
		
	}
	
	public void stopArm() {
		
	}
	
	// positive degToRotate lowers the arm
	// negative degToRotate raises the arm
	public void rotateArmXDegrees(double degToRotate) {
		double direction = (degToRotate) / Math.abs(degToRotate);
		double rotations = ((350.0 * degToRotate) / 360.0);
		double maxSpeed = direction * armRotatorMaxSpeed;
		armProfile.setMotionProfile(new SRXProfile(maxSpeed, armRotator.getPosition(), rotations, 250, 250, 10));
		armProfile.startMotionProfile();
	}
	
	public void rockAndRoll(double jubbs) {
		rollerz.set(jubbs);
	}

	public void toggleClaw(boolean jubbs) {
		santaClaw.set(jubbs);
	}
	
	public void process() {
		if (armRotator.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) 
										!= CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
			resetArm();
		} 
		armProfile.control();
		armRotator.set(armProfile.getSetValue().value);
	}
	
	public void resetArm() {
//	    armRotator.changeControlMode(TalonControlMode.PercentVbus);
//	    armRotator.setPosition(0);
	    armProfile.reset();
	    armRotator.set(armProfile.getSetValue().value);
	}
}
