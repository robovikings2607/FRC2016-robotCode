package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class PuncherArm {
	
	private CANTalon punchWinder , armRotator , rollerz;
	private SRXProfileDriver armProfile;
	private Solenoid punchLock , santaClaw;
	private double armRotatorEncPos;
	
	public PuncherArm(){
		punchWinder = new CANTalon(Constants.puncherMotor);
		armRotator = new CANTalon(Constants.armMotor);
		rollerz = new CANTalon(Constants.rollersMotor);
		
		armProfile = new SRXProfileDriver(armRotator);
		
		punchLock = new Solenoid(Constants.puncherLock);
		santaClaw = new Solenoid(Constants.clawOpener);
		
		armRotator.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		armRotator.changeControlMode(TalonControlMode.MotionProfile);
    	armRotator.reverseSensor(true);
    	armRotator.setProfile(0);
		armRotatorEncPos = armRotator.getPosition();
		armRotator.setF(0.001);
    	armRotator.setP(.022);
    	armRotator.setI(0);
    	armRotator.setD(0);
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
	public void rotateArmRaw(double jubbs) {
		armRotator.set(jubbs);
	}
	
	public void rotateArmXDegrees(double degToRotate) {
		armProfile.setMotionProfile(new SRXProfile())
		armProfile.startMotionProfile();
	}
	
	public void rockAndRoll(double jubbs) {
		rollerz.set(jubbs);
	}

	public void toggleClaw(boolean jubbs) {
		santaClaw.set(jubbs);
	}
	
	public void process() {
		armRotator.set(armProfile.getSetValue().value);
		armProfile.control();
	}
	
	public void resetArm() {
//	    armRotator.changeControlMode(TalonControlMode.PercentVbus);
//	    armRotator.setPosition(0);
	    armRotator.set(0);
	    armProfile.reset();
	}
}
