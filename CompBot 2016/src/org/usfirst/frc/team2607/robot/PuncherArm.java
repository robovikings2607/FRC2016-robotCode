package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;

public class PuncherArm {
	
	private CANTalon punchWinder , armRotator , rollerz;
	private Solenoid punchLock , santaClaw;
	
	public PuncherArm(){
		punchWinder = new CANTalon(Constants.puncherMotor);
		armRotator = new CANTalon(Constants.armMotor);
		rollerz = new CANTalon(Constants.rollersMotor);
		
		punchLock = new Solenoid(Constants.puncherLock);
		santaClaw = new Solenoid(Constants.clawOpener);
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
	public void rotateArm(double jubbs) {
		armRotator.set(jubbs);
	}
	
	public void rockAndRoll(double jubbs) {
		rollerz.set(jubbs);
	}

	public void openSesame() {
		santaClaw.set(true);
	}
	
	public void closeSesame() {
		santaClaw.set(false);
	}
}
