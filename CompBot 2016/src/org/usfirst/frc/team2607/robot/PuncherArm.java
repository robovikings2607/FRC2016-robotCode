package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;

public class PuncherArm {
	
	private CANTalon punchWinder , armRotator ;
	private Solenoid punchLock ;
	
	public PuncherArm(){
		punchWinder = new CANTalon(Constants.puncherMotor);
		armRotator = new CANTalon(Constants.armMotor);
		
		punchLock = new Solenoid(Constants.puncherLock);
		
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

}
