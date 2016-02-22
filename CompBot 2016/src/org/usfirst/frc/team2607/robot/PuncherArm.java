package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class PuncherArm {
	
	private CANTalon punchWinder , armRotator , rollerz;
	private SRXProfileDriver armProfile;
	private Solenoid punchLock , santaClaw;
	private DigitalInput armLimiter , shooterCocked;
	private double armDegreesFromStart;
	private final double armRotatorMaxSpeed = 18.0;  // cim rotations per second, for motion profiles
	private AutoWinder winderThread;
	private boolean shooterEnabled;
	
	private class AutoWinder extends Thread {

		private int step = 0;
		
		@Override
		public void run() {
			int sleepTime = 0;
			while (true) {
				try {
					switch (step) {
						case 0: 
							punchWinder.setPosition(0);
							sleepTime = 20; 
							break;
						case 1:			// starting from the cocked position...
							shoot();	// release lock
							sleepTime = 250;
							step += 1;
							break;
						case 2:		
							punchWinder.set(.6); // move forward to grab
							if (punchWinder.getPosition() >= 100) step +=1;
							sleepTime = 10;
							break;
						case 3: 
							punchWinder.set(0); // stop moving
							punchWinder.setPosition(0);
							lock();				// close the lock 
							sleepTime = 250;	// wait .25 secs
							step += 1;
							break;
						case 4: 
							punchWinder.set(-1.0);	// draw back
							sleepTime = 10;
							if (punchWinder.getPosition() <= -100) step += 1;
							break;
						case 5:
							punchWinder.set(0);		// stop, sequence complete
							sleepTime = 20;
							step = 0;
							break;
						// auto shoot & re-cock sequence stops here, below is zero'ing sequence
						case 10:					// move to home position
							shoot();
							sleepTime = 10;
							punchWinder.setPosition(0);
							punchWinder.set(-.3);
							System.out.println("going home, pos: " + punchWinder.getPosition());
							if (!shooterCocked.get() || punchWinder.getPosition() <= -100) step += 1;
							break;
						case 11:
							punchWinder.set(0);
							sleepTime = 20;
							shooterEnabled = true;
							break;
						default: sleepTime = 20; break;
					}
					Thread.sleep(sleepTime);
				} catch (Exception e) { }
			}
		}
		
		public void startFromUncockedPosition() {
			if (step == 0 && shooterEnabled) step = 3;
		}
		
		public void startFromCockedPosition() {
			if (step == 0 && shooterEnabled) step = 1;
		}
		
		public void goToHomePosition() {
			if (step == 0 && !shooterEnabled) step = 10;
		}
		
		public void stopSequence() {
			punchWinder.set(0);
			step = 0;
		}
	}
	
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
		
    	// check if the shooter is at home (cocked) position, if not disable it until the zero'ing process is run
    	shooterEnabled = !shooterCocked.get();
		winderThread = new AutoWinder();
		winderThread.start();
	
		punchLock = new Solenoid(1,Constants.puncherLock);
		santaClaw = new Solenoid(1,Constants.clawOpener);
		
		armLimiter = new DigitalInput(Constants.armLimiter);
		shooterCocked = new DigitalInput(Constants.shooterCocked);
		
		punchWinder.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		punchWinder.enableBrakeMode(true);
		punchWinder.reverseSensor(true);
		
		armRotator.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		armRotator.changeControlMode(TalonControlMode.MotionProfile);
    	armRotator.reverseSensor(true);
    	armRotator.setProfile(0);
    	armRotator.setPosition(0);
		armRotator.setForwardSoftLimit(0);
		armRotator.enableForwardSoftLimit(true);
		armRotator.setReverseSoftLimit(-82.6);
		//armRotator.setReverseSoftLimit(-15);
		armRotator.enableReverseSoftLimit(true);
    	armRotator.setF(0.003);
    	armRotator.setP(.03);
    	armRotator.setI(0.0001);
    	armRotator.setD(0);
    	armRotator.enableBrakeMode(true);
    	
    	armDegreesFromStart = 0.0;			// saves the manual movements of the arm
	}
	
	public void lock() {
		punchLock.set(false);
	}
	
	public void shoot() {
		punchLock.set(true);
	}
	
	//Basic method for setting the puncher winder motor to spin
	public void windPuncher(double jubbs) {
		punchWinder.set(jubbs);
	}

	public void stopWindingSequence() {
		if (winderThread != null) winderThread.stopSequence();
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
		armDegreesFromStart += degToRotate;
		System.out.println("Arm command: " + degToRotate + ", total from start pos: " + armDegreesFromStart);
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
	
	public boolean isShooterEnabled() { //for shooterEnabled field
		return shooterEnabled;
	}
	
	public boolean isShooterCocked() { //for photo-eye (false when it sees pickup)
		return shooterCocked.get();
	}
	
	public void runWinderZeroer(){
		winderThread.goToHomePosition();
	}
	
	public void startFromUncockedPosition() {
		winderThread.startFromUncockedPosition();
	}
	
	public void startFromCockedPosition(){
		winderThread.startFromCockedPosition();
	}
	
	public void stopSequence() {
		winderThread.stopSequence();
	}
}
