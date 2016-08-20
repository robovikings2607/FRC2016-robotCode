package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class PuncherArm {
	
	private CANTalon punchWinder , armRotator , rollerz;
	private RobovikingSRXProfileDriver armProfile;
	private Solenoid punchLock , santaClaw, armLocker;
	private DigitalInput armLimiter , shooterCocked;
//	private final double armRotatorMaxSpeed = 18.0;  // cim rotations per second, for motion profiles
	private AutoWinder winderThread;
	private ArmPositioningThread armPosThread;
	private boolean shooterEnabled, armEnabled;
	
	private class AutoWinder extends Thread {

		//private int step = 0;
		private AtomicInteger step = new AtomicInteger(0);
		
		@Override
		public void run() {
			int sleepTime = 0;
			double startPos = 0, curPos = 0;
			double startTime = 0, runTime = 0;
			System.out.println("Starting AutoWinder thread...");
			while (true) {
				try {
					switch (step.get()) {
						case 1:			// starting from the cocked position...
							//punchWinder.setPosition(0);
							startPos = punchWinder.getPosition();
							shoot();	// release lock
							sleepTime = 250;
							step.compareAndSet(1, 2);			//step += 1;
							startTime = System.currentTimeMillis();
							break;
						case 2:		
							punchWinder.set(.6); // move forward to grab
							runTime = System.currentTimeMillis() - startTime;
							curPos = Math.abs(punchWinder.getPosition() - startPos);
							if (curPos >= 100 || runTime >= 7000) {
								punchWinder.set(0);
								step.compareAndSet(2, 3);		//step +=1;
							}
							sleepTime = 10;
							break;
						case 3: 
							punchWinder.set(0); // stop moving
							lock();				// close the lock
							//punchWinder.setPosition(0);
							startPos = punchWinder.getPosition();
							sleepTime = 250;	// wait .25 secs
							step.compareAndSet(3, 4);			//step += 1;
							break;
						case 4: 
							punchWinder.set(-1.0);	// draw back
							sleepTime = 10;
							curPos = Math.abs(punchWinder.getPosition() - startPos);
							if (curPos >= 100 || !shooterCocked.get()) {
								try {Thread.sleep(100);} catch (Exception e) {}
								punchWinder.set(0);
								step.compareAndSet(4, 5);		//step += 1;
							}
							break;
						case 5:
							punchWinder.set(0);		// stop, sequence complete
							sleepTime = 100;
							step.set(0); 
							break;
						// auto shoot & re-cock sequence stops here, below is zero'ing sequence
						case 10:					// move to home position
							shoot();
							sleepTime = 10;
							punchWinder.setPosition(0);
							punchWinder.set(-.3);
							if (!shooterCocked.get() || punchWinder.getPosition() <= -100) {
								punchWinder.set(0);
								step.compareAndSet(10, 11);				//step += 1;
							}
							break;
						case 11:
							punchWinder.set(0);
							sleepTime = 100;
							shooterEnabled = true;
							step.set(0);
							break;
						default: sleepTime = 100; break;
					}
					Thread.sleep(sleepTime);
				} catch (Exception e) { }
			}
		}
		
		public void startFromUncockedPosition() {
			if (shooterEnabled) step.compareAndSet(0,3);
		}
		
		public void startFromCockedPosition() {
			if (shooterEnabled) step.compareAndSet(0, 1);
		}
		
		public void goToHomePosition() {
			if (!shooterEnabled) step.compareAndSet(0, 10);
		}
		
		public void handleRobotDisable() {
			int curStep = step.get();
			if (shooterEnabled && (curStep > 0 && curStep < 3)) {
				System.out.println("Aborting step " + curStep + ", moving to step 3, shooterEnabled: " + shooterEnabled);
				punchWinder.set(0);
				step.set(3); 
			}
		}	
		
		public void abortSequence() {
			System.out.println("Aborting winder sequence from step " + step.get() + ", shooterEnabled: " + shooterEnabled);
			punchWinder.set(0);
			step.set(0);
		}	

	}
		
	private class ArmPositioningThread extends Thread {

		private final double armLockPos = -11.27;
		//private boolean armLocked = false;
		//private int step = 0;
		private AtomicInteger step = new AtomicInteger(0);
		private double targetPosition, pendingTargetPosition;	// pendingTargetPosition is used to temporarily hold new
																// target position when interrupting a running MP
		private double armMaxRPM = 18.0;						// max speed to run arm (default 18.0)
		private boolean lockArmWhenDoneMoving = true;
		
		@Override
		public void run() {
			while (true) {
				int sleepTime = 50;
				switch (step.get()) {
					case 1:					// start of "check and move" sequence
						if (armLocker.get()) step.compareAndSet(1, 2);	//step += 1;    //if arm isn't locked, proceed to rotation step
						else {						 // otherwise, unlock and wait for a bit before proceeding to rotate
							armLocker.set(true); //changed from false
							sleepTime = 250;
							step.compareAndSet(1, 2);		//step +=1;
						}
						break;
					case 2: 
						doRotationProfile();		// move to the target position
						
						sleepTime = 50;
						step.compareAndSet(2, 3);	//step += 1;
						break;
					case 3:
						if (!armProfile.isMPRunning()) {
							if(!armLimiter.get()) armRotator.setPosition(0);
							armLocker.set(!lockArmWhenDoneMoving);		// solenoid false = lock upon completing movement
							step.compareAndSet(3, 0);	//step = 0;
						}
						sleepTime = 20;
						break;
					// "check and move" sequence ends here
					case 10:						// start of locking sequence  
						if (!armLocker.get()) step.compareAndSet(10, 0);		// if arm already locked, goto wait step
						else {
							//doRotationProfile();		// move to the target position (locking pos in this case) // no longer needed
							// interrupt any running MP
							armProfile.interruptMP();
							sleepTime = 100;
							step.compareAndSet(10, 11);	//step += 1;
						}
						break;
					case 11:
						if (!armProfile.isMPRunning()) {
							step.compareAndSet(11, 12);		//step += 1;		// if we're done moving, proceed to lock
						}
						sleepTime = 250;
						break;
					case 12:
						armLocker.set(false); // lock the arm, changed from true
						step.compareAndSet(12, 0);
						break;
					// locking sequence ends here
					case 20:					// start of "interrupt, check and move" sequence; only start this when arm is already in motion
						armProfile.interruptMP();	// interrupt the currently running profile
						sleepTime = 50;
						step.compareAndSet(20, 21);	//step += 1;
						break;
					case 21:					// wait for the armRotator talon to be ready for the next profile
						if (!armProfile.isMPRunning()) {
							targetPosition = pendingTargetPosition;
							step.compareAndSet(21, 1);		// interrupt done, do check and move
						}
						sleepTime = 200;
						break;
						
					default: sleepTime = 50; break;
				}
				try {Thread.sleep(sleepTime);} catch (Exception e) {}
				
			}

		}

		private void doRotationProfile() {
			//		- distance to travel is (-(currentPos - targetPos))
			double distance = -((armRotator.getPosition() - targetPosition));
			//		- distance and maxspeed must have same sign for profile generation to work
			double direction = (distance) / Math.abs(distance);
			double maxSpeed = direction * armMaxRPM;
			if (!armProfile.isMPRunning()) {
				armProfile.pushAndStartMP(new SRXProfile(maxSpeed, armRotator.getPosition(), distance, 250, 250, 10));
			}
		}

		// if arm is waiting, do the "check and move" sequence
		// if arm is rotating, do the "interrupt, check and move" sequence  
		public void checkAndRotateArm(double targetPos, double maxRPM) {
			System.out.println("checkAndRotate...enabled " + armEnabled + " step: " + step);
			if (!armEnabled) return;
			int curStep = step.get(); 
			if (curStep == 0) {
				targetPosition = targetPos;
				armMaxRPM = maxRPM;
				step.set(1); 
				return;
			} 
			if (curStep == 3){
				pendingTargetPosition = targetPos;
				armMaxRPM = maxRPM;
				step.set(20);
			}
		}
		
		public void lockArm() {
			if (!armEnabled) return;
			step.set(10);		// no need to rotate to lock position any more, just execute lock sequence
			/*
			if (armEnabled && step == 0) {
				targetPosition = armLockPos;	
				step = 10;
			}
			*/
		}
		
		public void manualArmLock(boolean lock) {
			// when lock = true, set solenoid to false to actually lock, and vice versa
			if (!armEnabled) return;
			if (step.get() == 0) {
				armLocker.set(!lock);
			}
		}
		
		public int getStep() {
			return step.get();
		}
		
		public void setArmToLockAfterMoving(boolean lock) {
			lockArmWhenDoneMoving = lock;
		}

	}
	
	private class ArmHomingThread extends Thread {
		// this thread will be started on operator command when the arm needs to be homed (armEnabled == false)
		//		it's intended to be run anonymously (e.g. new ArmHomingThread().start()) since it just runs and exits
		@Override
		public void run() {
			if (armEnabled) return;
			System.out.println("Starting ArmHoming Thread...");
			armLocker.set(true);	// unlock the arm
			try {Thread.sleep(200); } catch (Exception e) {}
			armRotator.changeControlMode(TalonControlMode.PercentVbus); 
			armRotator.setPosition(0);
			armRotator.set(.25);
			try { Thread.sleep(250); } catch (Exception e) {}
			while (armLimiter.get() && armRotator.getSpeed() > 0) {
				try { Thread.sleep(10);} catch (Exception e) {}
			} 
			armRotator.set(0);
			if (armLimiter.get()) {
				System.out.println("WARNING!  Arm homing sequence aborted, arm should be moving but encoder isn't moving");
				System.out.println("Leaving arm control disabled");
//				armLocker.set(false);	// lock the arm
				armEnabled = false;
			} else {
				armRotator.setPosition(0);
	    		armRotator.changeControlMode(TalonControlMode.MotionProfile);
//	    		armLocker.set(false);	// lock the arm
	    		armEnabled = true;
			}
		}
	}
	
	public PuncherArm(){
		punchWinder = new CANTalon(Constants.puncherMotor);
		armRotator = new CANTalon(Constants.armMotor);
		rollerz = new CANTalon(Constants.rollersMotor);
		
		armProfile = new RobovikingSRXProfileDriver(armRotator);
		armProfile.start();
		
    	// check if the shooter is at home (cocked) position, if not disable it until the zero'ing process is run
		armLimiter = new DigitalInput(Constants.armLimiter);
		shooterCocked = new DigitalInput(Constants.shooterCocked);

		shooterEnabled = !shooterCocked.get();		// photoeye reads false when made, i.e. when winder is fully retracted
		armEnabled = !armLimiter.get();				// photoeye reads false when made, i.e. when arm is fully lowered
		winderThread = new AutoWinder();
		winderThread.start();

		armPosThread = new ArmPositioningThread();
		armPosThread.start();
		
		punchLock = new Solenoid(1,Constants.puncherLock);
		santaClaw = new Solenoid(1,Constants.clawOpener);
		armLocker = new Solenoid(1,Constants.armLocker);
		
		punchWinder.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		punchWinder.enableBrakeMode(true);
		punchWinder.reverseSensor(true);
		
		armRotator.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	armRotator.reverseSensor(true);
		armRotator.setReverseSoftLimit(-82.6);
		//armRotator.setReverseSoftLimit(-15);
		armRotator.enableReverseSoftLimit(true);
		armRotator.enableForwardSoftLimit(false);
    	armRotator.setProfile(0);
    	armRotator.setF(.012);//0.003);
    	armRotator.setP(.05);//.03);
    	armRotator.setI(0.0);//0.0001);
    	armRotator.setD(0);
    	armRotator.enableBrakeMode(true);
    	
    	if (armEnabled) {
    		armRotator.changeControlMode(TalonControlMode.MotionProfile);
    		armRotator.setPosition(0);
    	}
    	System.out.println("PuncherArm ctor done;  armEnabled: " + armEnabled + " shooterEnabled: " + shooterEnabled);
	}
	
	public void lock() {
		punchLock.set(false);				// engage the shooter lock
	}
	
	public void shoot() {
		punchLock.set(true);				// release the shooter lock
	}
	
	//Basic method for setting the puncher winder motor to spin, to allow for manual maintenance in 
	//test mode
	public void winderManualRun(double jubbs) {
		if (jubbs != 0.0) shooterEnabled = false;
		punchWinder.set(jubbs);
	}

	public void stopWindingSequence() {
		if (winderThread != null) winderThread.abortSequence();
	}

	public void handleWinderInDisabled() {
		if (winderThread != null) winderThread.handleRobotDisable();
	}
	
	// positive degToRotate lowers the arm
	// negative degToRotate raises the arm
/*
	public void rotateArmXDegrees(double degToRotate) {
		double direction = (degToRotate) / Math.abs(degToRotate);
		double rotations = ((350.0 * degToRotate) / 360.0);
		double maxSpeed = direction * armRotatorMaxSpeed;
//		armProfile.setMotionProfile(new SRXProfile(maxSpeed, armRotator.getPosition(), rotations, 250, 250, 10));
//		armProfile.startMotionProfile();
		armProfile.pushAndStartMP(new SRXProfile(maxSpeed, armRotator.getPosition(), rotations, 250, 250, 10));
		System.out.println("Arm command: " + degToRotate);
	}
*/
	
	// move arm to an absolute encoder position (# of turns from 0)
	// will only work if arm is "homed" to a known 0 position 
	// assume this home 0 = fully lowered;  which means absolute position must always be < 0
	//		- negative/reverse travel raises arm;  positive/forward travel lowers arm
	// ?update the below to check if a profile is currently running, and safely interrupt it via the new driver if it is,
	// before sending a new profile
/*	
	public void rotateArmToPosition(double targetPosition) {
		//		- distance to travel is (-(currentPos - targetPos))
		double distance = -((armRotator.getPosition() - targetPosition));
		//		- distance and maxspeed must have same sign for profile generation to work
		double direction = (distance) / Math.abs(distance);
		double maxSpeed = direction * armRotatorMaxSpeed;
		if (!armProfile.isMPRunning()) {
			armProfile.pushAndStartMP(new SRXProfile(maxSpeed, armRotator.getPosition(), distance, 250, 250, 10));
		}
	}
*/
	public void rockAndRoll(double jubbs) {
		rollerz.set(jubbs);
	}

	public void toggleClaw(boolean jubbs) {
		santaClaw.set(jubbs);
	}
	
	public boolean isClawOpen() {
		return !santaClaw.get();
	}
	
	public void checkArmEncoderPresent() {
		if (armRotator.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) 
										!= CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
			resetArm();
			armEnabled = false;
		} 
//		armProfile.control();
//		armRotator.set(armProfile.getSetValue().value);
	}
	
	public void resetArm() {
//	    armRotator.changeControlMode(TalonControlMode.PercentVbus);
//	    armRotator.setPosition(0);
//	    armProfile.reset();
//	    armRotator.set(armProfile.getSetValue().value);

		// just interrupt any existing MP;  don't ever reset position
		armProfile.interruptMP();
		armLocker.set(true); //changed to true
	}
	
	public double getArmPosition() {
		return armRotator.getPosition();
	}
	
	public boolean isShooterEnabled() { 	//for shooterEnabled field
		return shooterEnabled;
	}
	
	public boolean isShooterCocked() { 		//for photo-eye (false when it sees pickup)
		return !shooterCocked.get();		// the photoeye reads false when the shooter is cocked, so reverse
	}

	public boolean getArmLimiter() {	// for photo-eye at arm down position (false when at down position)
		return armLimiter.get();
	}
	
	public boolean getShooterEye() {
		return shooterCocked.get();
	}

	public boolean isArmEnabled() {		
		return armEnabled;
	}
	
	public boolean isArmDown() {		// photo-eye reads false when it sees the arm in it's fully
		return !armLimiter.get();		// lowered position
	}
	
	public boolean isArmWaiting() {
		return (!armProfile.isMPRunning()) && (armPosThread.getStep() == 0);
	}
	
	public void executeWinderHomingSequence(){
		winderThread.goToHomePosition();
	}
	
	public void executeArmHomingSequence() {
		new ArmHomingThread().start();
	}
	
	public void startFromUncockedPosition() {
		winderThread.startFromUncockedPosition();
	}
	
	public void executeShootAndReloadSequence(){
//		santaClaw.set(false);						// open the pickup claw
		winderThread.startFromCockedPosition();
	}
	
	public void executeCheckAndRotate(double targetPos) {
		armPosThread.checkAndRotateArm(targetPos, 18.0);
	}
	
	public void executeCheckAndRotate(double targetPos, double maxRPM) {
		armPosThread.checkAndRotateArm(targetPos, maxRPM);
	}
	
	public void executeArmLocking() {
		armPosThread.lockArm();
	}
	
	public void executeManualArmLock(boolean lock) {
		armPosThread.manualArmLock(lock);
	}
}
