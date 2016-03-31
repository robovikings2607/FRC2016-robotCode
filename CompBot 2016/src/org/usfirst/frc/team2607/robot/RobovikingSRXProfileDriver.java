package org.usfirst.frc.team2607.robot;

import java.util.concurrent.atomic.AtomicInteger;

import org.usfirst.frc.team2607.robot.SRXProfileDriver.PeriodicRunnable;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

// this will consist of 2 threads
//		1) the main thread runs the state machine like SRXProfileDriver.control() would, just doesn't need to be called from
//		the Robot periodic methods
//			- the state machine will also be more robust in terms of interrupting running profiles, hopefully fixing
//			the wierd behavior we sometimes see...check that current profile actually gets disabled, before proceeding
//			to push new points
//		2) the private inner thread replaces the notifier from SRXProfileDriver; it just tells the MPE to process it's
//		available point

public class RobovikingSRXProfileDriver extends Thread {


	private SRXProfile motionProfile;		// the profile we'll push to the Talon and execute
	private CANTalon talonSRX;				// the Talon we're driving
	//private int state;						
	private AtomicInteger state;			// the state machine control variable
	Notifier notifier = new Notifier(new PeriodicRunnable());
	
	private CANTalon.MotionProfileStatus talonMPStatus = new CANTalon.MotionProfileStatus();

	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  talonSRX.processMotionProfileBuffer();    }
	}

	public RobovikingSRXProfileDriver(CANTalon t) {
		talonSRX = t;
		motionProfile = null;
		state = new AtomicInteger(0);
	}

	public void process() {

		if (talonSRX.getControlMode() != TalonControlMode.MotionProfile) return;
		talonSRX.getMotionProfileStatus(talonMPStatus);
		switch (state.get()) {
			default:					// the default state is just waiting for command to push MP, 
										// talon MP state could be either Hold or Disable
				break;	
			case 1:						// triggered start
				if (talonMPStatus.outputEnable == CANTalon.SetValueMotionProfile.Disable) { 
					talonSRX.clearMotionProfileTrajectories();	
					state.compareAndSet(1,2);		//state += 1;
				}
				talonSRX.set(CANTalon.SetValueMotionProfile.Disable.value);
				break;
			case 2:						// if we get here, talon is disabled and ready for MP push
				if (motionProfile == null) {
					state.compareAndSet(2, 0);		//state = 0;
					return;
				}
				motionProfile.generateAndPushProfile(talonSRX);
				state.compareAndSet(2, 3);			//state += 1;
				break;
			case 3:						// check if enough points have been streamed, and enable if so
				if (talonMPStatus.btmBufferCnt > 5) {
					talonSRX.set(CANTalon.SetValueMotionProfile.Enable.value);
					state.compareAndSet(3, 4);		//state += 1;
				}
				break;
			case 4:						// MP is running, when we get to end set talon to hold last point
				if (talonMPStatus.activePointValid && talonMPStatus.activePoint.isLastPoint) {
					talonSRX.set(CANTalon.SetValueMotionProfile.Disable.value);		// changed to disable due to locking
					state.compareAndSet(4, 0);		//state = 0;
				}
				break;
			case 10:					// reset state to interrupt and clear the running MP, and go back to wait	
				if (talonMPStatus.outputEnable == CANTalon.SetValueMotionProfile.Disable) {
					talonSRX.clearMotionProfileTrajectories();
					state.set(0);
				} else {
					talonSRX.set(CANTalon.SetValueMotionProfile.Disable.value);
				}
				break;
		}
	}
	
	@Override
	public void run() {
		System.out.println("Starting RobovikingSRXProfileDriver thread....");
		talonSRX.changeMotionControlFramePeriod(5);
		notifier.startPeriodic(.005);
		while (true) {
			process();
			try { Thread.sleep(20);} catch (Exception e) {}
		}
	}
	
	public void pushAndStartMP(SRXProfile mp) {
		// if we're in wait state, go ahead and try to start profile
		if (state.get() == 0) {
			motionProfile = mp;
			state.set(1);
		}
	}
	
	public void interruptMP() {
		// if we're running a profile (enabled), interrupt (disable and clear) and go to wait state
		if (state.get() != 0) state.set(10);
	}	
	
	public boolean isMPRunning() {
		return (state.get() != 0);
	}
/*	
	public int getMPState() {
		return state.get();
	}
*/
}
