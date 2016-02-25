package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
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
	private int state;						// the state machine control variable
		
	private CANTalon.MotionProfileStatus talonMPStatus = new CANTalon.MotionProfileStatus();

	private class PeriodicDriver extends Thread {
		// for now assume this will just run all the time, see what Omar responds as to whether
		// we should start/stop based on entering/leaving MP mode
		@Override
		public void run() {
			while (true) {
				talonSRX.processMotionProfileBuffer();
				try { Thread.sleep(5); } catch (Exception e) {}
			}			
		}	
	}
	
	public RobovikingSRXProfileDriver(CANTalon t) {
		talonSRX = t;
		motionProfile = null;
		state = 0;
	}

	public void process() {

		if (talonSRX.getControlMode() != TalonControlMode.MotionProfile) return;
		talonSRX.getMotionProfileStatus(talonMPStatus);
		switch (state) {
			default:					// the default state is just waiting for command to push MP, 
										// talon MP state could be either Hold or Disable
				break;	
			case 1:						// triggered start
				if (talonMPStatus.outputEnable == CANTalon.SetValueMotionProfile.Disable) state += 1;
				talonSRX.set(CANTalon.SetValueMotionProfile.Disable.value);
				talonSRX.clearMotionProfileTrajectories();
				break;
			case 2:						// if we get here, talon is disabled and ready for MP push
				if (motionProfile == null) {
					state = 0;
					return;
				}
				motionProfile.generateAndPushProfile(talonSRX);
				state += 1;
				break;
			case 3:						// check if enough points have been streamed, and enable if so
				if (talonMPStatus.btmBufferCnt > 5) {
					talonSRX.set(CANTalon.SetValueMotionProfile.Enable.value);
					state += 1;
				}
				break;
			case 4:						// MP is running, when we get to end set talon to hold last point
				if (talonMPStatus.activePointValid && talonMPStatus.activePoint.isLastPoint) {
					talonSRX.set(CANTalon.SetValueMotionProfile.Hold.value);
					state = 0;
				}
				break;
			case 10:					// reset state to interrupt and clear the running MP, and go back to wait	
				if (talonMPStatus.outputEnable == CANTalon.SetValueMotionProfile.Disable) state = 0;
				talonSRX.set(CANTalon.SetValueMotionProfile.Disable.value);
				talonSRX.clearMotionProfileTrajectories();
		}
	}
	
	@Override
	public void run() {
		talonSRX.changeMotionControlFramePeriod(5);
		new PeriodicDriver().start();
		while (true) {
			process();
			try { Thread.sleep(20);} catch (Exception e) {}
		}
	}
	
	public void pushAndStartMP(SRXProfile mp) {
		// if we're in wait state, go ahead and try to start profile
		if (state == 0) {
			motionProfile = mp;
			state = 1;
		}
	}
	
	public void interruptMP() {
		// if we're running a profile (enable or hold), interrupt and go to wait state
		if (state != 0) state = 10;
	}	
	
	public boolean isMPRunning() {
		return (state != 0);
	}
}
