package org.usfirst.frc.team2607.robot;

// this will consist of 2 threads
//		1) the main thread runs the state machine like SRXProfileDriver.control() would, just doesn't need to be called from
//		the Robot periodic methods
//			- the state machine will also be more robust in terms of interrupting running profiles, hopefully fixing
//			the wierd behavior we sometimes see
//		2) the private inner thread replaces the notifier from SRXProfileDriver; it just tells the MPE to process it's
//		available point

public class NewSRXProfileDriver {

	
	
	
	
}
