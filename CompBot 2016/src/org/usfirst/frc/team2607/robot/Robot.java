
package org.usfirst.frc.team2607.robot;

import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	Transmission leftMotors , rightMotors ;
	RobotDrive rDrive ;
	PuncherArm arm ;
	Solenoid shifter ;
	AHRS navX;
	
	RobovikingStick dController , oController ;
	
	AutonomousEngine autoEngine;
	
	private double moveVal , rotateVal ;
	private boolean controlSet , armInTestFlag, armOneShot;

	
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	shifter = new Solenoid(1,Constants.shifter);
    	leftMotors = new Transmission(Constants.leftDeviceIDs , true);
    	rightMotors = new Transmission(Constants.rightDeviceIDs , true);
    	rDrive = new RobotDrive(leftMotors , rightMotors);
    	rDrive.setSafetyEnabled(false);
    	arm = new PuncherArm();
    	System.out.println("I AM CUTE DINOSAUR, HEAR ME RAWR");
    	dController = new RobovikingStick(Constants.dControllerPort);
    	oController = new RobovikingStick(Constants.oControllerPort);
    	
    	controlSet = false;
    	armInTestFlag = false;
    	autoEngine = new AutonomousEngine(rDrive, arm, shifter);  	    	
    }
    
    public void autonomousInit() {
    	
    }

    public void autonomousPeriodic() {
    	
    }
    
    public void disabledPeriodic() {
    	
    	arm.stopWindingSequence();    	
    	arm.resetArm();
    	arm.process();
    	if(++counter >= 50){
    		System.out.println( "ShooterEnabled: " + arm.isShooterEnabled() + "  Shooter Eye: " + arm.isShooterCocked() + " Arm Eye: " + arm.getArmLimiter());
    		System.out.println("OPAD POV: " + oController.getPOV(0));
    		counter = 0;
    	}
    }

    int counter = 0;
    public void teleopPeriodic() {
    	
    	moveVal = -( dController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
    	rotateVal = - (dController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));
    	
    	// Driving!
    	shifter.set(!dController.getToggleButton(RobovikingStick.xBoxButtonRightStick));
    	rDrive.arcadeDrive(moveVal, rotateVal);
    	
    	if(oController.getRawButton(RobovikingStick.xBoxButtonLeftStick)){
    		controlSet = true;
    	} else {
    		controlSet = false;
    	}
    
    	//Shooting controls
    	if(oController.getTriggerPressed(RobovikingStick.xBoxRightTrigger) && arm.isShooterEnabled()) {  // right trigger = axis 3
//    		arm.shoot();
    		arm.executeShootAndReloadSequence();
    	}
    	else if(oController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
    		arm.lock();
    	}
    	
    	if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
    		arm.executeWinderHomingSequence();
    	}
    	
    	//Controlling the rollers
    	if(oController.getRawButton(RobovikingStick.xBoxRightBumper) && !controlSet) {
    		arm.rockAndRoll(-1.0);
    	}
    	else if(oController.getRawButton(RobovikingStick.xBoxLeftBumper) && !controlSet) {
    		arm.rockAndRoll(1.0);
    	}
    	else {
    		arm.rockAndRoll(0);
    	}
    	
    	//Controlling the claw (open or close)
    	arm.toggleClaw(oController.getToggleButton(RobovikingStick.xBoxButtonB));
    	
    	//Controlling the arm    	
    	//if (!controlSet) arm.resetArm();
    	arm.process();
/*    	
    	// raise the arm 5 degrees each time xBox Button Y is pressed while holding down left stick
    	// lower the arm 5 degrees each time xBox Button A is pressed while holding down left stick
    	if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY) && controlSet) {
    		arm.rotateArmXDegrees(-5.0); //(new SRXProfile(-18, -4.861, 250, 250, 10));
    	}
    	else if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonA) && controlSet) {
    		arm.rotateArmXDegrees(5.0); // new SRXProfile(18, 4.861, 250, 250, 10));
    	}
*/

    	if(!armInTestFlag){
/*
    		if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY) && !arm.getArmLimiter() && controlSet) {
	    		arm.rotateArmXDegrees(-47);
	    	}
	    	if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonA) && arm.getArmLimiter() && controlSet) {
	    		arm.rotateArmXDegrees(47);
	    	}
*/			switch (oController.getPOV(0)) {
				case 0:
					if (!armOneShot) arm.rotateArmXDegrees(-47);
					armOneShot = true;
					break;
				case 180:
					if (!armOneShot) arm.rotateArmXDegrees(47);
					armOneShot = true;
					break;
				case -1:
				default:
					armOneShot= false;
					break;
			}
    	}
    	
    	
    	
    	if(++counter >= 50){
    		System.out.println( "ShooterEnabled: " + arm.isShooterEnabled() + "  Shooter Eye: " + arm.isShooterCocked() + " Arm Eye: " + arm.getArmLimiter());
    		counter = 0;
    	}
        
    }
// WIERD TALON SRX BEHAVIORS:
/* For some reason, the drive Talons would lose master/follower when testing the below.  Could be due either to 
 * something in the PID Controller or Test Mode - need to figure out why this is.
 * 		- if we start in Teleop, Talons are in master/follower
 * 		- if we move to test and run the PID, only "motor2" (the master) runs...the other two do not follow    
 * As a workaround, updating the Transmission class to just set all three motors directly.
 *
 * Also the encoder positions seem to get messed up when switching from Test to Teleop - winder and arm do not work properly
 * even if we just restart code.  Need to power cycle to get normal behavior
 */    
    private RobovikingDriveTrainProfileDriver d;
    public void testInit() {
    	TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
        config.dt = .01;
        config.max_acc = 8.0;
        config.max_jerk = 60.0;
        config.max_vel = 6.0;
        
        final double kWheelbaseWidth = 25.25/12;

        WaypointSequence p = new WaypointSequence(10);
        p.addWaypoint(new WaypointSequence.Waypoint(0, 0, 0));
        //p.addWaypoint(new WaypointSequence.Waypoint(5, 7.0, 0));
        p.addWaypoint(new WaypointSequence.Waypoint(7.0, 7.0, Math.PI / 3.0));
        p.addWaypoint(new WaypointSequence.Waypoint(10.0, 10.0, 0));

        Path path = PathGenerator.makePath(p, config,
            kWheelbaseWidth, "Corn Dogs");
    	
    	d = new RobovikingDriveTrainProfileDriver(leftMotors, rightMotors, config.dt, path);
    	shifter.set(true);
    }
    
    public void testPeriodic() {
  	    	
    	if (++counter >= 30) {
    		System.out.println("Left SP: " + leftMotors.pidLoop.getSetpoint() + " Right SP: " + rightMotors.pidLoop.getSetpoint());
    		System.out.println("Left enc: " + leftMotors.enc.getRate() + " Right enc: " + rightMotors.enc.getRate());
    		counter = 0;
    	}
    	
    	// shooter winding manual control
    	if(oController.getRawButton(RobovikingStick.xBoxLeftBumper)) {  // drive plunger forward (loosen)
    		arm.winderManualRun(.6);
    	}
    	else if(oController.getRawButton(RobovikingStick.xBoxRightBumper) && arm.isShooterCocked()) {  // drive plunger back (tighten)
    		arm.winderManualRun(-.6);
    	}
    	else {
    		arm.winderManualRun(0);
    	}

    	// manual control of shooter latch
    	if(oController.getTriggerPressed(RobovikingStick.xBoxRightTrigger)) {  // right trigger = axis 3
    		arm.shoot();
    	}
    	else if(oController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
    		arm.lock();
    	}
    	
    	if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY) ) {
    		armInTestFlag = true;
    		arm.rotateArmXDegrees(-5.0); //(new SRXProfile(-18, -4.861, 250, 250, 10));
    	}
    	else if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonA) && arm.getArmLimiter() ) {
    		armInTestFlag = true;
    		arm.rotateArmXDegrees(5.0); // new SRXProfile(18, 4.861, 250, 250, 10));
    	}  	  	
    	
    	// testing of velocity PID on Transmission 
    	if (dController.getRawButton(RobovikingStick.xBoxButtonY)) {
    		leftMotors.enableVelPID();
    		leftMotors.setVelSP(4.0);
    		rightMotors.enableVelPID();
    		rightMotors.setVelSP(-4.0);
    	}  
    	
    	if (dController.getButtonReleasedOneShot(RobovikingStick.xBoxButtonY)){
    		leftMotors.disableVelPID();
    		rightMotors.disableVelPID();
    		leftMotors.set(0);
    		rightMotors.set(0);
    	}
    	
    	if (dController.getButtonPressedOneShot(RobovikingStick.xBoxButtonX)) {
    		d.followPath();
    	}
    }
    
}
