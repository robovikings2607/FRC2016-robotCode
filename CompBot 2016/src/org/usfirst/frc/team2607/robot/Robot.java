
package org.usfirst.frc.team2607.robot;

import com.kauailabs.navx.frc.AHRS;

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
	private boolean controlSet ;

	
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	shifter = new Solenoid(1,Constants.shifter);
    	leftMotors = new Transmission(Constants.leftDeviceIDs , false);
    	rightMotors = new Transmission(Constants.rightDeviceIDs , false);
    	rDrive = new RobotDrive(leftMotors , rightMotors);
    	rDrive.setSafetyEnabled(false);
    	arm = new PuncherArm();
    	System.out.println("I AM CUTE DINOSAUR, HEAR ME RAWR");
    	dController = new RobovikingStick(Constants.dControllerPort);
    	oController = new RobovikingStick(Constants.oControllerPort);
    	
    	controlSet = false;
    	autoEngine = new AutonomousEngine(rDrive, arm, shifter);  	    	
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    }
    
    public void disabledPeriodic() {
    	arm.resetArm();
    	arm.process();
    }

    /**
     * This function is called periodically (~50 times a second) during operator control
     */
    public void teleopPeriodic() {
    	
    	moveVal = -( dController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
    	rotateVal = - (dController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));
    	
    	// Driving!
    	shifter.set(dController.getToggleButton(RobovikingStick.xBoxButtonRightStick));
    	rDrive.arcadeDrive(moveVal, rotateVal);
    	
    	if(oController.getRawButton(RobovikingStick.xBoxButtonLeftStick)){
    		controlSet = true;
    	} else {
    		controlSet = false;
    	}
    
    	//Winding the puncher
    	if(oController.getRawButton(RobovikingStick.xBoxLeftBumper)) {  // drive plunger back (tighten)
    		arm.windPuncher(.6);
    	}
    	else if(oController.getRawButton(RobovikingStick.xBoxRightBumper)) {  // drive plunger up (loosen)
    		arm.windPuncher(-.6);
    	}
    	else {
    		arm.windPuncher(0);
    	}

    	//Shooting controls
    	if(oController.getTriggerPressed(RobovikingStick.xBoxRightTrigger)) {  // right trigger = axis 3
    		arm.shoot();
    	}
    	else if(oController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
    		arm.lock();
    	}
    	
    	//Controlling the rollers
    	if(oController.getRawButton(RobovikingStick.xBoxButtonY) && !controlSet) {
    		arm.rockAndRoll(1.0);
    	}
    	else if(oController.getRawButton(RobovikingStick.xBoxButtonA) && !controlSet) {
    		arm.rockAndRoll(-1.0);
    	}
    	else {
    		arm.rockAndRoll(0);
    	}
    	
    	//Controlling the claw (open or close)
    	arm.toggleClaw(oController.getToggleButton(RobovikingStick.xBoxButtonB));
    	
    	//Controlling the arm    	
    	//if (!controlSet) arm.resetArm();
    	arm.process();
    	
    	// raise the arm 5 degrees each time xBox Button Y is pressed while holding down left stick
    	// lower the arm 5 degrees each time xBox Button A is pressed while holding down left stick
    	if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY) && controlSet) {
    		arm.rotateArmXDegrees(-5.0); //(new SRXProfile(-18, -4.861, 250, 250, 10));
    	}
    	else if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonA) && controlSet) {
    		arm.rotateArmXDegrees(5.0); // new SRXProfile(18, 4.861, 250, 250, 10));
    	}
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
