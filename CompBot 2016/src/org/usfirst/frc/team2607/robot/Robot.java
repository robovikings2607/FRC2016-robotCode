
package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.FileReader;
import java.util.Scanner;

import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;

import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.io.TextFileDeserializer;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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
	private boolean armInTestFlag, armOneShot;
	private RobovikingDriveTrainProfileDriver mp;
	private int armPosIndex = 0;						// index into array of arm positions
	
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	shifter = new Solenoid(1,Constants.shifter);
    	leftMotors = new Transmission(Constants.leftDeviceIDs , true, RobovikingModPIDController.kTurnLeft, null); // null for gyro means not used
    	leftMotors.setName("leftMotors");
    	rightMotors = new Transmission(Constants.rightDeviceIDs , true, RobovikingModPIDController.kTurnRight, null); // null for gyro means not used
    	rightMotors.setName("rightMotors");
    	rDrive = new RobotDrive(leftMotors , rightMotors);
    	rDrive.setSafetyEnabled(false);

    	
    	arm = new PuncherArm();
    	armPosIndex = 0;
    	System.out.println("I AM CUTE DINOSAUR, HEAR ME RAWR");
    	dController = new RobovikingStick(Constants.dControllerPort);
    	oController = new RobovikingStick(Constants.oControllerPort);
    	
    	// default to high gear
    	shifter.set(true);    	
    	
    	armInTestFlag = false;
    	autoEngine = new AutonomousEngine(rDrive, arm, shifter);  	    	
    	
    	// for tuning....webserver to view PID logs
    	Server server = new Server(5801);
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(5801);
        server.addConnector(connector);

        ResourceHandler resource_handler = new ResourceHandler();
        resource_handler.setDirectoriesListed(true);
        resource_handler.setWelcomeFiles(new String[]{ "/home/lvuser/index.html" });

        resource_handler.setResourceBase(".");
        
        HandlerList handlers = new HandlerList();
        handlers.setHandlers(new Handler[] { resource_handler, new DefaultHandler() });
        server.setHandler(handlers);
        try {
			server.start();
			server.join();
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
    }
 
    public void autonomousInit() {
    	// NOTE:  must set rightMotors inverted in order to use motion profile, due to the completely messed-up 
    	// way the WPILib RobotDrive handles opposing sides of the robot drivetrain....again, not that we're complaining
    	// or anything

    	rightMotors.setInverted(true);			/// AAAAARRRRRGHHH!!!!!   WHYYYYYYYYY??????!!!!
    	Path path = null;
		try {
			Scanner trajFile = new Scanner(new FileReader(new File("/home/lvuser/testProfile.txt")));
			trajFile.useDelimiter("\\Z");
			String traj = trajFile.next();
			TextFileDeserializer tfds = new TextFileDeserializer();
			path = tfds.deserialize(traj);
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		mp = new RobovikingDriveTrainProfileDriver(leftMotors, rightMotors, path);
		mp.followPath();
    }

    public void autonomousPeriodic() {
    	
    	consoleMessage();
    	
    }
    
    public void disabledPeriodic() {
    	// arm.stopWindingSequence();  // don't stop the winding sequence in disabled; let it finish when we re-enable
    								   // since that is normal match behavior
    	arm.resetArm();
    	arm.checkArmEncoderPresent();
    	
    	consoleMessage();
    }

	@Override
	public void teleopInit() {
		// NOTE:  must set leftMotors and rightMotors to not-inverted so that RobotDrive works correctly after
		// running motion profile in autonomous mode
		//		This is due to the completely messed-up way WPILib currently handles opposing sides of the robot
		//		drivetrain....not that we're complaining or anything
		leftMotors.setInverted(false);
		rightMotors.setInverted(false);
	}
    
    int counter = 0, msgCount = 0;
    public void consoleMessage() {
    	if(++counter >= 50){
    		msgCount += 1;
    		System.out.println(msgCount + ": Shooter Enabled: " + arm.isShooterEnabled() + "  Shooter Cocked: " + arm.isShooterCocked() 
    							+ "Shooter Eye: " + arm.getShooterEye());
    		System.out.println(msgCount + ": Arm Eye: " + arm.getArmLimiter() + " Arm Enabled: " + arm.isArmEnabled()
    							+ " Arm Down: " + arm.isArmDown());
    		System.out.println(msgCount + ": Drive Profile running: " + mp.isRunning() + " ; Done: " + mp.isDone());
    		counter = 0;
    	}    	
    }
    
    public void teleopPeriodic() {

    	consoleMessage();
    	
    	moveVal = -( dController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
    	rotateVal = - (dController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));
    	
    	// Driving!
    	shifter.set(!dController.getToggleButton(RobovikingStick.xBoxButtonRightStick));  // defaults to high gear (true)
    	rDrive.arcadeDrive(moveVal, rotateVal);
    	   
    	//Shooting controls
    	if(oController.getTriggerPressed(RobovikingStick.xBoxRightTrigger) && arm.isShooterEnabled()) {  // right trigger = axis 3
    		if (!arm.isClawOpen()) {
    			oController.setRumble(Joystick.RumbleType.kLeftRumble, 1);
    			oController.setRumble(Joystick.RumbleType.kRightRumble, 1);
    		} else {
    			arm.executeShootAndReloadSequence();		// go ahead and shoot	
    		}    		
    	} else {
    		oController.setRumble(Joystick.RumbleType.kLeftRumble, 0);
    		oController.setRumble(Joystick.RumbleType.kRightRumble, 0);
    	}
    	
    	if(oController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
    		arm.lock();
    	}
    	
    	// Homing sequence for shooter
    	if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
    		arm.executeWinderHomingSequence();
    	}
    	// abort the homing sequence if you release the start button while it's running
    	if (oController.getButtonReleasedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
    		arm.stopWindingSequence();
    	}
    	
    	//Controlling the rollers
    	if(oController.getRawButton(RobovikingStick.xBoxRightBumper)) {
    		arm.rockAndRoll(-1.0);
    	}
    	else if(oController.getRawButton(RobovikingStick.xBoxLeftBumper)) {
    		arm.rockAndRoll(1.0);
    	}
    	else {
    		arm.rockAndRoll(0);
    	}
    	
    	//Controlling the claw (open or close)
    	arm.toggleClaw(oController.getToggleButton(RobovikingStick.xBoxButtonB));
    	
    	//Controlling the arm - check safeties: 
    	//	1) the position encoder is present and interrupt MP if not
    	arm.checkArmEncoderPresent();

    	if(!armInTestFlag){
    		
    		switch (oController.getPOV(0)) {
				case 0:
					if (!armOneShot && arm.isArmEnabled() && arm.isArmWaiting() && armPosIndex < (Constants.armPositions.length - 1)) { 
						armPosIndex += 1;
						arm.rotateArmToPosition(Constants.armPositions[armPosIndex]); //arm.rotateArmToPosition(-45.69); // arm.rotateArmXDegrees(-47);
					}
					armOneShot = true;
					break;
				case 180:
					if (!armOneShot && arm.isArmEnabled() && arm.isArmWaiting() && (armPosIndex > 0)) {
						armPosIndex -= 1;
						arm.rotateArmToPosition(Constants.armPositions[armPosIndex]);		//arm.rotateArmXDegrees(47);
					}
					armOneShot = true;
					break;
				case -1:
				default:
					armOneShot= false;
					break;
			}
    		
    	}
    	
        
    }
    
    
    
// WIERD TALON SRX BEHAVIORS:
/* For some reason, the drive Talons would lose master/follower when testing motion profiling in test mode.  
 * Could be due either to something in the PID Controller or Test Mode - need to figure out why this is.
 * 		- if we start in Teleop, Talons are in master/follower
 * 		- if we move to test and run the PID, only "motor2" (the master) runs...the other two do not follow    
 * As a workaround, updating the Transmission class to just set all three motors directly.
 *
 * Also the encoder positions seem to get messed up when switching from Test to Teleop - winder and arm do not work properly
 * even if we just restart code.  Need to power cycle to get normal behavior
 */    

    public void testInit() {
    	armInTestFlag = true;		// I don't trust test mode with the SRX, so prevent moving the arm back in teleop 
    								// unless we restart the code
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
    	else if(oController.getRawButton(RobovikingStick.xBoxRightBumper) && !arm.isShooterCocked()) {  // drive plunger back (tighten)
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
    	
    	/* // SP is not just a velocity anymore
    	 * // testing of velocity PID on Transmission 
    	if (dController.getRawButton(RobovikingStick.xBoxButtonY)) {
    		leftMotors.enableVelPID();
    		//leftMotors.setVelSP(4.0);				
    		rightMotors.enableVelPID();
    		//rightMotors.setVelSP(-4.0);			// the right side motors are reversed
    	}  
    	
    	if (dController.getButtonReleasedOneShot(RobovikingStick.xBoxButtonY)){
    		leftMotors.disableVelPID();
    		rightMotors.disableVelPID();
    		leftMotors.set(0);
    		rightMotors.set(0);
    	}*/
    	
    }
    
}
