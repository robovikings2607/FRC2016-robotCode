
package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.FileReader;
import java.text.DecimalFormat;
import java.util.Scanner;

import org.usfirst.frc.team2607.robot.auto.AutonomousEngine;

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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	

	public Transmission leftMotors , rightMotors ;
	public RobotDrive rDrive ;
	public PuncherArm arm ;
	public Solenoid shifter ;
	public AHRS navX;
	public PIDController turnPID;
	public double  kp = 0.053,	// 0.053
				   ki = 0.00012,
				   kd = 0.0,
				   feedForward = .36; // 0.1
	public boolean visionTurningActive = false;
	
	//public RobovikingDriveTrainProfileDriver mp;
	
	RobovikingStick dController , oController ;
	
	AutonomousEngine autoEngine;
	Thread autoThread = null;
//	SimpleTableServer dataTable;
	
	private double moveVal , rotateVal ;
	private boolean armInTestFlag, armOneShot, armLockOneShot = false, armVisionOneShot = false;
	private int armPosIndex = 0;						// index into array of arm positions
	private boolean tryingToShootWithPickupClosed = false, armDownButPickupOpen = false;
	private DecimalFormat floatFmt = new DecimalFormat("##.0000000");
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	navX = new AHRS(SPI.Port.kMXP);
    	navX.setPIDSourceType(PIDSourceType.kDisplacement);
    	turnPID = new PIDController(kp, ki, kd, feedForward, navX, this);
    	turnPID.setInputRange(-180.0, 180.0);
    	turnPID.setOutputRange(-.7, .7);
    	turnPID.setAbsoluteTolerance(.5);
    	SmartDashboard.putString("kp", floatFmt.format(kp));
    	SmartDashboard.putString("ki", floatFmt.format(ki));
    	SmartDashboard.putString("kd", floatFmt.format(kd));
    	SmartDashboard.putString("ff", floatFmt.format(feedForward));
    	shifter = new Solenoid(1,Constants.shifter);
    	leftMotors = new Transmission(Constants.leftDeviceIDs , true, RobovikingModPIDController.kTurnLeft, null); // null for gyro means not used
    	leftMotors.setName("Left");
    	rightMotors = new Transmission(Constants.rightDeviceIDs , true, RobovikingModPIDController.kTurnRight, null); // null for gyro means not used
    	rightMotors.setName("Right");
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

    	autoEngine = new AutonomousEngine(this); 
    	autoEngine.loadSavedMode();
/*
    	try {
    		dataTable = new SimpleTableServer();
    	} catch (Exception e) {
    		dataTable = null;
    	}
*/    	
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
 
    boolean autonModeRan = false;
    public void autonomousInit() {
    	// NOTE:  must set rightMotors inverted in order to use motion profile, due to the completely messed-up 
    	// way the WPILib RobotDrive handles opposing sides of the robot drivetrain....again, not that we're complaining
    	// or anything

    	rightMotors.setInverted(true);			/// AAAAARRRRRGHHH!!!!!   WHYYYYYYYYY??????!!!!
    	
    	leftMotors.log.enableLogging(true);
    	rightMotors.log.enableLogging(true);
    	
    	autoThread = new Thread(autoEngine);
//    	autoEngine.setMode(2);
    	if (autoEngine.getMode() == 2 || autoEngine.getMode() == 3) armPosIndex = 2;
    	autoThread.start(); 
    	autonModeRan = true;
    }

    public void autonomousPeriodic() {
    	
    	//consoleMessage();
    	
    }
    
    public void disabledPeriodic() {
    	
    	if (autonModeRan) {
    		autonModeRan = false;
    		if (autoThread != null) {
    			if (autoThread.isAlive()) { 
    				System.out.println("autoThread alive, interrupting");
    				autoThread.interrupt();
    			} else {
    				System.out.println("autoThread not alive");
    			}
    			
    		}
    	}
    	
    	arm.handleWinderInDisabled();
    	arm.resetArm();
    	arm.checkArmEncoderPresent();
    	
    	//consoleMessage();
    	
    	leftMotors.log.enableLogging(false);
    	rightMotors.log.enableLogging(false);
    	
    	if (dController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart)) {
    		autoEngine.selectMode();
    	}
    }

	@Override
	public void teleopInit() {
		// NOTE:  must set leftMotors and rightMotors to not-inverted so that RobotDrive works correctly after
		// running motion profile in autonomous mode
		//		This is due to the completely messed-up way WPILib currently handles opposing sides of the robot
		//		drivetrain....not that we're complaining or anything
		leftMotors.setInverted(false);
		rightMotors.setInverted(false);
		SmartDashboard.putNumber("manualArmAngle", -999);
	}
    
    int counter = 0, msgCount = 0;
    public void consoleMessage() {
    /*	
    	if (counter >= 25 && dataTable != null) {
    		dataTable.put("ArmPosition", arm.getArmPosition());
    	}
    */	   	
    	if(++counter >= 50){
    		msgCount += 1;
    		System.out.println(msgCount + ": Shooter Enabled: " + arm.isShooterEnabled() + "  Shooter Cocked: " + arm.isShooterCocked() 
    							+ " Shooter Eye: " + arm.getShooterEye());
    		System.out.println(msgCount + ": Arm Eye: " + arm.getArmLimiter() + " Arm Enabled: " + arm.isArmEnabled()
    							+ " Arm Down: " + arm.isArmDown());
    		System.out.println(msgCount + ": targetAngleInFOV: " + SmartDashboard.getNumber("targetAngleInFOV", 999) 
    									+ " degToRotate: " + SmartDashboard.getNumber("degToRotate", 999) 
    								    + " Arm Position: " + arm.getArmPosition() + "\n");
    		SmartDashboard.putNumber("Arm Position", arm.getArmPosition());
    		counter = 0;
    	}    	
    }
    
    public double calcTurn(double degToTurn) {
		long timeoutMilli = 3000;
		long startTime = System.currentTimeMillis();
		
		double kP = 0.053;
		double maxTurn = 0.7;
		double tolerance = 0.5;
				
		double error = navX.getYaw() - degToTurn;
		System.out.println("calcTurn error: " + error);
			
		double calcTurn = kP * error;
			if (error <= 0){
				calcTurn = Math.max(-maxTurn, calcTurn - .36);
			} else {
				calcTurn = Math.min(maxTurn, calcTurn + .36);
			}
			
		if (navX.getYaw() > (degToTurn - tolerance) && navX.getYaw() < (degToTurn + tolerance)){
			return 0.0;
		} else {
			System.out.println("CommandedVoltage: " + calcTurn);
			return calcTurn;
		}

    }
    
    
    boolean turnOneShot = false;
    double degFromVision = 990;
    public void teleopPeriodic() {

    	//consoleMessage();
    	SmartDashboard.putNumber("Arm Position", arm.getArmPosition());
    	
    	// Driving!
    	moveVal = -( dController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
    	rotateVal = - (dController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX));

    	if (dController.getRawButton(RobovikingStick.xBoxButtonA)) {
    		if (!turnOneShot) {
    			turnOneShot = true;
    			navX.zeroYaw();
    			degFromVision = SmartDashboard.getNumber("degToRotate", 999);
    		}
    		if (degFromVision != -999 && degFromVision != 999) {
    			visionTurningActive = true;
    			rDrive.arcadeDrive(0, calcTurn(degFromVision));
    		}
    	} else if (dController.getRawButton(RobovikingStick.xBoxButtonB)) {
    		if (!turnOneShot) {
    			turnOneShot = true;
    			navX.zeroYaw();
    			degFromVision = SmartDashboard.getNumber("degToRotate", 999);
    			kp = Double.parseDouble(SmartDashboard.getString("kp"));
    			ki = Double.parseDouble(SmartDashboard.getString("ki"));
    			kd = Double.parseDouble(SmartDashboard.getString("kd"));
    			feedForward = Double.parseDouble(SmartDashboard.getString("ff"));
    			turnPID.setPID(kp, ki, kd, feedForward);
        		if (degFromVision != -999 && degFromVision != 999) {
        			turnPID.setSetpoint(degFromVision);
        			turnPID.enable();
        			visionTurningActive = true;
        		}
    		}
    	} else {
    		turnOneShot = false;
    		if (turnPID.isEnabled()) {
    			turnPID.reset();
    		}
    		visionTurningActive = false;
    		rDrive.arcadeDrive(moveVal, rotateVal);	
    	}

    	// Shifting!
    	if (visionTurningActive) {
    		shifter.set(false);	//low gear
    	} else {
    		shifter.set(!dController.getToggleButton(RobovikingStick.xBoxButtonRightStick));  // defaults to high gear (true)	
    	}
    	
    	
    	//Shooting controls
    	if(oController.getTriggerPressed(RobovikingStick.xBoxRightTrigger) && arm.isShooterEnabled()) {  // right trigger = axis 3
    		if (!arm.isClawOpen()) {
    			tryingToShootWithPickupClosed = true;
    		} else {
    			arm.executeShootAndReloadSequence();  // go ahead and shoot
    			tryingToShootWithPickupClosed = false;
    		}    		
    	} else {
    		tryingToShootWithPickupClosed = false;
    	}
    	
    	
    	// Homing sequence for shooter
    	if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
    		arm.executeWinderHomingSequence();
    	}
    	// abort the homing sequence if you release the start button while it's running
    	if (oController.getButtonReleasedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
    		arm.stopWindingSequence();
    	}
    	
    	// Homing sequence for arm
    	if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonBack) && !arm.isArmEnabled()) {
    		arm.executeArmHomingSequence();
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

    	armDownButPickupOpen = arm.isArmDown() && arm.isClawOpen();

    	// controller rumble
    	if (tryingToShootWithPickupClosed || armDownButPickupOpen) {
    		oController.setRumble(Joystick.RumbleType.kLeftRumble, 1);
    		oController.setRumble(Joystick.RumbleType.kRightRumble, 1);
    	} else {
    		oController.setRumble(Joystick.RumbleType.kLeftRumble, 0);
    		oController.setRumble(Joystick.RumbleType.kRightRumble, 0);
    	}
    	
    	//Controlling the arm - check safeties: 
    	//	1) the position encoder is present and interrupt MP if not
    	arm.checkArmEncoderPresent();
 	    
    	// manual arm unlock
    	if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY)) {
    		arm.executeManualArmLock(false);
    	} 
    	
    	if(!armInTestFlag){

    		// removed arm.isArmWaiting() check from all commands, to allow for interrupt
    		
    		// lock arm in current position (interrupting movement if necessary)
    		if(oController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
        		if (!armLockOneShot && arm.isArmEnabled() && armPosIndex != -1) {
        			armPosIndex = -1;
        			arm.executeArmLocking();
        			armLockOneShot = true;
        		}
        	} else {
        		armLockOneShot = false;
        	}
    		
    		// move arm to position specified by manual entry on Smart Dashboard (for tuning purposes only)
    		if (dController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY) && arm.isArmEnabled()) {
    			double manualArmAngle = SmartDashboard.getNumber("manualArmAngle", 999);
    			if (manualArmAngle < 0 && manualArmAngle > -67.0) {
    				armPosIndex = -2;
    				arm.executeCheckAndRotate(manualArmAngle, 30.0);
    			}
    		}
    		
    		// move arm to high goal target angle based on vision tracking    		
    		if (oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonA) && !armVisionOneShot && arm.isArmEnabled()) {
    			double targetAngleInFOV = SmartDashboard.getNumber("targetAngleInFOV", 999);
    			double armPosition = Constants.getArmAngle(targetAngleInFOV);
    			/*targetAngleInFOV += (-.5);
    			double armPosition = .00014735 * Math.pow(targetAngleInFOV, 3) + 
    								 -.0282245209 * Math.pow(targetAngleInFOV, 2) + 
    								 1.051732632 * targetAngleInFOV + 
    								 -52.64559001  + 0.3 ; // + Fudge factor
    								 /*.0005909 * Math.pow(targetAngleInFOV, 3) + 
    								 -.07626081 * Math.pow(targetAngleInFOV, 2) +
    								 2.661478844 * targetAngleInFOV + 
    								 -68.3454292; */
    			
    			if (armPosition < 0 && armPosition > -67.0) {
    				armPosIndex = -2;
    				arm.executeCheckAndRotate(armPosition, 30.0);
    				armVisionOneShot = true;
    			} 
    		} else {
    			armVisionOneShot = false;
    		}
    		
    		// dpad manual arm positioning
    		switch (oController.getPOV(0)) {
				case 0:		// highest position
					if (!armOneShot && arm.isArmEnabled() && armPosIndex != 3)  { 
						armPosIndex = 3;
						//arm.rotateArmToPosition(Constants.armPositions[armPosIndex]); //arm.rotateArmToPosition(-45.69); // arm.rotateArmXDegrees(-47);
						System.out.println("Trying to move arm to position 3 : " + armPosIndex);
						arm.executeCheckAndRotate(Constants.armPositions[armPosIndex], 30.0);
					}
					armOneShot = true;
					break;
				case 90: 	// outerworks position (slightly lower than normal auton shot)
					if (!armOneShot && arm.isArmEnabled() && armPosIndex != 1) {
						armPosIndex = 1;
						//arm.rotateArmToPosition(Constants.armPositions[armPosIndex]);
						System.out.println("Trying to move arm to position 1 : " + armPosIndex);
						arm.executeCheckAndRotate(Constants.armPositions[armPosIndex], 30.0);
					}
					armOneShot = true;
					break;
				case 180:	// full down
					if (!armOneShot && arm.isArmEnabled() && armPosIndex != 0) {
						armPosIndex = 0;
						//arm.rotateArmToPosition(Constants.armPositions[armPosIndex]);		//arm.rotateArmXDegrees(47);
						System.out.println("Trying to move arm to position 0 : " + armPosIndex);
						arm.executeCheckAndRotate(Constants.armPositions[armPosIndex], 30.0);
					}
					armOneShot = true;
					break;
				case 270:	// normal shot (slightly higher than outerworks)
					if (!armOneShot && arm.isArmEnabled() && armPosIndex != 2) {
						armPosIndex = 2;
						//arm.rotateArmToPosition(Constants.armPositions[armPosIndex]);		//arm.rotateArmXDegrees(47);
						System.out.println("Trying to move arm to position 2 : " + armPosIndex);
						arm.executeCheckAndRotate(Constants.armPositions[armPosIndex], 30.0);
					}
					armOneShot = true;
					break;
				case -1:
				default:
					armOneShot= false;
					break;
			}
    		
    		if(oController.getRawButton(RobovikingStick.xBoxButtonX)){
    			if (!armOneShot && arm.isArmEnabled() && armPosIndex != 4) {
					armPosIndex = 4;
					//arm.rotateArmToPosition(Constants.armPositions[armPosIndex]);		//arm.rotateArmXDegrees(47);
					System.out.println("Trying to move arm to position 4 : " + armPosIndex);
					arm.executeCheckAndRotate(Constants.armPositions[armPosIndex], 30.0);
				}
				armOneShot = true;
    		} else {
    			armOneShot = false;
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
/*    	
    	if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonY) ) {
    		armInTestFlag = true;
    		arm.rotateArmXDegrees(-5.0); //(new SRXProfile(-18, -4.861, 250, 250, 10));
    	}
    	else if(oController.getButtonPressedOneShot(RobovikingStick.xBoxButtonA) && arm.getArmLimiter() ) {
    		armInTestFlag = true;
    		arm.rotateArmXDegrees(5.0); // new SRXProfile(18, 4.861, 250, 250, 10));
    	}  	  	
*/    	
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

	@Override
	public void pidWrite(double output) {
		rDrive.arcadeDrive(0.0, output);
	}
    
}
