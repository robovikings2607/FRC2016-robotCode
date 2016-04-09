
package org.usfirst.frc.team2607.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
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
	
	/*
	 * TODO 1) implement the PuncherArm class
	 * 		2) implement the solenoids and digitalInputs from the PuncherArm class
	 * 		3) add controls for the arm
	 */
	
	//Objects/Fields for Driving
	Transmission leftMotors , rightMotors;
	RobovikingStick driveController , operatorController;
	RobotDrive soulOfFinn;
	AHRS navX;
	private double moveVal , rotateVal;
	
	//Objects for Arm
	PuncherArm arm ;
	DigitalInput armLimiter , plungerLimiter;
	private int armPosIndex = 0;
	private boolean armLimitFlag , plungerLimitFlag;
	private boolean armInTestFlag, armOneShot, armLockOneShot = false;
	private boolean tryingToShootWithPickupClosed = false, runningRollersWithArmUp = false;

	//Objects for Pneumatics
	Solenoid shifter ;
	Compressor compressor;
	
	//Other Objects
	PowerLogger loggerTron;
	PIDController pidController;
	double kp , ki , kd , feedForward, turnSP;
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	//Driving
    	rightMotors = new Transmission(Constants.rightMotor1 , Constants.rightMotor2 , false);
    	leftMotors = new Transmission(Constants.leftMotor1 , Constants.leftMotor2 , false);
    	driveController = new RobovikingStick(Constants.driveController);
    	operatorController = new RobovikingStick(Constants.operatorController);
    	soulOfFinn = new RobotDrive(leftMotors , rightMotors);
    	soulOfFinn.setSafetyEnabled(false);
    	
    	//Arm
    	arm = new PuncherArm();
    	armLimiter = new DigitalInput(Constants.armLimiter);
    	plungerLimiter = new DigitalInput(Constants.plungerLimiter);
    	armPosIndex = 0;
    	armInTestFlag = false;
    	armLimitFlag = true;
    	plungerLimitFlag = true;
    	
    	//Pneumatics
    	//compressor = new Compressor(1);
    	shifter = new Solenoid( 1 , Constants.shifter );
    	shifter.set(true);

    	//Other
    	navX = new AHRS(SPI.Port.kMXP);
    	pidController = new PIDController(kp, ki, kd, feedForward, navX, this);
    	loggerTron = null;
    	kp = 0.053;	// 0.053
    	ki = 0.00012;
    	kd = 0.0;
    	feedForward = .51; // 0.1
    	turnSP = 0.0;
    	navX.setPIDSourceType(PIDSourceType.kDisplacement);
    	pidController.setInputRange(-180, 180);
    	pidController.setOutputRange(-1, 1);
    	pidController.setAbsoluteTolerance(0.5);
    	//compressor.start();
    	
    }
    
    @Override
    public void disabledInit() {
    	if (loggerTron != null) {
    		loggerTron.stopLogging();
    		loggerTron = null;
    	}
    }
    
	@Override
	public void disabledPeriodic() {
		
		arm.handleWinderInDisabled();
    	arm.resetArm();
    	arm.checkArmEncoderPresent();
    	
	}


	@Override
	public void teleopInit() {
//		loggerTron = new PowerLogger(this);
//		loggerTron.startLogging();
		SmartDashboard.putNumber("turnSP", turnSP);
	}

	boolean turnOneShot = false;
    double degFromVision = 990;
    int count = 0;
    long startTime, lineupTime;
    String robotInfo;
    public void teleopPeriodic() {
    	
    	//Controls Stuff
    	
    	if(driveController.getRawButton(RobovikingStick.xBoxButtonA)) {
    		if(!turnOneShot) {
    			turnOneShot = true;
    			navX.zeroYaw();
    			degFromVision = SmartDashboard.getNumber("degToRotate", 999);
    			turnSP = SmartDashboard.getNumber("turnSP", 0);
    			if (degFromVision != -999 && degFromVision != 999) {
    				pidController.setPID(kp, ki, kd, feedForward);
    				pidController.setSetpoint(turnSP);
    				startTime = System.currentTimeMillis();
    				pidController.enable();
    				robotInfo = "Turning ";
    				SmartDashboard.putBoolean("robotSaveFrames", true);
    			}
    		}
    		if (!pidController.onTarget()) {
    			lineupTime = System.currentTimeMillis() - startTime; 
    		}
    		robotInfo += ", onT: " + pidController.onTarget() + " ms: " + lineupTime + " CO: " + pidController.get();
    		SmartDashboard.putString("robotInfo", robotInfo);
    		SmartDashboard.putNumber("robotTurnSP", pidController.getSetpoint());
    		SmartDashboard.putNumber("robotTurnPV", navX.pidGet());
    		if (++count > 25) {
    			System.out.println("SP: " + pidController.getSetpoint() + " PV: " + navX.pidGet() + " CO: " + pidController.get()
    					+ " onTarget: " + pidController.onTarget() + " lineupTime: " + lineupTime);
    			count = 0;
    		}
    	} else {
    		if (turnOneShot) {
    			pidController.reset();
    			pidController.disable();
    			turnOneShot = false;
    		}
    		
    		plungerLimitFlag = plungerLimiter.get();
    		armLimitFlag = armLimiter.get();	
    		
    		if (++count > 50) {
    			System.out.println("Arm limiter: " + armLimitFlag + " , Punchie Limiter: " + plungerLimitFlag);
    			count = 0;
    		}
    		
    		SmartDashboard.putBoolean("robotSaveFrames", false);
    		SmartDashboard.putString("robotInfo", "--");
    		SmartDashboard.putNumber("robotTurnSP", -999);
    		SmartDashboard.putNumber("robotTurnPV", -999);
    		
        	//Driving Stuff
        	moveVal = ( driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
        	rotateVal =  ( driveController.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX) );
        	
        	shifter.set(driveController.getToggleButton(RobovikingStick.xBoxButtonRightStick));
        	soulOfFinn.arcadeDrive( moveVal , rotateVal);
        	
        	//Shooting Stuff
        	if(operatorController.getTriggerPressed(RobovikingStick.xBoxRightTrigger) && arm.isShooterEnabled()) {  // right trigger = axis 3
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
        	if (operatorController.getButtonPressedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
        		arm.executeWinderHomingSequence();
        	}
        	// abort the homing sequence if you release the start button while it's running
        	if (operatorController.getButtonReleasedOneShot(RobovikingStick.xBoxButtonStart) && !arm.isShooterEnabled()) {
        		arm.stopWindingSequence();
        	}
        	
        	// Homing sequence for arm
        	if (operatorController.getButtonPressedOneShot(RobovikingStick.xBoxButtonBack) && !arm.isArmEnabled()) {
        		arm.executeArmHomingSequence();
        	}
        	    	
        	//Controlling the rollers
        	if(operatorController.getRawButton(RobovikingStick.xBoxRightBumper)) {
        		arm.rockAndRoll(-1.0);
        		runningRollersWithArmUp = !arm.isArmDown();
        	}
        	else if(operatorController.getRawButton(RobovikingStick.xBoxLeftBumper)) {
        		arm.rockAndRoll(1.0);
        		runningRollersWithArmUp = !arm.isArmDown();
        	}
        	else {
        		arm.rockAndRoll(0);
        		runningRollersWithArmUp = false;
        	}
        	
        	//Controlling the claw (open or close)
        	arm.toggleClaw(operatorController.getToggleButton(RobovikingStick.xBoxButtonB));

        	// controller rumble
        	if (tryingToShootWithPickupClosed || runningRollersWithArmUp) {
        		operatorController.setRumble(Joystick.RumbleType.kLeftRumble, 1);
        		operatorController.setRumble(Joystick.RumbleType.kRightRumble, 1);
        	} else {
        		operatorController.setRumble(Joystick.RumbleType.kLeftRumble, 0);
        		operatorController.setRumble(Joystick.RumbleType.kRightRumble, 0);
        	}
        	
        	//Controlling the arm - check safeties: 
        	//	1) the position encoder is present and interrupt MP if not
        	arm.checkArmEncoderPresent();
        	
        	
        	if(!armInTestFlag){

        		// removed arm.isArmWaiting() check from all commands, to allow for interrupt
        		
        		// lock arm in current position (interrupting movement if necessary)
        		if(operatorController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
            		if (!armLockOneShot && arm.isArmEnabled() && armPosIndex != -1) {
            			armPosIndex = -1;
            			arm.executeArmLocking();
            			armLockOneShot = true;
            		}
            	} else {
            		armLockOneShot = false;
            	}
        		
        		// dpad manual arm positioning
        		switch (operatorController.getPOV(0)) {
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
        		
        	}
     	    	
    	}
    	
    }
    
    public double getDriverYIn() {
    	return moveVal;
    }
    
    public double getDriverXIn() {
    	return rotateVal;
    }

    public double getLeftCmd() {
    	return leftMotors.get();
    }
    
    public double getRightCmd() {
    	return rightMotors.get();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    

    	if (++counter >= 30) {
    		System.out.println("Left SP: " + leftMotors.pidLoop.getSetpoint() + " Right SP: " + rightMotors.pidLoop.getSetpoint());
    		System.out.println("Left enc: " + leftMotors.enc.getRate() + " Right enc: " + rightMotors.enc.getRate());
    		counter = 0;
    	}
    	
    	// shooter winding manual control
    	if(operatorController.getRawButton(RobovikingStick.xBoxLeftBumper)) {  // drive plunger forward (loosen)
    		arm.winderManualRun(.6);
    	}
    	else if(operatorController.getRawButton(RobovikingStick.xBoxRightBumper) && !arm.isShooterCocked()) {  // drive plunger back (tighten)
    		arm.winderManualRun(-.6);
    	}
    	else {
    		arm.winderManualRun(0);
    	}

    	// manual control of shooter latch
    	if(operatorController.getTriggerPressed(RobovikingStick.xBoxRightTrigger)) {  // right trigger = axis 3
    		arm.shoot();
    	}
    	else if(operatorController.getTriggerPressed(RobovikingStick.xBoxLeftTrigger)) {  // left trigger = axis 2
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
		soulOfFinn.arcadeDrive(0, output);
	}
	
	int counter = 0, msgCount = 0;
    public void consoleMessage() {
    	
    	if(++counter >= 50){
    		msgCount += 1;
    		System.out.println(msgCount + ": Shooter Enabled: " + arm.isShooterEnabled() + "  Shooter Cocked: " + arm.isShooterCocked() 
    							+ " Shooter Eye: " + arm.getShooterEye());
    		System.out.println(msgCount + ": Arm Eye: " + arm.getArmLimiter() + " Arm Enabled: " + arm.isArmEnabled()
    							+ " Arm Down: " + arm.isArmDown());
    		
    		SmartDashboard.putNumber("Arm Position", arm.getArmPosition());
    		counter = 0;
    	}    	
    }
    
}
