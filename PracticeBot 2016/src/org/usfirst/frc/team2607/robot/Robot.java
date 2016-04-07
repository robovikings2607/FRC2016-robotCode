
package org.usfirst.frc.team2607.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
	
	Transmission leftMotors , rightMotors;
	Solenoid shifter , clawinator , polarPlungerHook , shakeAndBrake ;
	Compressor compressor;
	RobovikingStick bearTech;
	RobotDrive soulOfFinn;
	PowerLogger loggerTron;
	PIDController pidController;
	AHRS navX;
	double kp , ki , kd , feedForward, turnSP;
	double moveVal , rotateVal;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	/*
    	 * claw  1
    	 * plunger 2
    	 * brake 3
    	 */
    	
    	rightMotors = new Transmission(1,2 , false);
    	leftMotors = new Transmission(3,4 , false);
    	bearTech = new RobovikingStick(0);
    	soulOfFinn = new RobotDrive(leftMotors , rightMotors);
    	
    	//compressor = new Compressor(1);
    	shifter = new Solenoid( 1 , Constants.shifter );
    	clawinator = new Solenoid( 1 , Constants.clawSolenoid);
    	polarPlungerHook = new Solenoid( 1 , Constants.plunger);
    	shakeAndBrake = new Solenoid( 1 , Constants.brake);
    	loggerTron = null;
    	
    	kp = 0.053;	// 0.053
    	ki = 0.00012;
    	kd = 0.0;
    	feedForward = .51; // 0.1
    	turnSP = 0.0;
    	navX = new AHRS(SPI.Port.kMXP);
    	navX.setPIDSourceType(PIDSourceType.kDisplacement);
    	pidController = new PIDController(kp, ki, kd, feedForward, navX, this);
    	
    	pidController.setInputRange(-180, 180);
    	pidController.setOutputRange(-1, 1);
    	pidController.setAbsoluteTolerance(0.5);
    	soulOfFinn.setSafetyEnabled(false);
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
	public void teleopInit() {
//		loggerTron = new PowerLogger(this);
//		loggerTron.startLogging();
		SmartDashboard.putNumber("turnSP", turnSP);
	}

	boolean turnOneShot = false;
    double degFromVision = 990;
    int count = 0;
    long startTime, lineupTime;
    
    /**
     * This function is called periodically during operator control
     */
    String robotInfo;
    public void teleopPeriodic() {
    	
    	//Controls Stuff
    	
    	if(bearTech.getRawButton(RobovikingStick.xBoxButtonA)) {
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
    		SmartDashboard.putBoolean("robotSaveFrames", false);
    		SmartDashboard.putString("robotInfo", "--");
    		SmartDashboard.putNumber("robotTurnSP", -999);
    		SmartDashboard.putNumber("robotTurnPV", -999);
        	//Driving Stuff
        	moveVal = ( bearTech.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
        	rotateVal =  ( bearTech.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX) );
        	
        	shifter.set(bearTech.getToggleButton(RobovikingStick.xBoxButtonRightStick));
        	soulOfFinn.arcadeDrive( moveVal , rotateVal);
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
    
    }

	@Override
	public void pidWrite(double output) {
		soulOfFinn.arcadeDrive(0, output);
	}
    
}
