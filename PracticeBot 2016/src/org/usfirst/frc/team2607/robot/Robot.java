
package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	Transmission leftMotors , rightMotors;
	RobovikingStick bearTech;
	RobotDrive mindOfFinn;
	PowerLogger loggerTron;
	double moveVal , rotateVal;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	rightMotors = new Transmission(1,2 , false);
    	leftMotors = new Transmission(3,4 , false);
    	bearTech = new RobovikingStick(0);
    	mindOfFinn = new RobotDrive(leftMotors , rightMotors);
    	loggerTron = null;
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
		loggerTron = new PowerLogger(this);
		loggerTron.startLogging();
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

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	moveVal = ( bearTech.getRawAxisWithDeadzone(RobovikingStick.xBoxLeftStickY) );
    	rotateVal =  ( bearTech.getRawAxisWithDeadzone(RobovikingStick.xBoxRightStickX) );
    	
    	mindOfFinn.arcadeDrive( moveVal , rotateVal);
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
    
}
