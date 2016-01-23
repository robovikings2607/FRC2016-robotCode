package edu.archwood.frc2607;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class ProtoBot extends IterativeRobot {
	
	TalonPair left , right ;
	
	RobotDrive robotDrive ;
	RobovikingStick controller ;
	Talon shooterMotor ;
	Compressor compressor ;
	Solenoid shooterRelease ;
	
	public void disabledInit() {
		
	}

	public void disabledPeriodic() {
		
	}

	public void robotInit() {
		
		left = new TalonPair(1 , 2);
		right = new TalonPair(3 , 4);
		shooterMotor = new Talon(5);
		robotDrive = new RobotDrive(left , right);
		compressor = new Compressor(1, 1); //Check if parameters match
		shooterRelease = new Solenoid(5);
		controller = new RobovikingStick(1);
		
		compressor.start();
		
	}

	public void teleopInit() {
		
	}

	public void teleopPeriodic() {
		
		robotDrive.arcadeDrive( -controller.getY() , controller.getRawAxis(5) );
		
		controlShooter();
	}
	
	public void controlShooter(){
		
		if(controller.getRawButton(5)){
			shooterMotor.set(0.3);
		} else if(controller.getRawButton(6)){
			shooterMotor.set(-0.3);
		} else {
			shooterMotor.set(0);
		}
		
		if(controller.getTriggerPressed(1)){
			shooterRelease.set(true);
		} else {
			shooterRelease.set(false);
		}
	}
	
}
