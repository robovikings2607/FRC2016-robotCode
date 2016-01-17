package edu.archwood.frc2607;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;

public class ProtoBot extends IterativeRobot {
	
	TalonPair left , right ;
	Solenoid airBender ;
	RobotDrive robotDrive ;
	RobovikingStick controller ;

	public void disabledInit() {
		
	}

	public void disabledPeriodic() {
		
	}

	public void robotInit() {
		
		left = new TalonPair(1 , 2);
		right = new TalonPair(3 , 4);
		
		airBender = new Solenoid(5);
		robotDrive = new RobotDrive(left , right);
		
		controller = new RobovikingStick(0);
		
	}

	public void teleopInit() {
		
	}

	public void teleopPeriodic() {
		
		robotDrive.arcadeDrive( -controller.getY() , -controller.getRawAxis(4) );
		
	}



}
