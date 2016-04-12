package org.usfirst.frc.team2607.robot;

public class Constants {

	public static int //Solenoid (PCM) Ports
		shifter = 0,
		clawSolenoid = 1,
		plungerHook = 3,
		brakeSolenoid = 2;
	
	public static int //Photoeye (DIO)Ports
		plungerLimiter = 0,
		armLimiter = 1;
	
	public static int //Motor (PWM) Ports
		rightMotor1 = 1,
		rightMotor2 = 2,
		leftMotor1 = 3,
		leftMotor2 = 4,
		
		rollersMotor = 5,
		puncherMotor = 7,
		armMotor = 10;
	
	public static int //Gamepad Ports
		driveController = 0,
		operatorController = 1;
	
	public static double[] armPositions = 
			//{0.0, -40.0, -44.0, -50.0};
			{0.0, -41, -54.0, -66.535, -12};
}
