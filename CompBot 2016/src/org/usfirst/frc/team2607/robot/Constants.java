package org.usfirst.frc.team2607.robot;

public class Constants {
	
	private static int 
				leftMotor1 = 1	,
				leftMotor2 = 2	,
				leftMotor3 = 3	,
				
				leftEncChannelA = 7	,
				leftEncChannelB = 6	;
				
	private static int
				rightMotor1 = 6	,
				rightMotor2 = 4	,
				rightMotor3 = 5	,
	
				rightEncChannelA = 5 ,
				rightEncChannelB = 4 ;
	
	public static int
				shifter = 0;
	
	public static int
				// TalonSRX's
				armMotor = 8	,
				puncherMotor = 10,	// was 7,
				rollersMotor = 9,
				
				// Solenoids
				armLocker= 3,
				puncherLock = 2	,
				clawOpener = 1	,
				
				// PhotoEye's
				armLimiter = 8	,
				shooterCocked = 9;
	
	public static int
				dControllerPort = 0	,
				oControllerPort = 1	;

	// armPositions are in CIM motor rotations, not in arm degrees
	// rotations = (350.0 * degrees) / 360.0
	// 0.0 is arm down;  all other positions are negative, since reverse motor drives the arm up
	public static double[] armPositions = {0.0, -11.27, -43.0, -45.69, -68.535};
	
	
	public static int[] leftDeviceIDs = { leftMotor1 , leftMotor2 , leftMotor3 , leftEncChannelA , leftEncChannelB };
	public static int[] rightDeviceIDs = { rightMotor1 , rightMotor2 , rightMotor3 , rightEncChannelA , rightEncChannelB };

		
}
