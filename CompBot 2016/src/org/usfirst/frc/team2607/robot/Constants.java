package org.usfirst.frc.team2607.robot;

public class Constants {
	
	private static int 
				leftMotor1 = 1	,
				leftMotor2 = 2	,
				leftMotor3 = 3	,
				
				leftEncChannelA = 0	,
				leftEncChannelB = 1	;
	
	private static int
				rightMotor1 = 6	,
				rightMotor2 = 4	,
				rightMotor3 = 5	,
	
				rightEncChannelA = 2 ,
				rightEncChannelB = 3 ;
	
	public static int
				shifter = 0;
	
	public static int
				armMotor = 8	,
				puncherMotor = 7,
				rollersMotor = 9,
				
				puncherLock = 2	,
				clawOpener = 1	;
	
	public static int
				dControllerPort = 0	,
				oControllerPort = 1	;
	
	
	public static int[] leftDeviceIDs = { leftMotor1 , leftMotor2 , leftMotor3 , leftEncChannelA , leftEncChannelB };
	public static int[] rightDeviceIDs = { rightMotor1 , rightMotor2 , rightMotor3 , rightEncChannelA , rightEncChannelB };

}
