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
	public static double[] armPositions = 
										//{0.0, -40.0, -44.0, -50.0};
										//{0.0, -40.0, -54.0, -66.535};
										//{0.0, -42.5, -54.0, -66.535, -12.0};
										//{0.0, -43.25, -54.0, -66.535, -12.0};
										{0.0, -43.25, -54.0, -66.535, -4.6};

	public static int[] leftDeviceIDs = { leftMotor1 , leftMotor2 , leftMotor3 , leftEncChannelA , leftEncChannelB };
	public static int[] rightDeviceIDs = { rightMotor1 , rightMotor2 , rightMotor3 , rightEncChannelA , rightEncChannelB };

	public static double getArmAngle (double targetAngleInFOV){
//		targetAngleInFOV += (4);
/* from MAR Champs....not enough data points
		return  .00014735 * Math.pow(targetAngleInFOV, 3) + 
							 -.0282245209 * Math.pow(targetAngleInFOV, 2) + 
							 1.051732632 * targetAngleInFOV + 
							 -52.64559001  + 0.3 ; // + Fudge factor
*/
/* from Westtown....
							 .0005909 * Math.pow(targetAngleInFOV, 3) + 
							 -.07626081 * Math.pow(targetAngleInFOV, 2) +
							 2.661478844 * targetAngleInFOV + 
							 -68.3454292; 
*/
/* from data collected on 11-Jun 
		// -0.000866964 x^3+0.104158 x^2-4.63308 x+26.3652
		return  -0.000866964 * Math.pow(targetAngleInFOV, 3) + 
				 0.104158 * Math.pow(targetAngleInFOV, 2) + 
				 -4.63308 * targetAngleInFOV + 
				 26.3652;
*/
/* from edited data points*/
		//-0.00123187 x^3+0.14784 x^2-6.22975 x+42.9779
		return  -0.00123187 * Math.pow(targetAngleInFOV, 3) + 
				0.14784 * Math.pow(targetAngleInFOV, 2) + 
				-6.22975 * targetAngleInFOV + 
				 42.9779;
	}
		
}
