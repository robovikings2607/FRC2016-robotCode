package org.usfirst.frc.team2607.robot;

import com.team254.lib.trajectory.Trajectory.Segment;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;


// lowGear maxRPS 2.39
// highGear maxRPS 7.16

/**
 * SpeedController for a 3-CIM Shifting gear box. One motor
 * spins in the opposite direction of the other Two motors due
 * to the physical design.
 * 
 * @author David
 */
public class Transmission implements SpeedController {
	
	CANTalon motor1 , motor2 , motor3;
	
	public Encoder enc;
	public RobovikingModPIDController pidLoop;
	public Gyro gyro;
	public PIDLogger log;
	private String name;
	
	private boolean encodersFlag = false;
	private boolean invertedFlag = false;
	private int brakePulseTick = 0;
	private boolean enableBrakeMode = false;
	private double curMaxSpeed;
	
	/**
	 * Constructor for the 'Transmission' SpeedController
	 * @param deviceID - array of the PWM port #'s
	 * @param useEncoders - whether or not encoders are used
	 */
	public Transmission( int[] deviceID , boolean useEncoders, boolean side, Gyro gyro ) {
		
		encodersFlag = useEncoders;
		invertedFlag = false;
		
		motor1 = new CANTalon(deviceID[0]);
		motor2 = new CANTalon(deviceID[1]);
		motor3 = new CANTalon(deviceID[2]);

/*		Commenting this out, as follower mode exhibits wierd behavior when used with PID Controller and/or test mode.
 * 		Need to diagnose fully in WPILib - don't know yet whether it's the PID Controller or test mode 
 * 		Specifically, the behavior is that motors 1&3 get switched out of follower mode, so only motor2 drives
 *		the reverseOutput also no longer takes effect
 *
		motor3.changeControlMode(TalonControlMode.Follower);
		motor3.set(deviceID[1]);
		motor1.changeControlMode(TalonControlMode.Follower);
		motor1.set(deviceID[1]);
*/		
	
		motor1.enableBrakeMode(false);
		motor2.enableBrakeMode(false);
		motor3.enableBrakeMode(false);
		
		if(encodersFlag) {
			// NOTE:  Encoder must be reversed for the right-side motors;  the "side" parameter of the 
			// ctor is false for left, and true for right, so use this as the "reversed" flag on the Encoder ctor
			enc = new Encoder(deviceID[3] , deviceID[4] , side, Encoder.EncodingType.k1X);
			enc.setPIDSourceType(PIDSourceType.kDisplacement);
			enc.reset();
			enc.setDistancePerPulse(0.00760);	// ((Wheel Di. (in) / 12) * pi) / enc counts
			if (side) {		// rightMotors
				pidLoop = new RobovikingModPIDController(0.5, 0.0, 0.0, 0.081, 0.03, 0.0, enc, this, gyro); // null for gyro means not used
			} else { 		// leftMotors
				pidLoop = new RobovikingModPIDController(0.5, 0.0, 0.0, 0.0823, 0.03, 0.0, enc, this, gyro); // null for gyro means not used
			}
			//0.14, 0.001, 0.0, 0.0151, 0.0022, -3.0/80.0,
			pidLoop.setTurnDirection(side);
			pidLoop.setPositionInputRange(-7000, 7000.0);
			pidLoop.setAccelerationInputRange(-20, 20);
			pidLoop.setVelocityInputRange(-15.0, 15.0);
			pidLoop.setHeadingInputRange(-360, 360);
			pidLoop.disable();
			//pidLoop.setAbsoluteTolerance(.5);
			
			log = new PIDLogger(this);
			log.start();
		
		}
		
	}

	public void disableVelPID() {
		if (!encodersFlag) return;
		pidLoop.disable();
		motor1.set(0);
		motor2.set(0);
		motor3.set(0);
	}
	
	public void enableVelPID() {
		if (!encodersFlag) return;
		pidLoop.enable();
				
	}
	
	public void setSP(double pos, double vel, double acc, double ang) {
		pidLoop.setSetpoint(pos, vel, acc, ang);
	}
	
	public void setSP(Segment s){
		pidLoop.setSetpoint(s);
	}
	
	@Override
	public double get() {
		if(!encodersFlag){
			return motor2.get();
		} else {
			return -0.0;
		}
	}

	@Override
	public void set(double speed, byte syncGroup) {
		set(speed);
	}

	@Override
	public void set(double s) {
		if(invertedFlag) {
			s = -s;
		}
/*
		if(!encodersFlag) {
			motor2.set(s);
		}
		else {
			
		}
*/
		motor1.set(-s);		
		motor2.set(s);
		motor3.set(s);
		
		if (Math.abs(s) <= .05) {
			if (++brakePulseTick >= 10) {
			enableBrakeMode = !enableBrakeMode;
			motor1.enableBrakeMode(enableBrakeMode);
			motor2.enableBrakeMode(enableBrakeMode);
			motor3.enableBrakeMode(enableBrakeMode);
			brakePulseTick = 0;
			}
		}
	}

	@Override
	public void setInverted(boolean isInverted) {
		invertedFlag = isInverted;
	}

	@Override
	public boolean getInverted() {
		return invertedFlag;
	}

	@Override
	public void disable() {
		if (!encodersFlag) return;
        enc.reset();
	}
	
	@Override
	public void pidWrite(double output) {
		set(output);
	}
	
	public void setName(String n){
		name = n;
	}
	
	public String getName(){
		return name;
	}
	
	@Override
	public String toString() {
		//System.out.println("SP POS: " + pidLoop.getSetpoint()[0] + " SP VEL: " + pidLoop.getSetpoint()[1] + 
			//	" REAL POS: " + enc.getDistance() + " REAL VEL: " + enc.getRate());
		
		return (pidLoop.getSetpoint()[0] + "," + enc.getDistance() + "," +
				pidLoop.getSetpoint()[1] + "," + enc.getRate() + "\n" +
				pidLoop.getSetpoint()[3] + "," + ((gyro == null) ? "null" : gyro.getAngle()));
	}

	@Override
	public void stopMotor() {
		//Must be implemented, for what?
	}
	
	public void resetEncoder(){
		enc.reset();
	}

}
