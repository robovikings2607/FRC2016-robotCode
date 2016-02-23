package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
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
	
	Encoder enc;
	RobovikingPIDController pidLoop;
	
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
	public Transmission( int[] deviceID , boolean useEncoders ) {
		
		encodersFlag = useEncoders;
		invertedFlag = false;
		
		motor1 = new CANTalon(deviceID[0]);
		motor2 = new CANTalon(deviceID[1]);
		motor3 = new CANTalon(deviceID[2]);
		
		motor3.changeControlMode(TalonControlMode.Follower);
		motor3.set(deviceID[1]);
		motor1.changeControlMode(TalonControlMode.Follower);
		motor1.set(deviceID[1]);
		motor1.reverseOutput(true);
		
		motor1.enableBrakeMode(false);
		motor2.enableBrakeMode(false);
		motor3.enableBrakeMode(false);
		
		if(encodersFlag) {
			enc = new SmoothedEncoder(deviceID[3] , deviceID[4] , false, Encoder.EncodingType.k1X);
			enc.setPIDSourceType(PIDSourceType.kRate);
			enc.reset();
			enc.setDistancePerPulse(0.00766990393942820614859043794746);	// ((Wheel Di. (in) / 12) * pi) / enc counts
			pidLoop = new RobovikingPIDController(0.0, 0.0, 0.0, .067, enc, motor2); 
			pidLoop.setInputRange(-15.0, 15.0);
			pidLoop.setOutputRange(-1.0, 1.0);
			pidLoop.disable();
			//pidLoop.setAbsoluteTolerance(.5);
		}
		
	}

	public void disableVelPID() {
		if (!encodersFlag) return;
		pidLoop.disable();
		motor2.set(0);
	}
	
	public void enableVelPID() {
		if (!encodersFlag) return;
		pidLoop.enable();
				
	}
	
	public void setVelSP(double speed) {		// passed in as fps, from motion profiler
		if (!encodersFlag) return;
		pidLoop.setSetpoint(speed);
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
		motor2.set(s);
		
		if (Math.abs(s) <= .1) {
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
		System.out.println("WARNING:  Transmission.pidWrite() called;  should not be being called by anything");
	}

}
