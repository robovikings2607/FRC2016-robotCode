package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PIDController;

/**
 * SpeedController for a 3-CIM Shifting gear box. One motor
 * spins in the opposite direction of the other Two motors due
 * to the physical design.
 * 
 * @author David
 */
public class Transmission implements SpeedController {
	
	CANTalon motor1 , motor2 , motor3;
	
	SmoothedEncoder enc;
	PIDController pidLoop;
	
	private boolean encodersFlag = false;
	private boolean invertedFlag = false;
	private int brakePulseTick = 0;
	private boolean enableBrakeMode = false;
	
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
			enc = new SmoothedEncoder(deviceID[3] , deviceID[4] , true , Encoder.EncodingType.k1X);
			//pidLoop = new PIDController() 
		}
		
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
		if(!encodersFlag) {
			motor2.set(s);
		}
		else {
			
		}
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
		
	}

}
