package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;

public class Transmission implements SpeedController {
	
	Talon motor1 , motor2;
	Solenoid shifter;
	
	SmoothedEncoder enc;
	PIDController pidLoop;
	
	private boolean encodersFlag = false;
	private boolean invertedFlag = false;
	
	/**
	 * New SpeedController that models three motors turning the same axle. The first motor will run opposite to
	 * the other motors, but at the same speed.
	 * @param deviceID - PWM port of the first Talon. The other Talons are set to the next two ports.
	 * @param useEncoders - whether you want to use encoders or not.
	 */
//	public Transmission( int deviceID , boolean useEncoders ) {
//		init(deviceID, deviceID + 1, useEncoders);
//	}

	public Transmission(int motor1DeviceID, int motor2DeviceID, 
						 boolean useEncoders) {
		init(motor1DeviceID, motor2DeviceID, useEncoders);
	}
	
	private void init(int motor1DeviceID, int motor2DeviceID, 
						 boolean useEncoders) {
		
		encodersFlag = useEncoders;
		invertedFlag = false;
		
		motor1 = new Talon(motor1DeviceID);
		motor2 = new Talon(motor2DeviceID);

		if (useEncoders)
			enc = new SmoothedEncoder(0 , 1 , true , Encoder.EncodingType.k1X);

//		shifter = new Solenoid(0);
		
//		shifter.set(false);

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
			motor1.set(s);
			motor2.set(s);
		}
		else {
			
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

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		
	}

}
