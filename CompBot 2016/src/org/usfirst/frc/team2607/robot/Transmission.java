package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;

public class Transmission implements SpeedController {
	
	CANTalon motor1 , motor2 , motor3;
	Solenoid shifter;
	
	SmoothedEncoder enc;
	PIDController pidLoop;
	
	private boolean encodersFlag = false;
	private boolean invertedFlag = false;
	
	/**
	 * 
	 * @param deviceID
	 * @param useEncoders
	 */
	public Transmission( int deviceID , boolean useEncoders ) {
		
		encodersFlag = useEncoders;
		invertedFlag = false;
		
		motor1 = new CANTalon(deviceID);
		motor2 = new CANTalon(deviceID + 1);
		motor3 = new CANTalon(deviceID + 2);
		
		motor3.changeControlMode(TalonControlMode.Follower);
		motor3.set(deviceID + 1);
		
		motor1.enableBrakeMode(false);
		motor2.enableBrakeMode(false);
		motor3.enableBrakeMode(false);
		
		enc = new SmoothedEncoder(0 , 1 , true , Encoder.EncodingType.k1X);
		shifter = new Solenoid(0);
		
		shifter.set(false);
	}

	@Override
	public double get() {
		if(!encodersFlag){
			return motor1.get();
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
			motor1.set(-s);
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

}
