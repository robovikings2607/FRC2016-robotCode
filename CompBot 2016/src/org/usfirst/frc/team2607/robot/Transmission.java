package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;

public class Transmission implements SpeedController {
	
	CANTalon motor1 , motor2 , motor3;
	Solenoid solenoid;
	
	SmoothedEncoder enc;
	PIDController pidLoop;
	
	private boolean encodersFlag = false;
	private boolean invertedFlag = false;
	
	public Transmission( int deviceID , boolean useEncoders ) {
		
		encodersFlag = useEncoders;
		
		motor1 = new CANTalon(deviceID);
		motor2 = new CANTalon(deviceID + 1);
		motor3 = new CANTalon(deviceID + 2);
		
		motor1.enableBrakeMode(true);
		motor2.enableBrakeMode(true);
		motor3.enableBrakeMode(true);
		
		enc = new SmoothedEncoder(0 , 1 , true , Encoder.EncodingType.k1X);
		
		solenoid = new Solenoid(0);
	}

	@Override
	public void pidWrite(double output) {
	}

	@Override
	public double get() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void set(double speed, byte syncGroup) {
		
	}

	@Override
	public void set(double s) {
		if(invertedFlag) {
			s = -s;
		}
		if(!encodersFlag) {
			motor1.set(s);
			motor2.set(s);
			motor3.set(s);
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
}
