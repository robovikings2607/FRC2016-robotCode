package edu.archwood.frc2607;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

/**
 * This code was borrowed from our 2014 robot code
 *
 */
public class TalonPair implements SpeedController {
	private Talon motor1 , motor2 ;
	
	 public TalonPair(int motor1PWM, int motor2PWM) {
	        motor1 = new Talon(motor1PWM);
	        if (motor2PWM > 0) {
	            motor2 = new Talon(motor2PWM);
	        } else {
	            motor2 = null;
	        }
	    }

		public void disable() {
			// TODO Auto-generated method stub
			
		}

		public double get() {
			// TODO Auto-generated method stub
			return 0;
		}

		
		public void set(double d) {
			if (motor2 != null) {
	            motor1.set(d);
	            motor2.set(d);
	        } else {
	            motor1.set(d);
	        }
		}

		public void set(double arg0, byte arg1) {
			// TODO Auto-generated method stub
			
		}

		public void pidWrite(double arg0) {
			// TODO Auto-generated method stub
			
		}


}
