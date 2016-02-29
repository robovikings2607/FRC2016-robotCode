package org.usfirst.frc.team2607.robot;

import java.util.TimerTask;

import com.team254.lib.trajectory.Trajectory.Segment;

import java.util.ArrayDeque;
import java.util.Queue;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.BoundaryException;

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes care of
 * the integral calculations, as well as writing the given PIDOutput
 */
public class RobovikingModPIDController implements LiveWindowSendable {

  public static final double kDefaultPeriod = .05;
  public static final boolean kTurnLeft = false;
  public static final boolean kTurnRight = true;
  private static int instances = 0;
  private double m_P; // factor for "proportional" control
  private double m_I; // factor for "integral" control
  private double m_D; // factor for "derivative" control
  private double m_V; // factor for velocity feedforward term
  private double m_A; // factor for acceleration feedforward term
  private double m_Turn; //factor for turning feedback control
  private double m_maximumOutput = 1.0; // |maximum output|
  private double m_minimumOutput = -1.0; // |minimum output|
  private double m_maximumInput_pos = 0.0; // maximum input - limit pos setpoint to this
  private double m_minimumInput_pos = 0.0; // minimum input - limit pos setpoint to this
  private double m_maximumInput_vel = 0.0; // maximum input - limit vel setpoint to this
  private double m_minimumInput_vel = 0.0; // minimum input - limit vel setpoint to this
  private double m_maximumInput_acc = 0.0; // maximum input - limit acc setpoint to this
  private double m_minimumInput_acc = 0.0; // minimum input - limit acc setpoint to this
  private double m_maximumInput_ang = 0.0; // maximum input - limit angle setpoint to this
  private double m_minimumInput_ang = 0.0; // minimum input - limit angle setpoint to this
  private boolean m_continuous = false; // do the endpoints wrap around? eg.
                                        // Absolute encoder
  private boolean m_turnDirection = false; //for Gyro
  									//False = Left | True = Right
  private boolean m_enabled = false; // is the pid controller enabled
  private double m_prevError = 0.0; // the prior error (used to compute
                                    // velocity)
  private double m_totalError = 0.0; // the sum of the errors for use in the
                                     // integral calc
  private Tolerance m_tolerance; // the tolerance object used to check if on
                                 // target
  private int m_bufLength = 1;
  private Queue<Double> m_buf;
  private double m_bufTotal = 0.0;
  private double m_setpoint_pos = 0.0;
  private double m_setpoint_vel = 0.0;
  private double m_setpoint_acc = 0.0;
  private double m_setpoint_ang = 0.0;
  private double m_prevSetpoint_pos = 0.0;
  private double m_error = 0.0;
  private double m_result = 0.0;
  private double m_period = kDefaultPeriod;
  protected PIDSource m_pidInput;
  protected PIDOutput m_pidOutput;
  protected Gyro m_Gyro;
  java.util.Timer m_controlLoop;
  Timer m_setpointTimer;
  private boolean m_freed = false;
  private boolean m_usingPercentTolerance;

  /**
   * Tolerance is the type of tolerance used to specify if the PID controller is
   * on target.
   *
   * The various implementations of this class such as PercentageTolerance and
   * AbsoluteTolerance specify types of tolerance specifications to use.
   */
  public interface Tolerance {
    public boolean onTarget();
  }

  /**
   * Used internally for when Tolerance hasn't been set.
   */
  public class NullTolerance implements Tolerance {
    @Override
    public boolean onTarget() {
      throw new RuntimeException("No tolerance value set when calling onTarget().");
    }
  }

  public class PercentageTolerance implements Tolerance {
    double percentage;

    PercentageTolerance(double value) {
      percentage = value;
    }

    @Override
    public boolean onTarget() {
      return isAvgErrorValid() && (Math.abs(getAvgError()) < percentage / 100 * (m_maximumInput_pos - m_minimumInput_pos));
    }
  }

  public class AbsoluteTolerance implements Tolerance {
    double value;

    AbsoluteTolerance(double value) {
      this.value = value;
    }

    @Override
    public boolean onTarget() {
      return isAvgErrorValid() && Math.abs(getAvgError()) < value;
    }
  }

  private class RobovikingPIDTask extends TimerTask {

    private RobovikingModPIDController m_controller;

    public RobovikingPIDTask(RobovikingModPIDController robovikingRobovikingModPIDController) {
      if (robovikingRobovikingModPIDController == null) {
        throw new NullPointerException("Given RobovikingModPIDController was null");
      }
      m_controller = robovikingRobovikingModPIDController;
    }

    @Override
    public void run() {
      m_controller.calculate();
    }
  }

  /**
   * Allocate a PID object with the given constants for P, I, D, and F
   *$
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   * @param Kf the feed forward term
   * @param source The PIDSource object that is used to get values
   * @param output The PIDOutput object that is set to the output percentage
   * @param period the loop time for doing calculations. This particularly
   *        effects calculations of the integral and differential terms. The
   *        default is 50ms.
   */
  public RobovikingModPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source,
      PIDOutput output, double period) {

    if (source == null) {
      throw new NullPointerException("Null PIDSource was given");
    }
    if (output == null) {
      throw new NullPointerException("Null PIDOutput was given");
    }

    m_controlLoop = new java.util.Timer();
    m_setpointTimer = new Timer();
    m_setpointTimer.start();

    m_P = Kp;
    m_I = Ki;
    m_D = Kd;
    m_V = Kf;

    m_pidInput = source;
    m_pidOutput = output;
    m_period = period;

    m_controlLoop.schedule(new RobovikingPIDTask(this), 0L, (long) (m_period * 1000));

    instances++;
    HLUsageReporting.reportPIDController(instances);
    m_tolerance = new NullTolerance();

    m_buf = new ArrayDeque<Double>(m_bufLength+1);
  }

  /**
   * Allocate a PID object with the given constants for P, I, D and period
   *$
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   * @param source the PIDSource object that is used to get values
   * @param output the PIDOutput object that is set to the output percentage
   * @param period the loop time for doing calculations. This particularly
   *        effects calculations of the integral and differential terms. The
   *        default is 50ms.
   */
  public RobovikingModPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output,
      double period) {
    this(Kp, Ki, Kd, 0.0, source, output, period);
  }

  /**
   * Allocate a PID object with the given constants for P, I, D, using a 50ms
   * period.
   *$
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   * @param source The PIDSource object that is used to get values
   * @param output The PIDOutput object that is set to the output percentage
   */
  public RobovikingModPIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
    this(Kp, Ki, Kd, source, output, kDefaultPeriod);
  }

  /**
   * Allocate a PID object with the given constants for P, I, D, using a 50ms
   * period.
   *$
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   * @param Kf the feed forward term
   * @param source The PIDSource object that is used to get values
   * @param output The PIDOutput object that is set to the output percentage
   */
  public RobovikingModPIDController(double Kp, double Ki, double Kd, double Kf, PIDSource source,
      PIDOutput output) {
    this(Kp, Ki, Kd, Kf, source, output, kDefaultPeriod);
  }
  
  /**
   * Allocate a PID object with the given constants for P, I, D, using a 50ms
   * period.
   *$
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   * @param Kf the feed forward term
   * @param source The PIDSource object that is used to get values
   * @param output The PIDOutput object that is set to the output percentage
   */
  public RobovikingModPIDController(double Kp, double Ki, double Kd, double Kf, double Ka, double KTurn, PIDSource source,
      PIDOutput output, Gyro g) {
    this(Kp, Ki, Kd, Kf, source, output, kDefaultPeriod);
    m_A = Ka;
    m_Turn = KTurn;
    m_Gyro = g;
  }

  /**
   * Free the PID object
   */
  public void free() {
    m_controlLoop.cancel();
    synchronized (this) {
      m_freed = true;
      m_pidOutput = null;
      m_pidInput = null;
      m_controlLoop = null;
    }
    if (this.table != null)
      table.removeTableListener(listener);
  }

  /**
   * Read the input, calculate the output accordingly, and write to the output.
   * This should only be called by the PIDTask and is created during
   * initialization.
   */
  protected void calculate() {
    boolean enabled;
    PIDSource pidInput;

    synchronized (this) {
      if (m_pidInput == null) {
        return;
      }
      if (m_pidOutput == null) {
        return;
      }
      enabled = m_enabled; // take snapshot of these values...
      pidInput = m_pidInput;
    }

    if (enabled) {
      double input;
      double result;
      PIDOutput pidOutput = null;
      synchronized (this) {
        input = pidInput.pidGet();
      }
      synchronized (this) {
        m_error = m_setpoint_pos - input;
        if (m_continuous) {
          if (Math.abs(m_error) > (m_maximumInput_pos - m_minimumInput_pos) / 2) {
            if (m_error > 0) {
              m_error = m_error - m_maximumInput_pos + m_minimumInput_pos;
            } else {
              m_error = m_error + m_maximumInput_pos - m_minimumInput_pos;
            }
          }
        }

        if (m_pidInput.getPIDSourceType().equals(PIDSourceType.kRate)) {
        	//System.out.println("RATE MODE");
          if (m_P != 0) {
            double potentialPGain = (m_totalError + m_error) * m_P;
            if (potentialPGain < m_maximumOutput) {
              if (potentialPGain > m_minimumOutput) {
                m_totalError += m_error;
              } else {
                m_totalError = m_minimumOutput / m_P;
              }
            } else {
              m_totalError = m_maximumOutput / m_P;
            }

            m_result = m_P * m_totalError + m_D * m_error +
                       calculateFeedForward() + calculateTurn();
          }
        }
        else {
        	//System.out.println("DISP MODE");
          if (m_I != 0) {
            double potentialIGain = (m_totalError + m_error) * m_I;
            if (potentialIGain < m_maximumOutput) {
              if (potentialIGain > m_minimumOutput) {
                m_totalError += m_error;
              } else {
                m_totalError = m_minimumOutput / m_I;
              }
            } else {
              m_totalError = m_maximumOutput / m_I;
            }
          }

          m_result = m_P * m_error + m_I * m_totalError +
                     m_D * (m_error - m_prevError) + calculateFeedForward() + calculateTurn();
        }
        m_prevError = m_error;

        if (m_result > m_maximumOutput) {
          m_result = m_maximumOutput;
        } else if (m_result < m_minimumOutput) {
          m_result = m_minimumOutput;
        }
        pidOutput = m_pidOutput;
        result = m_result;

        // Update the buffer.
        m_buf.add(m_error);
        m_bufTotal += m_error;
        // Remove old elements when the buffer is full.
        if (m_buf.size() > m_bufLength) {
          m_bufTotal -= m_buf.remove();
        }
      }

      pidOutput.pidWrite(result);
    }
  }

/**
   * Calculate the feed forward term
   *
   * Both of the provided feed forward calculations are velocity feed forwards.
   * If a different feed forward calculation is desired, the user can override
   * this function and provide his or her own. This function  does no
   * synchronization because the RobovikingModPIDController class only calls it in
   * synchronized code, so be careful if calling it oneself.
   *
   * If a velocity PID controller is being used, the F term should be set to 1
   * over the maximum setpoint for the output. If a position PID controller is
   * being used, the F term should be set to 1 over the maximum speed for the
   * output measured in setpoint units per this controller's update period (see
   * the default period in this class's constructor).
   */
  protected double calculateFeedForward() {
      return m_V * m_setpoint_vel + m_A * m_setpoint_acc;
  }
  
  /**
   * Calculate the heading correction term
   *
   * This term adds a correction factor to the PID equation to correct for the robot's
   * heading versus the requested heading. m_turnDirection is used to set the
   * polarity on the returned value (add to one robot side, subtract from the other
   * in order to cause the robot to turn).
   */
  protected double calculateTurn() {
	if (m_Gyro == null) return 0.0;
	double angError = m_setpoint_ang - (m_Gyro.getAngle() * (Math.PI / 180.0)); //Profile setpoint is in radians
			
	return m_Turn * angError * (m_turnDirection ? 1 : -1);
  }

  /**
   * Set the PID Controller gain parameters. Set the proportional, integral, and
   * differential coefficients.
   *$
   * @param p Proportional coefficient
   * @param i Integral coefficient
   * @param d Differential coefficient
   */
  public synchronized void setPID(double p, double i, double d) {
    m_P = p;
    m_I = i;
    m_D = d;

    if (table != null) {
      table.putNumber("p", p);
      table.putNumber("i", i);
      table.putNumber("d", d);
    }
  }

  /**
   * Set the PID Controller gain parameters. Set the proportional, integral, and
   * differential coefficients.
   *$
   * @param p Proportional coefficient
   * @param i Integral coefficient
   * @param d Differential coefficient
   * @param f Feed forward coefficient
   */
  public synchronized void setPID(double p, double i, double d, double f) {
    m_P = p;
    m_I = i;
    m_D = d;
    m_V = f;

    if (table != null) {
      table.putNumber("p", p);
      table.putNumber("i", i);
      table.putNumber("d", d);
      table.putNumber("f", f);
    }
  }
  
  public synchronized void setKTurn(double kTurn){
	  m_Turn = kTurn;
  }
  
  /**
   * Set the direction that the kTurn of the pidLoop will act in. This must be opposite for
   * each side of the drive train to act property.
   *
   * @param direction Direction that kTurn acts in (False = Left | True = Right)
   */
  public synchronized void setTurnDirection(boolean direction){
	  m_turnDirection = direction;
  }

  /**
   * Get the Proportional coefficient
   *$
   * @return proportional coefficient
   */
  public synchronized double getP() {
    return m_P;
  }

  /**
   * Get the Integral coefficient
   *$
   * @return integral coefficient
   */
  public synchronized double getI() {
    return m_I;
  }

  /**
   * Get the Differential coefficient
   *$
   * @return differential coefficient
   */
  public synchronized double getD() {
    return m_D;
  }

  /**
   * Get the Feed forward velocity coefficient
   *$
   * @return feed forward coefficient
   */
  public synchronized double getV() {
    return m_V;
  }
  
  /**
   * Get the Feed forward acceleration coefficient
   *$
   * @return feed forward coefficient
   */
  public synchronized double getA() {
    return m_A;
  }

  /**
   * Return the current PID result This is always centered on zero and
   * constrained the the max and min outs
   *$
   * @return the latest calculated output
   */
  public synchronized double get() {
    return m_result;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then
   * using the max and min in as constraints, it considers them to be the same
   * point and automatically calculates the shortest route to the setpoint.
   *$
   * @param continuous Set to true turns on continuous, false turns off
   *        continuous
   */
  public synchronized void setContinuous(boolean continuous) {
    m_continuous = continuous;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then
   * using the max and min in as constraints, it considers them to be the same
   * point and automatically calculates the shortest route to the setpoint.
   */
  public synchronized void setContinuous() {
    this.setContinuous(true);
  }

  /**
   * Sets the maximum and minimum values expected from the input and setpoint.
   *
   * @param minimumInput the minimum value expected from the input
   * @param maximumInput the maximum value expected from the input
   */
  public synchronized void setPositionInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumInput_pos = minimumInput;
    m_maximumInput_pos = maximumInput;
    setSetpoint(m_setpoint_pos, m_setpoint_vel, m_setpoint_acc, m_setpoint_ang);
  }
  
  /**
   * Sets the maximum and minimum values expected from the input and setpoint.
   *
   * @param minimumInput the minimum value expected from the input
   * @param maximumInput the maximum value expected from the input
   */
  public synchronized void setVelocityInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumInput_vel = minimumInput;
    m_maximumInput_vel = maximumInput;
    setSetpoint(m_setpoint_pos, m_setpoint_vel, m_setpoint_acc, m_setpoint_ang);
  }
  
  /**
   * Sets the maximum and minimum values expected from the input and setpoint.
   *
   * @param minimumInput the minimum value expected from the input
   * @param maximumInput the maximum value expected from the input
   */
  public synchronized void setAccelerationInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumInput_acc = minimumInput;
    m_maximumInput_acc = maximumInput;
    setSetpoint(m_setpoint_pos, m_setpoint_vel, m_setpoint_acc, m_setpoint_ang);
  }
  
  /**
   * Sets the maximum and minimum values expected from the input and setpoint.
   *
   * @param minimumInput the minimum value expected from the input
   * @param maximumInput the maximum value expected from the input
   */
  public synchronized void setHeadingInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumInput_ang = minimumInput;
    m_maximumInput_ang = maximumInput;
    setSetpoint(m_setpoint_pos, m_setpoint_vel, m_setpoint_acc, m_setpoint_ang);
  }

  /**
   * Sets the minimum and maximum values to write.
   *
   * @param minimumOutput the minimum percentage to write to the output
   * @param maximumOutput the maximum percentage to write to the output
   */
  public synchronized void setOutputRange(double minimumOutput, double maximumOutput) {
    if (minimumOutput > maximumOutput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumOutput = minimumOutput;
    m_maximumOutput = maximumOutput;
  }

  /**
   * Set the setpoint for the RobovikingModPIDController
   * Clears the queue for GetAvgError().
   *$
   * @param setpoint_pos the desired setpoint
   */
  public synchronized void setSetpoint(double setpoint_pos, double setpoint_vel, double setpoint_acc, double setpoint_ang) {
    if (m_maximumInput_pos > m_minimumInput_pos) {
      if (setpoint_pos > m_maximumInput_pos) {
        m_setpoint_pos = m_maximumInput_pos;
      } else if (setpoint_pos < m_minimumInput_pos) {
        m_setpoint_pos = m_minimumInput_pos;
      } else {
        m_setpoint_pos = setpoint_pos;
      }
    } else {
      m_setpoint_pos = setpoint_pos;
    }
    
    if (m_maximumInput_vel > m_minimumInput_vel) {
        if (setpoint_vel > m_maximumInput_vel) {
          m_setpoint_vel = m_maximumInput_vel;
        } else if (setpoint_vel < m_minimumInput_vel) {
          m_setpoint_vel = m_minimumInput_vel;
        } else {
          m_setpoint_vel = setpoint_vel;
        }
      } else {
        m_setpoint_vel = setpoint_vel;
      }

    if (m_maximumInput_acc > m_minimumInput_acc) {
        if (setpoint_acc > m_maximumInput_acc) {
          m_setpoint_acc = m_maximumInput_acc;
        } else if (setpoint_acc < m_minimumInput_acc) {
          m_setpoint_acc = m_minimumInput_acc;
        } else {
          m_setpoint_acc = setpoint_acc;
        }
      } else {
        m_setpoint_acc = setpoint_acc;
      }
    
    if (m_maximumInput_ang > m_minimumInput_ang) {
        if (setpoint_ang > m_maximumInput_ang) {
          m_setpoint_ang = m_maximumInput_ang;
        } else if (setpoint_ang < m_minimumInput_ang) {
          m_setpoint_ang = m_minimumInput_ang;
        } else {
          m_setpoint_ang = setpoint_ang;
        }
      } else {
        m_setpoint_ang = setpoint_ang;
      }

    m_buf.clear();

    if (table != null)
      table.putNumber("setpoint", m_setpoint_pos);
  }
  
  public void setSetpoint(Segment s){
	  setSetpoint(s.pos, s.vel, s.acc, s.heading);
  }

  /**
   * Returns the current setpoint of the RobovikingModPIDController
   *$
   * @return the current setpoint
   */
  public synchronized double[] getSetpoint() {
    return new double[] {m_setpoint_pos, m_setpoint_vel, m_setpoint_acc, m_setpoint_ang};
  }

  /**
   * Returns the change in setpoint over time of the RobovikingModPIDController
   *$
   * @return the change in setpoint over time
   */
  public synchronized double getDeltaSetpoint() {
    return (m_setpoint_pos - m_prevSetpoint_pos) / m_setpointTimer.get();
  }

  /**
   * Returns the current difference of the input from the setpoint
   *$
   * @return the current error
   */
  public synchronized double getError() {
    // return m_error;
    return getSetpoint()[0] - m_pidInput.pidGet();
  }

  /**
   * Sets what type of input the PID controller will use
   *$
   * @param pidSource the type of input
   */
  void setPIDSourceType(PIDSourceType pidSource) {
    m_pidInput.setPIDSourceType(pidSource);
  }

  /**
   * Returns the type of input the PID controller is using
   *$
   * @return the PID controller input type
   */
  PIDSourceType getPIDSourceType() {
    return m_pidInput.getPIDSourceType();
  }

  /**
   * Returns the current difference of the error over the past few iterations.
   * You can specify the number of iterations to average with
   * setToleranceBuffer() (defaults to 1). getAvgError() is used for the
   * onTarget() function.
   *$
   * @return the current average of the error
   */
  public synchronized double getAvgError() {
    double avgError = 0;
    // Don't divide by zero.
    if (m_buf.size() != 0) avgError = m_bufTotal / m_buf.size();
    return avgError;
  }

  /**
   * Returns whether or not any values have been collected. If no values
   * have been collected, getAvgError is 0, which is invalid.
   *
   * @return True if {@link #getAvgError()} is currently valid.
   */
  private synchronized boolean isAvgErrorValid() {
    return m_buf.size() != 0;
  }

  /**
   * Set the percentage error which is considered tolerable for use with
   * OnTarget. (Input of 15.0 = 15 percent)
   *$
   * @param percent error which is tolerable
   * @deprecated Use {@link #setPercentTolerance(double)} or
   *             {@link #setAbsoluteTolerance(double)} instead.
   */
  @Deprecated
  public synchronized void setTolerance(double percent) {
    m_tolerance = new PercentageTolerance(percent);
  }

  /**
   * Set the PID tolerance using a Tolerance object. Tolerance can be specified
   * as a percentage of the range or as an absolute value. The Tolerance object
   * encapsulates those options in an object. Use it by creating the type of
   * tolerance that you want to use: setTolerance(new
   * RobovikingModPIDController.AbsoluteTolerance(0.1))
   *$
   * @param tolerance a tolerance object of the right type, e.g.
   *        PercentTolerance or AbsoluteTolerance
   */
  public void setTolerance(Tolerance tolerance) {
    m_tolerance = tolerance;
  }

  /**
   * Set the absolute error which is considered tolerable for use with OnTarget.
   *$
   * @param absvalue absolute error which is tolerable in the units of the input
   *        object
   */
  public synchronized void setAbsoluteTolerance(double absvalue) {
    m_tolerance = new AbsoluteTolerance(absvalue);
  }

  /**
   * Set the percentage error which is considered tolerable for use with
   * OnTarget. (Input of 15.0 = 15 percent)
   *$
   * @param percentage percent error which is tolerable
   */
  public synchronized void setPercentTolerance(double percentage) {
    m_tolerance = new PercentageTolerance(percentage);
  }

  /**
   * Set the number of previous error samples to average for tolerancing. When
   * determining whether a mechanism is on target, the user may want to use a
   * rolling average of previous measurements instead of a precise position or
   * velocity. This is useful for noisy sensors which return a few erroneous
   * measurements when the mechanism is on target. However, the mechanism will
   * not register as on target for at least the specified bufLength cycles.
   * @param bufLength Number of previous cycles to average.
   */
  public synchronized void setToleranceBuffer(int bufLength) {
    m_bufLength = bufLength;

    // Cut the existing buffer down to size if needed.
    while (m_buf.size() > bufLength) {
      m_bufTotal -= m_buf.remove();
    }
  }

  /**
   * Return true if the error is within the percentage of the total input range,
   * determined by setTolerance. This assumes that the maximum and minimum input
   * were set using setInput.
   *$
   * @return true if the error is less than the tolerance
   */
  public synchronized boolean onTarget() {
    return m_tolerance.onTarget();
  }

  /**
   * Begin running the RobovikingModPIDController
   */
  public synchronized void enable() {
    m_enabled = true;

    if (table != null) {
      table.putBoolean("enabled", true);
    }
  }

  /**
   * Stop running the RobovikingModPIDController, this sets the output to zero before
   * stopping.
   */
  public synchronized void disable() {
    m_pidOutput.pidWrite(0);
    m_enabled = false;

    if (table != null) {
      table.putBoolean("enabled", false);
    }
  }

  /**
   * Return true if RobovikingModPIDController is enabled.
   *
   * @deprecated Call {@link #isEnabled()} instead.
   */
  @Deprecated
  public synchronized boolean isEnable() {
    return isEnabled();
  }

  /**
   * Return true if RobovikingModPIDController is enabled.
   */
  public boolean isEnabled() {
    return m_enabled;
  }

  /**
   * Reset the previous error,, the integral term, and disable the controller.
   */
  public synchronized void reset() {
    disable();
    m_prevError = 0;
    m_totalError = 0;
    m_result = 0;
  }

  @Override
  public String getSmartDashboardType() {
    return "RobovikingModPIDController";
  }

  private final ITableListener listener = new ITableListener() {
    @Override
    public void valueChanged(ITable table, String key, Object value, boolean isNew) {
      if (key.equals("p") || key.equals("i") || key.equals("d") || key.equals("f")) {
        if (getP() != table.getNumber("p", 0.0) || getI() != table.getNumber("i", 0.0)
            || getD() != table.getNumber("d", 0.0) || getV() != table.getNumber("f", 0.0))
          setPID(table.getNumber("p", 0.0), table.getNumber("i", 0.0), table.getNumber("d", 0.0),
              table.getNumber("f", 0.0));
      } else if (key.equals("setpoint")) {
        if (getSetpoint()[0] != ((Double) value).doubleValue()){}
          //setSetpoint(((Double) value).doubleValue());
      } else if (key.equals("enabled")) {
        if (isEnable() != ((Boolean) value).booleanValue()) {
          if (((Boolean) value).booleanValue()) {
            enable();
          } else {
            disable();
          }
        }
      }
    }
  };
  private ITable table;

  @Override
  public void initTable(ITable table) {
    if (this.table != null)
      this.table.removeTableListener(listener);
    this.table = table;
    if (table != null) {
      table.putNumber("p", getP());
      table.putNumber("i", getI());
      table.putNumber("d", getD());
      table.putNumber("f", getV());
      table.putNumber("setpoint", getSetpoint()[0]);
      table.putBoolean("enabled", isEnable());
      table.addTableListener(listener, false);
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public ITable getTable() {
    return table;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void updateTable() {}

  /**
   * {@inheritDoc}
   */
  @Override
  public void startLiveWindowMode() {
    disable();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void stopLiveWindowMode() {}
}
