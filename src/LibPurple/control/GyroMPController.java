package LibPurple.control;

import java.util.TimerTask;

import LibPurple.utils.Utils;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class GyroMPController extends MPController 
{

	/**
	 * A <b>structure</b> for storing Motion Profiling values
	 */
	public static class GyroMPValue extends MPValue
	{
		public double kGyro;
		
		/**
		 * Allocate an MPValue on the Motion Profiler
		 * 
		 * @param pid	PID value object for the position PID loop
		 * @param kv	Velocity feed-forward term
		 * @param ka	Acceleration feed-forward term
		 * @param maxV	Max velocity of the system
		 * @param maxA	Max acceleration of the system
		 * @param kGyro porpotional constant for gyro feed-back control, one side should get a positive value and the other one a negative value
		 */
		public GyroMPValue(PIDvalue pid, double kv, double ka, double kGyro)
		{
			super(pid, kv, ka);
			this.kGyro = kGyro;
		}
		
	}
	
	
    private AnalogGyro gyro;
	private double angleTolerance;
	
    public GyroMPController(MPValue values, SpeedController motor, PIDSource source)
    {
    	super(values, motor, source);
    }
    
	/**
	 * @param values	Motion Profiling values for generating the path
	 * @param motor		The motor to which we output the calculated value
	 * @param source	The sensor used for following the path
	 * @param profileType	The Motion Profiling trajectory type
	 */
	
	public GyroMPController(MPValue values, SpeedController motor, PIDSource source, AnalogGyro gyro) 
	{
		super(values, motor, source);
		this.gyro = gyro;
	}
    
	public void calculate()
    {
		if(!enabled)
		{
			try
			{
				disableTask();
			} catch (InterruptedException e) {
				Utils.printErr(e.toString());
			}
			return;
		}
		
    	now = Timer.getFPGATimestamp();
    	double dt = now - lastTime;
    	passedTime = now - startTime;
    	

    	// Indication if on target or trajectory time has passed
		if(passedTime > trajectory.getTotalTime())
		{
			setpoint.position = this.trajectory.getDistance();
			setpoint.velocity = 0;
			setpoint.acceleration = 0;
			
		}
		else
		{
			setpoint = trajectory.calculate(now - startTime);
		}
		
		if(setpoint == null) 
		{
			Utils.printErr("[MPController] Error in Trajectory Calculate(), setpoint is null");
			return;
		}
		
    	// Get the current position from the sensor
    	double currentPosition = source.pidGet();
    	   	
    	// Calculate the errors
    	double positionError = setpoint.position - currentPosition;
    	
    	
    	// Calculate the PID values
    	double P = positionError * values.positionPid.kP;
    	double I = errorIntegral * values.positionPid.kI;
    	double D = ((positionError - lastError) / dt) * values.positionPid.kD;
    	
    	double output = P + I + D + values.kv * setpoint.velocity + values.ka * setpoint.acceleration;
    	if(this.trajectory instanceof GyroTrajectory && this.gyro != null && this.values instanceof GyroMPValue)
    	{
	    	double headingError = ((GyroTrajectory) this.trajectory).getHeading() - getAngle();
	    	double gyroOutput = headingError * ((GyroMPValue) this.values).kGyro;
    	
	    	output += gyroOutput;
    	}
    	motor.set(output);
    	
    	lastError = positionError;
    	errorIntegral += positionError * dt;
    	lastTime = now;
    	
    }
	
	public void setGyro(AnalogGyro gyro) {
		this.gyro = gyro;
	}

	public void setAngleTolerance(double tolerance)
	{
		this.angleTolerance = tolerance;
	}
	
	public double getAngle()
	{
		return this.gyro.getAngle();
	}
	
	public double getOutput()
	{
		return this.motor.get();
	}
	
	public double getSetpointAngle()
	{
		return ((GyroTrajectory)this.trajectory).getHeading();
	}
	
	public synchronized boolean onAngle()
	{
		if(this.trajectory instanceof GyroTrajectory && this.gyro != null && this.values instanceof GyroMPValue)
    	{
			return Utils.inRange(gyro.getAngle(), ((GyroTrajectory) this.trajectory).getFinalAngle(), this.angleTolerance);
    	}
		return true;
	}

	public double getAngleTolerance()
	{
		return this.angleTolerance;
	}
}