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

public class GyroMPController implements Sendable 
{

	/**
	 * A <b>structure</b> for storing Motion Profiling values
	 */
	public static class MPValue
	{
		public PIDvalue positionPid;
		public double kv;
		public double ka;
		public double kGyro;
		
		public MPValue() {}
		
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
		public MPValue(PIDvalue pid, double kv, double ka, double kGyro)
		{
			this.positionPid = pid;
			this.kv = kv;
			this.ka = ka;
			this.kGyro = kGyro;
		}
		
	}
	
	/**
	 * An asynchronous task for calculating the output and
	 * following the path
	 */
    private class GyroMPTask extends TimerTask 
    {

        private GyroMPController controller;
        
        public GyroMPTask(GyroMPController controller)
        { 
        	super();
            this.controller = controller;
            controller.currTask = this;
        }

       @Override
       public void run() 
       {
            controller.calculate();
       }
        
    }

    private MPValue values;
	private SpeedController motor;
	private PIDSource source;
	private AnalogGyro gyro;
	private GyroTrajectory trajectory;
	
	private double positionTolerance = 0.1;
	private double angleTolerance = 1;
	private boolean enabled = false; // fixed you alon <3
	private java.util.Timer controllerLoop;
	private final double period = 0.01; 
	
	private GyroSetpoint setpoint;
	private double startTime;
	

	private double passedTime;
	private double now;
	private double dt;
	private double lastTime;
	private double lastError;
	private double errorIntegral;
	
	private TimerTask currTask;

	/**
	 * @param values	Motion Profiling values for generating the path
	 * @param motor		The motor to which we output the calculated value
	 * @param source	The sensor used for following the path
	 * @param profileType	The Motion Profiling trajectory type
	 */
	
	public GyroMPController(MPValue values, SpeedController motor, PIDSource source, AnalogGyro gyro) 
	{
		this.values = values;
		this.motor = motor;
		this.source = source;
		this.gyro = gyro;
		
		// make sure the sensor will give us position and not rate
		source.setPIDSourceType(PIDSourceType.kDisplacement);
				
		controllerLoop = new java.util.Timer();
		controllerLoop.schedule(new GyroMPTask(this), 0L, (long) (period * 1000));
	}
    
    public void setTrajectory(GyroTrajectory trajectory)
    {
    	this.trajectory = trajectory;
    	this.setpoint = new GyroSetpoint();
		this.now = 0;
		this.lastTime = 0;
		this.lastError = 0;
		this.errorIntegral = 0;
		startTime = Timer.getFPGATimestamp();
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
 
    	double headingError = setpoint.heading - this.gyro.getAngle();
    	double gyroOutput = headingError * this.values.kGyro;
    	
    	double output = P + I + D + values.kv * setpoint.velocity + values.ka * setpoint.acceleration + gyroOutput;
    	
    	motor.set(output);
    	
    	lastError = positionError;
    	errorIntegral += positionError * dt;
    	lastTime = now;
    	
    }
	
	private void endTask()
	{
		this.currTask.cancel();
	}
	
	/**
	 * @return	whether the system is on target (using the tolerance)
	 */
	public synchronized boolean onTarget() 
	{
		return Utils.inRange(source.pidGet(), this.trajectory.getDistance(), this.positionTolerance);
	}
	
	public synchronized boolean onAngle()
	{
		return Utils.inRange(gyro.getAngle(), this.trajectory.getFinalAngle(), this.angleTolerance);
	}
	
	public boolean isTimeUp()
	{
		return passedTime > trajectory.getTotalTime();
	}
	
	public MPValue getValues() 
	{
		return values;
	}

	public void setValues(MPValue values)        
	{
		this.values = values;
	}

	public double getPositionTolerance()
	{
		return positionTolerance;
	}
	
	public double getAngleTolerance()
	{
		return this.angleTolerance;
	}

	public void setPositionTolerance(double tolerance) 
	{
		this.positionTolerance = tolerance;
	}
	
	public void setAngleTolerance(double tolerance)
	{
		this.angleTolerance = tolerance;
	}
	
	public void enable() throws IllegalMonitorStateException
	{
		synchronized(currTask)
		{
			if(this.enabled == false)
			{
				this.enabled = true;
				currTask.notify();
			}
		}
	}

	private void disableTask() throws InterruptedException
	{
		synchronized(currTask)
		{
			this.enabled = false;
			currTask.wait();
		}
	}
	
	public void disable()
	{
		this.enabled = false;
	}
	
	public boolean isEnabled()
	{
		return enabled;
	}
	
	public Trajectory3075 getTrajectory() 
	{
		return trajectory;
	}
	
	public double getPositionError()
	{
		return setpoint.position - source.pidGet();
	}
	
	private ITable m_table;
	
	private final ITableListener m_listener = (table, key, value, isNew) -> {
	    if (key.equals("P") || key.equals("I") || key.equals("D") || key.equals("kV") || key.equals("kA")) {
	      if (values.positionPid.kP != table.getNumber("P", 0.0) || values.positionPid.kI != table.getNumber("I", 0.0)
	          || values.positionPid.kD != table.getNumber("D", 0.0)
//	          || values.kv != table.getNumber("kV", 0.0)
	          || values.ka != table.getNumber("kA", 0.0)) 
	      {
	        values.positionPid.kP = table.getNumber("P", 0.0);
	        values.positionPid.kI = table.getNumber("I", 0.0);
	        values.positionPid.kD = table.getNumber("D", 0.0);
	       
//	        values.kv = table.getNumber("kV", 0.0);
	        values.ka = table.getNumber("kA", 0.0);
	      }
	    }
	  };
	  
	public void initTable(ITable subtable) {
		m_table = subtable;
		m_table.putNumber("P", values.positionPid.kP);
		m_table.putNumber("I", values.positionPid.kI);
		m_table.putNumber("D", values.positionPid.kD);
		m_table.putNumber("kV", values.kv);
		m_table.putNumber("kA", values.ka);
		
		m_table.addTableListener(m_listener, false);
		updateTable();
	}

	public ITable getTable() {
		// TODO Auto-generated method stub
		return m_table;
	}

	public String getSmartDashboardType() {
		// TODO Auto-generated method stub
		return "Motion Profiling Controller";
	}
	
	public void updateTable() 
	{
	    if (m_table != null && setpoint != null) 
	    {
	      	m_table.putBoolean("Enabled", enabled);
	    	m_table.putNumber("Position Setpoint", setpoint.position);
	    	m_table.putNumber("Velocity Setpoint", setpoint.velocity);
	    	m_table.putNumber("Acceleration Setpoint", setpoint.acceleration);
	      	m_table.putNumber("Sensor position", source.pidGet());
	      	m_table.putNumber("Position Error", getPositionError());
	    }
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setName(String name) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getSubsystem() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setSubsystem(String subsystem) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
		
	}
	
	public Trajectory3075.Setpoint getSetpoint() {
		return setpoint;
	}
	
	public double getPassedTime()
	{
		return this.passedTime;
	}
}