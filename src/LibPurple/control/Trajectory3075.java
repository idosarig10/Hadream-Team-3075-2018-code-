package LibPurple.control;


public abstract class Trajectory3075 
{
	public static enum Type { SinosoidalMotionProfile, TrapizoidalMotionProfile }

	public static class Setpoint 
	{
		public double velocity;
		public double position;
		public double acceleration;
		
		
		public Setpoint(double position, double velocity, double acceleration)
		{
			this.velocity = velocity;
			this.position = position;
			this.acceleration = acceleration;
		}
		
		public Setpoint()
		{
			this.velocity = 0;
			this.position = 0;
			this.acceleration = 0;
		}
		
	}

	protected double maxV;
	protected double maxA;
	protected Trajectory3075.Setpoint setpoint;

	
	/**
	 * Calculate setpoint for current time, for a trajectory
	 * @param time Time passed since the beginning of the motion
	 * @return Trajectory's set-point for the given time
	 */
	public abstract Setpoint calculate(double time);
	
	public abstract double getTotalTime();
	
	public abstract double getDirection();
	
	public Setpoint getSetpoint() 
	{
		return setpoint;
	}
	
	public void setSetpoint(Setpoint setpoint) 
	{
		this.setpoint = setpoint;
	}
	
	public abstract double getDistance();
	
	/**
	 * this will set a new distance goal but WILL NOT generate a new trajectory;
	 * it will just save the value for later use.
	 * @param distance
	 */
	public abstract void setDistance(double distance);
	
}
